mod media;
use anyhow::{Context, Result, bail, ensure};
use clap::{Parser, Subcommand};
use kynput::{GamepadAxis, GamepadEvent, InputPacket, InputTarget, InputType, Payload};
use serde::{Deserialize, Serialize};
use std::{
    fs::OpenOptions,
    io::Write,
    net::SocketAddr,
    path::{Path, PathBuf},
    sync::Arc,
    time::{Duration, Instant, SystemTime, UNIX_EPOCH},
};
use tokio::io::{AsyncReadExt, AsyncWriteExt};

const ALPN: &[u8] = b"so101-kynput-position-v1";
const FRAME_LEN: usize = 24 + 6 * 9;
const DEADLINE: Duration = Duration::from_millis(200);
const AXES: [GamepadAxis; 6] = [
    GamepadAxis::LeftX,
    GamepadAxis::LeftY,
    GamepadAxis::RightX,
    GamepadAxis::RightY,
    GamepadAxis::TriggerLeft,
    GamepadAxis::TriggerRight,
];

#[derive(Parser)]
#[command(about = "SO-101 position samples carried by kynput over authenticated KyNet QUIC")]
struct Cli {
    #[command(subcommand)]
    command: Command,
}
#[derive(Subcommand)]
enum Command {
    /// Serve GStreamer H.264 through an authenticated KyProto GOP video channel.
    CameraServe {
        #[arg(long, default_value = "127.0.0.1:7446")]
        listen: SocketAddr,
        #[arg(long)]
        credentials: PathBuf,
    },
    /// Receive H.264 from the KyProto video channel; stdout is an Annex-B stream.
    CameraView {
        #[arg(long)]
        remote: SocketAddr,
        #[arg(long)]
        credentials: PathBuf,
        #[arg(long, default_value = "so101-server")]
        server_name: String,
    },
    /// Receive one authenticated publisher; record positions, never open motors.
    Receive {
        #[arg(long, default_value = "127.0.0.1:7445")]
        listen: SocketAddr,
        #[arg(long)]
        credentials: PathBuf,
        #[arg(long)]
        output: PathBuf,
    },
    /// Publish NDJSON samples from stdin, or generate a hardware-free signal.
    Publish {
        #[arg(long)]
        remote: SocketAddr,
        #[arg(long)]
        credentials: PathBuf,
        #[arg(long, default_value = "so101-server")]
        server_name: String,
        #[arg(long)]
        simulate: bool,
        #[arg(long)]
        seconds: Option<f64>,
        #[arg(long, default_value_t = 30)]
        hz: u32,
    },
}

#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
struct Sample {
    captured_unix_ns: u64,
    positions: [u16; 6],
}

fn now_ns() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap()
        .as_nanos() as u64
}

fn encode(challenge: u64, seq: u64, sample: &Sample) -> Result<Vec<u8>> {
    ensure!(
        sample.positions.iter().all(|&x| x <= 4095),
        "Position outside 0..4095"
    );
    let mut buf = Vec::with_capacity(FRAME_LEN);
    buf.extend(challenge.to_be_bytes());
    buf.extend(seq.to_be_bytes());
    buf.extend(sample.captured_unix_ns.to_be_bytes());
    for (axis, &position) in AXES.iter().zip(&sample.positions) {
        // Project-specific virtual six-axis controller. Values preserve raw ticks exactly.
        // These packets are consumed by our receiver, never injected into an OS gamepad.
        let packet = InputPacket::new(
            InputTarget::Host,
            Payload::GamepadEvent(GamepadEvent::Axis {
                id: 101,
                type_: *axis,
                value: position as i16,
            }),
        );
        buf.push(packet.get_type() as u8);
        buf.extend(packet.serialize());
    }
    ensure!(buf.len() == FRAME_LEN, "Unexpected kynput packet layout");
    Ok(buf)
}

fn decode(buf: &[u8], challenge: u64, previous: u64) -> Result<(u64, Sample)> {
    ensure!(buf.len() == FRAME_LEN, "Invalid frame length");
    let number = |i| u64::from_be_bytes(buf[i..i + 8].try_into().unwrap());
    ensure!(number(0) == challenge, "Expired challenge");
    let seq = number(8);
    ensure!(seq > previous, "Duplicate or reordered sample");
    let mut positions = [0u16; 6];
    for (i, chunk) in buf[24..].chunks_exact(9).enumerate() {
        // Validate lengths/discriminants before calling upstream deserializers, which
        // assume well-formed buffers and can panic on malformed input.
        ensure!(
            chunk[0] == InputType::GamepadEvent as u8
                && chunk[1] == 1
                && chunk[2..6] == 101u32.to_be_bytes()
                && chunk[6] == AXES[i] as u8,
            "Invalid joint event/order"
        );
        let packet = InputPacket::deserialize(chunk[0], InputTarget::Host, &chunk[1..])?;
        match packet.payload {
            Payload::GamepadEvent(GamepadEvent::Axis { value, .. })
                if (0..=4095).contains(&value) =>
            {
                positions[i] = value as u16;
            }
            _ => bail!("Invalid position event"),
        }
    }
    Ok((
        seq,
        Sample {
            captured_unix_ns: number(16),
            positions,
        },
    ))
}

fn certs(path: &Path) -> Result<Vec<rustls::pki_types::CertificateDer<'static>>> {
    rustls_pemfile::certs(&mut std::io::BufReader::new(std::fs::File::open(path)?))
        .collect::<std::io::Result<Vec<_>>>()
        .context("Read certificate")
}
fn key(path: &Path) -> Result<rustls::pki_types::PrivateKeyDer<'static>> {
    rustls_pemfile::private_key(&mut std::io::BufReader::new(std::fs::File::open(path)?))?
        .context("Missing private key")
}
fn roots(dir: &Path) -> Result<rustls::RootCertStore> {
    let mut store = rustls::RootCertStore::empty();
    for cert in certs(&dir.join("ca.crt"))? {
        store.add(cert)?;
    }
    Ok(store)
}
fn transport() -> Arc<quinn::TransportConfig> {
    let mut config = quinn::TransportConfig::default();
    config.max_idle_timeout(Some(Duration::from_secs(3).try_into().unwrap()));
    config.keep_alive_interval(Some(Duration::from_millis(250)));
    Arc::new(config)
}

async fn receive(listen: SocketAddr, credentials: &Path, output: &Path) -> Result<()> {
    // Refuse overwrite; opening before accepting a client catches storage errors early.
    let mut journal = OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(output)?;
    let verifier =
        rustls::server::WebPkiClientVerifier::builder(Arc::new(roots(credentials)?)).build()?;
    let mut tls = rustls::ServerConfig::builder()
        .with_client_cert_verifier(verifier)
        .with_single_cert(
            certs(&credentials.join("cert.pem"))?,
            key(&credentials.join("key.pem"))?,
        )?;
    tls.alpn_protocols = vec![ALPN.to_vec()];
    let mut config = quinn::ServerConfig::with_crypto(Arc::new(
        quinn::crypto::rustls::QuicServerConfig::try_from(tls)?,
    ));
    config.transport = transport();
    let endpoint = quinn::Endpoint::server(config, listen)?;
    eprintln!(
        "Listening on {} (record only; one publisher per session)",
        endpoint.local_addr()?
    );
    let incoming = endpoint.accept().await.context("Listener closed")?;
    let raw = tokio::time::timeout(Duration::from_secs(5), incoming).await??;
    let conn = kynet::Connection::from(raw);
    let (mut send, mut recv) =
        tokio::time::timeout(Duration::from_secs(3), conn.accept_bi()).await??;
    let mut marker = [0];
    tokio::time::timeout(DEADLINE, recv.read_exact(&mut marker)).await??;
    ensure!(marker == *b"P", "Invalid protocol marker");
    let mut seq = 0;
    let mut challenge = 1u64;
    let outcome: Result<()> = async {
        loop {
            let issued = Instant::now();
            let mut frame = [0u8; FRAME_LEN];
            tokio::time::timeout(DEADLINE, async {
                send.write_all(&challenge.to_be_bytes()).await?;
                recv.read_exact(&mut frame).await?;
                Ok::<_, anyhow::Error>(())
            }).await.context("Publisher freshness watchdog expired")??;
            if frame[..8] == challenge.to_be_bytes() && frame[8..].iter().all(|&byte| byte == 0) {
                break Ok(());
            }
            let (new_seq, sample) = decode(&frame, challenge, seq)?;
            let record = serde_json::json!({"version":1,"seq":new_seq,
                "captured_unix_ns":sample.captured_unix_ns,"received_unix_ns":now_ns(),
                "positions":sample.positions,"challenge_round_trip_ms":issued.elapsed().as_secs_f64()*1000.0});
            let line = serde_json::to_string(&record)?;
            writeln!(journal, "{line}")?;
            journal.flush()?;
            println!("{line}");
            seq = new_seq;
            challenge = challenge.checked_add(1).context("Challenge overflow")?;
        }
    }.await;
    conn.close(0, "Position session ended; do not auto-resume motion");
    journal.sync_all()?;
    eprintln!("Session ended after {seq} accepted samples");
    outcome
}

async fn publish(
    remote: SocketAddr,
    credentials: &Path,
    server_name: &str,
    simulate: bool,
    seconds: Option<f64>,
    hz: u32,
) -> Result<()> {
    ensure!(
        (10..=120).contains(&hz),
        "hz must be 10..120 to meet the watchdog deadline"
    );
    ensure!(
        seconds.is_none_or(|s| s.is_finite() && s > 0.0),
        "Invalid duration"
    );
    let (source_tx, mut source_rx) = tokio::sync::watch::channel::<Option<Sample>>(None);
    // A detached OS reader allows shutdown even if terminal stdin remains blocked.
    // A watch channel keeps only the newest sample, avoiding a motion-command backlog.
    if !simulate {
        std::thread::spawn(move || {
            let result: Result<()> = (|| {
                let mut stdin = std::io::stdin().lock();
                loop {
                    let mut line = Vec::new();
                    let mut bounded = std::io::Read::take(&mut stdin, 1025);
                    let size = std::io::BufRead::read_until(&mut bounded, b'\n', &mut line)?;
                    if size == 0 {
                        break;
                    }
                    ensure!(
                        size <= 1024 && line.last() == Some(&b'\n'),
                        "Oversized/unterminated sample"
                    );
                    let sample: Sample = serde_json::from_slice(&line)?;
                    ensure!(
                        sample.positions.iter().all(|&x| x <= 4095),
                        "Invalid raw ticks"
                    );
                    source_tx.send_replace(Some(sample));
                }
                Ok(())
            })();
            if let Err(error) = result {
                eprintln!("Leader source failed: {error:#}");
            }
        });
    }
    let mut tls = rustls::ClientConfig::builder()
        .with_root_certificates(roots(credentials)?)
        .with_client_auth_cert(
            certs(&credentials.join("cert.pem"))?,
            key(&credentials.join("key.pem"))?,
        )?;
    tls.alpn_protocols = vec![ALPN.to_vec()];
    let mut config = quinn::ClientConfig::new(Arc::new(
        quinn::crypto::rustls::QuicClientConfig::try_from(tls)?,
    ));
    config.transport_config(transport());
    let bind = if remote.is_ipv4() {
        "0.0.0.0:0"
    } else {
        "[::]:0"
    };
    let endpoint = quinn::Endpoint::client(bind.parse()?)?;
    // LeRobot imports/USB handshake can take seconds. Wait for the first
    // source sample BEFORE connecting, while no live watchdog/session exists.
    if !simulate && source_rx.borrow().is_none() {
        eprintln!("Waiting up to 30 seconds for the LeRobot leader source");
        tokio::time::timeout(Duration::from_secs(30), source_rx.changed())
            .await
            .context("Leader did not initialize within 30 seconds")??;
    }
    let raw = tokio::time::timeout(
        Duration::from_secs(5),
        endpoint.connect_with(config, remote, server_name)?,
    )
    .await??;
    let conn = kynet::Connection::from(raw);
    let (mut send, mut recv) = conn.open_bi().await?;
    // QUIC streams become visible to the receiver after their first byte.
    // Send a protocol marker before waiting for the first server challenge.
    send.write_all(b"P").await?;
    let start = Instant::now();
    let mut interval = tokio::time::interval(Duration::from_secs_f64(1.0 / hz as f64));
    interval.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
    let mut seq = 0;
    let outcome: Result<()> = async {
        loop {
            if seconds.is_some_and(|s| start.elapsed().as_secs_f64() >= s) {
                break;
            }
            let mut nonce = [0u8; 8];
            tokio::time::timeout(Duration::from_secs(2), recv.read_exact(&mut nonce)).await??;
            interval.tick().await;
            let sample = if simulate {
                let t = start.elapsed().as_secs_f64();
                Sample {
                    captured_unix_ns: now_ns(),
                    positions: std::array::from_fn(|i| {
                        (2048.0 + 300.0 * (t + i as f64 / 4.0).sin()) as u16
                    }),
                }
            } else {
                if source_rx.borrow().is_none() {
                    tokio::time::timeout(Duration::from_millis(100), source_rx.changed()).await??;
                }
                source_rx.borrow().clone().context("Input source ended")?
            };
            let age = now_ns()
                .checked_sub(sample.captured_unix_ns)
                .context("Source timestamp is in the future")?;
            ensure!(
                age <= 100_000_000,
                "Leader sample older than 100 ms; refusing stale input"
            );
            seq += 1;
            let frame = encode(u64::from_be_bytes(nonce), seq, &sample)?;
            tokio::time::timeout(DEADLINE, send.write_all(&frame)).await??;
        }
        Ok(())
    }
    .await;
    // Wait for the last frame to be accepted before closing.
    if outcome.is_ok() && seq > 0 {
        let mut acknowledgement = [0; 8];
        tokio::time::timeout(DEADLINE, recv.read_exact(&mut acknowledgement)).await??;
        let mut stop = [0; FRAME_LEN];
        stop[..8].copy_from_slice(&acknowledgement);
        tokio::time::timeout(DEADLINE, send.write_all(&stop)).await??;
        tokio::time::timeout(Duration::from_secs(1), conn.closed()).await??;
    }
    conn.close(0, "Publisher stopped");
    endpoint.wait_idle().await;
    eprintln!("Published {seq} position samples");
    outcome
}

#[tokio::main]
async fn main() -> Result<()> {
    rustls::crypto::ring::default_provider()
        .install_default()
        .ok();
    match Cli::parse().command {
        Command::CameraServe {
            listen,
            credentials,
        } => media::serve(listen, &credentials).await,
        Command::CameraView {
            remote,
            credentials,
            server_name,
        } => media::view(remote, &credentials, &server_name).await,
        Command::Receive {
            listen,
            credentials,
            output,
        } => receive(listen, &credentials, &output).await,
        Command::Publish {
            remote,
            credentials,
            server_name,
            simulate,
            seconds,
            hz,
        } => publish(remote, &credentials, &server_name, simulate, seconds, hz).await,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn exact_ticks_and_atomic_order() {
        let sample = Sample {
            captured_unix_ns: 123,
            positions: [0, 4095, 2048, 1, 200, 3000],
        };
        let frame = encode(4, 8, &sample).unwrap();
        let (seq, decoded) = decode(&frame, 4, 7).unwrap();
        assert_eq!(seq, 8);
        assert_eq!(decoded.positions, sample.positions);
        assert!(decode(&frame, 4, 8).is_err());
        assert!(decode(&frame, 5, 7).is_err());
        for length in 0..FRAME_LEN {
            assert!(decode(&frame[..length], 4, 7).is_err());
        }
        let mut corrupt = frame;
        corrupt[30] = 255;
        assert!(decode(&corrupt, 4, 7).is_err());
    }
}
