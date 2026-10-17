mod media;
mod tls;
use anyhow::{Context, Result, ensure};
use clap::{Parser, Subcommand};
use serde::{Deserialize, Serialize};
use std::{
    collections::{BTreeMap, VecDeque},
    fs::OpenOptions,
    io::Write,
    net::SocketAddr,
    path::{Path, PathBuf},
    sync::Arc,
    time::{Duration, Instant, SystemTime, UNIX_EPOCH},
};
use tokio::io::{AsyncReadExt, AsyncWriteExt};

const ALPN: &[u8] = b"so101-lerobot-action-datagram-v4";
const FRAME_LEN: usize = 24 + 6 * 8;
const DEADLINE: Duration = Duration::from_millis(200);
const JOINTS: [&str; 6] = [
    "shoulder_pan.pos",
    "shoulder_lift.pos",
    "elbow_flex.pos",
    "wrist_flex.pos",
    "wrist_roll.pos",
    "gripper.pos",
];

#[derive(Parser)]
#[command(about = "Native LeRobot SO-101 actions over Kyber KyNet QUIC")]
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
        #[arg(long, default_value = "credentials/server")]
        credentials: PathBuf,
        /// Encrypted LAN/Tailscale connection without application credentials or peer identity verification.
        #[arg(long, alias = "tailscale")]
        insecure_lan: bool,
        #[arg(long)]
        status_file: Option<PathBuf>,
    },
    /// Receive H.264 from the KyProto video channel; stdout is an Annex-B stream.
    CameraView {
        #[arg(long)]
        remote: SocketAddr,
        #[arg(long, default_value = "credentials/leader")]
        credentials: PathBuf,
        /// Encrypted LAN/Tailscale connection without application credentials or peer identity verification.
        #[arg(long, alias = "tailscale")]
        insecure_lan: bool,
        #[arg(long, default_value = "so101-server")]
        server_name: String,
        #[arg(long)]
        status_file: Option<PathBuf>,
    },
    /// Receive one authenticated publisher; record positions, never open motors.
    Receive {
        #[arg(long, default_value = "127.0.0.1:7445")]
        listen: SocketAddr,
        #[arg(long, default_value = "credentials/server")]
        credentials: PathBuf,
        /// Encrypted LAN/Tailscale connection without application credentials or peer identity verification.
        #[arg(long, alias = "tailscale")]
        insecure_lan: bool,
        #[arg(long)]
        output: PathBuf,
        /// Maximum rate of latest-action delivery to the follower.
        #[arg(long, default_value_t = 120)]
        hz: u32,
    },
    /// Publish NDJSON samples from stdin, or generate a hardware-free signal.
    Publish {
        #[arg(long)]
        remote: SocketAddr,
        #[arg(long, default_value = "credentials/leader")]
        credentials: PathBuf,
        /// Encrypted LAN/Tailscale connection without application credentials or peer identity verification.
        #[arg(long, alias = "tailscale")]
        insecure_lan: bool,
        #[arg(long, default_value = "so101-server")]
        server_name: String,
        #[arg(long)]
        simulate: bool,
        #[arg(long)]
        seconds: Option<f64>,
        #[arg(long, default_value_t = 120)]
        hz: u32,
    },
}

#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
struct Sample {
    captured_unix_ns: u64,
    action: BTreeMap<String, f64>,
}

fn now_ns() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap()
        .as_nanos() as u64
}

fn validate_action(action: &BTreeMap<String, f64>) -> Result<()> {
    ensure!(
        action.len() == JOINTS.len(),
        "Exactly six LeRobot .pos keys required"
    );
    for (index, name) in JOINTS.iter().enumerate() {
        let value = *action.get(*name).context("Missing LeRobot joint")?;
        let valid = if index == 5 {
            (0.0..=100.0).contains(&value)
        } else {
            (-360.0..=360.0).contains(&value)
        };
        ensure!(
            value.is_finite() && valid,
            "Invalid LeRobot joint value: {name}"
        );
    }
    Ok(())
}

fn encode(challenge: u64, seq: u64, sample: &Sample) -> Result<Vec<u8>> {
    validate_action(&sample.action)?;
    let mut buf = Vec::with_capacity(FRAME_LEN);
    buf.extend(challenge.to_be_bytes());
    buf.extend(seq.to_be_bytes());
    buf.extend(sample.captured_unix_ns.to_be_bytes());
    for name in JOINTS {
        buf.extend(sample.action[name].to_be_bytes());
    }
    Ok(buf)
}

fn decode(buf: &[u8], challenge: u64, previous: u64) -> Result<(u64, Sample)> {
    ensure!(buf.len() == FRAME_LEN, "Invalid frame length");
    let number = |i| u64::from_be_bytes(buf[i..i + 8].try_into().unwrap());
    ensure!(number(0) == challenge, "Expired challenge");
    let seq = number(8);
    ensure!(seq > previous, "Duplicate or reordered sample");
    let action = JOINTS
        .iter()
        .zip(buf[24..].chunks_exact(8))
        .map(|(name, chunk)| {
            (
                name.to_string(),
                f64::from_be_bytes(chunk.try_into().unwrap()),
            )
        })
        .collect();
    validate_action(&action)?;
    Ok((
        seq,
        Sample {
            captured_unix_ns: number(16),
            action,
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
    // A command fits in one datagram. Prefer new actions when the network stalls.
    config.datagram_send_buffer_size(FRAME_LEN * 2);
    config.datagram_receive_buffer_size(Some(FRAME_LEN * 4));
    Arc::new(config)
}

#[derive(Default)]
struct Challenges {
    issued: VecDeque<(u64, Instant)>,
    next: u64,
}

impl Challenges {
    fn issue(&mut self, now: Instant) -> Result<u64> {
        while self
            .issued
            .front()
            .is_some_and(|(_, at)| now.duration_since(*at) > DEADLINE)
        {
            self.issued.pop_front();
        }
        self.next = self.next.checked_add(1).context("Challenge overflow")?;
        self.issued.push_back((self.next, now));
        Ok(self.next)
    }

    fn age(&self, nonce: u64, now: Instant) -> Option<Duration> {
        self.issued.iter().find_map(|(value, at)| {
            let age = now.duration_since(*at);
            (*value == nonce && age <= DEADLINE).then_some(age)
        })
    }
}

async fn receive(
    listen: SocketAddr,
    credentials: &Path,
    output: &Path,
    insecure_lan: bool,
    hz: u32,
) -> Result<()> {
    ensure!((10..=120).contains(&hz), "hz must be 10..120");
    // Refuse overwrite; opening before accepting a client catches storage errors early.
    let mut journal = OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(output)?;
    tls::address(listen, insecure_lan)?;
    let config = tls::server(credentials, insecure_lan, ALPN)?;
    let endpoint = quinn::Endpoint::server(config, listen)?;
    eprintln!(
        "Listening on {} (record only; one publisher per session; max {hz} Hz)",
        endpoint.local_addr()?
    );
    let incoming = endpoint.accept().await.context("Listener closed")?;
    let raw = tokio::time::timeout(Duration::from_secs(5), incoming).await??;
    tls::address(raw.remote_address(), insecure_lan)?;
    let conn = kynet::Connection::from(raw);
    let (_send, mut recv) =
        tokio::time::timeout(Duration::from_secs(3), conn.accept_bi()).await??;
    let mut marker = [0];
    tokio::time::timeout(DEADLINE, recv.read_exact(&mut marker)).await??;
    ensure!(marker == *b"P", "Invalid protocol marker");
    let mut seq = 0;
    let mut delivered = 0;
    let mut challenges = Challenges::default();
    let mut pending = None;
    let mut last_input = tokio::time::Instant::now();
    let mut beacons = tokio::time::interval(Duration::from_millis(20));
    beacons.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
    let mut interval = tokio::time::interval(Duration::from_secs_f64(1.0 / hz as f64));
    interval.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
    let mut stop = [0];
    let outcome: Result<()> = async {
        loop {
            tokio::select! {
                _ = tokio::time::sleep_until(last_input + DEADLINE) => {
                    anyhow::bail!("Publisher freshness watchdog expired");
                }
                result = recv.read_exact(&mut stop) => {
                    result?;
                    ensure!(stop == *b"S", "Invalid stop marker");
                    break Ok(());
                }
                _ = beacons.tick() => {
                    let nonce = challenges.issue(Instant::now())?;
                    conn.send_datagram(bytes::Bytes::copy_from_slice(&nonce.to_be_bytes())).await?;
                }
                result = conn.read_datagram() => {
                    let frame = result?;
                    ensure!(frame.len() == FRAME_LEN, "Invalid action datagram length");
                    let nonce = u64::from_be_bytes(frame[..8].try_into()?);
                    let incoming_seq = u64::from_be_bytes(frame[8..16].try_into()?);
                    // Datagram loss/reordering is normal. Neither old sequence numbers
                    // nor an expired challenge can refresh the input watchdog.
                    if incoming_seq <= seq { continue; }
                    let Some(age) = challenges.age(nonce, Instant::now()) else { continue; };
                    let (new_seq, sample) = decode(&frame, nonce, seq)?;
                    seq = new_seq;
                    last_input = tokio::time::Instant::now();
                    pending = Some((new_seq, sample, now_ns(), age));
                }
                _ = interval.tick() => {
                    let Some((new_seq, sample, received, age)) = pending.take() else { continue; };
                    ensure!(now_ns().saturating_sub(received) <= 100_000_000, "Local action delivery stalled");
                    let record = serde_json::json!({"version":3,"transport_version":4,
                        "units":"lerobot_degrees_gripper_percent","seq":new_seq,
                        "captured_unix_ns":sample.captured_unix_ns,"received_unix_ns":received,
                        "action":sample.action,"challenge_age_ms":age.as_secs_f64()*1000.0});
                    let line = serde_json::to_string(&record)?;
                    writeln!(journal, "{line}")?;
                    println!("{line}");
                    delivered += 1;
                }
            }
        }
    }.await;
    conn.close(0, "Position session ended; do not auto-resume motion");
    journal.sync_all()?;
    eprintln!("Session ended after {delivered} delivered samples (last sequence {seq})");
    outcome
}

async fn publish(
    remote: SocketAddr,
    credentials: &Path,
    server_name: &str,
    simulate: bool,
    seconds: Option<f64>,
    hz: u32,
    insecure_lan: bool,
) -> Result<()> {
    tls::address(remote, insecure_lan)?;
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
                    validate_action(&sample.action)?;
                    source_tx.send_replace(Some(sample));
                }
                Ok(())
            })();
            if let Err(error) = result {
                eprintln!("Leader source failed: {error:#}");
            }
        });
    }
    let config = tls::client(credentials, insecure_lan, ALPN)?;
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
    tls::address(raw.remote_address(), insecure_lan)?;
    let conn = kynet::Connection::from(raw);
    let (mut send, _recv) = conn.open_bi().await?;
    // QUIC streams become visible to the receiver after their first byte.
    // Send a protocol marker before waiting for the first server challenge.
    send.write_all(b"P").await?;
    let start = Instant::now();
    let mut interval = tokio::time::interval(Duration::from_secs_f64(1.0 / hz as f64));
    interval.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
    let mut seq = 0;
    let mut challenge = None;
    let outcome: Result<()> = async {
        loop {
            tokio::select! {
                result = conn.read_datagram() => {
                    let beacon = result?;
                    ensure!(beacon.len() == 8, "Invalid challenge datagram");
                    let nonce = u64::from_be_bytes(beacon[..].try_into()?);
                    if challenge.is_none_or(|(previous, _)| nonce > previous) {
                        challenge = Some((nonce, Instant::now()));
                    }
                    continue;
                }
                _ = interval.tick() => {}
            }
            if seconds.is_some_and(|s| start.elapsed().as_secs_f64() >= s) {
                break;
            }
            let Some((nonce, received)) = challenge else {
                ensure!(start.elapsed() <= DEADLINE, "No server freshness challenge");
                continue;
            };
            ensure!(
                received.elapsed() <= DEADLINE,
                "Server freshness challenge expired"
            );
            let sample = if simulate {
                let t = start.elapsed().as_secs_f64();
                Sample {
                    captured_unix_ns: now_ns(),
                    action: JOINTS
                        .iter()
                        .enumerate()
                        .map(|(i, name)| {
                            let value = if i == 5 {
                                50.0 + 20.0 * t.sin()
                            } else {
                                30.0 * (t + i as f64 / 4.0).sin()
                            };
                            (name.to_string(), value)
                        })
                        .collect(),
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
            let frame = encode(nonce, seq, &sample)?;
            conn.send_datagram(bytes::Bytes::from(frame)).await?;
        }
        Ok(())
    }
    .await;
    if outcome.is_ok() {
        tokio::time::timeout(DEADLINE, send.write_all(b"S")).await??;
        tokio::time::timeout(Duration::from_secs(1), conn.closed()).await??;
    }
    if let Err(error) = &outcome {
        conn.close(1, &format!("Publisher failed: {error}"));
    } else {
        conn.close(0, "Publisher stopped");
    }
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
            insecure_lan,
            status_file,
        } => media::serve(listen, &credentials, status_file.as_deref(), insecure_lan).await,
        Command::CameraView {
            remote,
            credentials,
            insecure_lan,
            server_name,
            status_file,
        } => {
            media::view(
                remote,
                &credentials,
                &server_name,
                status_file.as_deref(),
                insecure_lan,
            )
            .await
        }
        Command::Receive {
            listen,
            credentials,
            insecure_lan,
            output,
            hz,
        } => receive(listen, &credentials, &output, insecure_lan, hz).await,
        Command::Publish {
            remote,
            credentials,
            insecure_lan,
            server_name,
            simulate,
            seconds,
            hz,
        } => {
            publish(
                remote,
                &credentials,
                &server_name,
                simulate,
                seconds,
                hz,
                insecure_lan,
            )
            .await
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn pipelined_challenges_retain_the_200_ms_expiry() {
        let mut challenges = Challenges::default();
        let start = Instant::now();
        let first = challenges.issue(start).unwrap();
        let next = challenges.issue(start + Duration::from_millis(20)).unwrap();
        assert!(
            challenges
                .age(first, start + Duration::from_millis(199))
                .is_some()
        );
        assert!(
            challenges
                .age(first, start + Duration::from_millis(201))
                .is_none()
        );
        assert!(
            challenges
                .age(next, start + Duration::from_millis(201))
                .is_some()
        );
        assert!(
            challenges
                .age(999, start + Duration::from_millis(201))
                .is_none()
        );
    }

    #[test]
    fn native_actions_preserve_signed_fractional_units_and_order() {
        let sample = Sample {
            captured_unix_ns: 123,
            action: JOINTS
                .iter()
                .zip([-150.125, 90.5, 0.0, -0.001, 180.0, 66.625])
                .map(|(name, value)| (name.to_string(), value))
                .collect(),
        };
        let frame = encode(4, 8, &sample).unwrap();
        let (seq, decoded) = decode(&frame, 4, 7).unwrap();
        assert_eq!(seq, 8);
        assert_eq!(decoded.action, sample.action);
        assert!(decode(&frame, 4, 8).is_err());
        assert!(decode(&frame, 5, 7).is_err());
        for length in 0..FRAME_LEN {
            assert!(decode(&frame[..length], 4, 7).is_err());
        }
        let mut corrupt = frame;
        corrupt[24..32].copy_from_slice(&f64::NAN.to_be_bytes());
        assert!(decode(&corrupt, 4, 7).is_err());
        let mut invalid = sample.clone();
        invalid.action.insert("gripper.pos".into(), 101.0);
        assert!(encode(4, 8, &invalid).is_err());
        invalid.action.remove("gripper.pos");
        assert!(encode(4, 8, &invalid).is_err());
    }
}
