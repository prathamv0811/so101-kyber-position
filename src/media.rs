//! GStreamer H.264 capture carried by the official KyProto GOP video channel.
use super::{now_ns, tls};
use anyhow::{Context, Result, ensure};
use bytes::Bytes;
use kymux_types::av::{AVPacket, CodecPacket, CodecPacketHeader, MediaPacket, MediaPacketHeader};
use kyproto::VideoProtocol;
use std::{
    net::SocketAddr,
    path::Path,
    time::{Duration, Instant},
};
use tokio::io::{AsyncReadExt, AsyncWriteExt};

const ALPN: &[u8] = b"so101-kyproto-h264-datagram-v2";
const H264: u32 = u32::from_be_bytes(*b"h264");

#[derive(Default)]
struct FrameGate {
    first_pts: Option<u64>,
    min_offset: i128,
    previous_pts: Option<u64>,
    need_key: bool,
    dropped: u64,
    queue_delay_us: u64,
}
impl FrameGate {
    fn accept(&mut self, pts: u64, is_key: bool, elapsed: Duration) -> bool {
        if self.first_pts.is_none() {
            self.first_pts = Some(pts);
            self.min_offset = elapsed.as_micros() as i128;
            self.need_key = true;
        }
        let offset = elapsed.as_micros() as i128 - (pts as i128 - self.first_pts.unwrap() as i128);
        self.min_offset = self.min_offset.min(offset);
        self.queue_delay_us = (offset - self.min_offset).max(0) as u64;
        let stale = self.queue_delay_us > 100_000;
        let gap = self
            .previous_pts
            .is_some_and(|old| pts <= old || pts.saturating_sub(old) > 50_000);
        self.previous_pts = Some(pts);
        if stale || gap {
            self.need_key = true;
        }
        if !stale && is_key {
            self.need_key = false;
        }
        if stale || self.need_key {
            self.dropped += 1;
            return false;
        }
        true
    }
}

fn configuration(data: &[u8]) -> Option<Vec<u8>> {
    let mut starts = Vec::new();
    let mut i = 0;
    while i + 3 < data.len() {
        let length = if data[i..].starts_with(&[0, 0, 0, 1]) {
            4
        } else if data[i..].starts_with(&[0, 0, 1]) {
            3
        } else {
            i += 1;
            continue;
        };
        starts.push((i, length));
        i += length;
    }
    let mut result = Vec::new();
    let (mut sps, mut pps) = (false, false);
    for (index, &(start, prefix)) in starts.iter().enumerate() {
        let end = starts.get(index + 1).map_or(data.len(), |next| next.0);
        if start + prefix >= end {
            continue;
        }
        match data[start + prefix] & 31 {
            7 => sps = true,
            8 => pps = true,
            _ => continue,
        }
        result.extend_from_slice(&data[start..end]);
    }
    (sps && pps).then_some(result)
}

fn packet(payload: Vec<u8>, is_key: bool, is_config: bool, pts: u64) -> AVPacket {
    AVPacket::Media(MediaPacket {
        header: MediaPacketHeader {
            is_config,
            is_key,
            pts,
            size: payload.len() as u32,
        },
        payload: Bytes::from(payload),
    })
}

fn progress(path: Option<&Path>, frames: u64, gate: Option<&FrameGate>) -> Result<()> {
    if let Some(path) = path {
        let temporary = path.with_extension("tmp");
        let mut status = serde_json::json!({
            "updated_unix_ns": now_ns(), "frames": frames,
            "transport": "KyProto Unreliable H.264",
        });
        if let Some(gate) = gate {
            status["queue_delay_ms"] = serde_json::json!(gate.queue_delay_us as f64 / 1000.0);
            status["dropped_frames"] = serde_json::json!(gate.dropped);
        }
        std::fs::write(&temporary, serde_json::to_vec(&status)?)?;
        std::fs::rename(temporary, path)?;
    }
    Ok(())
}

pub async fn serve(
    listen: SocketAddr,
    credentials: &Path,
    status_file: Option<&Path>,
    insecure_lan: bool,
) -> Result<()> {
    tls::address(listen, insecure_lan)?;
    let mut config = tls::server(credentials, insecure_lan, ALPN)?;
    let mut video_transport = super::transport();
    let settings = std::sync::Arc::get_mut(&mut video_transport).unwrap();
    settings.datagram_send_buffer_size(64 * 1024);
    settings.datagram_receive_buffer_size(Some(128 * 1024));
    config.transport = video_transport;
    let endpoint = quinn::Endpoint::server(config, listen)?;
    eprintln!(
        "KyProto H.264 camera listening on {}",
        endpoint.local_addr()?
    );
    // Retry failed camera setup only before video has started. No live stream
    // or motor session is automatically reconnected after a failure.
    let (conn, mut done_tx, mut done_rx, _media, mut video) = loop {
        let incoming = endpoint.accept().await.context("Camera listener closed")?;
        let raw = match tokio::time::timeout(Duration::from_secs(5), incoming).await {
            Ok(Ok(connection)) => connection,
            outcome => {
                eprintln!("Camera TLS setup failed: {outcome:?}; waiting for another client");
                continue;
            }
        };
        tls::address(raw.remote_address(), insecure_lan)?;
        eprintln!("Camera QUIC connected: {}", raw.remote_address());
        let conn = kynet::Connection::from(raw);
        let setup: Result<_> = async {
            let (done_tx, mut done_rx) =
                tokio::time::timeout(Duration::from_secs(5), conn.accept_bi())
                    .await
                    .context("Camera client did not open completion stream within 5 seconds")??;
            let mut marker = [0];
            tokio::time::timeout(Duration::from_secs(3), done_rx.read_exact(&mut marker))
                .await
                .context("Camera client did not send completion marker within 3 seconds")??;
            ensure!(marker == *b"M", "Invalid media completion marker");
            eprintln!("Camera completion stream ready; negotiating KyProto");
            let media = tokio::time::timeout(
                Duration::from_secs(5),
                kyproto::Connection::accept(conn.clone()),
            )
            .await
            .context("Camera KyProto control negotiation exceeded 5 seconds")??;
            let (id, channel) = tokio::time::timeout(
                Duration::from_secs(5),
                media.register_video_endpoint(VideoProtocol::Unreliable),
            )
            .await
            .context("Camera endpoint registration exceeded 5 seconds")??;
            ensure!(id == 0, "Unexpected video endpoint ID");
            eprintln!("Camera endpoint 0 registered; waiting for subscription");
            let video = tokio::time::timeout(Duration::from_secs(5), channel.ready())
                .await
                .context("Camera client did not subscribe to endpoint 0 within 5 seconds")??;
            Ok((done_tx, done_rx, media, video))
        }
        .await;
        match setup {
            Ok((done_tx, done_rx, media, video)) => {
                eprintln!("Camera client subscribed; starting H.264 video");
                break (conn, done_tx, done_rx, media, video);
            }
            Err(error) => {
                conn.close(1, "Camera startup failed; retry client explicitly");
                eprintln!("Camera startup failed: {error:#}; waiting for another client");
            }
        }
    };
    video
        .send(AVPacket::Codec(CodecPacket {
            header: CodecPacketHeader {
                codec: H264,
                rotation: 0,
                frame_size: 0,
            },
        }))
        .await?;
    let mut source = tokio::io::stdin();
    let (mut configured, mut have_key, mut last_pts) = (false, false, 0);
    let mut frames = 0;
    let mut previous_capture = None;
    loop {
        let mut header = [0; 13];
        let first = tokio::time::timeout(Duration::from_secs(1), source.read(&mut header[..1]))
            .await
            .context("GStreamer H.264 source produced no data for 1 second")??;
        if first == 0 {
            break;
        }
        source.read_exact(&mut header[1..]).await?;
        let size = u32::from_be_bytes(header[..4].try_into()?) as usize;
        ensure!(
            (4..=2_000_000).contains(&size) && header[12] <= 1,
            "Invalid H.264 source frame"
        );
        let mut data = vec![0; size];
        source.read_exact(&mut data).await?;
        let captured_ns = u64::from_be_bytes(header[4..12].try_into()?);
        let age = now_ns()
            .checked_sub(captured_ns)
            .context("Future camera timestamp")?;
        if age > 100_000_000 {
            have_key = false;
            continue;
        }
        let is_key = header[12] == 1;
        if previous_capture
            .is_some_and(|previous| captured_ns.saturating_sub(previous) > 50_000_000)
        {
            have_key = false;
        }
        previous_capture = Some(captured_ns);
        let pts = captured_ns / 1000;
        if let Some(config) = configuration(&data) {
            video.send(packet(config, false, true, pts)).await?;
            configured = true;
        }
        if !configured || (!have_key && !is_key) {
            continue;
        }
        tokio::time::timeout(
            Duration::from_millis(500),
            video.send(packet(data, is_key, false, pts)),
        )
        .await
        .context("Camera video send stalled for 500 ms")??;
        have_key = true;
        last_pts = pts;
        frames += 1;
        progress(status_file, frames, None)?;
    }
    done_tx.write_all(&last_pts.to_be_bytes()).await?;
    let mut acknowledgement = [0];
    tokio::time::timeout(
        Duration::from_secs(2),
        done_rx.read_exact(&mut acknowledgement),
    )
    .await??;
    ensure!(acknowledgement == *b"F", "Missing media acknowledgement");
    conn.close(0, "Camera source ended");
    endpoint.wait_idle().await;
    Ok(())
}

pub async fn view(
    remote: SocketAddr,
    credentials: &Path,
    server_name: &str,
    status_file: Option<&Path>,
    insecure_lan: bool,
) -> Result<()> {
    tls::address(remote, insecure_lan)?;
    let mut config = tls::client(credentials, insecure_lan, ALPN)?;
    let mut video_transport = super::transport();
    let settings = std::sync::Arc::get_mut(&mut video_transport).unwrap();
    settings.datagram_send_buffer_size(64 * 1024);
    settings.datagram_receive_buffer_size(Some(128 * 1024));
    config.transport_config(video_transport);
    let bind = if remote.is_ipv4() {
        "0.0.0.0:0"
    } else {
        "[::]:0"
    };
    let endpoint = quinn::Endpoint::client(bind.parse()?)?;
    let raw = tokio::time::timeout(
        Duration::from_secs(5),
        endpoint.connect_with(config, remote, server_name)?,
    )
    .await
    .context("Camera QUIC handshake timed out")??;
    tls::address(raw.remote_address(), insecure_lan)?;
    let conn = kynet::Connection::from(raw);
    let (mut done_tx, mut done_rx) = conn.open_bi().await?;
    done_tx.write_all(b"M").await?;
    let media = tokio::time::timeout(
        Duration::from_secs(3),
        kyproto::Connection::connect(conn.clone()),
    )
    .await
    .context("KyProto camera negotiation timed out")??;
    let channel = media.connect_video_endpoint(0, VideoProtocol::Unreliable)?;
    let mut video = tokio::time::timeout(Duration::from_secs(5), channel.ready())
        .await
        .context("Camera endpoint 0 subscription exceeded 5 seconds")??
        .make_cancel_safe();
    let mut completion = tokio::spawn(async move {
        let mut pts = [0; 8];
        done_rx.read_exact(&mut pts).await?;
        Ok::<_, std::io::Error>(u64::from_be_bytes(pts))
    });
    let mut stdout = tokio::io::stdout();
    let (mut received_pts, mut final_pts, mut codec_ok) = (0, None, false);
    let mut frames = 0;
    let started = Instant::now();
    let mut gate = FrameGate::default();
    let mut cached_config = Vec::new();
    let mut drain_deadline = None;
    loop {
        if final_pts.is_some_and(|last| received_pts >= last) {
            break;
        }
        tokio::select! {
            result = &mut completion, if final_pts.is_none() => {
                final_pts = Some(result??);
                drain_deadline = Some(tokio::time::Instant::now() + Duration::from_millis(200));
            }
            _ = async { tokio::time::sleep_until(drain_deadline.unwrap()).await }, if drain_deadline.is_some() => { break; }
            result = tokio::time::timeout(if frames == 0 { Duration::from_secs(5) } else { Duration::from_millis(750) }, video.recv()) => {
                let packet = result.context("Camera frames stalled")??.context("Video channel ended before completion")?;
                match packet {
                    AVPacket::Codec(codec) => {
                        ensure!(codec.header.codec == H264, "Unexpected video codec");
                        codec_ok = true;
                    }
                    AVPacket::Media(frame) => {
                        ensure!(codec_ok && frame.payload.len() <= 2_000_000, "Invalid video packet");
                        if frame.header.is_config {
                            cached_config = frame.payload.to_vec();
                            gate.need_key = true;
                            continue;
                        }
                        received_pts = received_pts.max(frame.header.pts);
                        if !gate.accept(frame.header.pts, frame.header.is_key, started.elapsed()) {
                            continue;
                        }
                        let deadline = if frames == 0 { Duration::from_secs(2) } else { Duration::from_millis(250) };
                        tokio::time::timeout(deadline, async {
                            if frame.header.is_key {
                                ensure!(!cached_config.is_empty(), "Missing H.264 configuration");
                                stdout.write_all(&cached_config).await?;
                            }
                            stdout.write_all(&frame.payload).await?;
                            stdout.flush().await?;
                            Ok::<_, anyhow::Error>(())
                        }).await.context("Video decoder stopped consuming fresh frames")??;
                        frames += 1;
                        progress(status_file, frames, Some(&gate))?;
                    }
                    AVPacket::Hole(_) => { gate.need_key = true; gate.dropped += 1; }
                }
            }
        }
    }
    done_tx.write_all(b"F").await?;
    tokio::time::timeout(Duration::from_secs(2), conn.closed()).await??;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn late_or_missing_frames_wait_for_a_fresh_keyframe() {
        let mut gate = FrameGate::default();
        let base = 1_790_000_000_000_000;
        assert!(gate.accept(base, true, Duration::ZERO));
        assert!(gate.accept(base + 33_333, false, Duration::from_micros(33_333)));
        assert!(!gate.accept(base + 66_666, false, Duration::from_millis(200)));
        assert!(!gate.accept(base + 100_000, false, Duration::from_millis(233)));
        assert!(gate.accept(base + 300_000, true, Duration::from_millis(300)));
        assert!(!gate.accept(base + 400_000, false, Duration::from_millis(400)));
        assert!(gate.accept(base + 433_333, true, Duration::from_micros(433_333)));
        assert_eq!(gate.dropped, 3);
    }

    #[test]
    fn sps_pps_are_extracted_without_picture_data() {
        let bytes = [0, 0, 0, 1, 0x67, 9, 0, 0, 1, 0x68, 8, 0, 0, 1, 0x65, 7];
        assert_eq!(configuration(&bytes).unwrap(), bytes[..11]);
        assert!(configuration(&[0, 0, 1, 0x65, 7]).is_none());
    }
}
