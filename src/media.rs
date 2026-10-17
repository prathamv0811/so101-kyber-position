//! GStreamer H.264 capture carried by the official KyProto GOP video channel.
use super::{now_ns, tls};
use anyhow::{Context, Result, ensure};
use bytes::Bytes;
use kymux_types::av::{AVPacket, CodecPacket, CodecPacketHeader, MediaPacket, MediaPacketHeader};
use kyproto::VideoProtocol;
use std::{net::SocketAddr, path::Path, time::Duration};
use tokio::io::{AsyncReadExt, AsyncWriteExt};

const ALPN: &[u8] = b"so101-kyproto-h264-v1";
const H264: u32 = u32::from_be_bytes(*b"h264");

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

fn progress(path: Option<&Path>, frames: u64) -> Result<()> {
    if let Some(path) = path {
        let temporary = path.with_extension("tmp");
        std::fs::write(
            &temporary,
            serde_json::to_vec(&serde_json::json!({
                "updated_unix_ns": now_ns(), "frames": frames,
            }))?,
        )?;
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
    let config = tls::server(credentials, insecure_lan, ALPN)?;
    let endpoint = quinn::Endpoint::server(config, listen)?;
    eprintln!(
        "KyProto H.264 camera listening on {}",
        endpoint.local_addr()?
    );
    let raw = tokio::time::timeout(
        Duration::from_secs(5),
        endpoint.accept().await.context("Listener closed")?,
    )
    .await??;
    tls::address(raw.remote_address(), insecure_lan)?;
    let conn = kynet::Connection::from(raw);
    // Establish a completion channel before KyProto takes ownership of routing.
    let (mut done_tx, mut done_rx) =
        tokio::time::timeout(Duration::from_secs(3), conn.accept_bi()).await??;
    let mut marker = [0];
    tokio::time::timeout(Duration::from_secs(1), done_rx.read_exact(&mut marker)).await??;
    ensure!(marker == *b"M", "Invalid media completion marker");
    let media = tokio::time::timeout(
        Duration::from_secs(3),
        kyproto::Connection::accept(conn.clone()),
    )
    .await??;
    let (id, channel) = media
        .register_video_endpoint(VideoProtocol::GopStream)
        .await?;
    ensure!(id == 0, "Unexpected video endpoint ID");
    let mut video = tokio::time::timeout(Duration::from_secs(3), channel.ready()).await??;
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
    loop {
        let mut header = [0; 13];
        let first =
            tokio::time::timeout(Duration::from_secs(1), source.read(&mut header[..1])).await??;
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
        if age > 250_000_000 {
            have_key = false;
            continue;
        }
        let is_key = header[12] == 1;
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
        .await??;
        have_key = true;
        last_pts = pts;
        frames += 1;
        progress(status_file, frames)?;
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
    let config = tls::client(credentials, insecure_lan, ALPN)?;
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
    let channel = media.connect_video_endpoint(0, VideoProtocol::GopStream)?;
    let mut video = tokio::time::timeout(Duration::from_secs(3), channel.ready())
        .await??
        .make_cancel_safe();
    let mut completion = tokio::spawn(async move {
        let mut pts = [0; 8];
        done_rx.read_exact(&mut pts).await?;
        Ok::<_, std::io::Error>(u64::from_be_bytes(pts))
    });
    let mut stdout = tokio::io::stdout();
    let (mut received_pts, mut final_pts, mut codec_ok) = (0, None, false);
    let mut frames = 0;
    loop {
        if final_pts.is_some_and(|last| received_pts >= last) {
            break;
        }
        tokio::select! {
            result = &mut completion, if final_pts.is_none() => { final_pts = Some(result??); }
            result = tokio::time::timeout(if frames == 0 { Duration::from_secs(5) } else { Duration::from_millis(750) }, video.recv()) => {
                let packet = result.context("Camera frames stalled")??.context("Video channel ended before completion")?;
                match packet {
                    AVPacket::Codec(codec) => {
                        ensure!(codec.header.codec == H264, "Unexpected video codec");
                        codec_ok = true;
                    }
                    AVPacket::Media(frame) => {
                        ensure!(codec_ok && frame.payload.len() <= 2_000_000, "Invalid video packet");
                        stdout.write_all(&frame.payload).await?;
                        stdout.flush().await?;
                        if !frame.header.is_config {
                            received_pts = frame.header.pts;
                            frames += 1;
                            progress(status_file, frames)?;
                        }
                    }
                    _ => anyhow::bail!("Unexpected video hole packet"),
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
    fn sps_pps_are_extracted_without_picture_data() {
        let bytes = [0, 0, 0, 1, 0x67, 9, 0, 0, 1, 0x68, 8, 0, 0, 1, 0x65, 7];
        assert_eq!(configuration(&bytes).unwrap(), bytes[..11]);
        assert!(configuration(&[0, 0, 1, 0x65, 7]).is_none());
    }
}
