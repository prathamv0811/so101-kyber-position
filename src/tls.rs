//! Optional file-free TLS for private LAN use; paired mutual TLS remains default.
use anyhow::{Result, ensure};
use rustls::{
    DigitallySignedStruct, SignatureScheme,
    client::danger::{HandshakeSignatureValid, ServerCertVerified, ServerCertVerifier},
    pki_types::{CertificateDer, PrivatePkcs8KeyDer, ServerName, UnixTime},
};
use std::{
    net::{IpAddr, SocketAddr},
    path::Path,
    sync::Arc,
};

pub fn address(address: SocketAddr, insecure: bool) -> Result<()> {
    let private = match address.ip() {
        IpAddr::V4(ip) => ip.is_private() || ip.is_loopback() || ip.is_link_local(),
        IpAddr::V6(ip) => ip.is_unique_local() || ip.is_loopback() || ip.is_unicast_link_local(),
    };
    ensure!(
        !insecure || private,
        "--insecure-lan requires an explicit private LAN or loopback address"
    );
    Ok(())
}

pub fn server(credentials: &Path, insecure: bool, alpn: &[u8]) -> Result<quinn::ServerConfig> {
    let mut tls = if insecure {
        let rcgen::CertifiedKey { cert, key_pair } =
            rcgen::generate_simple_self_signed(vec!["so101-server".into()])?;
        rustls::ServerConfig::builder()
            .with_no_client_auth()
            .with_single_cert(
                vec![cert.der().clone()],
                PrivatePkcs8KeyDer::from(key_pair.serialize_der()).into(),
            )?
    } else {
        let verifier =
            rustls::server::WebPkiClientVerifier::builder(Arc::new(super::roots(credentials)?))
                .build()?;
        rustls::ServerConfig::builder()
            .with_client_cert_verifier(verifier)
            .with_single_cert(
                super::certs(&credentials.join("cert.pem"))?,
                super::key(&credentials.join("key.pem"))?,
            )?
    };
    tls.alpn_protocols = vec![alpn.to_vec()];
    let mut config = quinn::ServerConfig::with_crypto(Arc::new(
        quinn::crypto::rustls::QuicServerConfig::try_from(tls)?,
    ));
    config.transport = super::transport();
    Ok(config)
}

pub fn client(credentials: &Path, insecure: bool, alpn: &[u8]) -> Result<quinn::ClientConfig> {
    let mut tls = if insecure {
        rustls::ClientConfig::builder()
            .dangerous()
            .with_custom_certificate_verifier(Arc::new(LanVerifier(Arc::new(
                rustls::crypto::ring::default_provider(),
            ))))
            .with_no_client_auth()
    } else {
        rustls::ClientConfig::builder()
            .with_root_certificates(super::roots(credentials)?)
            .with_client_auth_cert(
                super::certs(&credentials.join("cert.pem"))?,
                super::key(&credentials.join("key.pem"))?,
            )?
    };
    tls.alpn_protocols = vec![alpn.to_vec()];
    let mut config = quinn::ClientConfig::new(Arc::new(
        quinn::crypto::rustls::QuicClientConfig::try_from(tls)?,
    ));
    config.transport_config(super::transport());
    Ok(config)
}

#[derive(Debug)]
struct LanVerifier(Arc<rustls::crypto::CryptoProvider>);
impl ServerCertVerifier for LanVerifier {
    fn verify_server_cert(
        &self,
        _: &CertificateDer<'_>,
        _: &[CertificateDer<'_>],
        _: &ServerName<'_>,
        _: &[u8],
        _: UnixTime,
    ) -> Result<ServerCertVerified, rustls::Error> {
        Ok(ServerCertVerified::assertion())
    }
    fn verify_tls12_signature(
        &self,
        message: &[u8],
        cert: &CertificateDer<'_>,
        dss: &DigitallySignedStruct,
    ) -> Result<HandshakeSignatureValid, rustls::Error> {
        rustls::crypto::verify_tls12_signature(
            message,
            cert,
            dss,
            &self.0.signature_verification_algorithms,
        )
    }
    fn verify_tls13_signature(
        &self,
        message: &[u8],
        cert: &CertificateDer<'_>,
        dss: &DigitallySignedStruct,
    ) -> Result<HandshakeSignatureValid, rustls::Error> {
        rustls::crypto::verify_tls13_signature(
            message,
            cert,
            dss,
            &self.0.signature_verification_algorithms,
        )
    }
    fn supported_verify_schemes(&self) -> Vec<SignatureScheme> {
        self.0.signature_verification_algorithms.supported_schemes()
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn lan_mode_rejects_wildcard_and_public_addresses() {
        for addr in [
            "0.0.0.0:7445",
            "8.8.8.8:7445",
            "[::]:7445",
            "[2606:4700::1111]:7445",
        ] {
            assert!(super::address(addr.parse().unwrap(), true).is_err());
            assert!(super::address(addr.parse().unwrap(), false).is_ok());
        }
        for addr in [
            "192.168.101.50:7445",
            "127.0.0.1:7445",
            "[::1]:7445",
            "[fd00::1]:7445",
        ] {
            assert!(super::address(addr.parse().unwrap(), true).is_ok());
        }
    }
}
