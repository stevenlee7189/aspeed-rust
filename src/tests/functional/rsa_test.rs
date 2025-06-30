use crate::rsa::{RsaDigest, RsaPrivateKey, RsaPublicKey, RsaSignatureData};
use crate::tests::functional::rsa_test_vec::RSA_VERIFY_TV;
use crate::uart::UartController;
use embedded_io::Write;
use proposed_traits::rsa::{PaddingMode, RsaSign, RsaVerify};

pub fn run_rsa_signing_tests<'a, T>(uart: &mut UartController, engine: &mut T)
where
    T: RsaSign<PrivateKey = RsaPrivateKey<'a>, Message = RsaDigest, Signature = RsaSignatureData>,
{
    writeln!(uart, "\rRunning RSA Signing tests...").unwrap();

    for (i, vec) in RSA_VERIFY_TV.iter().enumerate() {
        let pubkey = RsaPrivateKey {
            m: vec.k.m,
            d: vec.k.d,
            m_bits: vec.k.m_bits as u32,
            d_bits: vec.k.d_bits as u32,
        };

        let mut digest = [0u8; 64];
        if vec.d_size > digest.len() {
            writeln!(
                uart,
                "\rRSA vector[{}] digest size {} exceeds buffer size {}",
                i,
                vec.d_size,
                digest.len()
            )
            .ok();
            continue;
        }

        digest[..vec.d_size].copy_from_slice(&vec.digest[..vec.d_size]);

        let message = RsaDigest {
            data: digest,
            len: vec.d_size,
        };

        let result = engine.sign(&pubkey, message, PaddingMode::Pkcs1v15);

        match result {
            Ok(signature) => {
                if signature.len != vec.s_size {
                    writeln!(
                        uart,
                        "\rRSA vector[{}] signature length mismatch: expected {}, got {}",
                        i, vec.s_size, signature.len
                    )
                    .ok();
                    writeln!(
                        uart,
                        "\rRSA vector[{}] signature mismatch:\r\n  expected: {:02x?}\r\n  got     : {:02x?}",
                        i,
                        &vec.signature[..vec.s_size],
                        &signature.data[..signature.len]
                    )
                        .ok();
                    continue;
                }
                if signature.data[..signature.len] != vec.signature[..vec.s_size] {
                    writeln!(
                        uart,
                        "\rRSA vector[{}] signature mismatch:\r\n  expected: {:02x?}\r\n  got     : {:02x?}",
                        i,
                        &vec.signature[..vec.s_size],
                        &signature.data[..signature.len]
                    )
                        .ok();
                    continue;
                }

                writeln!(uart, "\rRSA vector[{}] sign passed", i).ok();
            }
            Err(_err) => {
                writeln!(uart, "\rRSA vector[{}] sign failed", i).ok();
            }
        }
    }
}

pub fn run_rsa_verification_tests<'a, T>(uart: &mut UartController, engine: &mut T)
where
    T: RsaVerify<PublicKey = RsaPublicKey<'a>, Message = RsaDigest, Signature = RsaSignatureData>,
{
    writeln!(uart, "\rRunning RSA verification tests...").unwrap();

    for (i, vec) in RSA_VERIFY_TV.iter().enumerate() {
        let pubkey = RsaPublicKey {
            m: vec.k.m,
            e: vec.k.e,
            m_bits: vec.k.m_bits as u32,
            e_bits: vec.k.e_bits as u32,
        };

        let mut digest = [0u8; 64];
        if vec.d_size > digest.len() {
            writeln!(
                uart,
                "\rRSA vector[{}] digest size {} exceeds buffer size {}",
                i,
                vec.d_size,
                digest.len()
            )
            .ok();
            continue;
        }

        digest[..vec.d_size].copy_from_slice(&vec.digest[..vec.d_size]);

        let message = RsaDigest {
            data: digest,
            len: vec.d_size,
        };

        let mut sig = [0u8; 512];
        if vec.s_size > sig.len() {
            writeln!(
                uart,
                "\rVector[{}] signature size too large: {}",
                i, vec.s_size
            )
            .ok();
            continue;
        }
        sig[..vec.s_size].copy_from_slice(&vec.signature[..vec.s_size]);

        let signature = RsaSignatureData {
            data: sig,
            len: vec.s_size,
        };

        let result = engine.verify(&pubkey, message, PaddingMode::Pkcs1v15, &signature);

        match result {
            Ok(_decrypted) => {
                writeln!(uart, "\rRSA vector[{}] verify passed", i).ok();
            }
            Err(err) => {
                writeln!(uart, "\rRSA vector[{}] verify failed: {:?}", i, err).ok();
            }
        }
    }
}

pub fn run_rsa_tests<'a, T>(uart: &mut UartController, engine: &mut T)
where
    T: RsaVerify<PublicKey = RsaPublicKey<'a>, Message = RsaDigest, Signature = RsaSignatureData>
        + RsaSign<PrivateKey = RsaPrivateKey<'a>, Message = RsaDigest, Signature = RsaSignatureData>,
{
    writeln!(uart, "\r\nRunning RSA tests...").unwrap();
    run_rsa_verification_tests(uart, engine);
    run_rsa_signing_tests(uart, engine);
}
