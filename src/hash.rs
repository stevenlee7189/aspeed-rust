
 use crate::controller::HaceController;
use proposed_traits::digest::*;
use core::convert::Infallible;
use crate::controller::HashAlgo;



const HACE_SG_LAST: u32 = 1 << 31;
#[derive(Default, Copy, Clone)]
pub struct AspeedSg {
    pub len: u32,
    pub addr: u32,
}

impl AspeedSg {
    pub const fn new() -> Self {
        Self { len: 0, addr: 0 }
    }
}


#[repr(C)]
#[repr(align(64))]
pub struct AspeedHashContext {
    pub sg: [AspeedSg; 2],
    pub digest: [u8; 64],
    pub method: u32,
    pub block_size: u32,
    pub digcnt: [u64; 2],
    pub bufcnt: u32,
    pub buffer: [u8; 256],
    pub iv_size: u8,
}

impl Default for AspeedHashContext {
    fn default() -> Self {
        Self {
            sg: [AspeedSg::default(); 2],
            digest: [0; 64],
            method: 0,
            block_size: 0,
            digcnt: [0; 2],
            bufcnt: 0,
            buffer: [0; 256],
            iv_size: 0,
        }
    }
}

impl AspeedHashContext {
    pub const fn new() -> Self {
        Self {
            sg: [AspeedSg::new(), AspeedSg::new()],
            digest: [0; 64],
            method: 0,
            block_size: 0,
            digcnt: [0; 2],
            bufcnt: 0,
            buffer: [0; 256],
            iv_size: 0,
        }
    }
}


// DigestAlgorithm implementation for HashAlgo
impl DigestAlgorithm for HashAlgo {
    const OUTPUT_BITS: usize = 512; // Maximum size for all variants
    type DigestOutput = [u8; 64]; // Use the maximum size for all variants
}

pub trait IntoHashAlgo {
    fn to_hash_algo() -> HashAlgo;
}

pub struct Digest48(pub [u8; 48]);

impl Default for Digest48 {
    fn default() -> Self {
        Digest48([0u8; 48])
    }
}

impl AsRef<[u8]> for Digest48 {
    fn as_ref(&self) -> &[u8] {
        &self.0
    }
}

impl AsMut<[u8]> for Digest48 {
    fn as_mut(&mut self) -> &mut [u8] {
        &mut self.0
    }
}

pub struct Digest64(pub [u8; 64]);
impl Default for Digest64 {
    fn default() -> Self {
        Digest64([0u8; 64])
    }
}

impl AsRef<[u8]> for Digest64 {
    fn as_ref(&self) -> &[u8] {
        &self.0
    }
}

impl AsMut<[u8]> for Digest64 {
    fn as_mut(&mut self) -> &mut [u8] {
        &mut self.0
    }
}

pub struct Sha1;
pub struct Sha224;
pub struct Sha256;
pub struct Sha384;
pub struct Sha512;

impl DigestAlgorithm for Sha1 {
    const OUTPUT_BITS: usize = 160;
    type DigestOutput = [u8; 20];
}

impl DigestAlgorithm for Sha224 {
    const OUTPUT_BITS: usize = 224;
    type DigestOutput = [u8; 28];
}

impl DigestAlgorithm for Sha256 {
    const OUTPUT_BITS: usize = 256;
    type DigestOutput = [u8; 32];
}

impl DigestAlgorithm for Sha384 {
    const OUTPUT_BITS: usize = 384;
    type DigestOutput = Digest48; // Use Digest48 for 384 bits
}

impl DigestAlgorithm for Sha512 {
    const OUTPUT_BITS: usize = 512;
    type DigestOutput = Digest64; // Use Digest64 for 512 bits
}

impl Default for Sha256 {
    fn default() -> Self { Sha256 }
}

impl Default for Sha384 {
    fn default() -> Self { Sha384 }
}

impl Default for Sha512 {
    fn default() -> Self { Sha512 }
}


impl IntoHashAlgo for Sha256 {
    fn to_hash_algo() -> HashAlgo {
        HashAlgo::SHA256
    }
}

impl IntoHashAlgo for Sha384 {
    fn to_hash_algo() -> HashAlgo {
        HashAlgo::SHA384
    }
}

impl IntoHashAlgo for Sha512 {
    fn to_hash_algo() -> HashAlgo {
        HashAlgo::SHA512
    }
}


impl<'ctrl, A> DigestInit<A> for HaceController<'ctrl>
where
    A: DigestAlgorithm + IntoHashAlgo,
    A::DigestOutput: Default + AsMut<[u8]>,
{
    type OpContext<'a> = OpContextImpl<'a, 'ctrl, A> where Self: 'a; // Define your OpContext type here

    fn init<'a>(&'a mut self, _algo: A) -> Result<Self::OpContext<'a>, Self::Error> {
        self.algo = A::to_hash_algo();
        self.ctx_mut().method = self.algo.hash_cmd();
        self.copy_iv_to_digest();
        self.ctx_mut().block_size = self.algo.block_size() as u32;
        self.ctx_mut().bufcnt = 0;
        self.ctx_mut().digcnt = [0; 2];


        Ok(OpContextImpl {
            controller: self,
            _phantom: core::marker::PhantomData,
        })
    }
}

pub struct OpContextImpl<'a, 'ctrl, A: DigestAlgorithm + IntoHashAlgo> {
    pub controller: &'a mut HaceController<'ctrl>,
    _phantom: core::marker::PhantomData<A>,
}

impl<'a, 'ctrl, A> proposed_traits::digest::ErrorType for OpContextImpl<'a, 'ctrl, A>
where
    A: DigestAlgorithm + IntoHashAlgo,
{
    type Error = Infallible;
}

impl<'a, 'ctrl, A> DigestOp for OpContextImpl<'a, 'ctrl, A>
where
    A: DigestAlgorithm + IntoHashAlgo,
    A::DigestOutput: Default + AsMut<[u8]>
{
    type Output = A::DigestOutput;

    fn update(&mut self, _input: &[u8]) -> Result<(), Self::Error> {
        let input_len = _input.len() as u32;
        let (new_len, carry) = self.controller.ctx_mut().digcnt[0].overflowing_add(input_len as u64);

        self.controller.ctx_mut().digcnt[0] = new_len;
        if carry {
            self.controller.ctx_mut().digcnt[1] += 1;
        }

        let start = self.controller.ctx_mut().bufcnt as usize;
        let end = start + input_len as usize;
        if self.controller.ctx_mut().bufcnt + input_len < self.controller.ctx_mut().block_size {
            self.controller.ctx_mut().buffer[start..end].copy_from_slice(_input);
            self.controller.ctx_mut().bufcnt += input_len;
            return Ok(());
        }

        let remaining = (input_len + self.controller.ctx_mut().bufcnt) % self.controller.ctx_mut().block_size;
        let total_len = (input_len + self.controller.ctx_mut().bufcnt) - remaining;
        let mut i = 0;

        if self.controller.ctx_mut().bufcnt != 0 {
            self.controller.ctx_mut().sg[0].addr = self.controller.ctx_mut().buffer.as_ptr() as u32;
            self.controller.ctx_mut().sg[0].len = self.controller.ctx_mut().bufcnt;
            if total_len == self.controller.ctx_mut().bufcnt {
                self.controller.ctx_mut().sg[0].addr = _input.as_ptr() as u32;
                self.controller.ctx_mut().sg[0].len |= HACE_SG_LAST;
            }
            i += 1;
        }

        if total_len != self.controller.ctx_mut().bufcnt {
            self.controller.ctx_mut().sg[i].addr = _input.as_ptr() as u32;
            self.controller.ctx_mut().sg[i].len = (total_len - self.controller.ctx_mut().bufcnt) | HACE_SG_LAST;
        }

        self.controller.start_hash_operation(total_len);

        if remaining != 0 {
            let src_start = (total_len - self.controller.ctx_mut().bufcnt) as usize;
            let src_end = src_start + remaining as usize;

            self.controller.ctx_mut().buffer[..(remaining as usize)]
                .copy_from_slice(&_input[src_start..src_end]);
            self.controller.ctx_mut().bufcnt = remaining as u32;
        }
        Ok(())
    }


    fn finalize(self) -> Result<Self::Output, Self::Error> {
        self.controller.fill_padding(0);
        let digest_len = self.controller.algo.digest_size();

        let (digest_ptr, bufcnt) = {
            let ctx = self.controller.ctx_mut();

            ctx.sg[0].addr = ctx.buffer.as_ptr() as u32;
            ctx.sg[0].len = ctx.bufcnt | HACE_SG_LAST;

            (ctx.digest.as_ptr(), ctx.bufcnt)
        };

        self.controller.start_hash_operation(bufcnt);

    let slice = unsafe {
        core::slice::from_raw_parts(digest_ptr, digest_len)
    };

    let mut output = A::DigestOutput::default();
    output.as_mut()[..digest_len].copy_from_slice(slice);

        let ctx = self.controller.ctx_mut();
        ctx.bufcnt = 0;
        ctx.buffer.fill(0);
        ctx.digest.fill(0);
        ctx.digcnt = [0; 2];

        unsafe {
            self.controller.hace.hace30().write(|w| w.bits(0));
        }

        Ok(output) // Return the final output
    }
}
