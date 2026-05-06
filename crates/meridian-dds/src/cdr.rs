//! CDR (Common Data Representation) XCDR1 serializer/deserializer.
//!
//! Zero-allocation, operates on `&mut [u8]` buffers.  Little-endian (CDR LE).
//! Implements the subset of CDR needed for ROS2 message interop:
//!
//! - Primitives: u8, i8, u16, i16, u32, i32, u64, i64, f32, f64, bool
//! - Strings: u32 length (including NUL) + UTF-8 bytes + NUL
//! - Sequences: u32 length + N elements
//! - Structs: fields in declaration order, each naturally aligned
//!
//! The 4-byte encapsulation header `[0x00, 0x01, 0x00, 0x00]` (CDR LE) is
//! written/read by the caller before invoking field-level serialization.

/// CDR LE encapsulation header (prepended to every DDS sample).
pub const CDR_LE_HEADER: [u8; 4] = [0x00, 0x01, 0x00, 0x00];

/// Errors during CDR serialization/deserialization.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CdrError {
    /// Buffer is too small for the data being written.
    BufferOverflow,
    /// Buffer ended before all fields could be read.
    UnexpectedEnd,
    /// String is not valid UTF-8.
    InvalidUtf8,
    /// String exceeds the maximum allowed length.
    StringTooLong,
}

// ─── Writer ───────────────────────────────────────────────────

/// Zero-alloc CDR writer operating on a pre-allocated `&mut [u8]`.
pub struct CdrWriter<'a> {
    buf: &'a mut [u8],
    pos: usize,
}

impl<'a> CdrWriter<'a> {
    /// Create a new writer over the given buffer.
    pub fn new(buf: &'a mut [u8]) -> Self {
        Self { buf, pos: 0 }
    }

    /// Number of bytes written so far.
    pub fn position(&self) -> usize {
        self.pos
    }

    /// Align the write position to `align` bytes (must be a power of 2).
    fn align(&mut self, align: usize) -> Result<(), CdrError> {
        let mask = align - 1;
        let aligned = (self.pos + mask) & !mask;
        if aligned > self.buf.len() {
            return Err(CdrError::BufferOverflow);
        }
        // Zero-fill padding bytes
        for i in self.pos..aligned {
            self.buf[i] = 0;
        }
        self.pos = aligned;
        Ok(())
    }

    fn check_remaining(&self, n: usize) -> Result<(), CdrError> {
        if self.pos + n > self.buf.len() {
            Err(CdrError::BufferOverflow)
        } else {
            Ok(())
        }
    }

    pub fn write_u8(&mut self, v: u8) -> Result<(), CdrError> {
        self.check_remaining(1)?;
        self.buf[self.pos] = v;
        self.pos += 1;
        Ok(())
    }

    pub fn write_i8(&mut self, v: i8) -> Result<(), CdrError> {
        self.write_u8(v as u8)
    }

    pub fn write_bool(&mut self, v: bool) -> Result<(), CdrError> {
        self.write_u8(if v { 1 } else { 0 })
    }

    pub fn write_u16(&mut self, v: u16) -> Result<(), CdrError> {
        self.align(2)?;
        self.check_remaining(2)?;
        self.buf[self.pos..self.pos + 2].copy_from_slice(&v.to_le_bytes());
        self.pos += 2;
        Ok(())
    }

    pub fn write_i16(&mut self, v: i16) -> Result<(), CdrError> {
        self.write_u16(v as u16)
    }

    pub fn write_u32(&mut self, v: u32) -> Result<(), CdrError> {
        self.align(4)?;
        self.check_remaining(4)?;
        self.buf[self.pos..self.pos + 4].copy_from_slice(&v.to_le_bytes());
        self.pos += 4;
        Ok(())
    }

    pub fn write_i32(&mut self, v: i32) -> Result<(), CdrError> {
        self.write_u32(v as u32)
    }

    pub fn write_u64(&mut self, v: u64) -> Result<(), CdrError> {
        self.align(8)?;
        self.check_remaining(8)?;
        self.buf[self.pos..self.pos + 8].copy_from_slice(&v.to_le_bytes());
        self.pos += 8;
        Ok(())
    }

    pub fn write_i64(&mut self, v: i64) -> Result<(), CdrError> {
        self.write_u64(v as u64)
    }

    pub fn write_f32(&mut self, v: f32) -> Result<(), CdrError> {
        self.write_u32(v.to_bits())
    }

    pub fn write_f64(&mut self, v: f64) -> Result<(), CdrError> {
        self.write_u64(v.to_bits())
    }

    /// Write a CDR string: u32 length (including NUL) + bytes + NUL byte.
    pub fn write_string(&mut self, s: &str) -> Result<(), CdrError> {
        let len_with_nul = s.len() + 1;
        self.write_u32(len_with_nul as u32)?;
        self.check_remaining(len_with_nul)?;
        self.buf[self.pos..self.pos + s.len()].copy_from_slice(s.as_bytes());
        self.buf[self.pos + s.len()] = 0; // NUL terminator
        self.pos += len_with_nul;
        Ok(())
    }

    /// Write a CDR sequence length prefix. Caller writes elements after.
    pub fn write_sequence_len(&mut self, len: u32) -> Result<(), CdrError> {
        self.write_u32(len)
    }

    /// Write a fixed-size f64 array (e.g., covariance matrices).
    pub fn write_f64_array(&mut self, vals: &[f64]) -> Result<(), CdrError> {
        for &v in vals {
            self.write_f64(v)?;
        }
        Ok(())
    }

    /// Write a fixed-size f32 array.
    pub fn write_f32_array(&mut self, vals: &[f32]) -> Result<(), CdrError> {
        for &v in vals {
            self.write_f32(v)?;
        }
        Ok(())
    }
}

// ─── Reader ───────────────────────────────────────────────────

/// Zero-alloc CDR reader operating on a `&[u8]`.
pub struct CdrReader<'a> {
    buf: &'a [u8],
    pos: usize,
}

impl<'a> CdrReader<'a> {
    /// Create a new reader over the given buffer.
    pub fn new(buf: &'a [u8]) -> Self {
        Self { buf, pos: 0 }
    }

    /// Current read position.
    pub fn position(&self) -> usize {
        self.pos
    }

    /// Bytes remaining.
    pub fn remaining(&self) -> usize {
        self.buf.len().saturating_sub(self.pos)
    }

    fn align(&mut self, align: usize) -> Result<(), CdrError> {
        let mask = align - 1;
        let aligned = (self.pos + mask) & !mask;
        if aligned > self.buf.len() {
            return Err(CdrError::UnexpectedEnd);
        }
        self.pos = aligned;
        Ok(())
    }

    fn check_remaining(&self, n: usize) -> Result<(), CdrError> {
        if self.pos + n > self.buf.len() {
            Err(CdrError::UnexpectedEnd)
        } else {
            Ok(())
        }
    }

    pub fn read_u8(&mut self) -> Result<u8, CdrError> {
        self.check_remaining(1)?;
        let v = self.buf[self.pos];
        self.pos += 1;
        Ok(v)
    }

    pub fn read_i8(&mut self) -> Result<i8, CdrError> {
        Ok(self.read_u8()? as i8)
    }

    pub fn read_bool(&mut self) -> Result<bool, CdrError> {
        Ok(self.read_u8()? != 0)
    }

    pub fn read_u16(&mut self) -> Result<u16, CdrError> {
        self.align(2)?;
        self.check_remaining(2)?;
        let v = u16::from_le_bytes([self.buf[self.pos], self.buf[self.pos + 1]]);
        self.pos += 2;
        Ok(v)
    }

    pub fn read_i16(&mut self) -> Result<i16, CdrError> {
        Ok(self.read_u16()? as i16)
    }

    pub fn read_u32(&mut self) -> Result<u32, CdrError> {
        self.align(4)?;
        self.check_remaining(4)?;
        let v = u32::from_le_bytes([
            self.buf[self.pos], self.buf[self.pos + 1],
            self.buf[self.pos + 2], self.buf[self.pos + 3],
        ]);
        self.pos += 4;
        Ok(v)
    }

    pub fn read_i32(&mut self) -> Result<i32, CdrError> {
        Ok(self.read_u32()? as i32)
    }

    pub fn read_u64(&mut self) -> Result<u64, CdrError> {
        self.align(8)?;
        self.check_remaining(8)?;
        let v = u64::from_le_bytes([
            self.buf[self.pos], self.buf[self.pos + 1],
            self.buf[self.pos + 2], self.buf[self.pos + 3],
            self.buf[self.pos + 4], self.buf[self.pos + 5],
            self.buf[self.pos + 6], self.buf[self.pos + 7],
        ]);
        self.pos += 8;
        Ok(v)
    }

    pub fn read_i64(&mut self) -> Result<i64, CdrError> {
        Ok(self.read_u64()? as i64)
    }

    pub fn read_f32(&mut self) -> Result<f32, CdrError> {
        Ok(f32::from_bits(self.read_u32()?))
    }

    pub fn read_f64(&mut self) -> Result<f64, CdrError> {
        Ok(f64::from_bits(self.read_u64()?))
    }

    /// Read a CDR string. Returns the slice WITHOUT the NUL terminator.
    /// Since we are no_std, returns bytes — caller can convert to str.
    pub fn read_string_bytes(&mut self) -> Result<&'a [u8], CdrError> {
        let len_with_nul = self.read_u32()? as usize;
        if len_with_nul == 0 {
            return Ok(&[]);
        }
        self.check_remaining(len_with_nul)?;
        let start = self.pos;
        self.pos += len_with_nul;
        // Strip trailing NUL
        let end = if len_with_nul > 0 && self.buf[start + len_with_nul - 1] == 0 {
            start + len_with_nul - 1
        } else {
            start + len_with_nul
        };
        Ok(&self.buf[start..end])
    }

    /// Read sequence length.
    pub fn read_sequence_len(&mut self) -> Result<u32, CdrError> {
        self.read_u32()
    }

    /// Read N f64 values into a slice.
    pub fn read_f64_into(&mut self, out: &mut [f64]) -> Result<(), CdrError> {
        for v in out.iter_mut() {
            *v = self.read_f64()?;
        }
        Ok(())
    }

    /// Read N f32 values into a slice.
    pub fn read_f32_into(&mut self, out: &mut [f32]) -> Result<(), CdrError> {
        for v in out.iter_mut() {
            *v = self.read_f32()?;
        }
        Ok(())
    }
}

// ─── Traits ───────────────────────────────────────────────────

/// Serialize to CDR wire format.
pub trait CdrSerialize {
    fn cdr_serialize(&self, w: &mut CdrWriter<'_>) -> Result<(), CdrError>;
}

/// Deserialize from CDR wire format.
pub trait CdrDeserialize: Sized {
    fn cdr_deserialize(r: &mut CdrReader<'_>) -> Result<Self, CdrError>;
}

// ─── Tests ────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    /// Write into a temp buffer, copy to a second buffer, read from that.
    /// Avoids borrow checker issues with writer/reader on same buf.
    fn write_then_read<F, G>(write_fn: F, read_fn: G)
    where
        F: FnOnce(&mut CdrWriter<'_>),
        G: FnOnce(&mut CdrReader<'_>),
    {
        let mut buf = [0u8; 512];
        let end = {
            let mut w = CdrWriter::new(&mut buf);
            write_fn(&mut w);
            w.position()
        };
        let mut r = CdrReader::new(&buf[..end]);
        read_fn(&mut r);
    }

    #[test]
    fn test_u8_roundtrip() {
        write_then_read(
            |w| { w.write_u8(0xAB).unwrap(); },
            |r| { assert_eq!(r.read_u8().unwrap(), 0xAB); },
        );
    }

    #[test]
    fn test_f32_roundtrip() {
        write_then_read(
            |w| { w.write_f32(3.14).unwrap(); },
            |r| { assert!((r.read_f32().unwrap() - 3.14).abs() < 1e-6); },
        );
    }

    #[test]
    fn test_f64_roundtrip() {
        write_then_read(
            |w| { w.write_f64(2.718281828).unwrap(); },
            |r| { assert!((r.read_f64().unwrap() - 2.718281828).abs() < 1e-12); },
        );
    }

    #[test]
    fn test_string_roundtrip() {
        write_then_read(
            |w| { w.write_string("map").unwrap(); },
            |r| { assert_eq!(r.read_string_bytes().unwrap(), b"map"); },
        );
    }

    #[test]
    fn test_alignment() {
        write_then_read(
            |w| {
                w.write_u8(0xFF).unwrap();
                w.write_u32(0x12345678).unwrap();
                assert_eq!(w.position(), 8);
            },
            |r| {
                assert_eq!(r.read_u8().unwrap(), 0xFF);
                assert_eq!(r.read_u32().unwrap(), 0x12345678);
            },
        );
    }

    #[test]
    fn test_mixed_types() {
        write_then_read(
            |w| {
                w.write_i32(42).unwrap();
                w.write_u32(100).unwrap();
                w.write_f64(1.5).unwrap();
                w.write_f64(2.5).unwrap();
                w.write_u8(3).unwrap();
            },
            |r| {
                assert_eq!(r.read_i32().unwrap(), 42);
                assert_eq!(r.read_u32().unwrap(), 100);
                assert!((r.read_f64().unwrap() - 1.5).abs() < 1e-12);
                assert!((r.read_f64().unwrap() - 2.5).abs() < 1e-12);
                assert_eq!(r.read_u8().unwrap(), 3);
            },
        );
    }

    #[test]
    fn test_buffer_overflow() {
        let mut buf = [0u8; 2];
        assert_eq!(CdrWriter::new(&mut buf).write_u32(1), Err(CdrError::BufferOverflow));
    }

    #[test]
    fn test_empty_string() {
        write_then_read(
            |w| { w.write_string("").unwrap(); },
            |r| { assert_eq!(r.read_string_bytes().unwrap(), b""); },
        );
    }

    #[test]
    fn test_covariance_array() {
        let cov = [1.0f64, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 3.0];
        write_then_read(
            |w| { w.write_f64_array(&cov).unwrap(); },
            |r| {
                let mut out = [0.0f64; 9];
                r.read_f64_into(&mut out).unwrap();
                assert_eq!(out, cov);
            },
        );
    }
}
