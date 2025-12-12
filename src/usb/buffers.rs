use super::btable::{RxBlockSize, RxEntry, TxEntry};
use super::device::MAX_ENDPOINTS;

use core::fmt::Debug;
use core::marker::PhantomData;

use usb_device::{Result, UsbError};
use vcell::VolatileCell;

/// Packet memory word size
const WORD_SIZE: usize = core::mem::size_of::<u32>();
/// Buffer descriptors size per endpoint
const BTABLE_ENTRY_SIZE: usize = core::mem::size_of::<TxEntry>() + core::mem::size_of::<RxEntry>();

/// USB rx/tx buffer allocator
pub(super) struct Allocator<USB> {
    /// Total available memory
    total_bytes: usize,
    /// Next available memory offset
    next_free_byte: usize,
    marker: PhantomData<USB>,
}

impl<USB> Allocator<USB> {
    pub fn new(packet_memory_bytes: usize) -> Self {
        // First `MAX_ENDPOINTS * 8` bytes are reserved for buffer descriptor table.
        // The remaining memory is available for buffers.
        Self {
            total_bytes: packet_memory_bytes,
            next_free_byte: MAX_ENDPOINTS * BTABLE_ENTRY_SIZE,
            marker: PhantomData,
        }
    }

    /// Allocates buffer. Returns the buffer offset or error.
    pub fn alloc(&mut self, num_bytes: usize) -> Result<usize> {
        // Precondition: buffers always start at 4-byte boundary.
        debug_assert!(self.next_free_byte.is_multiple_of(4));

        if self.next_free_byte + num_bytes > self.total_bytes {
            return Err(UsbError::EndpointMemoryOverflow);
        }

        let offset = self.next_free_byte;
        self.next_free_byte += num_bytes.next_multiple_of(4);

        Ok(offset)
    }
}

pub(super) struct TxBuffer<USB> {
    mem: &'static [VolatileCell<u32>],
    tx_entry: &'static VolatileCell<TxEntry>,
    marker: PhantomData<USB>,
}

#[allow(unsafe_code)]
unsafe impl<USB> Send for TxBuffer<USB> {}

impl<USB> Debug for TxBuffer<USB> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("TxBuffer")
            .field("ptr", &self.mem.as_ptr())
            .field("len_words", &self.mem.len())
            .field("tx_entry", &self.tx_entry.get())
            .finish()
    }
}

impl<USB> PartialEq for TxBuffer<USB> {
    fn eq(&self, other: &Self) -> bool {
        self.mem.as_ptr() == other.mem.as_ptr() && self.mem.len() == other.mem.len()
    }
}

impl<USB> TxBuffer<USB> {
    #[allow(unsafe_code)]
    pub fn new(
        allocator: &mut Allocator<USB>,
        index: usize,
        num_bytes: usize,
        packet_memory: *const (),
    ) -> Result<Self> {
        if num_bytes > 1023 {
            // tx_count has only 10 bits
            return Err(UsbError::Unsupported);
        }

        let btable = packet_memory as *const VolatileCell<TxEntry>;
        let btable_entry = btable.wrapping_add(index * 2);

        let buffer_memory = packet_memory as *const VolatileCell<u32>;
        let buffer_words = num_bytes.div_ceil(WORD_SIZE);
        let buffer_offset = allocator.alloc(num_bytes)?;
        // Buffers must be word-aligned
        debug_assert!(buffer_offset % WORD_SIZE == 0);

        let tx_entry = unsafe { &(*btable_entry) };
        tx_entry.set(TxEntry::new(buffer_offset));

        Ok(Self {
            mem: unsafe {
                core::slice::from_raw_parts(
                    buffer_memory.wrapping_add(buffer_offset / WORD_SIZE),
                    buffer_words,
                )
            },
            tx_entry,
            marker: PhantomData,
        })
    }
}

impl<USB> TxBuffer<USB> {
    pub fn write(&self, data: &[u8]) -> Result<usize> {
        if data.len() > self.mem.len() * WORD_SIZE {
            return Err(UsbError::BufferOverflow);
        }

        let mut index = 0;
        let chunks = data.chunks_exact(WORD_SIZE);
        let remainder = chunks.remainder();

        for chunk in chunks {
            let word = u32::from_ne_bytes(chunk.try_into().unwrap());
            self.mem[index].set(word);
            index += 1;
        }

        if !remainder.is_empty() {
            let mut word_bytes = [0u8; WORD_SIZE];
            word_bytes[..remainder.len()].copy_from_slice(remainder);
            let word = u32::from_ne_bytes(word_bytes);
            self.mem[index].set(word);
        }

        let mut tx = self.tx_entry.get();
        tx.set_tx_count(data.len() as _);
        self.tx_entry.set(tx);

        Ok(data.len())
    }

    #[cfg(test)]
    fn as_ptr(&self) -> *const () {
        self.mem.as_ptr() as _
    }

    #[cfg(test)]
    fn len_in_bytes(&self) -> usize {
        self.mem.len() * WORD_SIZE
    }
}

pub(super) struct RxBuffer<USB> {
    mem: &'static [VolatileCell<u32>],
    rx_entry: &'static VolatileCell<RxEntry>,
    marker: PhantomData<USB>,
}

#[allow(unsafe_code)]
unsafe impl<USB> Send for RxBuffer<USB> {}

impl<USB> Debug for RxBuffer<USB> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("RxBuffer")
            .field("ptr", &self.mem.as_ptr())
            .field("len_words", &self.mem.len())
            .field("rx_entry", &self.rx_entry.get())
            .finish()
    }
}

impl<USB> PartialEq for RxBuffer<USB> {
    fn eq(&self, other: &Self) -> bool {
        self.mem.as_ptr() == other.mem.as_ptr() && self.mem.len() == other.mem.len()
    }
}

impl<USB> RxBuffer<USB> {
    #[allow(unsafe_code)]
    pub fn new(
        allocator: &mut Allocator<USB>,
        index: usize,
        num_bytes: usize,
        packet_memory: *const (),
    ) -> Result<Self> {
        let btable = packet_memory as *const VolatileCell<RxEntry>;
        let btable_entry = btable.wrapping_add(index * 2 + 1);

        let block_size = if num_bytes > 62 {
            RxBlockSize::Size32
        } else {
            RxBlockSize::Size2
        };
        let alloc_blocks = if block_size == RxBlockSize::Size32 {
            // 0 blocks = 32 bytes, 1 block = 64 bytes, 3 blocks = 96 bytes, etc
            // See RM0444, table 228
            (num_bytes - 1) / 32
        } else {
            num_bytes.div_ceil(2)
        };
        let alloc_bytes = if block_size == RxBlockSize::Size32 {
            (alloc_blocks + 1) * 32
        } else {
            alloc_blocks * 2
        };

        let buffer_memory = packet_memory as *const VolatileCell<u32>;
        let buffer_words = alloc_bytes.div_ceil(WORD_SIZE);
        let buffer_offset = allocator.alloc(alloc_bytes)?;
        // Buffers must be word-aligned
        debug_assert!(buffer_offset % WORD_SIZE == 0);

        let rx_entry = unsafe { &(*btable_entry) };
        rx_entry.set(RxEntry::new(buffer_offset, block_size, alloc_blocks));

        Ok(Self {
            mem: unsafe {
                core::slice::from_raw_parts(
                    buffer_memory.wrapping_add(buffer_offset / WORD_SIZE),
                    buffer_words,
                )
            },
            rx_entry,
            marker: PhantomData,
        })
    }
}

impl<USB> RxBuffer<USB> {
    pub fn read(&self, data: &mut [u8]) -> Result<usize> {
        let rx = self.rx_entry.get();
        let bytes_read = rx.rx_count() as usize;
        if bytes_read > data.len() {
            return Err(UsbError::BufferOverflow);
        }

        for index in 0..bytes_read / WORD_SIZE {
            // Buffers are safe to read as u32
            let [b1, b2, b3, b4] = self.mem[index].get().to_ne_bytes();
            data[index * 4] = b1;
            data[index * 4 + 1] = b2;
            data[index * 4 + 2] = b3;
            data[index * 4 + 3] = b4;
        }

        let remainder = bytes_read % WORD_SIZE;
        if remainder != 0 {
            // Still safe to read the whole word.
            let [b1, b2, b3, _] = self.mem[bytes_read / WORD_SIZE].get().to_ne_bytes();
            data[bytes_read - remainder] = b1;
            if remainder > 1 {
                data[bytes_read - remainder + 1] = b2;
            }
            if remainder > 2 {
                data[bytes_read - remainder + 2] = b3;
            }
        }

        Ok(bytes_read)
    }

    #[cfg(test)]
    fn as_ptr(&self) -> *const () {
        self.mem.as_ptr() as _
    }

    #[cfg(test)]
    fn len_in_bytes(&self) -> usize {
        self.mem.len() * WORD_SIZE
    }
}

#[cfg(test)]
mod test {
    use super::*;

    const BTABLE_SIZE: usize = MAX_ENDPOINTS * BTABLE_ENTRY_SIZE;

    #[test]
    fn test_allocations() {
        let mut allocator = Allocator::<()>::new(2048);

        let buffer1 = allocator.alloc(5);
        assert_eq!(buffer1, Ok(BTABLE_SIZE));

        let buffer2 = allocator.alloc(470);
        assert_eq!(buffer2, Ok(BTABLE_SIZE + 8));

        let buffer3 = allocator.alloc(30);
        assert_eq!(buffer3, Ok(BTABLE_SIZE + 480));
    }

    #[test]
    fn test_allocations_overflow() {
        let mut allocator = Allocator::<()>::new(2048);

        // Buffer descriptor table takes 64 bytes, allocations another 1512 bytes.
        // 2048 - 64 - 1000 - 504 = 480, third allocation shouldn't fit.
        allocator
            .alloc(1000)
            .expect("allocation unexpectedly failed");
        allocator
            .alloc(501)
            .expect("allocation unexpectedly failed");

        let alloc_result = allocator.alloc(481);
        assert_eq!(
            alloc_result,
            Err(UsbError::EndpointMemoryOverflow),
            "allocation didn't overflow"
        );

        let buffer = allocator.alloc(480);
        assert_eq!(buffer, Ok(BTABLE_SIZE + 1000 + 504));
    }

    #[test]
    fn test_tx_buffer_allocations() {
        let memory = [0u8; 2048];
        let packet_memory = memory.as_ptr() as *const ();
        let btable_start = packet_memory as *const VolatileCell<TxEntry>;
        let alloc_start = btable_start.wrapping_add(MAX_ENDPOINTS * 2) as *const ();
        let mut allocator = Allocator::<()>::new(memory.len());

        let buffer1 = TxBuffer::new(&mut allocator, 0, 5, packet_memory).unwrap();
        assert_eq!(buffer1.as_ptr(), alloc_start);
        assert_eq!(buffer1.len_in_bytes(), 8);
        assert_eq!(buffer1.tx_entry as *const _, btable_start);

        let buffer2 = TxBuffer::new(&mut allocator, 2, 470, packet_memory).unwrap();
        assert_eq!(buffer2.as_ptr(), alloc_start.wrapping_byte_add(8));
        assert_eq!(buffer2.len_in_bytes(), 472);
        assert_eq!(buffer2.tx_entry as *const _, btable_start.wrapping_add(4));

        let buffer3 = TxBuffer::new(&mut allocator, 1, 59, packet_memory).unwrap();
        assert_eq!(buffer3.as_ptr(), alloc_start.wrapping_byte_add(8 + 472));
        assert_eq!(buffer3.len_in_bytes(), 60);
        assert_eq!(buffer3.tx_entry as *const _, btable_start.wrapping_add(2));

        let buffer4 = TxBuffer::new(&mut allocator, 3, 600, packet_memory).unwrap();
        assert_eq!(
            buffer4.as_ptr(),
            alloc_start.wrapping_byte_add(8 + 472 + 60)
        );
        assert_eq!(buffer4.len_in_bytes(), 600);
        assert_eq!(buffer4.tx_entry as *const _, btable_start.wrapping_add(6));
    }

    #[test]
    fn test_tx_buffer_allocations_overflow() {
        let memory = [0u8; 2048];
        let packet_memory = memory.as_ptr() as *const ();
        let btable_start = packet_memory as *const VolatileCell<TxEntry>;
        let alloc_start = btable_start.wrapping_add(MAX_ENDPOINTS * 2) as *const ();
        let mut allocator = Allocator::<()>::new(memory.len());

        // Buffer descriptor table takes 64 bytes, allocations another 1512 bytes.
        // 2048 - 64 - 1024 - 512 = 448, third allocation shouldn't fit.
        TxBuffer::new(&mut allocator, 0, 1023, packet_memory)
            .expect("allocation unexpectedly failed");
        TxBuffer::new(&mut allocator, 1, 512, packet_memory)
            .expect("allocation unexpectedly failed");

        let alloc_result = TxBuffer::new(&mut allocator, 3, 449, packet_memory);
        assert_eq!(
            alloc_result,
            Err(UsbError::EndpointMemoryOverflow),
            "allocation didn't overflow"
        );

        let buffer = TxBuffer::new(&mut allocator, 3, 448, packet_memory).unwrap();
        assert_eq!(buffer.as_ptr(), alloc_start.wrapping_byte_add(1024 + 512));
        assert_eq!(buffer.len_in_bytes(), 448);
        assert_eq!(buffer.tx_entry as *const _, btable_start.wrapping_add(6));
    }

    #[test]
    fn test_rx_buffer_allocations() {
        let memory = [0u8; 2048];
        let packet_memory = memory.as_ptr() as *const ();
        let btable_start = packet_memory as *const VolatileCell<TxEntry>;
        let alloc_start = btable_start.wrapping_add(MAX_ENDPOINTS * 2) as *const ();
        let mut allocator = Allocator::<()>::new(memory.len());

        // 5 bytes are rounded up to 6
        let buffer1 = RxBuffer::new(&mut allocator, 0, 5, packet_memory).unwrap();
        assert_eq!(buffer1.as_ptr(), alloc_start);
        assert_eq!(buffer1.len_in_bytes(), 8);
        assert_eq!(
            buffer1.rx_entry as *const _,
            btable_start.wrapping_add(1) as *const _
        );

        // 470 bytes rounds up to 32-byte boundary, return 480 bytes.
        let buffer2 = RxBuffer::new(&mut allocator, 2, 470, packet_memory).unwrap();
        assert_eq!(buffer2.as_ptr(), alloc_start.wrapping_byte_add(8));
        assert_eq!(buffer2.len_in_bytes(), 480);
        assert_eq!(
            buffer2.rx_entry as *const _,
            btable_start.wrapping_add(5) as *const _
        );

        // Largest buffer we can allocate in 2-byte blocks is 62 bytes.
        let buffer3 = RxBuffer::new(&mut allocator, 1, 62, packet_memory).unwrap();
        assert_eq!(buffer3.as_ptr(), alloc_start.wrapping_byte_add(8 + 480));
        assert_eq!(buffer3.len_in_bytes(), 64);
        assert_eq!(
            buffer3.rx_entry as *const _,
            btable_start.wrapping_add(3) as *const _
        );

        // Allocation one byte over 62-byte boundary, rounds up to 32 bytes.
        let buffer4 = RxBuffer::new(&mut allocator, 1, 63, packet_memory).unwrap();
        assert_eq!(
            buffer4.as_ptr(),
            alloc_start.wrapping_byte_add(8 + 480 + 64)
        );
        assert_eq!(buffer4.len_in_bytes(), 64);
        assert_eq!(
            buffer4.rx_entry as *const _,
            btable_start.wrapping_add(3) as *const _
        );

        // 576 is a multiple of 32, no rounding.
        let buffer5 = RxBuffer::new(&mut allocator, 3, 576, packet_memory).unwrap();
        assert_eq!(
            buffer5.as_ptr(),
            alloc_start.wrapping_byte_add(8 + 480 + 64 + 64)
        );
        assert_eq!(buffer5.len_in_bytes(), 576);
        assert_eq!(
            buffer5.rx_entry as *const _,
            btable_start.wrapping_add(7) as *const _
        );
    }

    #[test]
    fn test_rx_buffer_allocations_overflow() {
        let memory = [0u8; 2048];
        let packet_memory = memory.as_ptr() as *const ();
        let btable_start = packet_memory as *const VolatileCell<TxEntry>;
        let alloc_start = btable_start.wrapping_add(MAX_ENDPOINTS * 2) as *const ();
        let mut allocator = Allocator::<()>::new(memory.len());

        // Buffer descriptor table takes 64 bytes, allocations another 1512 bytes.
        // 2048 - 64 - 1024 - 512 = 448, third allocation shouldn't fit.
        RxBuffer::new(&mut allocator, 0, 1000, packet_memory)
            .expect("allocation unexpectedly failed");
        RxBuffer::new(&mut allocator, 1, 501, packet_memory)
            .expect("allocation unexpectedly failed");

        let alloc_result = RxBuffer::new(&mut allocator, 3, 449, packet_memory);
        assert_eq!(
            alloc_result,
            Err(UsbError::EndpointMemoryOverflow),
            "allocation didn't overflow"
        );

        let buffer = RxBuffer::new(&mut allocator, 3, 448, packet_memory).unwrap();
        assert_eq!(buffer.as_ptr(), alloc_start.wrapping_byte_add(1024 + 512));
        assert_eq!(buffer.len_in_bytes(), 448);
        assert_eq!(
            buffer.rx_entry as *const _,
            btable_start.wrapping_add(7) as *const _
        );
    }

    #[test]
    fn test_tx_buffer_writes() {
        const MEMORY_SIZE: usize = 2048;
        let mut memory = [0u8; MEMORY_SIZE];
        let device_memory = memory.as_ptr() as *const ();
        let btable_start = memory.as_ptr() as *const VolatileCell<TxEntry>;
        #[allow(unsafe_code)]
        let tx_entry = unsafe { &*btable_start };
        let alloc = &mut memory[BTABLE_ENTRY_SIZE * MAX_ENDPOINTS..];
        let mut allocator = Allocator::<()>::new(MEMORY_SIZE);
        let buffer = TxBuffer::new(&mut allocator, 0, 1023, device_memory).unwrap();

        let mut pattern = [0u8; 2048];
        for i in 0..pattern.len() {
            // Fill with non-zero bytes
            pattern[i] = ((i % 255) + 1) as u8;
        }

        for length in 1..=1023 {
            // reset packet memory
            alloc.fill(0);
            let data = &pattern[0..length];
            let bytes_written = buffer.write(&data);

            assert_eq!(bytes_written, Ok(length));
            assert_eq!(tx_entry.get().tx_count(), length as _);
            assert_eq!(&alloc[0..length], data);
        }

        let result = buffer.write(&pattern[0..1025]);
        assert_eq!(result, Err(UsbError::BufferOverflow));
    }

    #[test]
    fn test_rx_buffer_reads() {
        const MEMORY_SIZE: usize = 2048;
        let mut memory = [0u8; MEMORY_SIZE];
        let device_memory = memory.as_ptr() as *const ();
        let btable_start = memory.as_ptr() as *const VolatileCell<RxEntry>;
        #[allow(unsafe_code)]
        let rx_entry = unsafe { &*btable_start.wrapping_add(1) };
        let alloc = &mut memory[BTABLE_ENTRY_SIZE * MAX_ENDPOINTS..];
        let mut allocator = Allocator::<()>::new(MEMORY_SIZE);
        let buffer = RxBuffer::new(&mut allocator, 0, 1023, device_memory).unwrap();
        let mut rx_entry_value = rx_entry.get();

        for i in 0..alloc.len() {
            // Fill with non-zero bytes
            alloc[i] = ((i % 255) + 1) as u8;
        }

        for length in 1..=1023 {
            rx_entry_value.set_rx_count(length as _);
            rx_entry.set(rx_entry_value);

            let mut data = [0u8; 1024];
            let bytes_read = buffer.read(&mut data);

            assert_eq!(bytes_read, Ok(length));
            assert_eq!(&data[0..length], &alloc[0..length]);
        }

        // Test overflow
        rx_entry_value.set_rx_count(117);
        rx_entry.set(rx_entry_value);
        let mut data = [0u8; 116];
        let bytes_read = buffer.read(&mut data);
        assert_eq!(bytes_read, Err(UsbError::BufferOverflow));
    }
}
