use bitfield::bitfield;

bitfield! {
    /// Transmit buffer descriptor
    #[derive(Clone, Copy)]
    pub(super) struct TxEntry(u32);
    impl Debug;
    u16;
    /// The number of bytes to be transmitted by the endpoint/channel associated
    /// with the register at the next IN token addressed to it.
    pub tx_count, set_tx_count: 25, 16;
    /// The starting address of the packet buffer containing data to be transmitted
    /// by the endpoint/channel associated with the register at the next
    /// IN token addressed to it. Bits 1 and 0 must always be written as “00” since
    /// packet memory is word wide and all packet buffers must be word aligned.
    tx_addr, _: 15, 0;
}

impl TxEntry {
    pub fn new(addr: usize) -> Self {
        Self(addr as u32)
    }
}

/// Allocation block size for rx buffers
#[derive(Debug, Clone, Copy, Eq, PartialEq)]
pub(super) enum RxBlockSize {
    Size2 = 0,
    Size32 = 1,
}

bitfield! {
    /// Receive buffer descriptor
    #[derive(Copy, Clone)]
    pub(super) struct RxEntry(u32);
    impl Debug;
    is_32byte_block, _: 31;
    num_blocks, _: 30, 26;
    /// The number of bytes received by the endpoint/channel associated with the
    /// register at the next OUT/SETUP transaction addressed to it.
    pub rx_count, set_rx_count: 25, 16;
    /// The starting address of the packet buffer containing data received by the
    /// endpoint/channel associated with the register at the next OUT/SETUP
    /// transaction addressed to it. Bits 1 and 0 must always be written as “00” since
    /// packet memory is word wide and all packet buffers must be word aligned.
    rx_addr, _: 15, 0;
}

impl RxEntry {
    pub fn new(addr: usize, block_size: RxBlockSize, num_blocks: usize) -> Self {
        debug_assert!(num_blocks <= 0b11111);
        Self(((block_size as u32) << 31) | ((num_blocks as u32) << 26) | (addr as u32))
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn test_tx_entry() {
        let mut tx_buffer = TxEntry::new(0x384);

        tx_buffer.set_tx_count(0x1f1);
        assert_eq!(tx_buffer.0, 0x1f10384);
        assert_eq!(tx_buffer.tx_count(), 0x1f1);
        assert_eq!(tx_buffer.tx_addr(), 0x384);
    }

    #[test]
    fn test_large_rx_entry() {
        let rx_buffer = RxEntry::new(0x222, RxBlockSize::Size32, 6);

        assert_eq!(rx_buffer.0, 0x98000222);
        assert!(rx_buffer.is_32byte_block());
        assert_eq!(rx_buffer.num_blocks(), 6);
        assert_eq!(rx_buffer.rx_count(), 0);
        assert_eq!(rx_buffer.rx_addr(), 0x222);
    }

    #[test]
    fn test_small_rx_entry() {
        let rx_buffer = RxEntry::new(0x828, RxBlockSize::Size2, 30);

        assert_eq!(rx_buffer.0, 0x78000828);
        assert!(!rx_buffer.is_32byte_block());
        assert_eq!(rx_buffer.num_blocks(), 30);
        assert_eq!(rx_buffer.rx_count(), 0);
        assert_eq!(rx_buffer.rx_addr(), 0x828);
    }
}
