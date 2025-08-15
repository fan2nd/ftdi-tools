use futures_lite::future::{block_on, zip};
use nusb::transfer::{Control, ControlType, Recipient, RequestBuffer};
use std::time::Duration;

#[derive(Debug, thiserror::Error)]
pub enum FtdiError {
    #[error("A USB transport error occurred.")]
    ///
    /// This variant is used for all errors reported by the operating system when performing a USB
    /// operation. It may indicate that the USB device was unplugged, that another application or an
    /// operating system driver is currently using it, or that the current user does not have
    /// permission to access it.
    Usb(#[from] nusb::Error),

    #[error("Open failed: {0}")]
    /// Error occurs when open.
    OpenFailed(String),

    #[error("Unsupported chip type: {0:?}")]
    /// The connected device is not supported by the driver.
    UnsupportedChipType(ChipType),

    #[error("Bad Mpsse Command: {0:#x}")]
    /// The connected device is not supported by the driver.
    BadMpsseCommand(u8),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ChipType {
    Am,
    Bm,
    FT2232C,
    R,
    FT2232H,
    FT4232H,
    FT232H,
    FT230X,
}
impl ChipType {
    pub fn interface_list(self) -> &'static [Interface] {
        match self {
            ChipType::FT232H => &[Interface::A],
            ChipType::FT2232H => &[Interface::A, Interface::B],
            ChipType::FT4232H => &[Interface::A, Interface::B, Interface::C, Interface::D],
            _ => &[],
        }
    }
    pub fn mpsse_list(self) -> &'static [Interface] {
        match self {
            ChipType::FT232H => &[Interface::A],
            ChipType::FT2232H | ChipType::FT4232H => &[Interface::A, Interface::B],
            _ => &[],
        }
    }
    pub fn upper_pins(self) -> usize {
        match self {
            ChipType::FT232H | ChipType::FT2232H => 8,
            ChipType::FT4232H => 0,
            _ => 0,
        }
    }
}
#[repr(C)]
#[expect(unused)]
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub(crate) enum BitMode {
    Reset = 0,
    Bitbang = 1,
    Mpsse = 2,
    SyncBb = 4,
    Mcu = 8,
    Opto = 16,
    Cbus = 32,
    SyncFf = 64,
    Ft1284 = 128,
}

#[repr(C)]
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Interface {
    A = 1,
    B = 2,
    C = 3,
    D = 4,
}

impl Interface {
    fn read_ep(self) -> u8 {
        match self {
            Interface::A => 0x81,
            Interface::B => 0x83,
            Interface::C => 0x85,
            Interface::D => 0x87,
        }
    }

    fn write_ep(self) -> u8 {
        match self {
            Interface::A => 0x02,
            Interface::B => 0x04,
            Interface::C => 0x06,
            Interface::D => 0x08,
        }
    }

    fn index(&self) -> u16 {
        *self as u16
    }

    pub(crate) fn interface_number(&self) -> u8 {
        (*self as u8) - 1
    }
}

pub(crate) struct FtdiContext {
    /// USB device handle
    handle: nusb::Interface,
    /// FTDI device interface
    interface: Interface,
    write_ep: u8,
    read_ep: u8,
    max_packet_size: usize,
}

impl FtdiContext {
    pub(crate) fn new(
        handle: nusb::Interface,
        interface: Interface,
        max_packet_size: usize,
    ) -> Self {
        Self {
            handle,
            interface,
            write_ep: interface.write_ep(),
            read_ep: interface.read_ep(),
            max_packet_size,
        }
    }
    pub(crate) fn into_mpsse(mut self, mask: u8) -> Result<Self, FtdiError> {
        self.usb_reset()?;
        self.usb_purge_buffers()?;
        self.set_latency_timer(16)?;
        self.set_bitmode(mask, BitMode::Mpsse)?;
        Ok(self)
    }
    fn sio_write(&mut self, request: u8, value: u16) -> Result<(), FtdiError> {
        self.handle
            .control_out_blocking(
                Control {
                    control_type: ControlType::Vendor,
                    recipient: Recipient::Device,
                    request,
                    value,
                    index: self.interface.index(),
                },
                &[],
                Duration::from_secs(1),
            )
            .map_err(std::io::Error::from)?;

        Ok(())
    }

    fn usb_reset(&mut self) -> Result<(), FtdiError> {
        const SIO_RESET_REQUEST: u8 = 0;
        const SIO_RESET_SIO: u16 = 0;

        self.sio_write(SIO_RESET_REQUEST, SIO_RESET_SIO)
    }

    /// Clears the write buffer on the chip.
    fn usb_purge_tx_buffer(&mut self) -> Result<(), FtdiError> {
        const SIO_RESET_REQUEST: u8 = 0;
        const SIO_RESET_PURGE_TX: u16 = 2;

        self.sio_write(SIO_RESET_REQUEST, SIO_RESET_PURGE_TX)
    }

    fn usb_purge_rx_buffer(&mut self) -> Result<(), FtdiError> {
        const SIO_RESET_REQUEST: u8 = 0;
        const SIO_RESET_PURGE_RX: u16 = 1;

        self.sio_write(SIO_RESET_REQUEST, SIO_RESET_PURGE_RX)?;

        Ok(())
    }

    fn usb_purge_buffers(&mut self) -> Result<(), FtdiError> {
        self.usb_purge_tx_buffer()?;
        self.usb_purge_rx_buffer()?;

        Ok(())
    }

    fn set_latency_timer(&mut self, value: u8) -> Result<(), FtdiError> {
        const SIO_SET_LATENCY_TIMER_REQUEST: u8 = 0x09;

        self.sio_write(SIO_SET_LATENCY_TIMER_REQUEST, value as u16)
    }

    fn set_bitmode(&mut self, bitmask: u8, mode: BitMode) -> Result<(), FtdiError> {
        const SIO_SET_BITMODE_REQUEST: u8 = 0x0B;

        self.sio_write(
            SIO_SET_BITMODE_REQUEST,
            u16::from_le_bytes([bitmask, mode as u8]),
        )?;

        Ok(())
    }
    pub(crate) fn write_read(&self, write: &[u8], read: &mut [u8]) -> Result<(), FtdiError> {
        let write = async {
            for batch in write.chunks(self.max_packet_size) {
                self.handle
                    .bulk_out(self.write_ep, Vec::from(batch))
                    .await
                    .into_result()
                    .map_err(std::io::Error::from)?;
            }
            Result::<(), FtdiError>::Ok(())
        };
        let read = async {
            let mut read_len = 0;
            while read_len < read.len() {
                let result = self
                    .handle
                    .bulk_in(self.read_ep, RequestBuffer::new(self.max_packet_size))
                    .await
                    .into_result()
                    .map_err(std::io::Error::from)?;
                if result.len() > 2 {
                    let (status, data) = result.split_at(2);
                    if status[0] == 0xFA {
                        return Err(FtdiError::BadMpsseCommand(status[1]));
                    }
                    let (_, read_buf) = read.split_at_mut(read_len);
                    let (read_buf, _) = read_buf.split_at_mut(data.len());
                    read_buf.copy_from_slice(data);
                    read_len += data.len()
                }
            }
            Result::<(), FtdiError>::Ok(())
        };
        let result = block_on(zip(write, read));
        if result.0.is_err() {
            result.0
        } else if result.1.is_err() {
            result.1
        } else {
            Ok(())
        }
    }
}
