#![no_std]
#![no_main]

use core::arch::asm;

use small_os_lib::{HeaderError, QueueError, check_critical, check_header_len, read_header, reply, reply_empty, send, send_empty};

/// Based on the Camera driver implementation at https://github.com/ArduCAM/PICO_SPI_CAM/blob/master/C/ArduCAM/ArduCAM.cpp (accessed 25/03/2026)

const TIMER_QUEUE: u32 = 2;

fn sleep(ms: u32) {
    send_empty(TIMER_QUEUE, 1, &ms.to_le_bytes()).unwrap();
}

mod cap_ctrl_register {
    pub const FRAMES_SHIFT: usize = 0;

    pub const FRAMES_MASK: u8 = 0xff << FRAMES_SHIFT;
}

mod sensor_timing_register {
    pub const HSYNC_POLARITY_SHIFT: usize = 0;
    pub const VSYNC_POLARITY_SHIFT: usize = 1;
    pub const PCLK_REVERSE_SHIFT: usize = 3;

    pub const HSYNC_POLARITY_MASK: u8 = 1 << HSYNC_POLARITY_SHIFT;
    pub const VSYNC_POLARITY_MASK: u8 = 1 << VSYNC_POLARITY_SHIFT;
    pub const PCLK_REVERSE_MASK: u8 = 1 << PCLK_REVERSE_SHIFT;
}

mod fifo_ctrl_register {
    pub const CLEAR_FIFO_SHIFT: usize = 0;
    pub const START_CAPTURE_SHIFT: usize = 1;
    pub const RESET_FIFO_SHIFT: usize = 2;

    pub const CLEAR_FIFO_MASK: u8 = 1 << CLEAR_FIFO_SHIFT;
    pub const START_CAPTURE_MASK: u8 = 1 << START_CAPTURE_SHIFT;
    pub const RESET_FIFO_MASK: u8 = 1 << RESET_FIFO_SHIFT;
}

mod test_mode_register {
    pub const CAM_SHIFT: usize = 0;
    pub const TEST_DATA_SHIFT: usize = 1;

    pub const CAM_MASK: u8 = 1 << CAM_SHIFT;
    pub const TEST_DATA_MASK: u8 = 1 << TEST_DATA_SHIFT;
}

mod gpio_write_register {
    pub const RESET_IO_SHIFT: usize = 0;
    pub const STANDBY_SHIFT: usize = 1;
    pub const POWER_ENABLE_SHIFT: usize = 2;

    pub const RESET_IO_MASK: u8 = 1 << RESET_IO_SHIFT;
    pub const STANDY_MASK: u8 = 1 << STANDBY_SHIFT;
    pub const POWER_ENABLE_MASK: u8 = 1 << POWER_ENABLE_SHIFT;
}

mod cpld_register {
    pub const RESET_CPLD_SHIFT: usize = 7;

    pub const RESET_CPLD_MASK: u8 = 1 << RESET_CPLD_SHIFT;
}

mod version_register {
    pub const DECIMAL_SHIFT: usize = 0;
    pub const INTEGER_SHIFT: usize = 4;

    pub const DECIMAL_MASK: u8 = 0xf << DECIMAL_SHIFT;
    pub const INTEGER_MASK: u8 = 0xf << INTEGER_SHIFT;
}

mod status_register {
    pub const VSYNC_SHIFT: usize = 0;
    pub const EXTERNAL_TRIGGER_SHIFT: usize = 1;
    pub const WRITE_FIFO_DONE_SHIFT: usize = 3;

    pub const VSYNC_MASK: u8 = 1 << VSYNC_SHIFT;
    pub const EXTERNAL_TRIGGER_MASK: u8 = 1 << EXTERNAL_TRIGGER_SHIFT;
    pub const WRITE_FIFO_DONE_MASK: u8 = 1 << WRITE_FIFO_DONE_SHIFT;
}

mod write_fifo_size0_register {
    pub const SIZE_SHIFT: usize = 0;

    pub const SIZE_MASK: u8 = 0xff << SIZE_SHIFT;
}

mod write_fifo_size1_register {
    pub const SIZE_SHIFT: usize = 0;

    pub const SIZE_MASK: u8 = 0xff << SIZE_SHIFT;
}

mod write_fifo_size2_register {
    pub const SIZE_SHIFT: usize = 0;

    pub const SIZE_MASK: u8 = 0xff << SIZE_SHIFT;
}

mod fifo_status_register {
    pub const FULL_SHIFT: usize = 0;

    pub const FULL_MASK: u8 = 1 << FULL_SHIFT;
}

mod year_register {
    pub const YEAR_SHIFT: usize = 0;

    pub const YEAR_MASK: u8 = 0x7f << YEAR_SHIFT;
}

mod month_register {
    pub const MONTH_SHIFT: usize = 0;

    pub const MONTH_MASK: u8 = 0xf << MONTH_SHIFT;
}

mod date_register {
    pub const DATE_SHIFT: usize = 0;

    pub const DATE_MASK: u8 = 0x1f << DATE_SHIFT;
}

#[derive(Debug, Clone, Copy)]
enum CamReg {
    Test = 0x0,
    CapCtrl = 0x1,
    SensorTiming = 0x3,
    FifoCtrl = 0x4,
    TestMode = 0x5,
    GPIOWrite = 0x6,
    CPLD = 0x7,
    BurstFIFO = 0x3c,
    SingleFIFO = 0x3d,
    Version = 0x40,
    Status = 0x41,
    WriteFIFOSize0 = 0x42,
    WriteFIFOSize1 = 0x43,
    WriteFIFOSize2 = 0x44,
    FifoStatus = 0x45,
    Year = 0x46,
    Month = 0x47,
    Date = 0x48
}

#[derive(Debug, Clone, Copy)]
enum DSPReg {
    BypassDSP = 0x5,
    QuantizationScale = 0x44,
    CtrlI = 0x50,
    HSize0 = 0x51,
    VSize0 = 0x52,
    OffsetX0 = 0x53,
    OffsetY0 = 0x54,
    SizeOffset1 = 0x55,
    DPSel = 0x56,
    HSize2 = 0x57,
    OutWidth0 = 0x5a,
    OutHeight0 = 0x5b,
    OutSize1 = 0x5c,
    AddrAccess = 0x7c,
    DataAccess = 0x7d,
    Ctrl2 = 0x86,
    Ctrl3 = 0x87,
    Size0 = 0x8c,
    HSize8 = 0xc0,
    VSize8 = 0xc1,
    Ctrl0 = 0xc2,
    Ctrl1 = 0xc3,
    DVPSP = 0xd3,
    ImageMode = 0xda,
    Reset = 0xe0,
    MasterSpeed = 0xf0,
    SlaveID = 0xf7,
    SlaveCtrl = 0xf8,
    MCBist = 0xf9,
    ProgMemAddr0 = 0xfa,
    ProgMemAddr1 = 0xfb,
    ProgMemAccess = 0xfc,
    ProtocolCmd = 0xfd,
    ProtocolStatus = 0xfe,
    BankSelect = 0xff
}

#[derive(Debug, Clone, Copy)]
enum SensorReg {
    Gain = 0x0,
    Com1 = 0x3,
    Reg4 = 0x4,
    Reg8 = 0x8,
    Com2 = 0x9,
    ProdIDNum1 = 0xa,
    ProdIDNum2 = 0xb,
    Com3 = 0xc,
    Com4 = 0xd,
    AutoExpCtrl = 0x10,
    ClockRateCtrl = 0x11,
    Com7 = 0x12,
    Com8 = 0xc7,
    Com9 = 0x14,
    Com10 = 0x15,
    HorizWindowStart = 0x17,
    HorizWindowEnd = 0x18,
    VertWindowStart = 0x19,
    VertWindowEnd = 0x1a,
    Manufacture1 = 0x1c,
    Manufacture0 = 0x1d,
    Luminance1 = 0x24,
    Luminance0 = 0x25,
    LargeStepThreshold = 0x26,
    Reg2A = 0x2a,
    FrameRate = 0x2b,
    VSyncPulse0 = 0x2d,
    VSyncPulse1 = 0x2e,
    LuminanceAvg = 0x2f,
    HSyncStart = 0x30,
    HSyncEnd = 0x31,
    Reg32 = 0x32,
    ARCom2 = 0x34,
    Reg45 = 0x45,
    FrameLength0 = 0x46,
    FrameLength1 = 0x47,
    Com19 = 0x48,
    Zoom = 0x49,
    Com22 = 0x4b,
    Com25 = 0x4e,
    Band50 = 0x4f,
    Band60 = 0x50,
    Reg5d = 0x5d,
    Reg5e = 0x5e,
    Reg5f = 0x5f,
    Reg60 = 0x60,
    HistLowLevel = 0x61,
    HistHighLevel = 0x62,
    BankSelect = 0xff
}

const SPI_QUEUE: u32 = 0;
const SYNC_TAG: u16 = 0;

const I2C_QUEUE: u32 = 1;
const I2C_TAG: u16 = 0;

const SPI_MAX_READ_SIZE: usize = 32;
const I2C_MAX_READ_SIZE: usize = 32;

const I2C_ADDR: u8 = 0x60 >> 1;

fn spi_send_read_cmd(reg: CamReg) -> u8 {
    let mut reg = [reg as u8];
    send(SPI_QUEUE, SYNC_TAG, &mut reg, 1, 1).unwrap();
    reg[0]
}

fn spi_send_burst_read_cmd(reg: CamReg, buffer: &mut [u8]) {
    let mut data = [0; 32];
    data[0] = reg as u8;
    let mut offset = 0;
    while offset < buffer.len() {
        let reply_len = SPI_MAX_READ_SIZE.min(buffer.len() - offset);
        send(SPI_QUEUE, SYNC_TAG, &mut data, 1, reply_len).unwrap();
        buffer[offset..offset + reply_len].copy_from_slice(&data[..reply_len]);
        offset += reply_len;
    }
}

fn spi_send_write_cmd(reg: CamReg, val: u8) {
    let reg = [(reg as u8) | 0x80, val];
    send_empty(SPI_QUEUE, SYNC_TAG, &reg).unwrap();
    sleep(1);
}

fn i2c_read_dsp_reg(reg: DSPReg) -> u8 {
    let mut reg = [I2C_ADDR, reg as u8];
    send(I2C_QUEUE, I2C_TAG, &mut reg, 2, 1).unwrap();
    reg[0]
}

fn i2c_read_sensor_reg(reg: SensorReg) -> u8 {
    let mut reg = [I2C_ADDR, reg as u8];
    send(I2C_QUEUE, I2C_TAG, &mut reg, 2, 1).unwrap();
    reg[0]
}

fn i2c_write_dsp_reg(reg: DSPReg, val: u8) {
    let reg = [I2C_ADDR, reg as u8, val];
    send_empty(I2C_QUEUE, I2C_TAG, &reg).unwrap();
}

fn i2c_write_sensor_reg(reg: SensorReg, val: u8) {
    let reg = [I2C_ADDR, reg as u8, val];
    send_empty(I2C_QUEUE, I2C_TAG, &reg).unwrap();
}

fn get_sensor_manufacture() -> u16 {
    let mut res = 0;
    i2c_write_dsp_reg(DSPReg::BankSelect, 1);
    res |= i2c_read_sensor_reg(SensorReg::ProdIDNum1) as u16;
    res |= (i2c_read_sensor_reg(SensorReg::ProdIDNum2) as u16) << 8;
    res
}

pub struct Camera {
    bank_sel: bool
}

impl Camera {
    pub fn new() -> Self {
        let mut res = Self { 
            bank_sel: false 
        };
        res.reset();
        res
    }

    pub fn reset(&mut self) {
        i2c_write_sensor_reg(SensorReg::BankSelect, 1);
        i2c_write_sensor_reg(SensorReg::Com7, 0x80);
        sleep(100);
        spi_send_write_cmd(CamReg::CPLD, cpld_register::RESET_CPLD_MASK);
        sleep(100);
        spi_send_write_cmd(CamReg::CPLD, 0);
        sleep(100);
        spi_send_write_cmd(CamReg::FifoCtrl, fifo_ctrl_register::RESET_FIFO_MASK | fifo_ctrl_register::CLEAR_FIFO_MASK);
        i2c_write_dsp_reg(DSPReg::Reset, 0xff);
        i2c_write_dsp_reg(DSPReg::Reset, 0);
        i2c_write_sensor_reg(SensorReg::BankSelect, 0);
        i2c_write_dsp_reg(DSPReg::ImageMode, 0);
        self.take_picture();
    }

    pub fn take_picture(&mut self) {
        spi_send_write_cmd(CamReg::CapCtrl, 0);
        loop {
            let status = spi_send_read_cmd(CamReg::Status);
            if status & status_register::WRITE_FIFO_DONE_MASK != 0 {
                break;
            }
        }
        let mut fifo_size: u32 = 0;
        fifo_size |= spi_send_read_cmd(CamReg::WriteFIFOSize0) as u32;
        fifo_size |= (spi_send_read_cmd(CamReg::WriteFIFOSize1) as u32) << 8;
        fifo_size |= (spi_send_read_cmd(CamReg::WriteFIFOSize2) as u32) << 16;
        let x = fifo_size;
    }
}

pub enum CamReplyError {
    SendError,
    InvalidRequest,
    InvalidSendBuffer,
    InvalidReplyBuffer
}

impl From<CamReplyError> for u32 {
    fn from(value: CamReplyError) -> Self {
        match value {
            CamReplyError::SendError => 1,
            CamReplyError::InvalidRequest => 2,
            CamReplyError::InvalidSendBuffer => 3,
            CamReplyError::InvalidReplyBuffer => 4
        }
    }
}

impl From<HeaderError> for CamReplyError {
    fn from(value: HeaderError) -> Self {
        match value {
            HeaderError::InvalidSendBuffer => Self::InvalidSendBuffer,
            HeaderError::InvalidReplyBuffer => Self::InvalidReplyBuffer
        }
    }
}

pub enum CamError {
    ReplyError(CamReplyError),
    QueueError(QueueError)
}

impl From<CamReplyError> for CamError {
    fn from(value: CamReplyError) -> Self {
        Self::ReplyError(value)
    }
}

impl From<QueueError> for CamError {
    fn from(value: QueueError) -> Self {
        Self::QueueError(value)
    }
}

impl From<HeaderError> for CamError {
    fn from(value: HeaderError) -> Self {
        Self::from(CamReplyError::from(value))
    }
}

pub enum Request {
    PowerOff,
    PowerOn,
    Standby,
    ExitStandy,
    Version,
    ManufactureID
}

impl Request {
    pub fn parse() -> Result<Self, CamError> {
        let header = read_header(0)?;
        match header.tag {
            0 => {
                // Power Off
                check_header_len(&header, 0, 0)?;
                Ok(Self::PowerOff)
            },
            1 => {
                // Power On
                check_header_len(&header, 0, 0)?;
                Ok(Self::PowerOn)
            },
            2 => {
                // Standby
                check_header_len(&header, 0, 0)?;
                Ok(Self::Standby)
            },
            3 => {
                // Exit Standby
                check_header_len(&header, 0, 0)?;
                Ok(Self::ExitStandy)
            },
            4 => {
                // Version
                check_header_len(&header, 0, 4)?;
                Ok(Self::Version)
            },
            5 => {
                // ManufactureID
                check_header_len(&header, 0, 2)?;
                Ok(Self::ManufactureID)
            },
            _ => {
                Err(CamError::ReplyError(CamReplyError::InvalidRequest))
            }
        }
    }
}
fn get_version() -> [u8; 4] {
    let mut res = [0; 4];
    res[0] = spi_send_read_cmd(CamReg::Version);
    res[1] = (spi_send_read_cmd(CamReg::Year) & year_register::YEAR_MASK) >> year_register::YEAR_SHIFT;
    res[2] = (spi_send_read_cmd(CamReg::Month) & month_register::MONTH_MASK) >> month_register::MONTH_SHIFT;
    res[3] = (spi_send_read_cmd(CamReg::Date) & date_register::DATE_MASK) >> date_register::DATE_SHIFT;
    res
}


/// Program entry point
/// Disables mangling so it can be called from assembly
#[unsafe(no_mangle)]
pub extern "C" fn main(num_args: usize) {
    assert!(num_args == 0);
    let mut cam = Camera::new();
    loop {
        match Request::parse() {
            Ok(request) => {
                match request {
                    Request::PowerOff => {
                        spi_send_write_cmd(CamReg::GPIOWrite, 0);
                        check_critical(reply_empty(0, 0)).unwrap_or(Ok(())).unwrap();
                    },
                    Request::PowerOn => {
                        cam.reset();
                        check_critical(reply_empty(0, 0)).unwrap_or(Ok(())).unwrap();
                    },
                    Request::Standby => {
                        spi_send_write_cmd(CamReg::GPIOWrite, gpio_write_register::STANDY_MASK | gpio_write_register::POWER_ENABLE_MASK);
                        check_critical(reply_empty(0, 0)).unwrap_or(Ok(())).unwrap();
                    },
                    Request::ExitStandy => {
                        spi_send_write_cmd(CamReg::GPIOWrite, gpio_write_register::POWER_ENABLE_MASK);
                        check_critical(reply_empty(0, 0)).unwrap_or(Ok(())).unwrap();
                    },
                    Request::Version => {
                        let version = get_version();
                        check_critical(reply(0, 0, &version)).unwrap_or(Ok(())).unwrap();
                    },
                    Request::ManufactureID => {
                        let manufacture_id = get_sensor_manufacture();
                        check_critical(reply(0, 0, &manufacture_id.to_le_bytes())).unwrap_or(Ok(())).unwrap();
                    },
                }
            },
            Err(err) => {
                match err {
                    CamError::ReplyError(err) => {
                        check_critical(reply_empty(0, u32::from(err))).unwrap_or(Ok(())).unwrap();
                    },
                    CamError::QueueError(err) => {
                        match err {
                            QueueError::Died => {},
                            QueueError::SenderInvalidMemoryAccess => {
                                check_critical(reply_empty(0, u32::from(CamReplyError::SendError))).unwrap_or(Ok(())).unwrap();
                            },
                            _ => {
                                panic!("{:?}", err);
                            }
                        }
                    }
                }
            }
        }
    }
}
