// use core intrinsics 
#![feature(core_intrinsics)]
// test framework
#![feature(custom_test_frameworks)]
#![test_runner(crate::test::test_runner)]

#![no_std]
#![no_main]
#![reexport_test_harness_main = "test_main"]

use small_os_lib::{HeaderError, QueueError, args, check_critical, check_header_len, read_header, receive, reply, reply_empty, send, send_empty};

/// Derived from the camera driver implementation at https://github.com/ArduCAM/PICO_SPI_CAM/blob/master/C/ArduCAM/ArduCAM.cpp, github.com/ArduCAM/PICO_SPI_CAM/blob/master/C/Examples/ArduCAM_Mini_2MP_Plus_4CAM_VideoStreaming/ArduCAM_Mini_2MP_Plus_4CAM_VideoStreaming.cpp,
/// (github.com/ArduCAM/PICO_SPI_CAM/blob/master/C/Examples/ArduCAM_Mini_2MP_Plus_4CAM_VideoStreaming/ArduCAM_Mini_2MP_Plus_4CAM_VideoStreaming.cpp (accessed 25/03/2026)

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
    pub const RESET_FIFO_SHIFT: usize = 4;

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
    Reserved0 = 0x00,
    BypassDSP = 0x05,
    Reserved12 = 0x12,
    Reserved2C = 0x2c,
    Reserved2E = 0x2e,
    Reserved33 = 0x33,
    Reserved3C = 0x3c,
    Reserved41 = 0x41,
    Reserved42 = 0x42,
    Reserved43 = 0x43,
    QuantizationScale = 0x44,
    Reserved4C = 0x4c,
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
    Reserved76 = 0x76,
    AddrAccess = 0x7c,
    DataAccess = 0x7d,
    Reserved7F = 0x7f,
    Ctrl2 = 0x86,
    Ctrl3 = 0x87,
    Reserved88 = 0x88,
    Size0 = 0x8c,
    Reserved90 = 0x90,
    Reserved91 = 0x91,
    Reserved92 = 0x92,
    Reserved93 = 0x93,
    Reserved96 = 0x96,
    Reserved97 = 0x97,
    ReservedA4 = 0xa4,
    ReservedA6 = 0xa6,
    ReservedA7 = 0xa7,
    ReservedA8 = 0xa8,
    ReservedB0 = 0xb0,
    ReservedB1 = 0xb1,
    ReservedB2 = 0xb2,
    ReservedB3 = 0xb3,
    ReservedB4 = 0xb4,
    ReservedB5 = 0xb5,
    ReservedB6 = 0xb6,
    ReservedB7 = 0xb7,
    ReservedB8 = 0xb8,
    ReservedB9 = 0xb9,
    ReservedBF = 0xbf,
    HSize8 = 0xc0,
    VSize8 = 0xc1,
    Ctrl0 = 0xc2,
    Ctrl1 = 0xc3,
    ReservedC4 = 0xc4,
    ReservedC5 = 0xc5,
    ReservedC6 = 0xc6,
    ReservedC7 = 0xc7,
    ReservedC8 = 0xc8,
    ReservedC9 = 0xc9,
    DVPSP = 0xd3,
    ReservedD7 = 0xd7,
    ReservedD9 = 0xd9,
    ImageMode = 0xda,
    ReservedDD = 0xdd,
    ReservedDF = 0xdf,
    Reset = 0xe0,
    ReservedE1 = 0xe1,
    ReservedE5 = 0xe5,
    MasterSpeed = 0xf0,
    SlaveID = 0xf7,
    SlaveCtrl = 0xf8,
    MCBist = 0xf9,
    ProgMemAddr0 = 0xfa,
    ProgMemAddr1 = 0xfb,
    ProgMemAccess = 0xfc,
    ProtocolCmd = 0xfd,
    ProtocolStatus = 0xfe,
    BankSelect = 0xff,
}

#[derive(Debug, Clone, Copy)]
enum SensorReg {
    Gain = 0x00,
    Com1 = 0x03,
    Reg4 = 0x04,
    Reserved6 = 0x06,
    Reserved7 = 0x07,
    Reg8 = 0x08,
    Com2 = 0x09,
    ProdIDNum1 = 0x0a,
    ProdIDNum2 = 0x0b,
    Com3 = 0x0c,
    Com4 = 0x0d,
    ReservedE = 0x0e,
    AutoExpCtrl = 0x10,
    ClockRateCtrl = 0x11,
    Com7 = 0x12,
    Com8 = 0x13,
    Com9 = 0x14,
    Com10 = 0x15,
    Reserved16 = 0x16,
    HorizWindowStart = 0x17,
    HorizWindowEnd = 0x18,
    VertWindowStart = 0x19,
    VertWindowEnd = 0x1a,
    Manufacture1 = 0x1c,
    Manufacture0 = 0x1d,
    Reserved20 = 0x20,
    Reserved21 = 0x21,
    Reserved22 = 0x22,
    Reserved23 = 0x23,
    Reserved24 = 0x24,
    Reserved25 = 0x25,
    LargeStepThreshold = 0x26,
    Reserved28 = 0x28,
    Reg2A = 0x2a,
    FrameRate = 0x2b,
    Reserved2C = 0x2c,
    VSyncPulse0 = 0x2d,
    VSyncPulse1 = 0x2e,
    LuminanceAvg = 0x2f,
    HSyncStart = 0x30,
    HSyncEnd = 0x31,
    Reg32 = 0x32,
    Reserved33 = 0x33,
    ARCom2 = 0x34,
    Reserved35 = 0x35,
    Reserved36 = 0x36,
    Reserved37 = 0x37,
    Reserved39 = 0x39,
    Reserved3A = 0x3a,
    Reserved3B = 0x3b,
    Reserved3C = 0x3c,
    Reserved3D = 0x3d,
    Reserved3E = 0x3e,
    Reserved42 = 0x42,
    Reserved43 = 0x43,
    Reg45 = 0x45,
    FrameLength0 = 0x46,
    FrameLength1 = 0x47,
    Com19 = 0x48,
    Zoom = 0x49,
    Reserved4A = 0x4a,
    Com22 = 0x4b,
    Reserved4C = 0x4c,
    Com25 = 0x4e,
    Band50 = 0x4f,
    Band60 = 0x50,
    Reserved5A = 0x5a,
    Reserved5B = 0x5b,
    Reserved5C = 0x5c,
    Reg5D = 0x5d,
    Reg5E = 0x5e,
    Reg5F = 0x5f,
    Reg60 = 0x60,
    HistLowLevel = 0x61,
    HistHighLevel = 0x62,
    Reserved63 = 0x63,
    Reserved6C = 0x6c,
    Reserved6D = 0x6d,
    Reserved6E = 0x6e,
    Reserved70 = 0x70,
    Reserved71 = 0x71,
    Reserved73 = 0x73,
    Reserved7C = 0x7c,
    BankSelect = 0xff,
}

enum I2CReg {
    DSP(DSPReg),
    Sensor(SensorReg),
}

const QVGA: [(I2CReg, u8); 188] = [
    (I2CReg::DSP(DSPReg::Reserved2C), 0xff),
    (I2CReg::DSP(DSPReg::Reserved2E), 0xdf),
    (I2CReg::Sensor(SensorReg::Reserved3C), 0x32),
    (I2CReg::Sensor(SensorReg::ClockRateCtrl), 0x0),
    (I2CReg::Sensor(SensorReg::Com2), 0x2),
    (I2CReg::Sensor(SensorReg::Reg4), 0xa8),
    (I2CReg::Sensor(SensorReg::Com8), 0xe5),
    (I2CReg::Sensor(SensorReg::Com9), 0x48),
    (I2CReg::Sensor(SensorReg::Reserved2C), 0xc),
    (I2CReg::Sensor(SensorReg::Reserved33), 0x78),
    (I2CReg::Sensor(SensorReg::Reserved3A), 0x33),
    (I2CReg::Sensor(SensorReg::Reserved3B), 0xfb),
    (I2CReg::Sensor(SensorReg::Reserved3E), 0x0),
    (I2CReg::Sensor(SensorReg::Reserved43), 0x11),
    (I2CReg::Sensor(SensorReg::Reserved16), 0x10),
    (I2CReg::Sensor(SensorReg::Reserved39), 0x2),
    (I2CReg::Sensor(SensorReg::Reserved35), 0x88),
    (I2CReg::Sensor(SensorReg::Reserved22), 0xa),
    (I2CReg::Sensor(SensorReg::Reserved37), 0x40),
    (I2CReg::Sensor(SensorReg::Reserved23), 0x0),
    (I2CReg::Sensor(SensorReg::ARCom2), 0xa0),
    (I2CReg::Sensor(SensorReg::Reserved6), 0x2),
    (I2CReg::Sensor(SensorReg::Reserved6), 0x88),
    (I2CReg::Sensor(SensorReg::Reserved7), 0xc0),
    (I2CReg::Sensor(SensorReg::Com4), 0xb7),
    (I2CReg::Sensor(SensorReg::ReservedE), 0x1),
    (I2CReg::Sensor(SensorReg::Reserved4C), 0x0),
    (I2CReg::Sensor(SensorReg::Reserved4A), 0x81),
    (I2CReg::Sensor(SensorReg::Reserved21), 0x99),
    (I2CReg::Sensor(SensorReg::Reserved24), 0x40),
    (I2CReg::Sensor(SensorReg::Reserved25), 0x38),
    (I2CReg::Sensor(SensorReg::LargeStepThreshold), 0x82),
    (I2CReg::Sensor(SensorReg::Reserved5C), 0x0),
    (I2CReg::Sensor(SensorReg::Reserved63), 0x0),
    (I2CReg::Sensor(SensorReg::FrameLength0), 0x22),
    (I2CReg::Sensor(SensorReg::Com3), 0x3a),
    (I2CReg::Sensor(SensorReg::Reg5D), 0x55),
    (I2CReg::Sensor(SensorReg::Reg5E), 0x7d),
    (I2CReg::Sensor(SensorReg::Reg5F), 0x7d),
    (I2CReg::Sensor(SensorReg::Reg60), 0x55),
    (I2CReg::Sensor(SensorReg::HistLowLevel), 0x70),
    (I2CReg::Sensor(SensorReg::HistHighLevel), 0x80),
    (I2CReg::Sensor(SensorReg::Reserved7C), 0x5),
    (I2CReg::Sensor(SensorReg::Reserved20), 0x80),
    (I2CReg::Sensor(SensorReg::Reserved28), 0x30),
    (I2CReg::Sensor(SensorReg::Reserved6C), 0x0),
    (I2CReg::Sensor(SensorReg::Reserved6D), 0x80),
    (I2CReg::Sensor(SensorReg::Reserved6E), 0x0),
    (I2CReg::Sensor(SensorReg::Reserved70), 0x2),
    (I2CReg::Sensor(SensorReg::Reserved71), 0x94),
    (I2CReg::Sensor(SensorReg::Reserved73), 0xc1),
    (I2CReg::Sensor(SensorReg::Reserved3D), 0x34),
    (I2CReg::Sensor(SensorReg::Com7), 0x4),
    (I2CReg::Sensor(SensorReg::Reserved5A), 0x57),
    (I2CReg::Sensor(SensorReg::Band50), 0xbb),
    (I2CReg::Sensor(SensorReg::Band60), 0x9c),
    (I2CReg::DSP(DSPReg::ReservedE5), 0x7f),
    (I2CReg::DSP(DSPReg::MCBist), 0xc0),
    (I2CReg::DSP(DSPReg::Reserved41), 0x24),
    (I2CReg::DSP(DSPReg::Reset), 0x14),
    (I2CReg::DSP(DSPReg::Reserved76), 0xff),
    (I2CReg::DSP(DSPReg::Reserved33), 0xa0),
    (I2CReg::DSP(DSPReg::Reserved42), 0x20),
    (I2CReg::DSP(DSPReg::Reserved43), 0x18),
    (I2CReg::DSP(DSPReg::Reserved4C), 0x0),
    (I2CReg::DSP(DSPReg::Ctrl3), 0xd0),
    (I2CReg::DSP(DSPReg::Reserved88), 0x3f),
    (I2CReg::DSP(DSPReg::ReservedD7), 0x3),
    (I2CReg::DSP(DSPReg::ReservedD9), 0x10),
    (I2CReg::DSP(DSPReg::DVPSP), 0x82),
    (I2CReg::DSP(DSPReg::ReservedC8), 0x8),
    (I2CReg::DSP(DSPReg::ReservedC9), 0x80),
    (I2CReg::DSP(DSPReg::AddrAccess), 0x0),
    (I2CReg::DSP(DSPReg::DataAccess), 0x0),
    (I2CReg::DSP(DSPReg::AddrAccess), 0x3),
    (I2CReg::DSP(DSPReg::DataAccess), 0x48),
    (I2CReg::DSP(DSPReg::DataAccess), 0x48),
    (I2CReg::DSP(DSPReg::AddrAccess), 0x8),
    (I2CReg::DSP(DSPReg::DataAccess), 0x20),
    (I2CReg::DSP(DSPReg::DataAccess), 0x10),
    (I2CReg::DSP(DSPReg::DataAccess), 0xe),
    (I2CReg::DSP(DSPReg::Reserved90), 0x0),
    (I2CReg::DSP(DSPReg::Reserved91), 0xe),
    (I2CReg::DSP(DSPReg::Reserved91), 0x1a),
    (I2CReg::DSP(DSPReg::Reserved91), 0x31),
    (I2CReg::DSP(DSPReg::Reserved91), 0x5a),
    (I2CReg::DSP(DSPReg::Reserved91), 0x69),
    (I2CReg::DSP(DSPReg::Reserved91), 0x75),
    (I2CReg::DSP(DSPReg::Reserved91), 0x7e),
    (I2CReg::DSP(DSPReg::Reserved91), 0x88),
    (I2CReg::DSP(DSPReg::Reserved91), 0x8f),
    (I2CReg::DSP(DSPReg::Reserved91), 0x96),
    (I2CReg::DSP(DSPReg::Reserved91), 0xa3),
    (I2CReg::DSP(DSPReg::Reserved91), 0xaf),
    (I2CReg::DSP(DSPReg::Reserved91), 0xc4),
    (I2CReg::DSP(DSPReg::Reserved91), 0xd7),
    (I2CReg::DSP(DSPReg::Reserved91), 0xe8),
    (I2CReg::DSP(DSPReg::Reserved91), 0x20),
    (I2CReg::DSP(DSPReg::Reserved92), 0x0),
    (I2CReg::DSP(DSPReg::Reserved93), 0x6),
    (I2CReg::DSP(DSPReg::Reserved93), 0xe3),
    (I2CReg::DSP(DSPReg::Reserved93), 0x3),
    (I2CReg::DSP(DSPReg::Reserved93), 0x3),
    (I2CReg::DSP(DSPReg::Reserved93), 0x0),
    (I2CReg::DSP(DSPReg::Reserved93), 0x2),
    (I2CReg::DSP(DSPReg::Reserved93), 0x0),
    (I2CReg::DSP(DSPReg::Reserved93), 0x0),
    (I2CReg::DSP(DSPReg::Reserved93), 0x0),
    (I2CReg::DSP(DSPReg::Reserved93), 0x0),
    (I2CReg::DSP(DSPReg::Reserved93), 0x0),
    (I2CReg::DSP(DSPReg::Reserved93), 0x0),
    (I2CReg::DSP(DSPReg::Reserved93), 0x0),
    (I2CReg::DSP(DSPReg::Reserved96), 0x0),
    (I2CReg::DSP(DSPReg::Reserved97), 0x8),
    (I2CReg::DSP(DSPReg::Reserved97), 0x19),
    (I2CReg::DSP(DSPReg::Reserved97), 0x2),
    (I2CReg::DSP(DSPReg::Reserved97), 0xc),
    (I2CReg::DSP(DSPReg::Reserved97), 0x24),
    (I2CReg::DSP(DSPReg::Reserved97), 0x30),
    (I2CReg::DSP(DSPReg::Reserved97), 0x28),
    (I2CReg::DSP(DSPReg::Reserved97), 0x26),
    (I2CReg::DSP(DSPReg::Reserved97), 0x2),
    (I2CReg::DSP(DSPReg::Reserved97), 0x98),
    (I2CReg::DSP(DSPReg::Reserved97), 0x80),
    (I2CReg::DSP(DSPReg::Reserved97), 0x0),
    (I2CReg::DSP(DSPReg::Reserved97), 0x0),
    (I2CReg::DSP(DSPReg::ReservedA4), 0x0),
    (I2CReg::DSP(DSPReg::ReservedA8), 0x0),
    (I2CReg::DSP(DSPReg::ReservedC5), 0x11),
    (I2CReg::DSP(DSPReg::ReservedC6), 0x51),
    (I2CReg::DSP(DSPReg::ReservedBF), 0x80),
    (I2CReg::DSP(DSPReg::ReservedC7), 0x10),
    (I2CReg::DSP(DSPReg::ReservedB6), 0x66),
    (I2CReg::DSP(DSPReg::ReservedB8), 0xa5),
    (I2CReg::DSP(DSPReg::ReservedB7), 0x64),
    (I2CReg::DSP(DSPReg::ReservedB9), 0x7c),
    (I2CReg::DSP(DSPReg::ReservedB3), 0xaf),
    (I2CReg::DSP(DSPReg::ReservedB4), 0x97),
    (I2CReg::DSP(DSPReg::ReservedB5), 0xff),
    (I2CReg::DSP(DSPReg::ReservedB0), 0xc5),
    (I2CReg::DSP(DSPReg::ReservedB1), 0x94),
    (I2CReg::DSP(DSPReg::ReservedB2), 0xf),
    (I2CReg::DSP(DSPReg::ReservedC4), 0x5c),
    (I2CReg::DSP(DSPReg::ReservedA6), 0x0),
    (I2CReg::DSP(DSPReg::ReservedA7), 0x20),
    (I2CReg::DSP(DSPReg::ReservedA7), 0xd8),
    (I2CReg::DSP(DSPReg::ReservedA7), 0x1b),
    (I2CReg::DSP(DSPReg::ReservedA7), 0x31),
    (I2CReg::DSP(DSPReg::ReservedA7), 0x0),
    (I2CReg::DSP(DSPReg::ReservedA7), 0x18),
    (I2CReg::DSP(DSPReg::ReservedA7), 0x20),
    (I2CReg::DSP(DSPReg::ReservedA7), 0xd8),
    (I2CReg::DSP(DSPReg::ReservedA7), 0x19),
    (I2CReg::DSP(DSPReg::ReservedA7), 0x31),
    (I2CReg::DSP(DSPReg::ReservedA7), 0x0),
    (I2CReg::DSP(DSPReg::ReservedA7), 0x18),
    (I2CReg::DSP(DSPReg::ReservedA7), 0x20),
    (I2CReg::DSP(DSPReg::ReservedA7), 0xd8),
    (I2CReg::DSP(DSPReg::ReservedA7), 0x19),
    (I2CReg::DSP(DSPReg::ReservedA7), 0x31),
    (I2CReg::DSP(DSPReg::ReservedA7), 0x0),
    (I2CReg::DSP(DSPReg::ReservedA7), 0x18),
    (I2CReg::DSP(DSPReg::Reserved7F), 0x0),
    (I2CReg::DSP(DSPReg::ReservedE5), 0x1f),
    (I2CReg::DSP(DSPReg::ReservedE1), 0x77),
    (I2CReg::DSP(DSPReg::ReservedDD), 0x7f),
    (I2CReg::DSP(DSPReg::Ctrl0), 0xe),
    (I2CReg::DSP(DSPReg::Reset), 0x4),
    (I2CReg::DSP(DSPReg::HSize8), 0xc8),
    (I2CReg::DSP(DSPReg::VSize8), 0x96),
    (I2CReg::DSP(DSPReg::Ctrl2), 0x3d),
    (I2CReg::DSP(DSPReg::HSize0), 0x90),
    (I2CReg::DSP(DSPReg::VSize0), 0x2c),
    (I2CReg::DSP(DSPReg::OffsetX0), 0x0),
    (I2CReg::DSP(DSPReg::OffsetY0), 0x0),
    (I2CReg::DSP(DSPReg::SizeOffset1), 0x88),
    (I2CReg::DSP(DSPReg::HSize2), 0x0),
    (I2CReg::DSP(DSPReg::CtrlI), 0x92),
    (I2CReg::DSP(DSPReg::OutWidth0), 0x50),
    (I2CReg::DSP(DSPReg::OutHeight0), 0x3c),
    (I2CReg::DSP(DSPReg::OutSize1), 0x0),
    (I2CReg::DSP(DSPReg::DVPSP), 0x4),
    (I2CReg::DSP(DSPReg::Reset), 0x0),
    (I2CReg::DSP(DSPReg::BypassDSP), 0x0),
    (I2CReg::DSP(DSPReg::ImageMode), 0x8),
    (I2CReg::DSP(DSPReg::ReservedD7), 0x3),
    (I2CReg::DSP(DSPReg::Reset), 0x0),
    (I2CReg::DSP(DSPReg::BypassDSP), 0x0),
];

const JPEG_INIT: [(I2CReg, u8); 187] = [
    (I2CReg::DSP(DSPReg::Reserved2C), 0xff),
    (I2CReg::DSP(DSPReg::Reserved2E), 0xdf),
    (I2CReg::Sensor(SensorReg::Reserved3C), 0x32),
    (I2CReg::Sensor(SensorReg::ClockRateCtrl), 0x0),
    (I2CReg::Sensor(SensorReg::Com2), 0x2),
    (I2CReg::Sensor(SensorReg::Reg4), 0x28),
    (I2CReg::Sensor(SensorReg::Com8), 0xe5),
    (I2CReg::Sensor(SensorReg::Com9), 0x48),
    (I2CReg::Sensor(SensorReg::Reserved2C), 0xc),
    (I2CReg::Sensor(SensorReg::Reserved33), 0x78),
    (I2CReg::Sensor(SensorReg::Reserved3A), 0x33),
    (I2CReg::Sensor(SensorReg::Reserved3B), 0xfb),
    (I2CReg::Sensor(SensorReg::Reserved3E), 0x0),
    (I2CReg::Sensor(SensorReg::Reserved43), 0x11),
    (I2CReg::Sensor(SensorReg::Reserved16), 0x10),
    (I2CReg::Sensor(SensorReg::Reserved39), 0x92),
    (I2CReg::Sensor(SensorReg::Reserved35), 0xda),
    (I2CReg::Sensor(SensorReg::Reserved22), 0x1a),
    (I2CReg::Sensor(SensorReg::Reserved37), 0xc3),
    (I2CReg::Sensor(SensorReg::Reserved23), 0x0),
    (I2CReg::Sensor(SensorReg::ARCom2), 0xc0),
    (I2CReg::Sensor(SensorReg::Reserved36), 0x1a),
    (I2CReg::Sensor(SensorReg::Reserved6), 0x88),
    (I2CReg::Sensor(SensorReg::Reserved7), 0xc0),
    (I2CReg::Sensor(SensorReg::Com4), 0x87),
    (I2CReg::Sensor(SensorReg::ReservedE), 0x41),
    (I2CReg::Sensor(SensorReg::Reserved4C), 0x0),
    (I2CReg::Sensor(SensorReg::Com19), 0x0),
    (I2CReg::Sensor(SensorReg::Reserved5B), 0x0),
    (I2CReg::Sensor(SensorReg::Reserved42), 0x3),
    (I2CReg::Sensor(SensorReg::Reserved4A), 0x81),
    (I2CReg::Sensor(SensorReg::Reserved21), 0x99),
    (I2CReg::Sensor(SensorReg::Reserved24), 0x40),
    (I2CReg::Sensor(SensorReg::Reserved25), 0x38),
    (I2CReg::Sensor(SensorReg::LargeStepThreshold), 0x82),
    (I2CReg::Sensor(SensorReg::Reserved5C), 0x0),
    (I2CReg::Sensor(SensorReg::Reserved63), 0x0),
    (I2CReg::Sensor(SensorReg::HistLowLevel), 0x70),
    (I2CReg::Sensor(SensorReg::HistHighLevel), 0x80),
    (I2CReg::Sensor(SensorReg::Reserved7C), 0x5),
    (I2CReg::Sensor(SensorReg::Reserved20), 0x80),
    (I2CReg::Sensor(SensorReg::Reserved28), 0x30),
    (I2CReg::Sensor(SensorReg::Reserved6C), 0x0),
    (I2CReg::Sensor(SensorReg::Reserved6D), 0x80),
    (I2CReg::Sensor(SensorReg::Reserved6E), 0x0),
    (I2CReg::Sensor(SensorReg::Reserved70), 0x2),
    (I2CReg::Sensor(SensorReg::Reserved71), 0x94),
    (I2CReg::Sensor(SensorReg::Reserved73), 0xc1),
    (I2CReg::Sensor(SensorReg::Com7), 0x40),
    (I2CReg::Sensor(SensorReg::HorizWindowStart), 0x11),
    (I2CReg::Sensor(SensorReg::HorizWindowEnd), 0x43),
    (I2CReg::Sensor(SensorReg::VertWindowStart), 0x0),
    (I2CReg::Sensor(SensorReg::VertWindowEnd), 0x4b),
    (I2CReg::Sensor(SensorReg::Reg32), 0x9),
    (I2CReg::Sensor(SensorReg::Reserved37), 0xc0),
    (I2CReg::Sensor(SensorReg::Band50), 0x60),
    (I2CReg::Sensor(SensorReg::Band60), 0xa8),
    (I2CReg::Sensor(SensorReg::Reserved6D), 0x0),
    (I2CReg::Sensor(SensorReg::Reserved3D), 0x38),
    (I2CReg::Sensor(SensorReg::FrameLength0), 0x3f),
    (I2CReg::Sensor(SensorReg::Band50), 0x60),
    (I2CReg::Sensor(SensorReg::Com3), 0x3c),
    (I2CReg::DSP(DSPReg::ReservedE5), 0x7f),
    (I2CReg::DSP(DSPReg::MCBist), 0xc0),
    (I2CReg::DSP(DSPReg::Reserved41), 0x24),
    (I2CReg::DSP(DSPReg::Reset), 0x14),
    (I2CReg::DSP(DSPReg::Reserved76), 0xff),
    (I2CReg::DSP(DSPReg::Reserved33), 0xa0),
    (I2CReg::DSP(DSPReg::Reserved42), 0x20),
    (I2CReg::DSP(DSPReg::Reserved43), 0x18),
    (I2CReg::DSP(DSPReg::Reserved4C), 0x0),
    (I2CReg::DSP(DSPReg::Ctrl3), 0xd5),
    (I2CReg::DSP(DSPReg::Reserved88), 0x3f),
    (I2CReg::DSP(DSPReg::ReservedD7), 0x3),
    (I2CReg::DSP(DSPReg::ReservedD9), 0x10),
    (I2CReg::DSP(DSPReg::DVPSP), 0x82),
    (I2CReg::DSP(DSPReg::ReservedC8), 0x8),
    (I2CReg::DSP(DSPReg::ReservedC9), 0x80),
    (I2CReg::DSP(DSPReg::AddrAccess), 0x0),
    (I2CReg::DSP(DSPReg::DataAccess), 0x0),
    (I2CReg::DSP(DSPReg::AddrAccess), 0x3),
    (I2CReg::DSP(DSPReg::DataAccess), 0x48),
    (I2CReg::DSP(DSPReg::DataAccess), 0x48),
    (I2CReg::DSP(DSPReg::AddrAccess), 0x8),
    (I2CReg::DSP(DSPReg::DataAccess), 0x20),
    (I2CReg::DSP(DSPReg::DataAccess), 0x10),
    (I2CReg::DSP(DSPReg::DataAccess), 0xe),
    (I2CReg::DSP(DSPReg::Reserved90), 0x0),
    (I2CReg::DSP(DSPReg::Reserved91), 0xe),
    (I2CReg::DSP(DSPReg::Reserved91), 0x1a),
    (I2CReg::DSP(DSPReg::Reserved91), 0x31),
    (I2CReg::DSP(DSPReg::Reserved91), 0x5a),
    (I2CReg::DSP(DSPReg::Reserved91), 0x69),
    (I2CReg::DSP(DSPReg::Reserved91), 0x75),
    (I2CReg::DSP(DSPReg::Reserved91), 0x7e),
    (I2CReg::DSP(DSPReg::Reserved91), 0x88),
    (I2CReg::DSP(DSPReg::Reserved91), 0x8f),
    (I2CReg::DSP(DSPReg::Reserved91), 0x96),
    (I2CReg::DSP(DSPReg::Reserved91), 0xa3),
    (I2CReg::DSP(DSPReg::Reserved91), 0xaf),
    (I2CReg::DSP(DSPReg::Reserved91), 0xc4),
    (I2CReg::DSP(DSPReg::Reserved91), 0xd7),
    (I2CReg::DSP(DSPReg::Reserved91), 0xe8),
    (I2CReg::DSP(DSPReg::Reserved91), 0x20),
    (I2CReg::DSP(DSPReg::Reserved92), 0x0),
    (I2CReg::DSP(DSPReg::Reserved93), 0x6),
    (I2CReg::DSP(DSPReg::Reserved93), 0xe3),
    (I2CReg::DSP(DSPReg::Reserved93), 0x5),
    (I2CReg::DSP(DSPReg::Reserved93), 0x5),
    (I2CReg::DSP(DSPReg::Reserved93), 0x0),
    (I2CReg::DSP(DSPReg::Reserved93), 0x4),
    (I2CReg::DSP(DSPReg::Reserved93), 0x0),
    (I2CReg::DSP(DSPReg::Reserved93), 0x0),
    (I2CReg::DSP(DSPReg::Reserved93), 0x0),
    (I2CReg::DSP(DSPReg::Reserved93), 0x0),
    (I2CReg::DSP(DSPReg::Reserved93), 0x0),
    (I2CReg::DSP(DSPReg::Reserved93), 0x0),
    (I2CReg::DSP(DSPReg::Reserved93), 0x0),
    (I2CReg::DSP(DSPReg::Reserved96), 0x0),
    (I2CReg::DSP(DSPReg::Reserved97), 0x8),
    (I2CReg::DSP(DSPReg::Reserved97), 0x19),
    (I2CReg::DSP(DSPReg::Reserved97), 0x2),
    (I2CReg::DSP(DSPReg::Reserved97), 0xc),
    (I2CReg::DSP(DSPReg::Reserved97), 0x24),
    (I2CReg::DSP(DSPReg::Reserved97), 0x30),
    (I2CReg::DSP(DSPReg::Reserved97), 0x28),
    (I2CReg::DSP(DSPReg::Reserved97), 0x26),
    (I2CReg::DSP(DSPReg::Reserved97), 0x2),
    (I2CReg::DSP(DSPReg::Reserved97), 0x98),
    (I2CReg::DSP(DSPReg::Reserved97), 0x80),
    (I2CReg::DSP(DSPReg::Reserved97), 0x0),
    (I2CReg::DSP(DSPReg::Reserved97), 0x0),
    (I2CReg::DSP(DSPReg::Ctrl1), 0xed),
    (I2CReg::DSP(DSPReg::ReservedA4), 0x0),
    (I2CReg::DSP(DSPReg::ReservedA8), 0x0),
    (I2CReg::DSP(DSPReg::ReservedC5), 0x11),
    (I2CReg::DSP(DSPReg::ReservedC6), 0x51),
    (I2CReg::DSP(DSPReg::ReservedBF), 0x80),
    (I2CReg::DSP(DSPReg::ReservedC7), 0x10),
    (I2CReg::DSP(DSPReg::ReservedB6), 0x66),
    (I2CReg::DSP(DSPReg::ReservedB8), 0xa5),
    (I2CReg::DSP(DSPReg::ReservedB7), 0x64),
    (I2CReg::DSP(DSPReg::ReservedB9), 0x7c),
    (I2CReg::DSP(DSPReg::ReservedB3), 0xaf),
    (I2CReg::DSP(DSPReg::ReservedB4), 0x97),
    (I2CReg::DSP(DSPReg::ReservedB5), 0xff),
    (I2CReg::DSP(DSPReg::ReservedB0), 0xc5),
    (I2CReg::DSP(DSPReg::ReservedB1), 0x94),
    (I2CReg::DSP(DSPReg::ReservedB2), 0xf),
    (I2CReg::DSP(DSPReg::ReservedC4), 0x5c),
    (I2CReg::DSP(DSPReg::HSize8), 0x64),
    (I2CReg::DSP(DSPReg::VSize8), 0x4b),
    (I2CReg::DSP(DSPReg::Size0), 0x0),
    (I2CReg::DSP(DSPReg::Ctrl2), 0x3d),
    (I2CReg::DSP(DSPReg::CtrlI), 0x0),
    (I2CReg::DSP(DSPReg::HSize0), 0xc8),
    (I2CReg::DSP(DSPReg::VSize0), 0x96),
    (I2CReg::DSP(DSPReg::OffsetX0), 0x0),
    (I2CReg::DSP(DSPReg::OffsetY0), 0x0),
    (I2CReg::DSP(DSPReg::SizeOffset1), 0x0),
    (I2CReg::DSP(DSPReg::OutWidth0), 0xc8),
    (I2CReg::DSP(DSPReg::OutHeight0), 0x96),
    (I2CReg::DSP(DSPReg::OutSize1), 0x0),
    (I2CReg::DSP(DSPReg::DVPSP), 0x0),
    (I2CReg::DSP(DSPReg::Ctrl1), 0xed),
    (I2CReg::DSP(DSPReg::Reserved7F), 0x0),
    (I2CReg::DSP(DSPReg::ImageMode), 0x0),
    (I2CReg::DSP(DSPReg::ReservedE5), 0x1f),
    (I2CReg::DSP(DSPReg::ReservedE1), 0x67),
    (I2CReg::DSP(DSPReg::Reset), 0x0),
    (I2CReg::DSP(DSPReg::ReservedDD), 0x7f),
    (I2CReg::DSP(DSPReg::BypassDSP), 0x0),
    (I2CReg::DSP(DSPReg::Reserved12), 0x40),
    (I2CReg::DSP(DSPReg::DVPSP), 0x4),
    (I2CReg::DSP(DSPReg::HSize8), 0x16),
    (I2CReg::DSP(DSPReg::VSize8), 0x12),
    (I2CReg::DSP(DSPReg::Size0), 0x0),
    (I2CReg::DSP(DSPReg::Ctrl2), 0x3d),
    (I2CReg::DSP(DSPReg::CtrlI), 0x0),
    (I2CReg::DSP(DSPReg::HSize0), 0x2c),
    (I2CReg::DSP(DSPReg::VSize0), 0x24),
    (I2CReg::DSP(DSPReg::OffsetX0), 0x0),
    (I2CReg::DSP(DSPReg::OffsetY0), 0x0),
    (I2CReg::DSP(DSPReg::SizeOffset1), 0x0),
    (I2CReg::DSP(DSPReg::OutWidth0), 0x2c),
    (I2CReg::DSP(DSPReg::OutHeight0), 0x24),
    (I2CReg::DSP(DSPReg::OutSize1), 0x0),
];

const YUV422: [(I2CReg, u8); 8] = [
    (I2CReg::DSP(DSPReg::BypassDSP), 0x0),
    (I2CReg::DSP(DSPReg::ImageMode), 0x10),
    (I2CReg::DSP(DSPReg::ReservedD7), 0x3),
    (I2CReg::DSP(DSPReg::ReservedDF), 0x0),
    (I2CReg::DSP(DSPReg::Reserved33), 0x80),
    (I2CReg::DSP(DSPReg::Reserved3C), 0x40),
    (I2CReg::DSP(DSPReg::ReservedE1), 0x77),
    (I2CReg::DSP(DSPReg::Reserved0), 0x0),
];

const JPEG: [(I2CReg, u8); 7] = [
    (I2CReg::DSP(DSPReg::Reset), 0x14),
    (I2CReg::DSP(DSPReg::ReservedE1), 0x77),
    (I2CReg::DSP(DSPReg::ReservedE5), 0x1f),
    (I2CReg::DSP(DSPReg::ReservedD7), 0x3),
    (I2CReg::DSP(DSPReg::ImageMode), 0x10),
    (I2CReg::DSP(DSPReg::Reset), 0x0),
    (I2CReg::Sensor(SensorReg::Reg4), 0x8),
];

const JPEG_160x120: [(I2CReg, u8); 37] = [
    (I2CReg::Sensor(SensorReg::Com7), 0x40),
    (I2CReg::Sensor(SensorReg::HorizWindowStart), 0x11),
    (I2CReg::Sensor(SensorReg::HorizWindowEnd), 0x43),
    (I2CReg::Sensor(SensorReg::VertWindowStart), 0x0),
    (I2CReg::Sensor(SensorReg::VertWindowEnd), 0x4b),
    (I2CReg::Sensor(SensorReg::Reg32), 0x9),
    (I2CReg::Sensor(SensorReg::Band50), 0xca),
    (I2CReg::Sensor(SensorReg::Band60), 0xa8),
    (I2CReg::Sensor(SensorReg::Reserved5A), 0x23),
    (I2CReg::Sensor(SensorReg::Reserved6D), 0x0),
    (I2CReg::Sensor(SensorReg::Reserved39), 0x12),
    (I2CReg::Sensor(SensorReg::Reserved35), 0xda),
    (I2CReg::Sensor(SensorReg::Reserved22), 0x1a),
    (I2CReg::Sensor(SensorReg::Reserved37), 0xc3),
    (I2CReg::Sensor(SensorReg::Reserved23), 0x0),
    (I2CReg::Sensor(SensorReg::ARCom2), 0xc0),
    (I2CReg::Sensor(SensorReg::Reserved36), 0x1a),
    (I2CReg::Sensor(SensorReg::Reserved6), 0x88),
    (I2CReg::Sensor(SensorReg::Reserved7), 0xc0),
    (I2CReg::Sensor(SensorReg::Com4), 0x87),
    (I2CReg::Sensor(SensorReg::ReservedE), 0x41),
    (I2CReg::Sensor(SensorReg::Reserved4C), 0x0),
    (I2CReg::DSP(DSPReg::Reset), 0x4),
    (I2CReg::DSP(DSPReg::HSize8), 0x64),
    (I2CReg::DSP(DSPReg::VSize8), 0x4b),
    (I2CReg::DSP(DSPReg::Ctrl2), 0x35),
    (I2CReg::DSP(DSPReg::CtrlI), 0x92),
    (I2CReg::DSP(DSPReg::HSize0), 0xc8),
    (I2CReg::DSP(DSPReg::VSize0), 0x96),
    (I2CReg::DSP(DSPReg::OffsetX0), 0x0),
    (I2CReg::DSP(DSPReg::OffsetY0), 0x0),
    (I2CReg::DSP(DSPReg::SizeOffset1), 0x0),
    (I2CReg::DSP(DSPReg::HSize2), 0x0),
    (I2CReg::DSP(DSPReg::OutWidth0), 0x28),
    (I2CReg::DSP(DSPReg::OutHeight0), 0x1e),
    (I2CReg::DSP(DSPReg::OutSize1), 0x0),
    (I2CReg::DSP(DSPReg::Reset), 0x0),
];

const JPEG_176x144: [(I2CReg, u8); 37] = [
    (I2CReg::Sensor(SensorReg::Com7), 0x40),
    (I2CReg::Sensor(SensorReg::HorizWindowStart), 0x11),
    (I2CReg::Sensor(SensorReg::HorizWindowEnd), 0x43),
    (I2CReg::Sensor(SensorReg::VertWindowStart), 0x0),
    (I2CReg::Sensor(SensorReg::VertWindowEnd), 0x4b),
    (I2CReg::Sensor(SensorReg::Reg32), 0x9),
    (I2CReg::Sensor(SensorReg::Band50), 0xca),
    (I2CReg::Sensor(SensorReg::Band60), 0xa8),
    (I2CReg::Sensor(SensorReg::Reserved5A), 0x23),
    (I2CReg::Sensor(SensorReg::Reserved6D), 0x0),
    (I2CReg::Sensor(SensorReg::Reserved39), 0x12),
    (I2CReg::Sensor(SensorReg::Reserved35), 0xda),
    (I2CReg::Sensor(SensorReg::Reserved22), 0x1a),
    (I2CReg::Sensor(SensorReg::Reserved37), 0xc3),
    (I2CReg::Sensor(SensorReg::Reserved23), 0x0),
    (I2CReg::Sensor(SensorReg::ARCom2), 0xc0),
    (I2CReg::Sensor(SensorReg::Reserved36), 0x1a),
    (I2CReg::Sensor(SensorReg::Reserved6), 0x88),
    (I2CReg::Sensor(SensorReg::Reserved7), 0xc0),
    (I2CReg::Sensor(SensorReg::Com4), 0x87),
    (I2CReg::Sensor(SensorReg::ReservedE), 0x41),
    (I2CReg::Sensor(SensorReg::Reserved4C), 0x0),
    (I2CReg::DSP(DSPReg::Reset), 0x4),
    (I2CReg::DSP(DSPReg::HSize8), 0x64),
    (I2CReg::DSP(DSPReg::VSize8), 0x4b),
    (I2CReg::DSP(DSPReg::Ctrl2), 0x35),
    (I2CReg::DSP(DSPReg::CtrlI), 0x92),
    (I2CReg::DSP(DSPReg::HSize0), 0xc8),
    (I2CReg::DSP(DSPReg::VSize0), 0x96),
    (I2CReg::DSP(DSPReg::OffsetX0), 0x0),
    (I2CReg::DSP(DSPReg::OffsetY0), 0x0),
    (I2CReg::DSP(DSPReg::SizeOffset1), 0x0),
    (I2CReg::DSP(DSPReg::HSize2), 0x0),
    (I2CReg::DSP(DSPReg::OutWidth0), 0x2c),
    (I2CReg::DSP(DSPReg::OutHeight0), 0x24),
    (I2CReg::DSP(DSPReg::OutSize1), 0x0),
    (I2CReg::DSP(DSPReg::Reset), 0x0),
];

const JPEG_320x240: [(I2CReg, u8); 37] = [
    (I2CReg::Sensor(SensorReg::Com7), 0x40),
    (I2CReg::Sensor(SensorReg::HorizWindowStart), 0x11),
    (I2CReg::Sensor(SensorReg::HorizWindowEnd), 0x43),
    (I2CReg::Sensor(SensorReg::VertWindowStart), 0x0),
    (I2CReg::Sensor(SensorReg::VertWindowEnd), 0x4b),
    (I2CReg::Sensor(SensorReg::Reg32), 0x9),
    (I2CReg::Sensor(SensorReg::Band50), 0xca),
    (I2CReg::Sensor(SensorReg::Band60), 0xa8),
    (I2CReg::Sensor(SensorReg::Reserved5A), 0x23),
    (I2CReg::Sensor(SensorReg::Reserved6D), 0x0),
    (I2CReg::Sensor(SensorReg::Reserved39), 0x12),
    (I2CReg::Sensor(SensorReg::Reserved35), 0xda),
    (I2CReg::Sensor(SensorReg::Reserved22), 0x1a),
    (I2CReg::Sensor(SensorReg::Reserved37), 0xc3),
    (I2CReg::Sensor(SensorReg::Reserved23), 0x0),
    (I2CReg::Sensor(SensorReg::ARCom2), 0xc0),
    (I2CReg::Sensor(SensorReg::Reserved36), 0x1a),
    (I2CReg::Sensor(SensorReg::Reserved6), 0x88),
    (I2CReg::Sensor(SensorReg::Reserved7), 0xc0),
    (I2CReg::Sensor(SensorReg::Com4), 0x87),
    (I2CReg::Sensor(SensorReg::ReservedE), 0x41),
    (I2CReg::Sensor(SensorReg::Reserved4C), 0x0),
    (I2CReg::DSP(DSPReg::Reset), 0x4),
    (I2CReg::DSP(DSPReg::HSize8), 0x64),
    (I2CReg::DSP(DSPReg::VSize8), 0x4b),
    (I2CReg::DSP(DSPReg::Ctrl2), 0x35),
    (I2CReg::DSP(DSPReg::CtrlI), 0x89),
    (I2CReg::DSP(DSPReg::HSize0), 0xc8),
    (I2CReg::DSP(DSPReg::VSize0), 0x96),
    (I2CReg::DSP(DSPReg::OffsetX0), 0x0),
    (I2CReg::DSP(DSPReg::OffsetY0), 0x0),
    (I2CReg::DSP(DSPReg::SizeOffset1), 0x0),
    (I2CReg::DSP(DSPReg::HSize2), 0x0),
    (I2CReg::DSP(DSPReg::OutWidth0), 0x50),
    (I2CReg::DSP(DSPReg::OutHeight0), 0x3c),
    (I2CReg::DSP(DSPReg::OutSize1), 0x0),
    (I2CReg::DSP(DSPReg::Reset), 0x0),
];

const JPEG_352x288: [(I2CReg, u8); 37] = [
    (I2CReg::Sensor(SensorReg::Com7), 0x40),
    (I2CReg::Sensor(SensorReg::HorizWindowStart), 0x11),
    (I2CReg::Sensor(SensorReg::HorizWindowEnd), 0x43),
    (I2CReg::Sensor(SensorReg::VertWindowStart), 0x0),
    (I2CReg::Sensor(SensorReg::VertWindowEnd), 0x4b),
    (I2CReg::Sensor(SensorReg::Reg32), 0x9),
    (I2CReg::Sensor(SensorReg::Band50), 0xca),
    (I2CReg::Sensor(SensorReg::Band60), 0xa8),
    (I2CReg::Sensor(SensorReg::Reserved5A), 0x23),
    (I2CReg::Sensor(SensorReg::Reserved6D), 0x0),
    (I2CReg::Sensor(SensorReg::Reserved39), 0x12),
    (I2CReg::Sensor(SensorReg::Reserved35), 0xda),
    (I2CReg::Sensor(SensorReg::Reserved22), 0x1a),
    (I2CReg::Sensor(SensorReg::Reserved37), 0xc3),
    (I2CReg::Sensor(SensorReg::Reserved23), 0x0),
    (I2CReg::Sensor(SensorReg::ARCom2), 0xc0),
    (I2CReg::Sensor(SensorReg::Reserved36), 0x1a),
    (I2CReg::Sensor(SensorReg::Reserved6), 0x88),
    (I2CReg::Sensor(SensorReg::Reserved7), 0xc0),
    (I2CReg::Sensor(SensorReg::Com4), 0x87),
    (I2CReg::Sensor(SensorReg::ReservedE), 0x41),
    (I2CReg::Sensor(SensorReg::Reserved4C), 0x0),
    (I2CReg::DSP(DSPReg::Reset), 0x4),
    (I2CReg::DSP(DSPReg::HSize8), 0x64),
    (I2CReg::DSP(DSPReg::VSize8), 0x4b),
    (I2CReg::DSP(DSPReg::Ctrl2), 0x35),
    (I2CReg::DSP(DSPReg::CtrlI), 0x89),
    (I2CReg::DSP(DSPReg::HSize0), 0xc8),
    (I2CReg::DSP(DSPReg::VSize0), 0x96),
    (I2CReg::DSP(DSPReg::OffsetX0), 0x0),
    (I2CReg::DSP(DSPReg::OffsetY0), 0x0),
    (I2CReg::DSP(DSPReg::SizeOffset1), 0x0),
    (I2CReg::DSP(DSPReg::HSize2), 0x0),
    (I2CReg::DSP(DSPReg::OutWidth0), 0x58),
    (I2CReg::DSP(DSPReg::OutHeight0), 0x48),
    (I2CReg::DSP(DSPReg::OutSize1), 0x0),
    (I2CReg::DSP(DSPReg::Reset), 0x0),
];

const JPEG_640x480: [(I2CReg, u8); 38] = [
    (I2CReg::Sensor(SensorReg::ClockRateCtrl), 0x1),
    (I2CReg::Sensor(SensorReg::Com7), 0x0),
    (I2CReg::Sensor(SensorReg::HorizWindowStart), 0x11),
    (I2CReg::Sensor(SensorReg::HorizWindowEnd), 0x75),
    (I2CReg::Sensor(SensorReg::Reg32), 0x36),
    (I2CReg::Sensor(SensorReg::VertWindowStart), 0x1),
    (I2CReg::Sensor(SensorReg::VertWindowEnd), 0x97),
    (I2CReg::Sensor(SensorReg::Com1), 0xf),
    (I2CReg::Sensor(SensorReg::Reserved37), 0x40),
    (I2CReg::Sensor(SensorReg::Band50), 0xbb),
    (I2CReg::Sensor(SensorReg::Band60), 0x9c),
    (I2CReg::Sensor(SensorReg::Reserved5A), 0x57),
    (I2CReg::Sensor(SensorReg::Reserved6D), 0x80),
    (I2CReg::Sensor(SensorReg::Reserved3D), 0x34),
    (I2CReg::Sensor(SensorReg::Reserved39), 0x2),
    (I2CReg::Sensor(SensorReg::Reserved35), 0x88),
    (I2CReg::Sensor(SensorReg::Reserved22), 0xa),
    (I2CReg::Sensor(SensorReg::Reserved37), 0x40),
    (I2CReg::Sensor(SensorReg::ARCom2), 0xa0),
    (I2CReg::Sensor(SensorReg::Reserved6), 0x2),
    (I2CReg::Sensor(SensorReg::Com4), 0xb7),
    (I2CReg::Sensor(SensorReg::ReservedE), 0x1),
    (I2CReg::DSP(DSPReg::Reset), 0x4),
    (I2CReg::DSP(DSPReg::HSize8), 0xc8),
    (I2CReg::DSP(DSPReg::VSize8), 0x96),
    (I2CReg::DSP(DSPReg::Ctrl2), 0x3d),
    (I2CReg::DSP(DSPReg::CtrlI), 0x89),
    (I2CReg::DSP(DSPReg::HSize0), 0x90),
    (I2CReg::DSP(DSPReg::VSize0), 0x2c),
    (I2CReg::DSP(DSPReg::OffsetX0), 0x0),
    (I2CReg::DSP(DSPReg::OffsetY0), 0x0),
    (I2CReg::DSP(DSPReg::SizeOffset1), 0x88),
    (I2CReg::DSP(DSPReg::HSize2), 0x0),
    (I2CReg::DSP(DSPReg::OutWidth0), 0xa0),
    (I2CReg::DSP(DSPReg::OutHeight0), 0x78),
    (I2CReg::DSP(DSPReg::OutSize1), 0x0),
    (I2CReg::DSP(DSPReg::DVPSP), 0x4),
    (I2CReg::DSP(DSPReg::Reset), 0x0),
];

const JPEG_800x600: [(I2CReg, u8); 38] = [
    (I2CReg::Sensor(SensorReg::ClockRateCtrl), 0x1),
    (I2CReg::Sensor(SensorReg::Com7), 0x0),
    (I2CReg::Sensor(SensorReg::HorizWindowStart), 0x11),
    (I2CReg::Sensor(SensorReg::HorizWindowEnd), 0x75),
    (I2CReg::Sensor(SensorReg::Reg32), 0x36),
    (I2CReg::Sensor(SensorReg::VertWindowStart), 0x1),
    (I2CReg::Sensor(SensorReg::VertWindowEnd), 0x97),
    (I2CReg::Sensor(SensorReg::Com1), 0xf),
    (I2CReg::Sensor(SensorReg::Reserved37), 0x40),
    (I2CReg::Sensor(SensorReg::Band50), 0xbb),
    (I2CReg::Sensor(SensorReg::Band60), 0x9c),
    (I2CReg::Sensor(SensorReg::Reserved5A), 0x57),
    (I2CReg::Sensor(SensorReg::Reserved6D), 0x80),
    (I2CReg::Sensor(SensorReg::Reserved3D), 0x34),
    (I2CReg::Sensor(SensorReg::Reserved39), 0x2),
    (I2CReg::Sensor(SensorReg::Reserved35), 0x88),
    (I2CReg::Sensor(SensorReg::Reserved22), 0xa),
    (I2CReg::Sensor(SensorReg::Reserved37), 0x40),
    (I2CReg::Sensor(SensorReg::ARCom2), 0xa0),
    (I2CReg::Sensor(SensorReg::Reserved6), 0x2),
    (I2CReg::Sensor(SensorReg::Com4), 0xb7),
    (I2CReg::Sensor(SensorReg::ReservedE), 0x1),
    (I2CReg::DSP(DSPReg::Reset), 0x4),
    (I2CReg::DSP(DSPReg::HSize8), 0xc8),
    (I2CReg::DSP(DSPReg::VSize8), 0x96),
    (I2CReg::DSP(DSPReg::Ctrl2), 0x35),
    (I2CReg::DSP(DSPReg::CtrlI), 0x89),
    (I2CReg::DSP(DSPReg::HSize0), 0x90),
    (I2CReg::DSP(DSPReg::VSize0), 0x2c),
    (I2CReg::DSP(DSPReg::OffsetX0), 0x0),
    (I2CReg::DSP(DSPReg::OffsetY0), 0x0),
    (I2CReg::DSP(DSPReg::SizeOffset1), 0x88),
    (I2CReg::DSP(DSPReg::HSize2), 0x0),
    (I2CReg::DSP(DSPReg::OutWidth0), 0xc8),
    (I2CReg::DSP(DSPReg::OutHeight0), 0x96),
    (I2CReg::DSP(DSPReg::OutSize1), 0x0),
    (I2CReg::DSP(DSPReg::DVPSP), 0x2),
    (I2CReg::DSP(DSPReg::Reset), 0x0),
];

const JPEG_1024x768: [(I2CReg, u8); 36] = [
    (I2CReg::Sensor(SensorReg::ClockRateCtrl), 0x1),
    (I2CReg::Sensor(SensorReg::Com7), 0x0),
    (I2CReg::Sensor(SensorReg::HorizWindowStart), 0x11),
    (I2CReg::Sensor(SensorReg::HorizWindowEnd), 0x75),
    (I2CReg::Sensor(SensorReg::Reg32), 0x36),
    (I2CReg::Sensor(SensorReg::VertWindowStart), 0x1),
    (I2CReg::Sensor(SensorReg::VertWindowEnd), 0x97),
    (I2CReg::Sensor(SensorReg::Com1), 0xf),
    (I2CReg::Sensor(SensorReg::Reserved37), 0x40),
    (I2CReg::Sensor(SensorReg::Band50), 0xbb),
    (I2CReg::Sensor(SensorReg::Band60), 0x9c),
    (I2CReg::Sensor(SensorReg::Reserved5A), 0x57),
    (I2CReg::Sensor(SensorReg::Reserved6D), 0x80),
    (I2CReg::Sensor(SensorReg::Reserved3D), 0x34),
    (I2CReg::Sensor(SensorReg::Reserved39), 0x2),
    (I2CReg::Sensor(SensorReg::Reserved35), 0x88),
    (I2CReg::Sensor(SensorReg::Reserved22), 0xa),
    (I2CReg::Sensor(SensorReg::Reserved37), 0x40),
    (I2CReg::Sensor(SensorReg::ARCom2), 0xa0),
    (I2CReg::Sensor(SensorReg::Reserved6), 0x2),
    (I2CReg::Sensor(SensorReg::Com4), 0xb7),
    (I2CReg::Sensor(SensorReg::ReservedE), 0x1),
    (I2CReg::DSP(DSPReg::HSize8), 0xc8),
    (I2CReg::DSP(DSPReg::VSize8), 0x96),
    (I2CReg::DSP(DSPReg::Size0), 0x0),
    (I2CReg::DSP(DSPReg::Ctrl2), 0x3d),
    (I2CReg::DSP(DSPReg::CtrlI), 0x0),
    (I2CReg::DSP(DSPReg::HSize0), 0x90),
    (I2CReg::DSP(DSPReg::VSize0), 0x2c),
    (I2CReg::DSP(DSPReg::OffsetX0), 0x0),
    (I2CReg::DSP(DSPReg::OffsetY0), 0x0),
    (I2CReg::DSP(DSPReg::SizeOffset1), 0x88),
    (I2CReg::DSP(DSPReg::OutWidth0), 0x0),
    (I2CReg::DSP(DSPReg::OutHeight0), 0xc0),
    (I2CReg::DSP(DSPReg::OutSize1), 0x1),
    (I2CReg::DSP(DSPReg::DVPSP), 0x2),
];

const JPEG_1280x1024: [(I2CReg, u8); 38] = [
    (I2CReg::Sensor(SensorReg::ClockRateCtrl), 0x1),
    (I2CReg::Sensor(SensorReg::Com7), 0x0),
    (I2CReg::Sensor(SensorReg::HorizWindowStart), 0x11),
    (I2CReg::Sensor(SensorReg::HorizWindowEnd), 0x75),
    (I2CReg::Sensor(SensorReg::Reg32), 0x36),
    (I2CReg::Sensor(SensorReg::VertWindowStart), 0x1),
    (I2CReg::Sensor(SensorReg::VertWindowEnd), 0x97),
    (I2CReg::Sensor(SensorReg::Com1), 0xf),
    (I2CReg::Sensor(SensorReg::Reserved37), 0x40),
    (I2CReg::Sensor(SensorReg::Band50), 0xbb),
    (I2CReg::Sensor(SensorReg::Band60), 0x9c),
    (I2CReg::Sensor(SensorReg::Reserved5A), 0x57),
    (I2CReg::Sensor(SensorReg::Reserved6D), 0x80),
    (I2CReg::Sensor(SensorReg::Reserved3D), 0x34),
    (I2CReg::Sensor(SensorReg::Reserved39), 0x2),
    (I2CReg::Sensor(SensorReg::Reserved35), 0x88),
    (I2CReg::Sensor(SensorReg::Reserved22), 0xa),
    (I2CReg::Sensor(SensorReg::Reserved37), 0x40),
    (I2CReg::Sensor(SensorReg::ARCom2), 0xa0),
    (I2CReg::Sensor(SensorReg::Reserved6), 0x2),
    (I2CReg::Sensor(SensorReg::Com4), 0xb7),
    (I2CReg::Sensor(SensorReg::ReservedE), 0x1),
    (I2CReg::DSP(DSPReg::Reset), 0x4),
    (I2CReg::DSP(DSPReg::HSize8), 0xc8),
    (I2CReg::DSP(DSPReg::VSize8), 0x96),
    (I2CReg::DSP(DSPReg::Ctrl2), 0x3d),
    (I2CReg::DSP(DSPReg::CtrlI), 0x0),
    (I2CReg::DSP(DSPReg::HSize0), 0x90),
    (I2CReg::DSP(DSPReg::VSize0), 0x2c),
    (I2CReg::DSP(DSPReg::OffsetX0), 0x0),
    (I2CReg::DSP(DSPReg::OffsetY0), 0x0),
    (I2CReg::DSP(DSPReg::SizeOffset1), 0x88),
    (I2CReg::DSP(DSPReg::HSize2), 0x0),
    (I2CReg::DSP(DSPReg::OutWidth0), 0x40),
    (I2CReg::DSP(DSPReg::OutHeight0), 0xf0),
    (I2CReg::DSP(DSPReg::OutSize1), 0x1),
    (I2CReg::DSP(DSPReg::DVPSP), 0x2),
    (I2CReg::DSP(DSPReg::Reset), 0x0),
];

const JPEG_1600x1200: [(I2CReg, u8); 38] = [
    (I2CReg::Sensor(SensorReg::ClockRateCtrl), 0x1),
    (I2CReg::Sensor(SensorReg::Com7), 0x0),
    (I2CReg::Sensor(SensorReg::HorizWindowStart), 0x11),
    (I2CReg::Sensor(SensorReg::HorizWindowEnd), 0x75),
    (I2CReg::Sensor(SensorReg::Reg32), 0x36),
    (I2CReg::Sensor(SensorReg::VertWindowStart), 0x1),
    (I2CReg::Sensor(SensorReg::VertWindowEnd), 0x97),
    (I2CReg::Sensor(SensorReg::Com1), 0xf),
    (I2CReg::Sensor(SensorReg::Reserved37), 0x40),
    (I2CReg::Sensor(SensorReg::Band50), 0xbb),
    (I2CReg::Sensor(SensorReg::Band60), 0x9c),
    (I2CReg::Sensor(SensorReg::Reserved5A), 0x57),
    (I2CReg::Sensor(SensorReg::Reserved6D), 0x80),
    (I2CReg::Sensor(SensorReg::Reserved3D), 0x34),
    (I2CReg::Sensor(SensorReg::Reserved39), 0x2),
    (I2CReg::Sensor(SensorReg::Reserved35), 0x88),
    (I2CReg::Sensor(SensorReg::Reserved22), 0xa),
    (I2CReg::Sensor(SensorReg::Reserved37), 0x40),
    (I2CReg::Sensor(SensorReg::ARCom2), 0xa0),
    (I2CReg::Sensor(SensorReg::Reserved6), 0x2),
    (I2CReg::Sensor(SensorReg::Com4), 0xb7),
    (I2CReg::Sensor(SensorReg::ReservedE), 0x1),
    (I2CReg::DSP(DSPReg::Reset), 0x4),
    (I2CReg::DSP(DSPReg::HSize8), 0xc8),
    (I2CReg::DSP(DSPReg::VSize8), 0x96),
    (I2CReg::DSP(DSPReg::Ctrl2), 0x3d),
    (I2CReg::DSP(DSPReg::CtrlI), 0x0),
    (I2CReg::DSP(DSPReg::HSize0), 0x90),
    (I2CReg::DSP(DSPReg::VSize0), 0x2c),
    (I2CReg::DSP(DSPReg::OffsetX0), 0x0),
    (I2CReg::DSP(DSPReg::OffsetY0), 0x0),
    (I2CReg::DSP(DSPReg::SizeOffset1), 0x88),
    (I2CReg::DSP(DSPReg::HSize2), 0x0),
    (I2CReg::DSP(DSPReg::OutWidth0), 0x90),
    (I2CReg::DSP(DSPReg::OutHeight0), 0x2c),
    (I2CReg::DSP(DSPReg::OutSize1), 0x5),
    (I2CReg::DSP(DSPReg::DVPSP), 0x2),
    (I2CReg::DSP(DSPReg::Reset), 0x0),
];

const SVGA: [(I2CReg, u8); 173] = [
    (I2CReg::Sensor(SensorReg::Com7), 0x80),
    (I2CReg::DSP(DSPReg::Reserved2C), 0xff),
    (I2CReg::DSP(DSPReg::Reserved2E), 0xdf),
    (I2CReg::Sensor(SensorReg::Reserved3C), 0x32),
    (I2CReg::Sensor(SensorReg::ClockRateCtrl), 0x1),
    (I2CReg::Sensor(SensorReg::Com2), 0x2),
    (I2CReg::Sensor(SensorReg::Reg4), 0x28),
    (I2CReg::Sensor(SensorReg::Com8), 0xe5),
    (I2CReg::Sensor(SensorReg::Com9), 0x48),
    (I2CReg::Sensor(SensorReg::Reserved2C), 0xc),
    (I2CReg::Sensor(SensorReg::Reserved33), 0x78),
    (I2CReg::Sensor(SensorReg::Reserved3A), 0x33),
    (I2CReg::Sensor(SensorReg::Reserved3B), 0xfb),
    (I2CReg::Sensor(SensorReg::Reserved3E), 0x0),
    (I2CReg::Sensor(SensorReg::Reserved43), 0x11),
    (I2CReg::Sensor(SensorReg::Reserved16), 0x10),
    (I2CReg::Sensor(SensorReg::Reserved39), 0x92),
    (I2CReg::Sensor(SensorReg::Reserved35), 0xda),
    (I2CReg::Sensor(SensorReg::Reserved22), 0x1a),
    (I2CReg::Sensor(SensorReg::Reserved37), 0xc3),
    (I2CReg::Sensor(SensorReg::Reserved23), 0x0),
    (I2CReg::Sensor(SensorReg::ARCom2), 0xc0),
    (I2CReg::Sensor(SensorReg::Reserved36), 0x1a),
    (I2CReg::Sensor(SensorReg::Reserved6), 0x88),
    (I2CReg::Sensor(SensorReg::Reserved7), 0xc0),
    (I2CReg::Sensor(SensorReg::Com4), 0x87),
    (I2CReg::Sensor(SensorReg::ReservedE), 0x41),
    (I2CReg::Sensor(SensorReg::Reserved4C), 0x0),
    (I2CReg::Sensor(SensorReg::Com19), 0x0),
    (I2CReg::Sensor(SensorReg::Reserved5B), 0x0),
    (I2CReg::Sensor(SensorReg::Reserved42), 0x3),
    (I2CReg::Sensor(SensorReg::Reserved4A), 0x81),
    (I2CReg::Sensor(SensorReg::Reserved21), 0x99),
    (I2CReg::Sensor(SensorReg::Reserved24), 0x40),
    (I2CReg::Sensor(SensorReg::Reserved25), 0x38),
    (I2CReg::Sensor(SensorReg::LargeStepThreshold), 0x82),
    (I2CReg::Sensor(SensorReg::Reserved5C), 0x0),
    (I2CReg::Sensor(SensorReg::Reserved63), 0x0),
    (I2CReg::Sensor(SensorReg::HistLowLevel), 0x70),
    (I2CReg::Sensor(SensorReg::HistHighLevel), 0x80),
    (I2CReg::Sensor(SensorReg::Reserved7C), 0x5),
    (I2CReg::Sensor(SensorReg::Reserved20), 0x80),
    (I2CReg::Sensor(SensorReg::Reserved28), 0x30),
    (I2CReg::Sensor(SensorReg::Reserved6C), 0x0),
    (I2CReg::Sensor(SensorReg::Reserved6D), 0x80),
    (I2CReg::Sensor(SensorReg::Reserved6E), 0x0),
    (I2CReg::Sensor(SensorReg::Reserved70), 0x2),
    (I2CReg::Sensor(SensorReg::Reserved71), 0x94),
    (I2CReg::Sensor(SensorReg::Reserved73), 0xc1),
    (I2CReg::Sensor(SensorReg::Com7), 0x40),
    (I2CReg::Sensor(SensorReg::HorizWindowStart), 0x11),
    (I2CReg::Sensor(SensorReg::HorizWindowEnd), 0x43),
    (I2CReg::Sensor(SensorReg::VertWindowStart), 0x0),
    (I2CReg::Sensor(SensorReg::VertWindowEnd), 0x4b),
    (I2CReg::Sensor(SensorReg::Reg32), 0x9),
    (I2CReg::Sensor(SensorReg::Reserved37), 0xc0),
    (I2CReg::Sensor(SensorReg::Band50), 0x60),
    (I2CReg::Sensor(SensorReg::Band60), 0xa8),
    (I2CReg::Sensor(SensorReg::Reserved6D), 0x0),
    (I2CReg::Sensor(SensorReg::Reserved3D), 0x38),
    (I2CReg::Sensor(SensorReg::FrameLength0), 0x3f),
    (I2CReg::Sensor(SensorReg::Band50), 0x60),
    (I2CReg::Sensor(SensorReg::Com3), 0x3c),
    (I2CReg::DSP(DSPReg::ReservedE5), 0x7f),
    (I2CReg::DSP(DSPReg::MCBist), 0xc0),
    (I2CReg::DSP(DSPReg::Reserved41), 0x24),
    (I2CReg::DSP(DSPReg::Reset), 0x14),
    (I2CReg::DSP(DSPReg::Reserved76), 0xff),
    (I2CReg::DSP(DSPReg::Reserved33), 0xa0),
    (I2CReg::DSP(DSPReg::Reserved42), 0x20),
    (I2CReg::DSP(DSPReg::Reserved43), 0x18),
    (I2CReg::DSP(DSPReg::Reserved4C), 0x0),
    (I2CReg::DSP(DSPReg::Ctrl3), 0xd5),
    (I2CReg::DSP(DSPReg::Reserved88), 0x3f),
    (I2CReg::DSP(DSPReg::ReservedD7), 0x3),
    (I2CReg::DSP(DSPReg::ReservedD9), 0x10),
    (I2CReg::DSP(DSPReg::DVPSP), 0x82),
    (I2CReg::DSP(DSPReg::ReservedC8), 0x8),
    (I2CReg::DSP(DSPReg::ReservedC9), 0x80),
    (I2CReg::DSP(DSPReg::AddrAccess), 0x0),
    (I2CReg::DSP(DSPReg::DataAccess), 0x0),
    (I2CReg::DSP(DSPReg::AddrAccess), 0x3),
    (I2CReg::DSP(DSPReg::DataAccess), 0x48),
    (I2CReg::DSP(DSPReg::DataAccess), 0x48),
    (I2CReg::DSP(DSPReg::AddrAccess), 0x8),
    (I2CReg::DSP(DSPReg::DataAccess), 0x20),
    (I2CReg::DSP(DSPReg::DataAccess), 0x10),
    (I2CReg::DSP(DSPReg::DataAccess), 0xe),
    (I2CReg::DSP(DSPReg::Reserved90), 0x0),
    (I2CReg::DSP(DSPReg::Reserved91), 0xe),
    (I2CReg::DSP(DSPReg::Reserved91), 0x1a),
    (I2CReg::DSP(DSPReg::Reserved91), 0x31),
    (I2CReg::DSP(DSPReg::Reserved91), 0x5a),
    (I2CReg::DSP(DSPReg::Reserved91), 0x69),
    (I2CReg::DSP(DSPReg::Reserved91), 0x75),
    (I2CReg::DSP(DSPReg::Reserved91), 0x7e),
    (I2CReg::DSP(DSPReg::Reserved91), 0x88),
    (I2CReg::DSP(DSPReg::Reserved91), 0x8f),
    (I2CReg::DSP(DSPReg::Reserved91), 0x96),
    (I2CReg::DSP(DSPReg::Reserved91), 0xa3),
    (I2CReg::DSP(DSPReg::Reserved91), 0xaf),
    (I2CReg::DSP(DSPReg::Reserved91), 0xc4),
    (I2CReg::DSP(DSPReg::Reserved91), 0xd7),
    (I2CReg::DSP(DSPReg::Reserved91), 0xe8),
    (I2CReg::DSP(DSPReg::Reserved91), 0x20),
    (I2CReg::DSP(DSPReg::Reserved92), 0x0),
    (I2CReg::DSP(DSPReg::Reserved93), 0x6),
    (I2CReg::DSP(DSPReg::Reserved93), 0xe3),
    (I2CReg::DSP(DSPReg::Reserved93), 0x5),
    (I2CReg::DSP(DSPReg::Reserved93), 0x5),
    (I2CReg::DSP(DSPReg::Reserved93), 0x0),
    (I2CReg::DSP(DSPReg::Reserved93), 0x4),
    (I2CReg::DSP(DSPReg::Reserved93), 0x0),
    (I2CReg::DSP(DSPReg::Reserved93), 0x0),
    (I2CReg::DSP(DSPReg::Reserved93), 0x0),
    (I2CReg::DSP(DSPReg::Reserved93), 0x0),
    (I2CReg::DSP(DSPReg::Reserved93), 0x0),
    (I2CReg::DSP(DSPReg::Reserved93), 0x0),
    (I2CReg::DSP(DSPReg::Reserved93), 0x0),
    (I2CReg::DSP(DSPReg::Reserved96), 0x0),
    (I2CReg::DSP(DSPReg::Reserved97), 0x8),
    (I2CReg::DSP(DSPReg::Reserved97), 0x19),
    (I2CReg::DSP(DSPReg::Reserved97), 0x2),
    (I2CReg::DSP(DSPReg::Reserved97), 0xc),
    (I2CReg::DSP(DSPReg::Reserved97), 0x24),
    (I2CReg::DSP(DSPReg::Reserved97), 0x30),
    (I2CReg::DSP(DSPReg::Reserved97), 0x28),
    (I2CReg::DSP(DSPReg::Reserved97), 0x26),
    (I2CReg::DSP(DSPReg::Reserved97), 0x2),
    (I2CReg::DSP(DSPReg::Reserved97), 0x98),
    (I2CReg::DSP(DSPReg::Reserved97), 0x80),
    (I2CReg::DSP(DSPReg::Reserved97), 0x0),
    (I2CReg::DSP(DSPReg::Reserved97), 0x0),
    (I2CReg::DSP(DSPReg::Ctrl1), 0xed),
    (I2CReg::DSP(DSPReg::ReservedA4), 0x0),
    (I2CReg::DSP(DSPReg::ReservedA8), 0x0),
    (I2CReg::DSP(DSPReg::ReservedC5), 0x11),
    (I2CReg::DSP(DSPReg::ReservedC6), 0x51),
    (I2CReg::DSP(DSPReg::ReservedBF), 0x80),
    (I2CReg::DSP(DSPReg::ReservedC7), 0x10),
    (I2CReg::DSP(DSPReg::ReservedB6), 0x66),
    (I2CReg::DSP(DSPReg::ReservedB8), 0xa5),
    (I2CReg::DSP(DSPReg::ReservedB7), 0x64),
    (I2CReg::DSP(DSPReg::ReservedB9), 0x7c),
    (I2CReg::DSP(DSPReg::ReservedB3), 0xaf),
    (I2CReg::DSP(DSPReg::ReservedB4), 0x97),
    (I2CReg::DSP(DSPReg::ReservedB5), 0xff),
    (I2CReg::DSP(DSPReg::ReservedB0), 0xc5),
    (I2CReg::DSP(DSPReg::ReservedB1), 0x94),
    (I2CReg::DSP(DSPReg::ReservedB2), 0xf),
    (I2CReg::DSP(DSPReg::ReservedC4), 0x5c),
    (I2CReg::DSP(DSPReg::HSize8), 0x64),
    (I2CReg::DSP(DSPReg::VSize8), 0x4b),
    (I2CReg::DSP(DSPReg::Size0), 0x0),
    (I2CReg::DSP(DSPReg::Ctrl2), 0x3d),
    (I2CReg::DSP(DSPReg::CtrlI), 0x0),
    (I2CReg::DSP(DSPReg::HSize0), 0xc8),
    (I2CReg::DSP(DSPReg::VSize0), 0x96),
    (I2CReg::DSP(DSPReg::OffsetX0), 0x0),
    (I2CReg::DSP(DSPReg::OffsetY0), 0x0),
    (I2CReg::DSP(DSPReg::SizeOffset1), 0x0),
    (I2CReg::DSP(DSPReg::OutWidth0), 0xc8),
    (I2CReg::DSP(DSPReg::OutHeight0), 0x96),
    (I2CReg::DSP(DSPReg::OutSize1), 0x0),
    (I2CReg::DSP(DSPReg::DVPSP), 0x82),
    (I2CReg::DSP(DSPReg::Ctrl1), 0xed),
    (I2CReg::DSP(DSPReg::Reserved7F), 0x0),
    (I2CReg::DSP(DSPReg::ImageMode), 0x0),
    (I2CReg::DSP(DSPReg::ReservedE5), 0x1f),
    (I2CReg::DSP(DSPReg::ReservedE1), 0x67),
    (I2CReg::DSP(DSPReg::Reset), 0x0),
    (I2CReg::DSP(DSPReg::ReservedDD), 0x7f),
    (I2CReg::DSP(DSPReg::BypassDSP), 0x0),
];

const SPI_QUEUE: u32 = 0;
const SYNC_TAG: u16 = 0;

const I2C_QUEUE: u32 = 1;
const I2C_TAG: u16 = 0;

const SPI_MAX_READ_SIZE: usize = 1024;
const I2C_MAX_READ_SIZE: usize = 32;

const I2C_ADDR: u8 = 0x60 >> 1;

fn spi_send_read_cmd(reg: CamReg) -> u8 {
    let mut reg = [reg as u8];
    send(SPI_QUEUE, SYNC_TAG, &mut reg, 1, 1).unwrap();
    reg[0]
}

fn spi_send_burst_read_cmd(reg: CamReg, buffer: &mut [u8]) {
    let mut offset = 0;
    while offset < buffer.len() {
        let reply_len = SPI_MAX_READ_SIZE.min(buffer.len() - offset);
        buffer[offset] = reg as u8;
        send(SPI_QUEUE, SYNC_TAG, &mut buffer[offset..offset + reply_len], 1, reply_len).unwrap();
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

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum JPEGFormat {
    Size160x120,
    Size176x144,
    Size320x240,
    Size352x288,
    Size640x480,
    Size800x600,
    Size1024x768,
    Size1280x1024,
    Size1600x1200
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ImageFormat {
    JPEG(JPEGFormat),
    QVGA
}

impl TryFrom<u8> for ImageFormat {
    type Error = u8;
    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0 => Ok(Self::QVGA),
            1 => Ok(Self::JPEG(JPEGFormat::Size160x120)),
            2 => Ok(Self::JPEG(JPEGFormat::Size176x144)),
            3 => Ok(Self::JPEG(JPEGFormat::Size320x240)),
            4 => Ok(Self::JPEG(JPEGFormat::Size352x288)),
            5 => Ok(Self::JPEG(JPEGFormat::Size640x480)),
            6 => Ok(Self::JPEG(JPEGFormat::Size800x600)),
            7 => Ok(Self::JPEG(JPEGFormat::Size1024x768)),
            8 => Ok(Self::JPEG(JPEGFormat::Size1280x1024)),
            9 => Ok(Self::JPEG(JPEGFormat::Size1600x1200)),
            _ => Err(value)
        }
    }
}

pub struct Camera {
    bank_sel: bool,
    capturing: bool,
    image_format: ImageFormat,
    fifo_offset: usize
}

impl Camera {
    pub fn new() -> Self {
        let mut res = Self { 
            bank_sel: false,
            capturing: false,
            image_format: ImageFormat::JPEG(JPEGFormat::Size320x240),
            fifo_offset: 0
        };
        res.reset();
        res
    }

    pub fn reset(&mut self) {
        i2c_write_sensor_reg(SensorReg::BankSelect, 1);
        self.bank_sel = true;
        i2c_write_sensor_reg(SensorReg::Com7, 0x80);
        sleep(100);
        spi_send_write_cmd(CamReg::CPLD, cpld_register::RESET_CPLD_MASK);
        sleep(100);
        spi_send_write_cmd(CamReg::CPLD, 0);
        sleep(100);
        self.set_jpeg();
        self.set_jpeg_size(JPEGFormat::Size320x240);
        self.write_registers(&JPEG_320x240);
        self.image_format = ImageFormat::JPEG(JPEGFormat::Size320x240);
        self.fifo_offset = 0;
        sleep(1000);
    }

    fn write_registers(&mut self, regs: &[(I2CReg, u8)]) {
        for (reg, val) in regs {
            match reg {
                I2CReg::DSP(reg) => {
                    if self.bank_sel {
                        i2c_write_sensor_reg(SensorReg::BankSelect, 0);
                        self.bank_sel = false;
                    }
                    i2c_write_dsp_reg(*reg, *val);
                },
                I2CReg::Sensor(reg) => {
                    if !self.bank_sel {
                        i2c_write_sensor_reg(SensorReg::BankSelect, 1);
                        self.bank_sel = true;
                    }
                    i2c_write_sensor_reg(*reg, *val);
                }
            }
        }
    }

    pub fn set_format(&mut self, format: ImageFormat) {
        match (self.image_format, format) {
            (ImageFormat::QVGA, ImageFormat::JPEG(format)) => {
                self.set_jpeg();
                self.set_jpeg_size(format);
            },
            (ImageFormat::JPEG(_), ImageFormat::QVGA) => {
                self.set_qvga();
            },
            (ImageFormat::JPEG(size0), ImageFormat::JPEG(size1)) => {
                if size0 != size1 {
                    self.set_jpeg_size(size1);
                }
            },
            (ImageFormat::QVGA, ImageFormat::QVGA) => {}
        }
        sleep(1000);
    }

    fn set_jpeg(&mut self) {
        self.write_registers(&JPEG_INIT);
        self.write_registers(&YUV422);
        self.write_registers(&JPEG);
        self.write_registers(&[(I2CReg::Sensor(SensorReg::Com10), 0x0)]);
    }
    
    fn set_qvga(&mut self) {
        self.write_registers(&QVGA);
    }

    fn set_jpeg_size(&mut self, size: JPEGFormat) {
        match size {
            JPEGFormat::Size160x120 => self.write_registers(&JPEG_160x120),
            JPEGFormat::Size176x144 => self.write_registers(&JPEG_176x144),
            JPEGFormat::Size320x240 => self.write_registers(&JPEG_320x240),
            JPEGFormat::Size352x288 => self.write_registers(&JPEG_352x288),
            JPEGFormat::Size640x480 => self.write_registers(&JPEG_640x480),
            JPEGFormat::Size800x600 => self.write_registers(&JPEG_800x600),
            JPEGFormat::Size1024x768 => self.write_registers(&JPEG_1024x768),
            JPEGFormat::Size1280x1024 => self.write_registers(&JPEG_1280x1024),
            JPEGFormat::Size1600x1200 => self.write_registers(&JPEG_1600x1200),
        }
    }

    pub fn start_captures(&mut self, captures: u8) -> Result<(), CamReplyError> {
        assert!(captures > 0);
        if !self.capturing || self.captures_done() {
            self.reset_fifo();
            sleep(500);
            spi_send_write_cmd(CamReg::CapCtrl, (captures - 1) << cap_ctrl_register::FRAMES_SHIFT);
            spi_send_write_cmd(CamReg::FifoCtrl, fifo_ctrl_register::START_CAPTURE_MASK);
            self.capturing = true;
            Ok(())
        } else {
            Err(CamReplyError::CaptureInProgress)
        }
    }

    pub fn captures_done(&mut self) -> bool {
        if !self.capturing {
            true
        } else {
            let status = spi_send_read_cmd(CamReg::Status);
            if status & status_register::WRITE_FIFO_DONE_MASK != 0 {
                self.capturing = false;
                true
            } else {
                false
            }
        }
    }

    pub fn get_fifo_size(&self) -> usize {
        let mut res = 0;
        res |= spi_send_read_cmd(CamReg::WriteFIFOSize0) as usize;
        res |= (spi_send_read_cmd(CamReg::WriteFIFOSize1) as usize) << 8;
        res |= (spi_send_read_cmd(CamReg::WriteFIFOSize2) as usize) << 16;
        res
    }

    pub fn fifo_full(&self) -> bool {
        spi_send_read_cmd(CamReg::FifoStatus) & fifo_status_register::FULL_MASK != 0
    }

    pub fn read_fifo(&mut self, buffer_len: u16) {
        let mut buffer = [0; 1028];
        let fifo_size = self.get_fifo_size() - self.fifo_offset;
        let len = fifo_size.min(buffer_len as usize);
        spi_send_burst_read_cmd(CamReg::BurstFIFO, &mut buffer[4.. 4 + buffer_len as usize]);
        self.fifo_offset += len;
        buffer[..4].copy_from_slice(&len.to_le_bytes());
        check_critical(reply(0, 0, &buffer[..4 + buffer_len as usize])).unwrap_or(Ok(())).unwrap();
    }

    pub fn reset_fifo(&mut self) {
        spi_send_write_cmd(CamReg::FifoCtrl, fifo_ctrl_register::RESET_FIFO_MASK);
        spi_send_write_cmd(CamReg::FifoCtrl, fifo_ctrl_register::CLEAR_FIFO_MASK);
        self.fifo_offset = 0;
    }
}

pub enum CamReplyError {
    SendError,
    InvalidRequest,
    InvalidImageFormat,
    InvalidNumberOfCaptures,
    CaptureInProgress,
    InvalidSendBuffer,
    InvalidReplyBuffer
}

impl From<CamReplyError> for u32 {
    fn from(value: CamReplyError) -> Self {
        match value {
            CamReplyError::SendError => 1,
            CamReplyError::InvalidRequest => 2,
            CamReplyError::InvalidImageFormat => 3,
            CamReplyError::InvalidNumberOfCaptures => 4,
            CamReplyError::CaptureInProgress => 5,
            CamReplyError::InvalidSendBuffer => 6,
            CamReplyError::InvalidReplyBuffer => 7
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
    SetImageFormat(ImageFormat),
    CaptureImages(u8),
    FifoSize,
    FifoFull,
    CapturesDone,
    FifoRead(u16)
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
                // Set Image Format
                check_header_len(&header, 1, 0)?;
                let mut buffer = [0];
                _ = receive(0, &mut buffer)?;
                let format = buffer[0];
                let format = ImageFormat::try_from(format).map_err(|_| CamReplyError::InvalidImageFormat)?;
                Ok(Self::SetImageFormat(format))
            },
            5 => {
                // Capture Images
                check_header_len(&header, 1, 0)?;
                let mut buffer = [0];
                _ = receive(0, &mut buffer)?;
                let len = buffer[0];
                if len != 0 {
                    Ok(Self::CaptureImages(len))
                } else {
                    Err(CamError::ReplyError(CamReplyError::InvalidNumberOfCaptures))
                }
            },
            6 => {
                // Fifo Size
                check_header_len(&header, 0, 4)?;
                Ok(Self::FifoSize)
            },
            7 => {
                // Fifo Full
                check_header_len(&header, 0, 1)?;
                Ok(Self::FifoFull)
            },
            8 => {
                // Captures Done
                check_header_len(&header, 0, 1)?;
                Ok(Self::CapturesDone)
            },
            9 => {
                // Fifo Read
                if header.send_len > 0 {
                    return Err(CamError::ReplyError(CamReplyError::InvalidSendBuffer));
                }
                if header.reply_len < 4 || header.reply_len as usize > 1028 {
                    return Err(CamError::ReplyError(CamReplyError::InvalidReplyBuffer));
                }
                Ok(Request::FifoRead(header.reply_len - 4))
            },
            _ => {
                Err(CamError::ReplyError(CamReplyError::InvalidRequest))
            }
        }
    }
}

/// Program entry point
/// Disables mangling so it can be called from assembly
#[unsafe(no_mangle)]
pub extern "C" fn main() {
    let args = args();
    assert_eq!(args.len(), 0);
    let mut cam = Camera::new();
    #[cfg(test)]
    test_main();
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
                    Request::SetImageFormat(format) => {
                        cam.set_format(format);
                        check_critical(reply_empty(0, 0)).unwrap_or(Ok(())).unwrap();
                    },
                    Request::CaptureImages(num) => {
                        match cam.start_captures(num) {
                            Ok(()) => {
                                check_critical(reply_empty(0, 0)).unwrap_or(Ok(())).unwrap();
                            },
                            Err(err) => {
                                check_critical(reply_empty(0, u32::from(err))).unwrap_or(Ok(())).unwrap();
                            }
                        }
                    },
                    Request::FifoSize => {
                        let fifo_size = cam.get_fifo_size() as u32;
                        check_critical(reply(0, 0, &fifo_size.to_le_bytes())).unwrap_or(Ok(())).unwrap();
                    },
                    Request::FifoFull => {
                        let fifo_full = cam.fifo_full() as u8;
                        check_critical(reply(0, 0, &[fifo_full])).unwrap_or(Ok(())).unwrap();
                    },
                    Request::CapturesDone => {
                        let captures_done = cam.captures_done() as u8;
                        check_critical(reply(0, 0, &[captures_done])).unwrap_or(Ok(())).unwrap();
                    },
                    Request::FifoRead(request) => {
                        cam.read_fifo(request);
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

/// Test framework which runs all the tests
/// Based off https://os.phil-opp.com/testing/ accessed 6/02/2026
#[cfg(test)]
mod test {
    use small_os_lib::kprintln;

    pub fn test_runner(tests: &[&dyn Fn()]) {
        kprintln!("Running {} tests for camera", tests.len());
        for test in tests {
            test();
        }
    }
}
