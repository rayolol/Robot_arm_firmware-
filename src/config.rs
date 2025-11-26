// config.rs
// System-wide configuration for motor controller board

// ============================================================================
// SYSTEM CONFIGURATION
// ============================================================================

/// System clock frequency (STM32F103 max is 72MHz)
pub const SYSCLK_HZ: u32 = 72_000_000;

/// APB1 peripheral clock (max 36MHz for STM32F103)
pub const PCLK1_HZ: u32 = 36_000_000;

/// APB2 peripheral clock
pub const PCLK2_HZ: u32 = 72_000_000;

// ============================================================================
// MOTOR CONFIGURATION
// ============================================================================

/// Number of TMC stepper drivers
pub const NUM_TMC_MOTORS: usize = 4;

/// Number of TB6600 stepper drivers
pub const NUM_TB6600_MOTORS: usize = 6;

/// Total number of motors
pub const TOTAL_MOTORS: usize = NUM_TMC_MOTORS + NUM_TB6600_MOTORS;

/// Number of AS5600 encoders
pub const NUM_ENCODERS: usize = 6;

// ============================================================================
// CONTROL LOOP TIMING
// ============================================================================

/// PID control loop frequency (1kHz is good for steppers)
pub const PID_FREQUENCY_HZ: u32 = 1_000;

/// Encoder reading frequency (should match PID frequency)
pub const ENCODER_READ_HZ: u32 = 1_000;

/// TMC driver service rate (100Hz is plenty for UART communication)
pub const TMC_SERVICE_HZ: u32 = 100;

/// Status reporting to master frequency
pub const STATUS_REPORT_HZ: u32 = 100;

// ============================================================================
// COMMUNICATION
// ============================================================================

/// TMC UART baud rate (9600 is standard for TMC)
pub const TMC_BAUD: u32 = 9_600;

/// Master controller UART baud rate (higher for faster updates)
pub const MASTER_BAUD: u32 = 115_200;

/// TMC UART bit time in CPU cycles
pub const TMC_BIT_CYCLES: u32 = SYSCLK_HZ / TMC_BAUD;

/// TMC byte time in CPU cycles (10 bits: 1 start + 8 data + 1 stop)
pub const TMC_BYTE_CYCLES: u32 = TMC_BIT_CYCLES * 10;

/// I2C frequency for encoder communication
pub const I2C_FREQUENCY_HZ: u32 = 400_000; // 400kHz fast mode

// ============================================================================
// HARDWARE LIMITS
// ============================================================================

/// Maximum step rate per motor (20kHz is safe for most steppers)
pub const MAX_STEP_RATE_HZ: u32 = 20_000;

/// Maximum encoder RPM
pub const MAX_ENCODER_RPM: u32 = 300;

/// Minimum pulse width for TB6600 step signal (microseconds)
pub const TB6600_MIN_PULSE_US: u32 = 2;

// ============================================================================
// PID DEFAULTS
// ============================================================================

/// Default proportional gain
pub const DEFAULT_KP: f32 = 1.0;

/// Default integral gain
pub const DEFAULT_KI: f32 = 0.1;

/// Default derivative gain
pub const DEFAULT_KD: f32 = 0.05;

/// PID output limits (step rate or current)
pub const PID_OUTPUT_MIN: f32 = -1000.0;
pub const PID_OUTPUT_MAX: f32 = 1000.0;

/// Anti-windup limit for integral term
pub const PID_INTEGRAL_LIMIT: f32 = 100.0;

// ============================================================================
// TMC MOTOR DEFAULTS
// ============================================================================

/// Default TMC motor current in milliamps
pub const TMC_DEFAULT_CURRENT_MA: u16 = 800;

/// Default TMC microsteps (0=256, 1=128, 2=64, 3=32, 4=16, 5=8, 6=4, 7=2, 8=1)
pub const TMC_DEFAULT_MICROSTEPS: u8 = 4; // 16 microsteps

/// TMC standby current (percentage of run current, 0-31)
pub const TMC_IHOLD: u8 = 16; // 50% of run current

/// TMC run current (0-31, but calculated from current_ma)
pub const TMC_IRUN: u8 = 31;

/// TMC hold delay (0-15, time to reduce to standby current)
pub const TMC_IHOLDDELAY: u8 = 10;

// ============================================================================
// TB6600 DEFAULTS
// ============================================================================

/// Default TB6600 microstepping (set by DIP switches on driver)
/// This is just for reference in calculations
pub const TB6600_DEFAULT_MICROSTEPS: u8 = 8;

// ============================================================================
// ENCODER CONFIGURATION
// ============================================================================

/// AS5600 I2C address
pub const AS5600_I2C_ADDR: u8 = 0x36;

/// Encoder resolution (AS5600 is 12-bit = 4096 counts per revolution)
pub const ENCODER_RESOLUTION: u16 = 4096;

/// Encoder counts to degrees conversion
pub const ENCODER_TO_DEGREES: f32 = 360.0 / ENCODER_RESOLUTION as f32;

/// TCA9548A I2C multiplexer address
pub const TCA9548A_I2C_ADDR: u8 = 0x70;

// ============================================================================
// BUFFER SIZES
// ============================================================================

/// Command queue size
pub const COMMAND_QUEUE_SIZE: usize = 16;

/// Response buffer size
pub const RESPONSE_BUFFER_SIZE: usize = 64;

/// UART RX buffer size
pub const UART_RX_BUFFER_SIZE: usize = 128;

// ============================================================================
// SAFETY & LIMITS
// ============================================================================

/// Maximum position error before triggering fault (degrees)
pub const MAX_POSITION_ERROR_DEG: f32 = 10.0;

/// Maximum velocity error before triggering fault (RPM)
pub const MAX_VELOCITY_ERROR_RPM: f32 = 50.0;

/// Watchdog timeout for master communication (milliseconds)
pub const MASTER_COMM_TIMEOUT_MS: u32 = 1000;

/// Motor disable timeout after no commands (milliseconds)
pub const MOTOR_TIMEOUT_MS: u32 = 5000;

// ============================================================================
// FEATURE FLAGS
// ============================================================================

/// Enable closed-loop position control
pub const FEATURE_POSITION_CONTROL: bool = true;

/// Enable closed-loop velocity control
pub const FEATURE_VELOCITY_CONTROL: bool = true;

/// Enable current sensing (if ADC is available)
pub const FEATURE_CURRENT_SENSING: bool = false;

/// Enable homing sequence
pub const FEATURE_HOMING: bool = false;

pub const PCF8574_I2C_ADDR: u8 = 0x20;


// ============================================================================
// DEBUG CONFIGURATION
// ============================================================================

/// Enable verbose logging via RTT
pub const DEBUG_VERBOSE: bool = true;

/// Log PID values every N iterations
pub const DEBUG_PID_LOG_INTERVAL: u32 = 1000;

/// Log encoder values every N iterations
pub const DEBUG_ENCODER_LOG_INTERVAL: u32 = 1000;

// ============================================================================
// VERSION INFO
// ============================================================================

pub const FIRMWARE_VERSION_MAJOR: u8 = 1;
pub const FIRMWARE_VERSION_MINOR: u8 = 0;
pub const FIRMWARE_VERSION_PATCH: u8 = 0;