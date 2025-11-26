use heapless::Vec;
use minicbor::{decode, encode, Decode, Encode};
use minicbor::encode::Write;
use rtt_target::rprintln;

#[derive(Debug)]
enum ParseError {
    InvalidLength,
    InvalidChecksum,
    InvalidCommand,
    BadSof,
    FrameTooLarge,
}

const START: u8 = 0xAA;
const LEN_BYTES: usize = 2;
const CRC_LEN: usize = 2;
const FRAME_CAPACITY: usize = 64;
const HEADER_LEN: usize = 1 + LEN_BYTES;
const MAX_PAYLOAD_LEN: usize = FRAME_CAPACITY - HEADER_LEN - CRC_LEN;

type FrameBuffer = Vec<u8, FRAME_CAPACITY>;
type PayloadBuffer = Vec<u8, MAX_PAYLOAD_LEN>;

struct HeaplessWrite<'a, const N: usize> {
    buf: &'a mut Vec<u8, N>,
}

impl<'a, const N: usize> Write for HeaplessWrite<'a, N> {
    type Error = ();

    fn write_all(&mut self, bytes: &[u8]) -> Result<(), Self::Error> {
        self.buf.extend_from_slice(bytes).map_err(|_| ())
    }
}

#[derive(Debug, Clone, Encode, Decode)]
#[cbor(array)]
enum CommandType {
    #[n(0x01)]
    TargetAngles {
        #[n(0)]
        angles: [f32; 6]
    },
    #[n(0x02)]
    EnableMotor,
    #[n(0x03)]
    DisableMotor,
    #[n(0x04)]
    EmergencyStop,
    #[n(0x05)]
    ReadMotorStatus,
    #[n(0x06)]
    GetSystemStatus,
    #[n(0x07)]
    SetControlMode,
    #[n(0x08)]
    ConfigureMotors,
    #[n(0x09)]
    HomeMotors,
    }
    
fn encode_frame(cmd: &CommandType) -> Result<FrameBuffer, ParseError> {
    let payload = encode_payload(cmd)?;
    let payload_len = payload.len();
    let len_bytes = (payload_len as u16).to_le_bytes();

    let mut frame = FrameBuffer::new();
    frame.push(START).map_err(|_| ParseError::FrameTooLarge)?;
    frame.extend_from_slice(&len_bytes)
        .map_err(|_| ParseError::FrameTooLarge)?;
    frame
        .extend_from_slice(&payload)
        .map_err(|_| ParseError::FrameTooLarge)?;

    let crc = crc16(&frame[1..]);
    frame
        .extend_from_slice(&crc.to_le_bytes())
        .map_err(|_| ParseError::FrameTooLarge)?;

    Ok(frame)
}

fn encode_payload(cmd: &CommandType) -> Result<PayloadBuffer, ParseError> {
    let mut payload = PayloadBuffer::new();
    let writer = HeaplessWrite { buf: &mut payload };
    encode(cmd, writer).map_err(|_| ParseError::FrameTooLarge)?;
    Ok(payload)
}

#[derive(Clone, Copy)]
enum RxState {
    WaitStart,
    Header,
    Payload,
    CrcLo,
    CrcHi,
}

struct CommandParser {
    buffer: [u8; FRAME_CAPACITY],
    index: usize,
    state: RxState,
    wanted_bytes: usize,

    rx: heapless::spsc::Queue<CommandType, 8>,
    tx: heapless::spsc::Queue<CommandType, 8>,
}

impl CommandParser {
    pub fn rx_loop<Serial: embedded_io::Read>(&mut self, mut serial: Serial) {

        let mut tmp = [0u8; 1];
        match serial.read(&mut tmp) {
            Ok(n) => {
                if n == 0 {return;};
                let byte = tmp[0];

                match self.state {
                    RxState::WaitStart => {
                        if byte == START {
                            self.state = RxState::Header;
                            self.index = 0;
                            self.buffer[self.index] = byte;
                            self.index = 1;
                            self.wanted_bytes = LEN_BYTES;
                        }
                    }
                    RxState::Header => {
                        self.buffer[self.index] = byte;
                        self.index += 1;
                        self.wanted_bytes -= 1;

                        if self.wanted_bytes == 0 {
                            let len = self.payload_len();
                            if len > MAX_PAYLOAD_LEN
                                || HEADER_LEN + len + CRC_LEN > self.buffer.len()
                            {
                                self.reset();
                            } else {
                                self.state = RxState::Payload;
                                self.wanted_bytes = len;
                                if self.wanted_bytes == 0 {
                                    self.state = RxState::CrcLo;
                                    self.wanted_bytes = 1;
                                }
                            }
                        }
                    }
                    RxState::Payload => {
                        self.buffer[self.index] = byte;
                        self.index += 1;
                        if self.wanted_bytes == 0 {
                            self.reset();
                            return;
                        }
                        self.wanted_bytes -= 1;

                        if self.wanted_bytes == 0 {
                            self.state = RxState::CrcLo;
                            self.wanted_bytes = 1;
                        }
                    }
                    RxState::CrcLo => {
                        self.buffer[self.index] = byte;
                        self.index += 1;
                        self.state = RxState::CrcHi;
                    }
                    RxState::CrcHi => {
                        self.buffer[self.index] = byte;
                        self.index += 1;

                        if self.check_crc() {
                            let command = self.handle_packet().map_err(|e| {
                                rprintln!("Error handling packet: {:?}", e);
                            });

                            if let Ok(cmd) = command {
                                self.rx.enqueue(cmd).ok();
                            }
                        }
                        self.reset();
                    }
                }
            }       
            Err(_e) => {}
        }
    }


    pub fn tx_loop<Serial: embedded_io::Write>(&mut self, mut serial: Serial) {
        if let Some(cmd) = self.tx.dequeue() {
            match encode_frame(&cmd) {
                Ok(packet) => {
                    let _ = serial.write(&packet);
                }
                Err(err) => {
                    rprintln!("Failed to encode frame: {:?}", err);
                }
            }
        }
    }

    pub fn get_command(&mut self) -> Option<CommandType> {
        self.rx.dequeue()
    }

    pub fn send_response(&mut self, response: CommandType) {
        self.tx.enqueue(response).ok();
    }

    fn reset(&mut self) {
        self.state = RxState::WaitStart;
        self.index = 0;
        self.wanted_bytes = 0;
    }

    fn payload_len(&self) -> usize {
        let len = u16::from_le_bytes([self.buffer[1], self.buffer[2]]);
        len as usize
    }

    fn handle_packet(&mut self) -> Result<CommandType, ParseError> {
        let len = self.payload_len();
        let payload = &self.buffer[HEADER_LEN..HEADER_LEN + len];
        decode(payload).map_err(|_| ParseError::InvalidCommand)
    }

    fn check_crc(&self) -> bool {
        let len = self.payload_len();
        if HEADER_LEN + len + CRC_LEN > self.index {
            return false;
        }

        let packet_crc = &self.buffer[1..HEADER_LEN + len];
        let crc_index = HEADER_LEN + len;
        let received_crc =
            u16::from_le_bytes([self.buffer[crc_index], self.buffer[crc_index + 1]]);
        let calculated_crc = crc16(packet_crc);
        received_crc == calculated_crc
    }
}

fn crc16(data: &[u8]) -> u16 {
    let mut crc: u16 = 0xFFFF;
    for &b in data {
        crc ^= b as u16;
        for _ in 0..8 {
            if (crc & 1) != 0 {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    crc & 0xFFFF
}

