use rtic_sync::{channel::*, make_channel};
use heapless::Vec;
use crate::control::motor::{MotorConfig};

#[derive(serde::Serialize, serde::Deserialize, Hash, Eq, PartialEq)]

enum CommandType {
    SetMotorVelocity {motor_id: u8, velocity: i32},
    SetMotorPosition {motor_id: u8, position: i32},
    EnableMotor {},
    DisableMotor {},
    EmergencyStop {},
    ReadMotorStatus {},
    GetSystemStatus {},
    SetControlMode {},
    ConfigureMotors {},
    HomeMotors {},
}

fn encode_command(cmd: &CommandType) -> Vec<u8, 64> {
    let mut buffer = Vec::new();
    postcard::to_slice(cmd, &mut buffer).unwrap();
    buffer
}

fn decode_command(data: &[u8]) -> Result<CommandType, postcard::Error> {
    postcard::from_bytes(data)
}
const SOF: u8 = 0xAA;
const HEADER_LEN: usize = 4;
const MAX_PAYLOAD_LEN: usize = 64;
const CRC_LEN: usize = 4;
enum ParseError {
    InvalidLength,
    InvalidChecksum,
    InvalidCommand,
    BadSof,
}

enum SystemStatus {
    //TODO: implement system status fields
}

enum Response {
    //TODO: implement response types
    MotorStatus { motor_id: u8, },
    SystemStatus { status: SystemStatus },
    Acknowledgement,
    PANIC { reason: &'static str },
}

struct Command {
    command_type: CommandType,
    data: Option<heapless::Vec<u8, 2>>,
}


struct CommandExecutor {
    command_channel: Receiver<'static, Command, 8>,
    response_channel: Sender<'static, Response, 8>,
    execute_queue: heapless::spsc::Queue<Command, 10>,
    handlers: heapless::index_map::FnvIndexMap<CommandType, fn(Command), 8>,
}

impl CommandExecutor {
    pub fn new(command_channel: Receiver<'static, Command, 8>, response_channel: Sender<'static, Response, 8>) -> Self {
        CommandExecutor {
            command_channel,
            response_channel,
            execute_queue: heapless::spsc::Queue::new(),
            handlers: heapless::index_map::FnvIndexMap::new(),
        }
    }

    fn encode_frame(&self, command: &Command) -> Vec<u8, {MAX_PAYLOAD_LEN + HEADER_LEN + CRC_LEN}> {
        let mut frame: Vec<u8, {MAX_PAYLOAD_LEN + HEADER_LEN + CRC_LEN}> = Vec::new();
        frame.push(SOF).unwrap();

        let payload = encode_command(&command.command_type);
        let payload_len = payload.len() as u16;

        frame.extend_from_slice(&payload_len.to_le_bytes()).unwrap();
        frame.extend_from_slice(&payload).unwrap();

        let mut crc = crc_any::CRC::crc16ccitt_false();
        crc.digest(&frame); // Exclude SOF from CRC
        let crc = crc.get_crc();
    
        frame.extend_from_slice(&crc.to_le_bytes()).unwrap();

        frame
    }

    fn decode_frame(&self, frame: &[u8]) -> Result<Command, ParseError> {
        if frame.len() < HEADER_LEN + CRC_LEN {
            return Err(ParseError::InvalidLength);
        }
        if frame[0] != SOF {
            return Err(ParseError::BadSof);
        }

        let payload_len = u16::from_le_bytes([frame[1], frame[2]]) as usize;
        if frame.len() != HEADER_LEN + payload_len + CRC_LEN {
            return Err(ParseError::InvalidLength);
        }

        let received_crc = u16::from_le_bytes([frame[HEADER_LEN + payload_len], frame[HEADER_LEN + payload_len + 1]]);
        let mut crc = crc_any::CRC::crc16ccitt_false();
        crc.digest(&frame[..HEADER_LEN + payload_len]); // Exclude SOF from CRC
        let calculated_crc = crc.get_crc();

        if received_crc != calculated_crc as u16 {
            return Err(ParseError::InvalidChecksum);
        }

        let command_type = decode_command(&frame[HEADER_LEN..HEADER_LEN + payload_len])
            .map_err(|_| ParseError::InvalidCommand)?;

        Ok(Command {
            command_type,
            data: None,
        })
    }

    pub fn parse_packet(&mut self, packet: &[u8]) -> Result<(), ParseError> {
        let cmd = self.decode_frame(packet)?;
        self.execute_queue.enqueue(cmd).ok();
        Ok(())  
    }


    fn register_handler(&mut self, command_type: CommandType, handler: fn(Command)) {
        self.handlers.insert(command_type, handler).ok();
    }
      

    fn read_fileterd(&mut self, command: Vec<Command, 10>) -> Option<Command> {
        for cmd in command {
            if let Ok(received_command) = self.command_channel.try_recv() {
                return Some(received_command);
            }
        }
        None
    }

    fn execute(&mut self) {
        if let Some(command) = self.execute_queue.dequeue() {
            self.handlers.get(&command.command_type).unwrap()(command);
        }
    }

    fn publish_response(&mut self, response: Response) {
        self.response_channel.try_send(response).ok();
    }
}