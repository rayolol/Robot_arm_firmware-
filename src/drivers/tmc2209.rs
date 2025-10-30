use rtt_target::rprintln;
use tmc2209::*;
use crate::halfduplex::HalfDuplexSerial;

pub enum TMCError {
    UARTError,
    TMCInternalError,
    TMCDisabled
}

#[derive(Clone, Copy, Debug)]  // ← Add these
pub enum Register {
    GCONF,
    GSTAT,
    CHOPCONF,
    IHOLDIRUN,
    IFCNT,
    DRVSTATUS,
    TSTEP,
}

pub enum MotorCommand {
    Configure(TMCConfig),
    SetCurrent(u16),
    SetMicrosteps(u32),
    GetStatus,
}

pub struct MotorResponse {
    motor_id: u8,
    success: bool,
    status: Option<reg::DRV_STATUS>,
    error: Option<TMCError>,
}



#[derive(Clone, Copy, Debug)]
pub struct TmcResponse {
    motor: u8,
    register: Register,
    value: u32,
    success: bool
}

pub struct TMCConfig {
    enable_spreadcycle: bool,
    shaft_dir: bool,
    index_otpw: bool,
    index_step: bool,
    multistep_filt: bool,
   
}

pub struct TmcDriver {
    motor_id: u8,
    enable: bool,
    current_ma: u16,
    microsteps: u32,
    reader: Reader
}

impl TmcDriver {
    pub fn new(motor_id: u8, current_ma: u16, microsteps: u32) -> Self {
        Self {
            motor_id,
            enable: false,
            current_ma,
            microsteps,
            reader: Reader::default(),
        }
    }
    pub fn enable(&mut self) -> Result<(), TMCError> {
        self.enable = true;

        Ok(())
    }

    pub fn disable(&mut self) -> Result<(), TMCError> {
        self.enable = false;

        Ok(())
    }

    pub fn config(&mut self, config: TMCConfig, uart: &mut HalfDuplexSerial) -> Result<(), TMCError> {
        if !self.enable {return Err(TMCError::TMCDisabled)}

        let mut conf = reg::GCONF(0);
        conf.set_en_spread_cycle(config.enable_spreadcycle);
        conf.set_i_scale_analog(false);
        conf.set_index_otpw(config.index_otpw);
        conf.set_index_step(config.index_step);
        conf.set_internal_rsense(false);
        conf.set_mstep_reg_select(true);
        conf.set_multistep_filt(config.multistep_filt);
        conf.set_pdn_disable(true);
        conf.set_shaft(config.shaft_dir);
        conf.set_test_mode(false);

        let req = write_request::<reg::GCONF>(self.motor_id, conf);

        let response = uart.send_and_get(req.bytes()).map_err(|_| TMCError::UARTError)?;

        if !response.is_empty() {
            rprintln!("TMC2209[{}] got response {:?}", self.motor_id, response);
        } else { 
            rprintln!("TMC2209[{}] got no response", self.motor_id);
        }


        Ok(())
    }
    pub fn set_current(&mut self, current_ma: u16, uart: &mut HalfDuplexSerial) -> Result<(), TMCError> {
        if !self.enable {return Err(TMCError::TMCDisabled)}

        self.current_ma = current_ma;
        
        let irun = Self::current_to_irun(current_ma);
        let ihold = irun / 2;
        let ihold_delay = 10;

        let mut value = reg::IHOLD_IRUN(0);
        value.set_ihold(ihold);
        value.set_irun(irun);
        value.set_ihold_delay(ihold_delay);

        let req = write_request::<reg::IHOLD_IRUN>(self.motor_id, value);
            
        
        let response = uart.send_and_get(req.bytes() ).map_err(|_| TMCError::UARTError)?;

        if !response.is_empty() {
            rprintln!("TMC2209[{}] got response {:?}", self.motor_id, response);
        } else { 
            rprintln!("TMC2209[{}] got no response", self.motor_id);
        }
        
        Ok(())
    }
    
    pub fn set_microsteps(&mut self, msteps: u32, uart: &mut HalfDuplexSerial) -> Result<(), TMCError> {
        if !self.enable {return Err(TMCError::TMCDisabled)}

        self.microsteps = msteps;
        
        let mres = match msteps {
            256 => 0, 
            128 => 1, 
            64 => 2, 
            32 => 3,
            16 => 4, 
            8 => 5, 
            4 => 6, 
            2 => 7, 
            1 => 8,
            _ => 4,
        };
        
        let mut chopconf = reg::CHOPCONF(0);
        chopconf.set_mres(mres);
        chopconf.set_tbl(2);
        chopconf.set_toff(3);
        chopconf.set_intpol(true);
            
        let req = write_request(self.motor_id, chopconf);
        
        let response = uart.send_and_get(req.bytes())
            .map_err(|_| TMCError::UARTError)?;

        if !response.is_empty() {
            rprintln!("TMC2209[{}] got response", self.motor_id);
        } else { 
            rprintln!("TMC2209[{}] got no response", self.motor_id);
        }
        
        Ok(())
    }
    pub fn get_status(&mut self, uart: &mut HalfDuplexSerial) -> Result<reg::DRV_STATUS, TMCError> {
        

        let read = read_request::<reg::DRV_STATUS>(self.motor_id);

        let response = uart.send_and_get(read.bytes()).map_err(|_| TMCError::UARTError)
            .map_err(|_| TMCError::UARTError)?;

        let array = response.as_ref();
        let reg = self.reader.read_response(array);
        let data = reg::DRV_STATUS(reg.1.unwrap().data_u32());
        
        Ok(data)
    }

    fn current_to_irun(ma: u16) -> u8 {
        ((ma as u32 * 32) / 2000).min(31) as u8
    }

    pub fn execute(&mut self, cmd: MotorCommand, uart: &mut HalfDuplexSerial) -> MotorResponse {
        match cmd {
            MotorCommand::Configure(cfg) => {
                let res = self.config(cfg, uart); 
                MotorResponse {
                    motor_id: self.motor_id,
                    success: res.is_ok(),
                    status: None,
                    error: res.err(),
                }
            }
            MotorCommand::SetCurrent(ma) => {
                let res = self.set_current(ma, uart);
                MotorResponse {
                    motor_id: self.motor_id,
                    success: res.is_ok(),
                    status: None,
                    error: res.err(),
                }
            }
            MotorCommand::GetStatus => {
                let res = self.get_status(uart);
                MotorResponse {
                    motor_id: self.motor_id,
                    success: res.is_ok(),
                    status: res.as_ref().ok().cloned(),
                    error: res.err(),
                }
            }
            _ => unimplemented!(),
        }
    }
}

