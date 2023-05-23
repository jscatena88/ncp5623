#![no_std]
//! No-std rust driver for the ON Semiconductor RGB LED driver
//! Built on the Embbedded HAl I2C Write trait

use derive_new::new;
use embedded_hal::blocking::i2c::Write;

const DEFAULT_ADDRESS: u8 = 0x38;

/// 7-bit I2C address of the NCP5623
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Address {
    Default,
    Custom(u8),
}

impl From<Address> for u8 {
    fn from(value: Address) -> Self {
        match value {
            Address::Default => DEFAULT_ADDRESS,
            Address::Custom(address) => address,
        }
    }
}

/// Error type
#[derive(Debug, PartialEq, Eq)]
pub enum Error<T: PartialEq>
{
    InvalidValue,
    WriteError(T),
}

impl<T: PartialEq> From<T> for Error<T> {
    fn from(value: T) -> Self {
        Self::WriteError(value)
    }
}

/// Enum of valid commands that can be sent, for use with the [`NCP5623::send_command`] function
#[derive(Clone, Copy)]
pub enum Command {
    Shutdown = 0b0,
    SetMaxCurrent = 0b00100000,
    SetRed = 0b01000000,
    SetGreen = 0b01100000,
    SetBlue = 0b10000000,
    UpwardTarget = 0b10100000,
    DownwardTarget = 0b11000000,
    DimmingStart = 0b11100000,
}

/// Instance of a NCP5623
/// See the chips's data sheet for details on the API
/// All commands (except [`NCP5623::shutdown`]) accept a 5 bit value as input (i.e. 0->31 inclusive). An [`Error::InvalidValue`]
/// is returned if the value passed in was not in the valid range
#[derive(new)]
pub struct NCP5623<I> {
    i2c: I,
    address: Address
}

impl<I> NCP5623<I> 
where
    I: Write,
    <I as Write>::Error: core::fmt::Debug + PartialEq
{
    pub fn new_default_address(i2c: I) -> Self{
        Self::new(i2c, Address::Default)
    }

    pub fn shutdown(&mut self) -> Result<(), Error<<I as Write>::Error>> {
        self.send_command(Command::Shutdown, 0)
    }

    pub fn set_red(&mut self, pwm_value: u8) ->  Result<(), Error<<I as Write>::Error>> {
        Self::validate_value(&pwm_value)?;
        self.send_command(Command::SetRed, pwm_value)
    }

    pub fn set_green(&mut self, pwm_value: u8) ->  Result<(), Error<<I as Write>::Error>> {
        Self::validate_value(&pwm_value)?;
        self.send_command(Command::SetGreen, pwm_value)
    }

    pub fn set_blue(&mut self, pwm_value: u8) ->  Result<(), Error<<I as Write>::Error>> {
        Self::validate_value(&pwm_value)?;
        self.send_command(Command::SetBlue, pwm_value)
    }

    pub fn set_rgb(&mut self, red_pwm_value: u8, green_pwm_value: u8, blue_pwm_value: u8) ->  Result<(), Error<<I as Write>::Error>> {
        Self::validate_value(&red_pwm_value)?;
        Self::validate_value(&green_pwm_value)?;
        Self::validate_value(&blue_pwm_value)?;
        self.send_command(Command::SetRed, red_pwm_value)?;
        self.send_command(Command::SetGreen, green_pwm_value)?;
        self.send_command(Command::SetBlue, blue_pwm_value)
    }

    pub fn set_max_current(&mut self, max_current: u8) ->  Result<(), Error<<I as Write>::Error>> {
        Self::validate_value(&max_current)?;
        self.send_command(Command::SetMaxCurrent, max_current)
    }

    pub fn set_upward_target(&mut self, target: u8) ->  Result<(), Error<<I as Write>::Error>> {
        Self::validate_value(&target)?;
        self.send_command(Command::UpwardTarget, target)
    }

    pub fn set_downward_target(&mut self, target: u8) ->  Result<(), Error<<I as Write>::Error>> {
        Self::validate_value(&target)?;
        self.send_command(Command::DownwardTarget, target)
    }

    pub fn start_dimming(&mut self, period: u8) ->  Result<(), Error<<I as Write>::Error>> {
        Self::validate_value(&period)?;
        self.send_command(Command::DimmingStart, period)
    }

    pub fn send_command(&mut self, cmd: Command, value: u8 ) ->  Result<(), Error<<I as Write>::Error>> {
        self.i2c.write(self.address.into(), &[Self::build_command(cmd, value)]).map_err(Error::from)
    }

    fn validate_value(value: &u8) ->  Result<(), Error<<I as Write>::Error>> {
        if *value > 0b11111 {
            Err(Error::InvalidValue)
        } else {
            Ok(())
        }   
    }

    fn build_command(cmd: Command, value: u8) -> u8 {
        (cmd as u8) | value
    }
}

#[cfg(test)]
mod test {
    use super::*;
    use embedded_hal::blocking::i2c::Write;

    #[derive(Debug, PartialEq, Eq)]
    enum MockError {
        Mock,
    }

    struct MockI2cOk;
    impl Write for MockI2cOk {
        type Error = MockError;
        fn write(&mut self, _: u8, _: &[u8]) -> Result<(), Self::Error> {
            Ok(())
        }        
    }

    struct MockI2cErr;
    impl Write for MockI2cErr {
        type Error = MockError;
        fn write(&mut self, _: u8, _: &[u8]) -> Result<(), Self::Error> {
            Err(MockError::Mock)
        }        
    }

    #[test]
    fn test_build_command() {
        assert_eq!(NCP5623::<MockI2cOk>::build_command(Command::Shutdown, 0), 0u8);
        assert_eq!(NCP5623::<MockI2cOk>::build_command(Command::SetRed, 0b0110), 0b01000110);
        assert_eq!(NCP5623::<MockI2cOk>::build_command(Command::SetRed, 0b11111), 0b01011111);
        assert_eq!(NCP5623::<MockI2cOk>::build_command(Command::SetRed, 0), 0b01000000);
    }

    #[test]
    fn test_validate_value() {
        assert_eq!(NCP5623::<MockI2cOk>::validate_value(&0b0), Ok(()));
        assert_eq!(NCP5623::<MockI2cOk>::validate_value(&0b1010), Ok(()));
        assert_eq!(NCP5623::<MockI2cOk>::validate_value(&0b11111), Ok(()));
        assert_eq!(NCP5623::<MockI2cOk>::validate_value(&0b100000), Err(Error::InvalidValue));
        assert_eq!(NCP5623::<MockI2cOk>::validate_value(&u8::MAX), Err(Error::InvalidValue));    
    }
}