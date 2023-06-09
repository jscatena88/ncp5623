#![no_std]
//! No-std rust driver for the ON Semiconductor RGB LED driver
//! Built on the Embbedded HAl I2C Write trait

#[cfg(test)]
extern crate std;

use derive_new::new;
use embedded_hal::blocking::i2c::Write;

const DEFAULT_ADDRESS: u8 = 0x38;

/// 7-bit I2C address of the NCP5623
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
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
pub enum Error<T> {
    InvalidValue,
    WriteError(T),
}

impl<T> From<T> for Error<T> {
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
    address: Address,
}

impl<I> NCP5623<I>
where
    I: Write,
    <I as Write>::Error: core::fmt::Debug,
{
    pub fn new_default_address(i2c: I) -> Self {
        Self::new(i2c, Address::Default)
    }

    pub fn shutdown(&mut self) -> Result<(), Error<<I as Write>::Error>> {
        self.send_command(Command::Shutdown, 0)
    }

    pub fn set_red(&mut self, pwm_value: u8) -> Result<(), Error<<I as Write>::Error>> {
        Self::validate_value(&pwm_value)?;
        self.send_command(Command::SetRed, pwm_value)
    }

    pub fn set_green(&mut self, pwm_value: u8) -> Result<(), Error<<I as Write>::Error>> {
        Self::validate_value(&pwm_value)?;
        self.send_command(Command::SetGreen, pwm_value)
    }

    pub fn set_blue(&mut self, pwm_value: u8) -> Result<(), Error<<I as Write>::Error>> {
        Self::validate_value(&pwm_value)?;
        self.send_command(Command::SetBlue, pwm_value)
    }

    pub fn set_rgb(
        &mut self,
        red_pwm_value: u8,
        green_pwm_value: u8,
        blue_pwm_value: u8,
    ) -> Result<(), Error<<I as Write>::Error>> {
        Self::validate_value(&red_pwm_value)?;
        Self::validate_value(&green_pwm_value)?;
        Self::validate_value(&blue_pwm_value)?;
        self.send_command(Command::SetRed, red_pwm_value)?;
        self.send_command(Command::SetGreen, green_pwm_value)?;
        self.send_command(Command::SetBlue, blue_pwm_value)
    }

    pub fn set_max_current(&mut self, max_current: u8) -> Result<(), Error<<I as Write>::Error>> {
        Self::validate_value(&max_current)?;
        self.send_command(Command::SetMaxCurrent, max_current)
    }

    pub fn set_upward_target(&mut self, target: u8) -> Result<(), Error<<I as Write>::Error>> {
        Self::validate_value(&target)?;
        self.send_command(Command::UpwardTarget, target)
    }

    pub fn set_downward_target(&mut self, target: u8) -> Result<(), Error<<I as Write>::Error>> {
        Self::validate_value(&target)?;
        self.send_command(Command::DownwardTarget, target)
    }

    pub fn start_dimming(&mut self, period: u8) -> Result<(), Error<<I as Write>::Error>> {
        Self::validate_value(&period)?;
        self.send_command(Command::DimmingStart, period)
    }

    pub fn send_command(
        &mut self,
        cmd: Command,
        value: u8,
    ) -> Result<(), Error<<I as Write>::Error>> {
        self.i2c
            .write(self.address.into(), &[Self::build_command(cmd, value)])
            .map_err(Error::from)
    }

    fn validate_value(value: &u8) -> Result<(), Error<<I as Write>::Error>> {
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
    use mockall::{mock, predicate::eq};

    #[derive(Copy, Clone, Debug, PartialEq, Eq)]
    pub enum MockError {
        Mock,
    }

    mock! {
        pub I2C {}
        impl Write for I2C {
            type Error = MockError;
            fn write(&mut self, address: u8, data: &[u8]) -> Result<(), <Self as Write>::Error>;
        }
    }

    #[test]
    fn test_build_command() {
        assert_eq!(NCP5623::<MockI2C>::build_command(Command::Shutdown, 0), 0u8);
        assert_eq!(
            NCP5623::<MockI2C>::build_command(Command::SetRed, 0b0110),
            0b01000110
        );
        assert_eq!(
            NCP5623::<MockI2C>::build_command(Command::SetRed, 0b11111),
            0b01011111
        );
        assert_eq!(
            NCP5623::<MockI2C>::build_command(Command::SetRed, 0),
            0b01000000
        );
    }

    #[test]
    fn test_validate_value() {
        assert_eq!(NCP5623::<MockI2C>::validate_value(&0b0), Ok(()));
        assert_eq!(NCP5623::<MockI2C>::validate_value(&0b1010), Ok(()));
        assert_eq!(NCP5623::<MockI2C>::validate_value(&0b11111), Ok(()));
        assert_eq!(
            NCP5623::<MockI2C>::validate_value(&0b100000),
            Err(Error::InvalidValue)
        );
        assert_eq!(
            NCP5623::<MockI2C>::validate_value(&u8::MAX),
            Err(Error::InvalidValue)
        );
    }

    #[test]
    fn test_u8_from_address() {
        assert_eq!(0x42u8, u8::from(Address::Custom(0x42)));
        assert_eq!(0x38u8, u8::from(Address::Default));
    }

    #[test]
    fn test_error_from_write_error() {
        assert_eq!(Error::WriteError(MockError::Mock), Error::from(MockError::Mock));
    }

    #[test]
    fn test_new_default_address() {
        let ncp = NCP5623::new_default_address(MockI2C::default());
        assert_eq!(u8::from(ncp.address), DEFAULT_ADDRESS);
    }

    #[test]
    fn test_shutdown() {
        let mut mock = MockI2C::default();
        let data = [0b0u8];
        mock.expect_write().with(eq(DEFAULT_ADDRESS), eq(data)).return_const(Ok(()));
        let mut ncp = NCP5623::new_default_address(mock);
        ncp.shutdown().unwrap();
    }

    #[test]
    fn test_set_red_ok() {
        let mut mock = MockI2C::default();
        let data = [0b01000000 | 0x11u8];
        mock.expect_write().with(eq(DEFAULT_ADDRESS), eq(data)).return_const(Ok(()));
        let mut ncp = NCP5623::new_default_address(mock);
        ncp.set_red(0x11).unwrap();
    }

    #[test]
    fn test_set_green_ok() {
        let mut mock = MockI2C::default();
        let data = [0b01100000 | 0x11u8];
        mock.expect_write().with(eq(DEFAULT_ADDRESS), eq(data)).return_const(Ok(()));
        let mut ncp = NCP5623::new_default_address(mock);
        ncp.set_green(0x11).unwrap();
    }

    #[test]
    fn test_set_blue_ok() {
        let mut mock = MockI2C::default();
        let data = [0b10000000 | 0x11u8];
        mock.expect_write().with(eq(DEFAULT_ADDRESS), eq(data)).return_const(Ok(()));
        let mut ncp = NCP5623::new_default_address(mock);
        ncp.set_blue(0x11).unwrap();
    }

    #[test]
    fn test_rgb() {
        let mut mock = MockI2C::default();
        let data_red = [0b01000000 | 0x11u8];
        let data_green = [0b01100000 | 0x12u8];
        let data_blue = [0b10000000 | 0x13u8];
        mock.expect_write().with(eq(DEFAULT_ADDRESS), eq(data_red)).return_const(Ok(()));
        mock.expect_write().with(eq(DEFAULT_ADDRESS), eq(data_green)).return_const(Ok(()));
        mock.expect_write().with(eq(DEFAULT_ADDRESS), eq(data_blue)).return_const(Ok(()));
        let mut ncp = NCP5623::new_default_address(mock);
        ncp.set_rgb(0x11,0x12,0x13).unwrap();
    }

    #[test]
    fn test_max_current() {
        let mut mock = MockI2C::default();
        let data = [0b00100000 | 0x11u8];
        mock.expect_write().with(eq(DEFAULT_ADDRESS), eq(data)).return_const(Ok(()));
        let mut ncp = NCP5623::new_default_address(mock);
        ncp.set_max_current(0x11).unwrap();
    }

    #[test]
    fn test_upward_target() {
        let mut mock = MockI2C::default();
        let data = [0b10100000 | 0x11u8];
        mock.expect_write().with(eq(DEFAULT_ADDRESS), eq(data)).return_const(Ok(()));
        let mut ncp = NCP5623::new_default_address(mock);
        ncp.set_upward_target(0x11).unwrap();
    }

    #[test]
    fn test_downward_target() {
        let mut mock = MockI2C::default();
        let data = [0b11000000 | 0x11u8];
        mock.expect_write().with(eq(DEFAULT_ADDRESS), eq(data)).return_const(Ok(()));
        let mut ncp = NCP5623::new_default_address(mock);
        ncp.set_downward_target(0x11).unwrap();
    }

    #[test]
    fn test_start_dimming() {
        let mut mock = MockI2C::default();
        let data = [0b11100000 | 0x11u8];
        mock.expect_write().with(eq(DEFAULT_ADDRESS), eq(data)).return_const(Ok(()));
        let mut ncp = NCP5623::new_default_address(mock);
        ncp.start_dimming(0x11).unwrap();
    }

    #[test]
    fn test_send_command_err() {
        let mut mock = MockI2C::default();
        let data = [0b11000000 | 0x11u8];
        mock.expect_write().with(eq(DEFAULT_ADDRESS), eq(data)).return_const(Err(MockError::Mock));
        let mut ncp = NCP5623::new_default_address(mock);
        assert_eq!(ncp.set_downward_target(0x11), Err(Error::WriteError(MockError::Mock)));
    }
}
