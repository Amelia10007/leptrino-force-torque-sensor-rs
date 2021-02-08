//! Unofficial device driver for [Leptrino force torque sensors](https://www.leptrino.co.jp/).
//!
//! Line up of Leptrino sensor is available [here](https://www.leptrino.co.jp/PDF/CFSHP.pdf).
//!
//! # Examples
//! ```no_run
//! use leptrino_force_torque_sensor::{LeptrinoSensor, Product};
//!
//! let mut sensor = LeptrinoSensor::open(Product::Pfs055Ya251U6, "/dev/ttyUSB0").unwrap();
//!
//! let wrench = sensor.update().unwrap();
//! println!("{:?}", wrench);
//! ```
//!
//! # Dependency under Linux environment
//! `libudev-dev` is required under Linux environment. Please install it by  
//! `sudo apt install libudev-dev`
//!
//! # Setup
//! It may be required to customize udev rules if you use usb-connected sensors.
//!
//! [This shell script](https://github.com/Amelia10007/leptrino-force-torque-sensor-rs/blob/master/examples/setup_udev_rule.sh) can be useful for customize (see the file in detail).
//!
//! # Note
//! I tested this crate only by Pfs055Ya251U6 sensor because I have no other Leptrino sensor.
#![warn(missing_docs)]

mod parse;

use itertools::Itertools;
pub use pair_macro::Triplet;
pub use parse::ParseError;
pub use serialport;
use serialport::{DataBits, FlowControl, Parity, SerialPort, StopBits};
use std::borrow::Cow;
use std::convert::TryInto;
use std::fmt::{self, Display, Formatter};
use std::time::Duration;

/// Leptrino 6-axis force-torque sensor.
/// # Examples
/// ```no_run
/// use leptrino_force_torque_sensor::{LeptrinoSensor, Product};
///
/// let mut sensor = LeptrinoSensor::open(Product::Pfs055Ya251U6, "/dev/ttyUSB0").unwrap();
///
/// let wrench = sensor.update().unwrap();
/// println!("{:?}", wrench);
/// ```
pub struct LeptrinoSensor {
    /// Product kind of the connected sensor.
    product: Product,
    /// Serial port device.
    port: Box<dyn SerialPort>,
    /// The latest wrench acquired by `update`.
    last_raw_wrench: Wrench,
    /// The rated wrench acquired from the sensor.
    rated_wrench: Wrench,
    /// Offset used by latest_wrench() and zeroed().
    offset: Wrench,
}

impl LeptrinoSensor {
    /// Connects to the Leptrino force torque sensor.
    /// # Params
    /// 1. `product` specify a sensor product.
    /// 1. `path` The sensor's path.
    ///
    /// # Returns
    /// `Ok(sensor)` if successfully connected, `Err(reason)` if failed.
    ///
    /// # Examples
    /// See the example [here](`LeptrinoSensor`).
    pub fn open<'a>(
        product: Product,
        path: impl Into<Cow<'a, str>>,
    ) -> Result<LeptrinoSensor, Error> {
        // These settings were determined according to the hardware configuration.
        let mut port = serialport::new(path, 9600)
            .data_bits(DataBits::Eight)
            .flow_control(FlowControl::None)
            .parity(Parity::None)
            .stop_bits(StopBits::One)
            .timeout(Duration::from_millis(1))
            .open()
            .map_err(Error::SerialPort)?;

        // Obtain the rated wrench, parsing the response.
        send_command(&mut port, &[0x04, 0xFF, 0x2B, 0x00])?;
        std::thread::sleep(port.timeout());

        let res = receive_message(&mut port)?;
        let (fx, fy, fz, mx, my, mz) = (0..6)
            .map(|i| 4 + i * 4)
            .filter_map(|start| res.get(start..start + 4))
            .map(|res| f32::from_le_bytes(res.try_into().unwrap()))
            .map(|digital| digital as f64)
            .next_tuple()
            .ok_or(Error::ParseData)?;
        let force = Triplet::new(fx, fy, fz);
        let torque = Triplet::new(mx, my, mz);
        let rated_wrench = Wrench::new(force, torque);

        //
        let mut sensor = Self {
            product,
            port,
            last_raw_wrench: Wrench::zeroed(),
            rated_wrench,
            offset: Wrench::zeroed(),
        };

        sensor.request_next_wrench()?;

        Ok(sensor)
    }

    /// Returns the latest wrench that is stored in this instance without communicaing the sensor.
    ///
    /// Use [`Self::update`] instead to obtain a new wrench from the sensor.
    /// # Returns
    /// `Ok(sensor)` if successfully connected, `Err(reason)` if failed.
    pub fn last_wrench(&self) -> Wrench {
        let f = self
            .last_raw_wrench
            .force
            .map_entrywise(self.offset.force, |raw, o| raw - o);
        let t = self
            .last_raw_wrench
            .torque
            .map_entrywise(self.offset.torque, |raw, o| raw - o);

        Wrench::new(f, t)
    }

    /// Communicating to the sensor, updates the latest wrench.
    /// # Returns
    /// `Ok(wrench)` if succeeds, `Err(reason)` if failed.
    pub fn update(&mut self) -> Result<Wrench, Error> {
        let res = receive_message(&mut self.port);

        // Regardless of success or failure of receive_message(), request the next single data.
        // If we do not so, after updating failed once, updating will fail everytime due to no reception from the sensor.
        self.request_next_wrench()?;

        let res = match res {
            Ok(res) => res,
            Err(e) => {
                return Err(e);
            }
        };

        let (fx, fy, fz, mx, my, mz) = (0..6)
            .map(|i| 4 + i * 2)
            .filter_map(|start| res.get(start..start + 2))
            .map(|res| i16::from_le_bytes(res.try_into().unwrap()))
            .map(|digital| digital as f64)
            .next_tuple()
            .ok_or(Error::ParseData)?;

        let rated_binary = self.product.rated_binary();
        let force = Triplet::new(fx, fy, fz)
            .map_entrywise(self.rated_wrench.force, |left, right| {
                left / rated_binary * right
            });
        let torque = Triplet::new(mx, my, mz)
            .map_entrywise(self.rated_wrench.torque, |left, right| {
                left / rated_binary * right
            });

        self.last_raw_wrench = Wrench::new(force, torque);

        Ok(self.last_wrench())
    }

    /// Set the offset so that the current wrench is zeroed.  
    /// This methos is useful for zero-point calibration.
    /// # Examples
    /// ```no_run
    /// use leptrino_force_torque_sensor::{LeptrinoSensor, Product, Triplet};
    ///
    /// let mut sensor = LeptrinoSensor::open(Product::Pfs055Ya251U6, "/dev/ttyUSB0").unwrap();
    ///
    /// sensor.update().unwrap();
    /// sensor.zeroed();
    /// let wrench = sensor.last_wrench();
    ///
    /// assert_eq!(wrench.force, Triplet::new(0.0, 0.0, 0.0));
    /// assert_eq!(wrench.torque, Triplet::new(0.0, 0.0, 0.0));
    /// ```
    pub fn zeroed(&mut self) {
        self.offset = self.last_raw_wrench;
    }

    /// Reads the product info from the sensor.
    /// # Returns
    /// `Ok(product_info)` if succeeds, `Err(reason)` if failed.
    pub fn receive_product_info(&mut self) -> Result<ProductInfo, Error> {
        self.communicate_pausing_wrench(|sensor| {
            send_command(&mut sensor.port, &[0x04, 0xFF, 0x2A, 0x00])?;
            std::thread::sleep(sensor.port.timeout());

            let res = receive_message(&mut sensor.port)?;

            let parse = |bytes: Option<&[u8]>| {
                bytes
                    .map(|bytes| bytes.to_vec())
                    .and_then(|bytes| String::from_utf8(bytes).ok())
                    .ok_or(Error::ParseData)
            };

            let product_type = parse(res.get(4..20))?;
            let serial = parse(res.get(20..28))?;
            let firmware_version = parse(res.get(28..32))?;
            let output_rate = parse(res.get(32..38))?;

            let product_info = ProductInfo {
                product_type,
                serial,
                firmware_version,
                output_rate,
            };

            Ok(product_info)
        })
    }

    /// Gets the builtin filter's cutoff frequency in Hertz.
    /// # Returns
    /// `Ok(Some(hertz))` if the filter is enabled.
    /// `Ok(None)` if the filter is disabled.
    /// `Err(reason)` if an error occurred during communication to the sensor.
    pub fn receive_builtin_filter_cutoff_hertz(&mut self) -> Result<Option<u32>, Error> {
        self.communicate_pausing_wrench(|sensor| {
            send_command(&mut sensor.port, &[0x04, 0xFF, 0xB6, 0x00])?;
            std::thread::sleep(sensor.port.timeout());

            let res = receive_message(&mut sensor.port)?;
            let raw = res.get(4).copied().ok_or(Error::ParseData)?;

            sensor.product.builtin_filter_cutoff_hertz(raw)
        })
    }

    /// Enable or disable the builtin filter.
    ///
    /// The modified setting will be applied after rebooting the sensor.
    /// # Params
    /// 1. `cutoff_frequency` The cutoff frequency of the filter.
    /// Specify `Some(hertz)` if you want to enable the filter.
    /// Specity `None` if you want to disable the filter.
    ///
    /// # Examples
    /// ```no_run
    /// use leptrino_force_torque_sensor::{LeptrinoSensor, Product};
    ///
    /// let mut sensor = LeptrinoSensor::open(Product::Pfs055Ya251U6, "/dev/ttyUSB0").unwrap();
    ///
    /// // Disable the filter.
    /// sensor.set_builtin_filter_cutoff_hertz(None).unwrap();
    /// ```
    pub fn set_builtin_filter_cutoff_hertz(
        &mut self,
        cutoff_hertz: Option<u32>,
    ) -> Result<(), Error> {
        self.communicate_pausing_wrench(|sensor| {
            let raw = sensor.product.builtin_filter_raw(cutoff_hertz)?;
            let command = [0x08, 0xFF, 0xA6, 0x00, raw, 0x00, 0x00, 0x00];

            send_command(&mut sensor.port, &command)?;
            std::thread::sleep(sensor.port.timeout());

            Ok(())
        })
    }

    /// Returns the reference to the serial port for the sensor.
    pub fn inner_port(&self) -> &Box<dyn SerialPort> {
        &self.port
    }

    /// Request single wrench.
    fn request_next_wrench(&mut self) -> Result<(), Error> {
        send_command(&mut self.port, &[0x04, 0xFF, 0x30, 0x00])
    }

    /// Stops communicating wrench information, then starts the specified communication.
    /// Finally, regardless of success or failure of the communication succeeded or not,
    /// restarts communication of wrench information.
    fn communicate_pausing_wrench<T, F>(&mut self, mut f: F) -> Result<T, Error>
    where
        F: FnMut(&mut Self) -> Result<T, Error>,
    {
        let value = self
            .port
            .clear(serialport::ClearBuffer::All)
            .map_err(Error::SerialPort)
            .and_then(|_| f(self));

        self.request_next_wrench()?;

        value
    }
}

/// Parses the specified command, then sends it.
fn send_command(port: &mut Box<dyn SerialPort>, command: &[u8]) -> Result<(), Error> {
    let mut message = parse::parse_command(&command);
    port.write_all(&mut message).map_err(Error::IO)
}

/// Receives all available bytes from the port, then parses it.
fn receive_message(port: &mut Box<dyn SerialPort>) -> Result<Vec<u8>, Error> {
    let count = port.bytes_to_read().map_err(Error::SerialPort)? as usize;
    let mut buf = vec![0; count as usize];
    port.read_exact(&mut buf).map_err(Error::IO)?;

    parse::parse_reception(&buf).map_err(Error::ParseResponse)
}

/// Specify a product type of Leptrino force torque sensor.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Product {
    /// PFS series, 250 N rated force, 6 Nm rated torque, 6-axis.
    Pfs055Ya251U6,
}

impl Product {
    /// Returns a raw output of the sensor under the rated force and torque.
    fn rated_binary(&self) -> f64 {
        match self {
            Product::Pfs055Ya251U6 => 10000.0,
        }
    }

    /// Converts from raw message from the sensor into filter's cutoff frequency.
    fn builtin_filter_cutoff_hertz(&self, raw_value: u8) -> Result<Option<u32>, Error> {
        match self {
            Product::Pfs055Ya251U6 => match raw_value {
                0 => Ok(None),
                1 => Ok(Some(10)),
                2 => Ok(Some(100)),
                3 => Ok(Some(200)),
                _ => Err(Error::ParseData),
            },
        }
    }

    /// Converts from filter's cutoff frequency into raw message to the sensor.
    fn builtin_filter_raw(&self, cutoff_hertz: Option<u32>) -> Result<u8, Error> {
        match self {
            Product::Pfs055Ya251U6 => match cutoff_hertz {
                Some(10) => Ok(1),
                Some(100) => Ok(2),
                Some(200) => Ok(3),
                Some(_) => Err(Error::InvalidParameter),
                None => Ok(0),
            },
        }
    }
}

/// Represents product information of a connected sensor.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ProductInfo {
    /// Product type.
    pub product_type: String,
    /// Serial number of the sensor.
    pub serial: String,
    /// Firmware of the sensor.
    pub firmware_version: String,
    /// Output rate in Hertz.
    pub output_rate: String,
}

/// Represents an error occurred while communicating sensors.
#[derive(Debug)]
pub enum Error {
    /// Failed to open the port for the sensor.
    SerialPort(serialport::Error),
    /// Failed to read or write data during communication.
    IO(std::io::Error),
    /// Received an invalid format message from the sensor.
    ParseResponse(ParseError),
    /// The received message has an invalid data part.
    ParseData,
    /// An invalid parameter was specified via sensor API.
    InvalidParameter,
}

impl Display for Error {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        match self {
            Error::SerialPort(e) => write!(f, "SerialPort: {}", e),
            Error::IO(e) => write!(f, "IO: {}", e),
            Error::ParseResponse(e) => write!(f, "Parse: {}", e),
            Error::ParseData => write!(f, "Failed to parse the response into data."),
            Error::InvalidParameter => write!(f, "An invalid parameter was specified."),
        }
    }
}

impl std::error::Error for Error {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Error::SerialPort(e) => Some(e),
            Error::IO(e) => Some(e),
            Error::ParseResponse(e) => Some(e),
            _ => None,
        }
    }
}

/// A pair of force and torque.
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct Wrench {
    /// 3-dimensional force in Newton.
    pub force: Triplet<f64>,
    /// 3-dimensional torque in NewtonMeter.
    pub torque: Triplet<f64>,
}

impl Wrench {
    /// Returns a new wrench.
    pub fn new(force: Triplet<f64>, torque: Triplet<f64>) -> Wrench {
        Self { force, torque }
    }

    /// Returns a new wrench, initializing it to 0 Newton and 0 NewtonMeter.
    pub fn zeroed() -> Wrench {
        Wrench::new(Triplet::default(), Triplet::default())
    }
}
