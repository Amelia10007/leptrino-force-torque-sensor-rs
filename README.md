# leptrino-force-torque-sensor
[![crates.io](https://img.shields.io/crates/v/leptrino-force-torque-sensor.svg)](https://crates.io/crates/leptrino-force-torque-sensor)
[![Build Status](https://travis-ci.org/Amelia10007/leptrino-force-torque-sensor-rs.svg?branch=master)](https://travis-ci.org/Amelia10007/leptrino-force-torque-sensor-rs)

Unofficial device driver for [Leptrino force torque sensors](https://www.leptrino.co.jp/).

Line up of Leptrino sensor is available [here](https://www.leptrino.co.jp/PDF/CFSHP.pdf).

Inspired the works of [ROS package](https://github.com/hiveground-ros-package/leptrino_force_torque) (It is written in C/C++ and for ROS).

# Example
```rust
use leptrino_force_torque_sensor::{LeptrinoSensor, Product};

let mut sensor = LeptrinoSensor::open(Product::Pfs055Ya251U6, "/dev/ttyUSB0").unwrap();

let wrench = sensor.update().unwrap();
println!("{:?}", wrench);
```

# Dependency under Linux environment
`libudev-dev` is required under Linux environment. Please install it by  
`$ sudo apt install libudev-dev`

# Setup
It may be required to customize udev rules if you use usb-connected sensors.

[This shell script](./examples/setup_udev_rule.sh) can be useful for customize (see the file in detail).

# Run a demo for an usb-connected sensor
1. Clone this repository.
1. Setup udev rule by using [this shell script](./examples/setup_udev_rule.sh).
1. Connect your sensor.
1. Run the example by ```cargo run --example demo```

# License
MIT

# Note
I tested this crate only by Pfs055Ya251U6 sensor because I have no other Leptrino sensor.