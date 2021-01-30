use leptrino_force_torque_sensor::serialport;
use leptrino_force_torque_sensor::{LeptrinoSensor, Product};

fn search_usb_sensor_path() -> Result<Option<String>, serialport::Error> {
    // Leptrino vendor ID.
    let vendor_id = 0x0483;
    // NOTE: The following product-ID may be specific for PFS055YA251U6.
    let product_id = 0x5740;

    let ports = serialport::available_ports()?;
    let path = ports
        .into_iter()
        .filter(move |port| match &port.port_type {
            // Takes only USB-connected device
            serialport::SerialPortType::UsbPort(usb) => {
                usb.vid == vendor_id && usb.pid == product_id
            }
            _ => false,
        })
        .map(|sensor_port| sensor_port.port_name)
        .next();

    Ok(path)
}

fn main() {
    println!("leptrino-force-torque-sensor demo started.");
    println!("Make sure that the sensor is connected to the computer.");
    println!("Make sure setting udev rule. See examples/setup_udev_rule.sh in detail.");

    // Search USB-connected leptrino sensor.
    let path = match search_usb_sensor_path() {
        Ok(Some(path)) => path,
        Ok(None) => {
            println!("No leptrino sensor is connected.");
            return;
        }
        Err(e) => {
            println!("{}", e);
            return;
        }
    };
    println!("Found a sensor. Path: {}", path);

    // Connect the found sensor.
    let product_kind = Product::Pfs055Ya251U6;
    let mut sensor = match LeptrinoSensor::open(product_kind, path) {
        Ok(sensor) => sensor,
        Err(e) => {
            println!("{}", e);
            return;
        }
    };
    println!("Successfully opened the sensor.");

    // Correct zero-point
    sensor.zeroed();

    // Repeatedly receive wrenches from the sensor.
    let measurement_count = 1000;
    for i in 0..measurement_count {
        std::thread::sleep(sensor.inner_port().timeout());

        match sensor.update() {
            Ok(w) => println!("[{}/{}] {:?}", i + 1, measurement_count, w),
            Err(e) => println!("[{}/{}] {}", i + 1, measurement_count, e),
        }
    }

    // Info
    println!("Product info: {:?}", sensor.receive_product_info());

    println!("leptrino-force-torque-sensor demo finished.");
}
