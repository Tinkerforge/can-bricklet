use std::{error::Error, io, thread};
use tinkerforge::{can_bricklet::*, ip_connection::IpConnection};

const HOST: &str = "127.0.0.1";
const PORT: u16 = 4223;
const UID: &str = "XYZ"; // Change XYZ to the UID of your CAN Bricklet

fn main() -> Result<(), Box<dyn Error>> {
    let ipcon = IpConnection::new(); // Create IP connection
    let can = CanBricklet::new(UID, &ipcon); // Create device object

    ipcon.connect((HOST, PORT)).recv()??; // Connect to brickd
                                          // Don't use device before ipcon is connected

    // Configure transceiver for loopback mode
    can.set_configuration(
        CAN_BRICKLET_BAUD_RATE_1000KBPS,
        CAN_BRICKLET_TRANSCEIVER_MODE_LOOPBACK,
        0,
    );

    let frame_read_receiver = can.get_frame_read_callback_receiver();

    // Spawn thread to handle received events.
    // This thread ends when the `can` object
    // is dropped, so there is no need for manual cleanup.
    thread::spawn(move || {
        for frame_read in frame_read_receiver {
            println!("Frame Type: {}", frame_read.frame_type);
            println!("Identifier: {}", frame_read.identifier);
            print!("Data (Length: {}):", frame_read.length);
            for item in frame_read.data.iter() {
                print!(" {}", item);
            }
            println!();
            println!();
        }
    });

    // Enable frame read callback
    can.enable_frame_read_callback();

    let data = [42u8, 23, 17, 0, 0, 0, 0, 0];
    can.write_frame(CAN_BRICKLET_FRAME_TYPE_STANDARD_DATA, 1742, data, 3);

    println!("Press enter to exit.");
    let mut _input = String::new();
    io::stdin().read_line(&mut _input)?;
    can.disable_frame_read_callback();
    ipcon.disconnect();
    Ok(())
}
