use std::{error::Error, io, thread};
use tinkerforge::{can_bricklet::*, ipconnection::IpConnection};

const HOST: &str = "127.0.0.1";
const PORT: u16 = 4223;
const UID: &str = "XYZ"; // Change XYZ to the UID of your CAN Bricklet

fn main() -> Result<(), Box<dyn Error>> {
    let ipcon = IpConnection::new(); // Create IP connection
    let can_bricklet = CANBricklet::new(UID, &ipcon); // Create device object

    ipcon.connect(HOST, PORT).recv()??; // Connect to brickd
                                        // Don't use device before ipcon is connected

    // Configure transceiver for loopback mode
    can_bricklet.set_configuration(CAN_BRICKLET_BAUD_RATE_1000KBPS, CAN_BRICKLET_TRANSCEIVER_MODE_LOOPBACK, 0);

    //Create listener for frame read events.
    let frame_read_listener = can_bricklet.get_frame_read_receiver();
    // Spawn thread to handle received events. This thread ends when the can_bricklet
    // is dropped, so there is no need for manual cleanup.
    thread::spawn(move || {
        for event in frame_read_listener {
            println!("Frame Type: {}", event.frame_type);
            println!("Identifier: {}", event.identifier);
            print!("Data (Length: {}):", event.length);
            for item in event.data.iter() {
                print!(" {}", item);
            }
            println!();
            println!();
        }
    });

    // Enable frame read callback
    can_bricklet.enable_frame_read_callback();

    let data = [42u8, 23, 17, 0, 0, 0, 0, 0];
    can_bricklet.write_frame(CAN_BRICKLET_FRAME_TYPE_STANDARD_DATA, 1742, data, 3);

    println!("Press enter to exit.");
    let mut _input = String::new();
    io::stdin().read_line(&mut _input)?;
    can_bricklet.disable_frame_read_callback();
    ipcon.disconnect();
    Ok(())
}
