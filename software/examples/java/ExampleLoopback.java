import com.tinkerforge.IPConnection;
import com.tinkerforge.BrickletCAN;

public class ExampleLoopback {
	private static final String HOST = "localhost";
	private static final int PORT = 4223;

	// Change XYZ to the UID of your CAN Bricklet
	private static final String UID = "XYZ";

	// Note: To make the example code cleaner we do not handle exceptions. Exceptions
	//       you might normally want to catch are described in the documentation
	public static void main(String args[]) throws Exception {
		IPConnection ipcon = new IPConnection(); // Create IP connection
		BrickletCAN can = new BrickletCAN(UID, ipcon); // Create device object

		ipcon.connect(HOST, PORT); // Connect to brickd
		// Don't use device before ipcon is connected

		// Configure transceiver for loopback mode
		can.setConfiguration(BrickletCAN.BAUD_RATE_1000KBPS,
		                     BrickletCAN.TRANSCEIVER_MODE_LOOPBACK, 0);

		// Add frame read listener
		can.addFrameReadListener(new BrickletCAN.FrameReadListener() {
			public void frameRead(short frameType, long identifier, short[] data,
			                      short length) {
				System.out.println("Frame Type: " + frameType);
				System.out.println("Identifier: " + identifier);
				System.out.print("Data (Length: " + length + "):");

				for (short i = 0; i < length && i < 8; ++i) {
					System.out.print(" " + data[i]);
				}

				System.out.println("");
				System.out.println("");
			}
		});

		// Enable frame read callback
		can.enableFrameReadCallback();

		// Write standard data frame with identifier 1742 and 3 bytes of data
		short[] data = new short[]{42, 23, 17, 0, 0, 0, 0, 0};
		can.writeFrame(BrickletCAN.FRAME_TYPE_STANDARD_DATA, (short)1742, data, (short)3);

		System.out.println("Press key to exit"); System.in.read();
		can.disableFrameReadCallback();
		ipcon.disconnect();
	}
}
