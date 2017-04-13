using System;
using Tinkerforge;

class Example
{
	private static string HOST = "localhost";
	private static int PORT = 4223;
	private static string UID = "XYZ"; // Change XYZ to the UID of your CAN Bricklet

	// Callback function for frame read callback
	static void FrameReadCB(BrickletCAN sender, byte frameType,
	                        long identifier, byte[] data, byte length)
	{
		Console.WriteLine("Frame Type: " + frameType);
		Console.WriteLine("Identifier: " + identifier);

		string s = "Data (Length: " + length + "):";
		for (int i = 0; i < length && i < 8; ++i) {
			s += " " + data[i];
		}

		Console.WriteLine(s);
	}

	static void Main()
	{
		IPConnection ipcon = new IPConnection(); // Create IP connection
		BrickletCAN can = new BrickletCAN(UID, ipcon); // Create device object

		ipcon.Connect(HOST, PORT); // Connect to brickd
		// Don't use device before ipcon is connected

		// Configure transceiver for loopback mode
		can.SetConfiguration(BrickletCAN.BAUD_RATE_1000KBPS,
		                     BrickletCAN.TRANSCEIVER_MODE_LOOPBACK, 0);

		// Register frame read callback to function FrameReadCB
		can.FrameReadCallback += FrameReadCB;

		// Enable frame read callback
		can.EnableFrameReadCallback();

		// Write standard data frame with identifier 1742 and 3 bytes of data
		byte[] data = new byte[8]{42, 23, 17, 0, 0, 0, 0, 0};
		can.WriteFrame(BrickletCAN.FRAME_TYPE_STANDARD_DATA, 1742, data, 3);

		Console.WriteLine("Press enter to exit");
		Console.ReadLine();
		can.DisableFrameReadCallback();
		ipcon.Disconnect();
	}
}
