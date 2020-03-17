package main

import (
	"fmt"
	"github.com/Tinkerforge/go-api-bindings/can_bricklet"
	"github.com/Tinkerforge/go-api-bindings/ipconnection"
)

const ADDR string = "localhost:4223"
const UID string = "XYZ" // Change XYZ to the UID of your CAN Bricklet.

func main() {
	ipcon := ipconnection.New()
	defer ipcon.Close()
	can, _ := can_bricklet.New(UID, &ipcon) // Create device object.

	ipcon.Connect(ADDR) // Connect to brickd.
	defer ipcon.Disconnect()
	// Don't use device before ipcon is connected.

	// Configure transceiver for loopback mode
	can.SetConfiguration(can_bricklet.BaudRate1000kbps,
		can_bricklet.TransceiverModeLoopback, 0)

	can.RegisterFrameReadCallback(func(frameType can_bricklet.FrameType, identifier uint32, data [8]uint8, length uint8) {
		if frameType == can_bricklet.FrameTypeStandardData {
			fmt.Println("Frame Type: Standard Data")
		} else if frameType == can_bricklet.FrameTypeStandardRemote {
			fmt.Println("Frame Type: Standard Remote")
		} else if frameType == can_bricklet.FrameTypeExtendedData {
			fmt.Println("Frame Type: Extended Data")
		} else if frameType == can_bricklet.FrameTypeExtendedRemote {
			fmt.Println("Frame Type: Extended Remote")
		}

		fmt.Printf("Identifier: %d\n", identifier)
		fmt.Printf("Data (Length: %d):", length)

		for _, item := range data {
			fmt.Printf(" %d", item)
		}

		fmt.Println()
		fmt.Println()
	})

	// Enable frame read callback
	can.EnableFrameReadCallback()

	data := [8]uint8{42, 23, 17, 0, 0, 0, 0, 0}
	can.WriteFrame(can_bricklet.FrameTypeStandardData, 1742, data, 3)

	fmt.Print("Press enter to exit.")
	fmt.Scanln()

	can.DisableFrameReadCallback()
}
