#!/bin/sh
# Connects to localhost:4223 by default, use --host and --port to change this

uid=XYZ # Change XYZ to the UID of your CAN Bricklet

# Configure transceiver for loopback mode
tinkerforge call can-bricklet $uid set-configuration 1000kbps loopback 0

# Handle incoming frame read callback
tinkerforge dispatch can-bricklet $uid frame-read &

# Enable frame read callback
tinkerforge call can-bricklet $uid enable-frame-read-callback

# Write standard data frame with identifier 1742 and 3 bytes of data
tinkerforge call can-bricklet $uid write-frame standard-data 1742 42,23,1,.. 3

echo "Press key to exit"; read dummy

tinkerforge call can-bricklet $uid disable-frame-read-callback

kill -- -$$ # Stop callback dispatch in background
