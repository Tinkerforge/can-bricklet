#!/usr/bin/env ruby
# -*- ruby encoding: utf-8 -*-

require 'tinkerforge/ip_connection'
require 'tinkerforge/bricklet_can'

include Tinkerforge

HOST = 'localhost'
PORT = 4223
UID = 'XYZ' # Change XYZ to the UID of your CAN Bricklet

ipcon = IPConnection.new # Create IP connection
can = BrickletCAN.new UID, ipcon # Create device object

ipcon.connect HOST, PORT # Connect to brickd
# Don't use device before ipcon is connected

# Configure transceiver for loopback mode
can.set_configuration BrickletCAN::BAUD_RATE_1000KBPS, \
                      BrickletCAN::TRANSCEIVER_MODE_LOOPBACK, 0

# Register frame read callback
can.register_callback(BrickletCAN::CALLBACK_FRAME_READ) do |frame_type, identifier, data,
                                                            length|
  if frame_type == BrickletCAN::FRAME_TYPE_STANDARD_DATA
    puts "Frame Type: Standard Data"
  elsif frame_type == BrickletCAN::FRAME_TYPE_STANDARD_REMOTE
    puts "Frame Type: Standard Remote"
  elsif frame_type == BrickletCAN::FRAME_TYPE_EXTENDED_DATA
    puts "Frame Type: Extended Data"
  elsif frame_type == BrickletCAN::FRAME_TYPE_EXTENDED_REMOTE
    puts "Frame Type: Extended Remote"
  end

  puts "Identifier: #{identifier}"
  puts "Data (Length: #{length}): #{data[0, [length, 8].min].join(' ')}"
  puts ''
end

# Enable frame read callback
can.enable_frame_read_callback

# Write standard data frame with identifier 1742 and 3 bytes of data
can.write_frame BrickletCAN::FRAME_TYPE_STANDARD_DATA, 1742, [42, 23, 17, 0, 0, 0, 0, 0], 3

puts 'Press key to exit'
$stdin.gets

can.disable_frame_read_callback

ipcon.disconnect
