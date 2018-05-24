#!/usr/bin/perl

use strict;
use Tinkerforge::IPConnection;
use Tinkerforge::BrickletCAN;

use constant HOST => 'localhost';
use constant PORT => 4223;
use constant UID => 'XYZ'; # Change XYZ to the UID of your CAN Bricklet

# Callback subroutine for frame read callback
sub cb_frame_read
{
    my ($frame_type, $identifier, $data, $length) = @_;

    print "Frame Type: " . $frame_type . "\n";
    print "Identifier: " . $identifier . "\n";
    print "Data (Length: " . $length . "): " . join(" ", @{$data}[0..($length, 8)[$length > 8] - 1]) . "\n";
}

my $ipcon = Tinkerforge::IPConnection->new(); # Create IP connection
my $can = Tinkerforge::BrickletCAN->new(&UID, $ipcon); # Create device object

$ipcon->connect(&HOST, &PORT); # Connect to brickd
# Don't use device before ipcon is connected

# Configure transceiver for loopback mode
$can->set_configuration($can->BAUD_RATE_1000KBPS, $can->TRANSCEIVER_MODE_LOOPBACK, 0);

# Register frame read callback to subroutine cb_frame_read
$can->register_callback($can->CALLBACK_FRAME_READ, 'cb_frame_read');

# Enable frame read callback
$can->enable_frame_read_callback();

# Write standard data frame with identifier 1742 and 3 bytes of data
my $data = [42, 23, 17, 0, 0, 0, 0, 0];
$can->write_frame($can->FRAME_TYPE_STANDARD_DATA, 1742, $data, 3);

print "Press key to exit\n";
<STDIN>;
$can->disable_frame_read_callback();
$ipcon->disconnect();
