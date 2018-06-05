<?php

require_once('Tinkerforge/IPConnection.php');
require_once('Tinkerforge/BrickletCAN.php');

use Tinkerforge\IPConnection;
use Tinkerforge\BrickletCAN;

const HOST = 'localhost';
const PORT = 4223;
const UID = 'XYZ'; // Change XYZ to the UID of your CAN Bricklet

// Callback function for frame read callback
function cb_frameRead($frame_type, $identifier, $data, $length)
{
    echo "Frame Type: " . $frame_type . "\n";
    echo "Identifier: " . $identifier . "\n";
    echo "Data (Length: " . $length . "):";

    for ($i = 0; $i < $length && $i < 8; ++$i) {
        echo " " . $data[$i];
    }

    echo "\n";
    echo "\n";
}

$ipcon = new IPConnection(); // Create IP connection
$can = new BrickletCAN(UID, $ipcon); // Create device object

$ipcon->connect(HOST, PORT); // Connect to brickd
// Don't use device before ipcon is connected

// Configure transceiver for loopback mode
$can->setConfiguration(BrickletCAN::BAUD_RATE_1000KBPS,
                       BrickletCAN::TRANSCEIVER_MODE_LOOPBACK, 0);

// Register frame read callback to function cb_frameRead
$can->registerCallback(BrickletCAN::CALLBACK_FRAME_READ, 'cb_frameRead');

// Enable frame read callback
$can->enableFrameReadCallback();

// Write standard data frame with identifier 1742 and 3 bytes of data
$data = [42, 23, 17, 0, 0, 0, 0, 0];
$can->writeFrame(BrickletCAN::FRAME_TYPE_STANDARD_DATA, 1742, $data, 3);

echo "Press ctrl+c to exit\n";
$ipcon->dispatchCallbacks(-1); // Dispatch callbacks forever

?>
