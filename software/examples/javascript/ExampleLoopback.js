var Tinkerforge = require('tinkerforge');

var HOST = 'localhost';
var PORT = 4223;
var UID = 'XYZ'; // Change XYZ to the UID of your CAN Bricklet

var ipcon = new Tinkerforge.IPConnection(); // Create IP connection
var can = new Tinkerforge.BrickletCAN(UID, ipcon); // Create device object

ipcon.connect(HOST, PORT,
    function (error) {
        console.log('Error: ' + error);
    }
); // Connect to brickd
// Don't use device before ipcon is connected

ipcon.on(Tinkerforge.IPConnection.CALLBACK_CONNECTED,
    function (connectReason) {
        // Configure transceiver for loopback mode
        can.setConfiguration(Tinkerforge.BrickletCAN.BAUD_RATE_1000KBPS,
                             Tinkerforge.BrickletCAN.TRANSCEIVER_MODE_LOOPBACK, 0);

        // Enable frame read callback
        can.enableFrameReadCallback();

        // Write standard data frame with identifier 1742 and 3 bytes of data
        var data = [42, 23, 17, 0, 0, 0, 0, 0];
        can.writeFrame(Tinkerforge.BrickletCAN.FRAME_TYPE_STANDARD_DATA, 1742, data, 3);
    }
);

// Register frame read callback
can.on(Tinkerforge.BrickletCAN.CALLBACK_FRAME_READ,
    // Callback function for frame read callback
    function (frameType, identifier, data, len) {
        console.log('Frame Type: ' + frameType);
        console.log('Identifier: ' + identifier);

        var str = 'Data (Length: ' + len + '):';
        for (var i = 0; i < len && i < 8; ++i) {
            str += ' ' + data[i];
        }

        console.log(str);
        console.log();
    }
);

console.log('Press key to exit');
process.stdin.on('data',
    function (data) {
        can.disableFrameReadCallback();
        ipcon.disconnect();
        process.exit(0);
    }
);
