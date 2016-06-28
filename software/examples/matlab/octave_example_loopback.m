function octave_example_loopback()
    more off;

    HOST = "localhost";
    PORT = 4223;
    UID = "XYZ"; % Change to your UID

    ipcon = java_new("com.tinkerforge.IPConnection"); % Create IP connection
    can = java_new("com.tinkerforge.BrickletCAN", UID, ipcon); % Create device object

    ipcon.connect(HOST, PORT); % Connect to brickd
    % Don't use device before ipcon is connected

    % Configure transceiver for loopback mode
    can.setConfiguration(can.BAUD_RATE_1000KBPS, can.TRANSCEIVER_MODE_LOOPBACK, 0);

    % Register read callback to function cb_read
    can.addFrameReadCallback(@cb_frame_read);

    % Enable frame read callback
    can.enableFrameReadCallback();

    % Write standard data frame with identifier 1742 and 3 bytes of data
    data = [42, 23, 17, 0, 0, 0, 0, 0];
    can.writeFrame(can.FRAME_TYPE_STANDARD_DATA, 1742, data, 3);

    input("Press key to exit\n", "s");
    can.disableFrameReadCallback();
    ipcon.disconnect();
end

% Callback function for frame read callback
function cb_frame_read(e)
    data = e.data;
    len = java2int(e.length);

    fprintf("Frame Type: %d\n", java2int(e.frameType));
    fprintf("Identifier: %d\n", java2int(e.identifier));
    fprintf("Data (Length: %d):", len);

    for i = 1:min(len, 8)
        fprintf(" %d", java2int(data(i)));
    end

    fprintf("\n");
end

function int = java2int(value)
    if compare_versions(version(), "3.8", "<=")
        int = value.intValue();
    else
        int = value;
    end
end
