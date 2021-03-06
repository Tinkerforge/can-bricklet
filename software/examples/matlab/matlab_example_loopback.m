function matlab_example_loopback()
    import com.tinkerforge.IPConnection;
    import com.tinkerforge.BrickletCAN;

    HOST = 'localhost';
    PORT = 4223;
    UID = 'XYZ'; % Change XYZ to the UID of your CAN Bricklet

    ipcon = IPConnection(); % Create IP connection
    can = handle(BrickletCAN(UID, ipcon), 'CallbackProperties'); % Create device object

    ipcon.connect(HOST, PORT); % Connect to brickd
    % Don't use device before ipcon is connected

    % Configure transceiver for loopback mode
    can.setConfiguration(BrickletCAN.BAUD_RATE_1000KBPS, ...
                         BrickletCAN.TRANSCEIVER_MODE_LOOPBACK, 0);

    % Register frame read callback to function cb_frame_read
    set(can, 'FrameReadCallback', @(h, e) cb_frame_read(e));

    % Enable frame read callback
    can.enableFrameReadCallback();

    % Write standard data frame with identifier 1742 and 3 bytes of data
    data = [42, 23, 17, 0, 0, 0, 0, 0];
    can.writeFrame(BrickletCAN.FRAME_TYPE_STANDARD_DATA, 1742, data, 3);

    input('Press key to exit\n', 's');

    can.disableFrameReadCallback();

    ipcon.disconnect();
end

% Callback function for frame read callback
function cb_frame_read(e)
    if e.frameType == com.tinkerforge.BrickletCAN.FRAME_TYPE_STANDARD_DATA
        fprintf('Frame Type: Standard Data\n');
    elseif e.frameType == com.tinkerforge.BrickletCAN.FRAME_TYPE_STANDARD_REMOTE
        fprintf('Frame Type: Standard Remote\n');
    elseif e.frameType == com.tinkerforge.BrickletCAN.FRAME_TYPE_EXTENDED_DATA
        fprintf('Frame Type: Extended Data\n');
    elseif e.frameType == com.tinkerforge.BrickletCAN.FRAME_TYPE_EXTENDED_REMOTE
        fprintf('Frame Type: Extended Remote\n');
    end

    fprintf('Identifier: %i\n', e.identifier);
    fprintf('Data (Length: %i):', e.length);

    for i = 1:min(e.length, 8)
        fprintf(' %i', e.data(i));
    end

    fprintf('\n');
    fprintf('\n');
end
