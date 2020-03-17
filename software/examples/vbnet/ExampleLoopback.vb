Imports System
Imports Tinkerforge

Module ExampleLoopback
    Const HOST As String = "localhost"
    Const PORT As Integer = 4223
    Const UID As String = "XYZ" ' Change XYZ to the UID of your CAN Bricklet

    ' Callback subroutine for frame read callback
    Sub FrameReadCB(ByVal sender As BrickletCAN, ByVal frameType As Byte, _
                    ByVal identifier As Long, ByVal data As Byte(), _
                    ByVal length As Byte)
        Console.WriteLine("Frame Type: {0}", frameType)
        Console.WriteLine("Identifier: {0}", identifier)
        Console.Write("Data (Length: {0}):", length)

        Dim i As Integer
        For i = 0 To Math.Min(length - 1, 7)
            Console.Write(" " + data(i).ToString())
        Next i

        Console.WriteLine("")
        Console.WriteLine("")
    End Sub

    Sub Main()
        Dim ipcon As New IPConnection() ' Create IP connection
        Dim can As New BrickletCAN(UID, ipcon) ' Create device object

        ipcon.Connect(HOST, PORT) ' Connect to brickd
        ' Don't use device before ipcon is connected

        ' Configure transceiver for loopback mode
        can.SetConfiguration(BrickletCAN.BAUD_RATE_1000KBPS, _
                             BrickletCAN.TRANSCEIVER_MODE_LOOPBACK, 0)

        ' Register frame read callback to subroutine FrameReadCB
        AddHandler can.FrameReadCallback, AddressOf FrameReadCB

        ' Enable frame read callback
        can.EnableFrameReadCallback()

        ' Write standard data frame with identifier 1742 and 3 bytes of data
        Dim data As Byte() = {42, 23, 17, 0, 0, 0, 0, 0}
        can.WriteFrame(BrickletCAN.FRAME_TYPE_STANDARD_DATA, 1742, data, 3)

        Console.WriteLine("Press key to exit")
        Console.ReadLine()

        can.DisableFrameReadCallback()

        ipcon.Disconnect()
    End Sub
End Module
