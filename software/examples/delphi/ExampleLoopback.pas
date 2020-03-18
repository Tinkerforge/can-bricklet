program ExampleLoopback;

{$ifdef MSWINDOWS}{$apptype CONSOLE}{$endif}
{$ifdef FPC}{$mode OBJFPC}{$H+}{$endif}

uses
  SysUtils, IPConnection, BrickletCAN;

type
  TExample = class
  private
    ipcon: TIPConnection;
    can: TBrickletCAN;
  public
    procedure FrameReadCB(sender: TBrickletCAN; const frameType: byte;
                          const identifier: longword; const data: TArray0To7OfUInt8;
                          const length_: byte);
    procedure Execute;
  end;

const
  HOST = 'localhost';
  PORT = 4223;
  UID = 'XYZ'; { Change XYZ to the UID of your CAN Bricklet }

var
  e: TExample;

{ Callback procedure for frame read callback }
procedure TExample.FrameReadCB(sender: TBrickletCAN; const frameType: byte;
                               const identifier: longword; const data: TArray0To7OfUInt8;
                               const length_: byte);
var i: integer;
begin
  if (frameType = BRICKLET_CAN_FRAME_TYPE_STANDARD_DATA) then begin
    WriteLn('Frame Type: Standard Data');
  end
  else if (frameType = BRICKLET_CAN_FRAME_TYPE_STANDARD_REMOTE) then begin
    WriteLn('Frame Type: Standard Remote');
  end
  else if (frameType = BRICKLET_CAN_FRAME_TYPE_EXTENDED_DATA) then begin
    WriteLn('Frame Type: Extended Data');
  end
  else if (frameType = BRICKLET_CAN_FRAME_TYPE_EXTENDED_REMOTE) then begin
    WriteLn('Frame Type: Extended Remote');
  end;

  WriteLn(Format('Identifier: %d', [identifier]));
  Write(Format('Data (Length: %d):', [length_]));
  for i := 0 to (length_ - 1) do begin
    if i < 8 then begin
      Write(Format(' %d', [data[i]]));
    end;
  end;

  WriteLn('');
  WriteLn('');
end;

procedure TExample.Execute;
var data: TArray0To7OfUInt8;
begin
  { Create IP connection }
  ipcon := TIPConnection.Create;

  { Create device object }
  can := TBrickletCAN.Create(UID, ipcon);

  { Connect to brickd }
  ipcon.Connect(HOST, PORT);
  { Don't use device before ipcon is connected }

  { Configure transceiver for loopback mode }
  can.SetConfiguration(BRICKLET_CAN_BAUD_RATE_1000KBPS,
                       BRICKLET_CAN_TRANSCEIVER_MODE_LOOPBACK, 0);

  { Register frame read callback to procedure FrameReadCB }
  can.OnFrameRead := {$ifdef FPC}@{$endif}FrameReadCB;

  { Enable frame read callback }
  can.EnableFrameReadCallback;

  { Write standard data frame with identifier 1742 and 3 bytes of data }
  data[0] := 42; data[1] := 23; data[2] := 17;
  can.WriteFrame(BRICKLET_CAN_FRAME_TYPE_STANDARD_DATA, 1742, data, 3);

  WriteLn('Press key to exit');
  ReadLn;

  can.DisableFrameReadCallback;

  ipcon.Destroy; { Calls ipcon.Disconnect internally }
end;

begin
  e := TExample.Create;
  e.Execute;
  e.Destroy;
end.
