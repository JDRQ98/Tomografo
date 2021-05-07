clear HC05
HC05 = bluetooth('Tomografo',1);
configureTerminator(HC05,"CR/LF");
%configureCallback(HC05,"byte",4,@ParseBluetooth);
%%
Cam = CameraController;
Cam.session.folder = 'D:\Tomografias\Test';
Cam.camera.isonumber = 100;
Cam.camera.fnumber = 10;
Cam.camera.shutterspeed = 1/4;
Cam.camera.compressionsetting = 'Large Fine JPEG';
Cam.camera.drive_mode = 'Single-Frame Shooting';
Cam.Capture();
%%
N = 64; %Number of images to take
writeline(HC05, "RESET.");
while(HC05.NumBytesAvailable < 4)
end
msg = read(HC05,HC05.NumBytesAvailable,"char");
pause(1);

writeline(HC05, append("MAX=", num2str(N), "."));
while(HC05.NumBytesAvailable < 4)
end
msg = read(HC05,HC05.NumBytesAvailable,"char");
pause(1);

for i = 1:N
    writeline(HC05, "STEP.")
    while(HC05.NumBytesAvailable < 4)
    end
    msg = read(HC05,HC05.NumBytesAvailable,"char");
    if (startsWith(msg,"OK"))
        Cam.Capture() %capture (set custom filename)
    end
end



%% FUNCTIONS

function TakePicture(Camera, ImgNumber)
name = append("PIC_" + num2str(ImgNumber));
Camera.Capture(append("PIC_" + num2str(ImgNumber))) %capture (set custom filename)
pause(1)
end

function Ack(bluetoothdevice)
writeline(bluetoothdevice,"OK");
end

