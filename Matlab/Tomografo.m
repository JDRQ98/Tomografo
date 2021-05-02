clear HC05
HC05 = bluetooth('Tomografo',1);
configureTerminator(HC05,"CR/LF");
%configureCallback(HC05,"byte",4,@ParseBluetooth);
%%
N = 16; %Number of images to take
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
       TakePicture(i);
    end
end



%% FUNCTIONS

function TakePicture(ImgNumber)
append("Taking picture " + num2str(ImgNumber))
pause(1)
end

function Ack(bluetoothdevice)
writeline(bluetoothdevice,"OK");
end

