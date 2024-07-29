%advances car   
%write(udp, "A", 'uint8', '192.168.0.161', 8888);
%pause(5);
%turns car left
%write(udp, "L", 'uint8', '192.168.0.161', 8888);
%pause(5);
%turns car right
%write(udp, "R", 'uint8', '192.168.0.161', 8888);
%pause(5);
%backs up car
%write(udp, "B", 'uint8', '192.168.0.161', 8888);
%pause(5);
%write(udp, "E", 'uint8', '192.168.0.161', 8888);
clear;
clc;
Lspeed = num2str(0);
Rspeed = num2str(0);
udp = udpport;
doot = strcat("L"+Lspeed, "R"+Rspeed)
write(udp, "L"+Lspeed, 'uint8', '192.168.0.161', 8888);
write(udp, "R"+Rspeed, 'uint8', '192.168.0.161', 8888);

