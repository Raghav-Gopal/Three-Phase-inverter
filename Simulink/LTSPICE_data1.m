function PMOS_A = LTSPICE_data1(freq,t,Midpoint_message,m)
Cf = 20000;
% Mf = 50;
Mf = freq;
% t = 0:0.0001:1;
% x=t;
PWMa = zeros(1,size(t,2));
PWMb = zeros(1,size(t,2));
carrier = sawtooth(2*pi*Cf*t,Midpoint_message);
message = m.*sin(2*pi*Mf*t);
inv_message = sin(2*pi*Mf*t+pi);
for i=1:1:size(t,2)
    if message(1,i)>carrier(1,i)
        PWMa(1,i) = 1;
    else 
        PWMa(1,i)=0;
    end
    if inv_message(1,i)>carrier(1,i)
        PWMb(1,i) = 1;
    else
        PWMb(1,i)=0;
    end
end
%plot(t,carrier);
PMOS_A = PWMa;
% PMOS_B = PWMb;
% NMOS_A = ~PWMa;
% NMOS_B = ~PWMb;
% Define file names for each voltage source
% filenames = {'PMOS_A.txt', 'PMOS_B.txt', 'NMOS_A.txt', 'NMOS_B.txt'};
% 
% % Data arrays
% data = {PMOS_A, PMOS_B, NMOS_A, NMOS_B};
% 
% % Loop through each signal and write to separate files
% for idx = 1:length(filenames)
%     fileID = fopen(filenames{idx}, 'w');
% 
%     % Write LTSpice header for clarity
%     % Write data in PWL format
%     for i = 1:length(t)
%         fprintf(fileID, '%f %d\n', t(i), data{idx}(i));
%     end
% 
%     % Close the file
%     fclose(fileID);
% 
%     fprintf('Data successfully exported to %s\n', filenames{idx});
% end
end