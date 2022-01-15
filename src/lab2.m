%%
% RBE3001 - Laboratory 2
%
% Lines 15-37 perform necessary library initializations. You can skip reading
% to line 38.
clear
clear java;
clear classes;

vid = hex2dec('16c0');
pid = hex2dec('0486');

%disp (vid);
%disp (pid);

javaaddpath ../lib/SimplePacketComsJavaFat-0.6.4.jar;
import edu.wpi.SimplePacketComs.*;
import edu.wpi.SimplePacketComs.device.*;
import edu.wpi.SimplePacketComs.phy.*;
import java.util.*;
import org.hid4java.*;
version -java;
myHIDSimplePacketComs=HIDfactory.get();
myHIDSimplePacketComs.setPid(pid);
myHIDSimplePacketComs.setVid(vid);
myHIDSimplePacketComs.connect();

% Create a PacketProcessor object to send data to the nucleo firmware
pp = Robot(myHIDSimplePacketComs);
model = Model(pp);
try
 
    SERV_ID = 1848;            % we will be talking to server ID 1848 on
    % the Nucleo
    SERVER_ID_READ =1910; % ID of the read packet
    DEBUG   = false;          % enables/disables debug prints
    
    tic
    
    %Start Programming here
    positions =[100,100,50;120,90,40;80,-40,100];
    planner = Traj_Planner(positions);
    planner.cubic_traj(0,1,10,-20,0,0)
    
catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end

% Clear up memory upon termination
pp.shutdown()

toc
