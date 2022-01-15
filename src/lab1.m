%%
% RBE3001 - Laboratory 1
%
% Instructions
% ------------
% Welcome again! This MATLAB script is your starting point for Lab
% 1 of RBE3001. The sample code below demonstrates how to establish
% communication between this script and the Nucleo firmware, send
% setpoint commands and receive sensor data.
%
% IMPORTANT - understanding the code below requires being familiar
% with the Nucleo firmware. Read that code first.

% Lines 15-37 perform necessary library initializations. You can skip reading
% to line 38.
clear
clear java
clear classes;

vid = hex2dec('16c0');
pid = hex2dec('0486');

disp (vid);
disp (pid);

javaaddpath ../lib/SimplePacketComsJavaFat-0.6.4.jar;
import edu.wpi.SimplePacketComs.*;
import edu.wpi.SimplePacketComs.device.*;
import edu.wpi.SimplePacketComs.phy.*;
import java.util.*;
import org.hid4java.*;
version -java
myHIDSimplePacketComs=HIDfactory.get();
myHIDSimplePacketComs.setPid(pid);
myHIDSimplePacketComs.setVid(vid);
myHIDSimplePacketComs.connect();

% Create a PacketProcessor object to send data to the nucleo firmware
pp = Robot(myHIDSimplePacketComs);
try
 
    SERV_ID = 1848;            % we will be talking to server ID 1848 on
    % the Nucleo
    SERVER_ID_READ =1910; % ID of the read packet
    DEBUG   = false;          % enables/disables debug prints
    
    % Instantiate a packet - the following instruction allocates 60
    % bytes for this purpose. Recall that the HID interface supports
    % packet sizes up to 64 bytes.
    packet = zeros(15, 1, 'single');
    joint_pos = zeros(1000,3);
    timestamps = zeros(1000,1);
    timejumps = timestamps;
    pp.servo_jp(SERV_ID,[0,0,0]);
    pause(1);
    tic
    pp.interpolate_jp(SERV_ID,[45,0,0],3000);
    curr_pos = pp.measured_js(1,1);
    row_counter=1;
    current_time=0;
    prev_time=0;
    while(abs(curr_pos(1)-45)>2)
       curr_pos = pp.measured_js(1,1);
       current_time = toc;
       timejumps(row_counter,:) = current_time-prev_time;
       timestamps(row_counter,:)= current_time;
       joint_pos(row_counter,:) = curr_pos(1,:);
       row_counter = row_counter + 1;
       prev_time = current_time;
    end
    combo_matrix = [joint_pos timestamps];
    writematrix(combo_matrix,'lab1.csv');
    tiledlayout(3,1)
    nexttile
    plot(timestamps,joint_pos(:,1));
    hold on
    plot(timestamps,joint_pos(:,2));
    plot(timestamps,joint_pos(:,3))
    xlabel("Time (s)");
    ylabel("Angle (deg)");
    title("Joint Positions During Movement");
    legend("Base Angle","Shoulder Angle","Wrist Angle");
    hold off
    nexttile
    histogram(timejumps)
    xlabel("Time step size (s)");
    ylabel("Frequency");
    title("Frequency of Time Increments");
    nexttile
    histogram(timejumps)
    xlim([0,.005])
    xlabel("Time step size (s)");
    ylabel("Frequency");
    title("Frequency of Time Increments(without outliers)");
    
    % Closes then opens the gripper
    %pp.closeGripper()
    %pause(1)
    %pp.openGripper()
    
catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end

% Clear up memory upon termination
pp.shutdown()

toc
