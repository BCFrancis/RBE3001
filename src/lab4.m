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
    
    %Arb 1 - -60,30,15 (60,-104.5,110.9)
    %Arb 2 - 0,30,15 (120.7,0,110.9)
    %Arb 3 - 70,75,-70 (67.1,184.5,112.2)
    positions = [55 100 45; 100 0 195; 150 -100 75; 55 100 45];
    planner = Traj_Planner(positions);
    recorded_linear = zeros(50, 4);
    recorded_angular = zeros(50, 4);
    
    j = 1;

    tic
    %Start Programming here
    for i=1:size(positions)
        q0 = pp.measured_cp();
        
        % cubic trajectories for each joint
        c1 = planner.quintic_traj(0,1.5,q0(1, 4),positions(i, 1),0,0,0,0);
        c2 = planner.quintic_traj(0,1.5,q0(2, 4),positions(i, 2),0,0,0,0);
        c3 = planner.quintic_traj(0,1.5,q0(3, 4),positions(i, 3),0,0,0,0);

        it = toc;
        dist = 100000;
        while dist > 10
            
            qs = pp.measured_js(1,1);
            velocities = pp.fdk3001(qs);
            
            recorded_linear(j, :) = [velocities(1:3)' toc];
            recorded_angular(j, :) = [velocities(4:6)' toc];
            
            angles = pp.get_euler();
            tip = pp.measured_cp();  
            
            j = j+1;
            dist = norm(tip(1:3, 4) - positions(i, :)');
            
            % set servo values based on next_angle
            angles = pp.ik3001([planner.next_angle_quintic(c1, toc-it), planner.next_angle_quintic(c2, toc-it), planner.next_angle_quintic(c3, toc-it)]);
            pp.servo_jp(SERV_ID, angles)
            
         
            model.plot_arm(angles)
            
            
        end
    end
    
    csvwrite('angular.csv', recorded_angular);
    csvwrite('linear.csv', recorded_linear);
    
catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end

% Clear up memory upon termination
pp.shutdown()

toc
