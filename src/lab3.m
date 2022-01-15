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
    positions = [55 100 45; 100 0 195; 150 -100, 75; 55 100 45];
    planner = Traj_Planner(positions);
    recorded_tip = zeros(50, 4);
    recorded_angles = zeros(50, 4);
    recorded_velocities = zeros(50, 4);
    recorded_accelerations = zeros(50, 4);

    
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
            angles = pp.get_euler();
            tip = pp.measured_cp();  
            recorded_angles(j, :) = [angles toc];
            recorded_tip(j, :) = [tip(1:3, 4)' toc];
            if j > 1
                curr_tip = recorded_tip(j,:);
                previous_tip = recorded_tip(j-1,:);
                recorded_velocities(j, :) = [(curr_tip(1)-previous_tip(1))/(curr_tip(4)-previous_tip(4)) (curr_tip(2)-previous_tip(2))/(curr_tip(4)-previous_tip(4)) (curr_tip(3)-previous_tip(3))/(curr_tip(4)-previous_tip(4)) toc];
            end
            if j > 2
                curr_velo = recorded_velocities(j,:);
                previous_velo = recorded_velocities(j-1,:);
                recorded_accelerations(j, :) = [(curr_velo(1)-previous_velo(1))/(curr_velo(4)-previous_velo(4)) (curr_velo(2)-previous_velo(2))/(curr_velo(4)-previous_velo(4)) (curr_velo(3)-previous_velo(3))/(curr_velo(4)-previous_velo(4)) toc];
            end
            j = j+1;
            dist = norm(tip(1:3, 4) - positions(i, :)');
            
            % set servo values based on next_angle
            angles = pp.ik3001([planner.next_angle_quintic(c1, toc-it), planner.next_angle_quintic(c2, toc-it), planner.next_angle_quintic(c3, toc-it)]);
            pp.servo_jp(SERV_ID, angles)
        end
    end
    
    csvwrite('tip_position.csv', recorded_tip)
    csvwrite('angles.csv', recorded_angles)
    csvwrite('velocities.csv', recorded_velocities)
    csvwrite('accelerations.csv', recorded_velocities)

    model.plot_tip(recorded_tip);
    figure()
    model.plot_angles(recorded_angles);
    figure()
    model.plot_velo(recorded_velocities);
    figure()
    model.plot_accel(recorded_accelerations);
    figure()
    
    plot3(recorded_tip(:, 1), recorded_tip(:, 2), recorded_tip(:, 3))
    title("Arm Path")
    xlabel("X Axis")
    ylabel("Y Axis")
    zlabel("Z Axis")
catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end

% Clear up memory upon termination
pp.shutdown()

toc
