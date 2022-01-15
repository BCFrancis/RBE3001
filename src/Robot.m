classdef Robot < handle
    
    properties
        myHIDSimplePacketComs;
        pol; 
        GRIPPER_ID = 1962
        curr_goal = zeros(1,3)
    end
    
    methods
          
        %The is a shutdown function to clear the HID hardware connection
        function  shutdown(robot)
	    %Close the device
            robot.myHIDSimplePacketComs.disconnect();
        end
        
        % Create a packet processor for an HID device with USB PID 0x007
        function robot = Robot(dev)
            robot.myHIDSimplePacketComs=dev; 
            robot.pol = java.lang.Boolean(false);
        end
        
        %Perform a command cycle. This function will take in a command ID
        %and a list of 32 bit floating point numbers and pass them over the
        %HID interface to the device, it will take the response and parse
        %them back into a list of 32 bit floating point numbers as well
        function com = command(robot, idOfCommand, values)
                com= zeros(15, 1, 'single');
                try
                    ds = javaArray('java.lang.Double',length(values));
                    for i=1:length(values)
                        ds(i)= java.lang.Double(values(i));
                    end
                    % Default packet size for HID
                    intid = java.lang.Integer(idOfCommand);
                    robot.myHIDSimplePacketComs.writeFloats(intid,  ds);
                    ret = 	robot.myHIDSimplePacketComs.readFloats(intid) ;
                    for i=1:length(com)
                       com(i)= ret(i).floatValue();
                    end
                catch exception
                    getReport(exception)
                    disp('Command error, reading too fast');
                end
        end
        
        function com = read(robot, idOfCommand)
                com= zeros(15, 1, 'single');
                try

                    % Default packet size for HID
                    intid = java.lang.Integer(idOfCommand);
                    ret = 	robot.myHIDSimplePacketComs.readFloats(intid) ;
                    for i=1:length(com)
                       com(i)= ret(i).floatValue();
                    end
                catch exception
                  getReport(exception)
                    disp('Command error, reading too fast');
                end
        end
        
        function  write(robot, idOfCommand, values)
                try
                    ds = javaArray('java.lang.Double',length(values));
                    for i=1:length(values)
                        ds(i)= java.lang.Double(values(i));
                    end
                    % Default packet size for HID
                    intid = java.lang.Integer(idOfCommand);
                    robot.myHIDSimplePacketComs.writeFloats(intid,  ds,robot.pol);

                catch exception
                    getReport(exception)
                    disp('Command error, reading too fast');
                end
        end
        
        % Specifies a position to the gripper
        function writeGripper(robot, value)
            try
                ds = javaArray('java.lang.Byte',length(1));
                ds(1)= java.lang.Byte(value);
                intid = java.lang.Integer(robot.GRIPPER_ID);
                robot.myHIDSimplePacketComs.writeBytes(intid, ds, robot.pol);
            catch exception
                getReport(exception)
                disp('Command error, reading too fast');
            end
        end
        
        % Opens the gripper
        function openGripper(robot)
            robot.writeGripper(180);
        end
        
        % Closes the gripper
        function closeGripper(robot)
            robot.writeGripper(0);
        end
                
       
       
        % moves the servos to a specific set of positions
        function servo_jp(robot, s_id, targets)
            packet = zeros(15, 1, 'single');
            packet(1) = 0; % one second time
            packet(2) = 0; % linear interpolation
            packet(3) = targets(1); % First link
            packet(4) = targets(2); % Second link
            packet(5) = targets(3); % Third link
            robot.curr_goal = [packet(3),packet(4),packet(5)];
            % Write the packet
            robot.write(s_id, packet);
            return
        end
        
        % moves the servos to a specific set of positions and change
        % interpolation value
        function interpolate_jp(robot, s_id, targets, time)
            packet = zeros(15, 1, 'single');
            packet(1) = time; % time in ms
            packet(2) = 0; % linear interpolation
            packet(3) = targets(1); % First link
            packet(4) = targets(2); % Second link
            packet(5) = targets(3); % Third link
            robot.curr_goal = [packet(3),packet(4),packet(5)];
            % Write the packet
            robot.write(s_id, packet);
            return
        end
       
        % gets the position and velocity of the arm if requested
        function out = measured_js(robot, getpos, getvel)
            out = [0 0 0; 0 0 0];
            if getpos
                pos = robot.read(1910);
                out(1) = pos(3);
                out(3) = pos(5);
                out(5) = pos(7);
            end 
            
            if getvel
                pos = robot.read(1822);
                out(2) = pos(3);
                out(4) = pos(6);
                out(6) = pos(9);
            end
        end

        %reutrns euler angles
        function angles= get_euler(robot)
            curr = robot.measured_js(1,1);
            angles = curr(1,:);
        end
        
        %Returns joint setpoints in degrees
        function setpoint_array = setpoint_js(robot)
            %disp('Angle setpoints');
            temp_arr = robot.read(1910);
            setpoint1 = temp_arr(2);
            setpoint2 = temp_arr(4);
            setpoint3 = temp_arr(6);
            setpoint_array = [setpoint1,setpoint2,setpoint3];
        end
        
        %returns final goal array 
        function goal_array = goal_js(robot)
            goal_array = robot.curr_goal;
        end
        
        %Return the 4x4 transformation matrix for one row of a DH table
        %given the parameters
        function T = dh2mat(robot, dhvalues)
            T =[cosd(dhvalues(1)),-sind(dhvalues(1))*cosd(dhvalues(4)),sind(dhvalues(1))*sind(dhvalues(4)),dhvalues(3)*cosd(dhvalues(1));
            sind(dhvalues(1)),cosd(dhvalues(1))*cosd(dhvalues(4)),-cosd(dhvalues(1))*sind(dhvalues(4)),dhvalues(3)*sind(dhvalues(1));
            0,sind(dhvalues(4)),cosd(dhvalues(4)),dhvalues(2);
            0,0,0,1];
        end
       
        %Takes the DH-table and returns the final transformation matrix
        function T = dh2fk(robot, dhtable)
            s = size(dhtable);
            T = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];
            for i = 1:s(1)
                T1 = robot.dh2mat(dhtable(i,:));
                T = T * T1;
            end  
        end
        
        %Takes the DH-table and returns the final transformation matrix
        function jt = dh2fk_forPlot(robot, dhtable)
             jt=zeros(4,4,4);
            s = size(dhtable);
            T = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];
            for i = 1:s(1)
                T1 = robot.dh2mat(dhtable(i,:));
                T = T * T1;
                jt(:,:,i) = T;
            end  
        end 
        
        % Function to create the DH parameters for our case (3x1)
        function rot=fk3001(robot, angles)
            % angles is an nx1 array of angle positions (theta_star 1, 2,
            % ...)
            % base to link 1
            dh = [  0               55      0       0; 
                    angles(1)       40      0       -90; 
                    angles(2)-90  0       100     0; 
                    angles(3)+90  0       100     0 ];
                
            rot = robot.dh2fk(dh);
        end      
        
        % Function to create the DH parameters for our case (3x1)
        function rot=fk3001_forPlot(robot, angles)
            % angles is an nx1 array of angle positions (theta_star 1, 2,
            % ...)
            % base to link 1
            dh = [  0               55      0       0; 
                    angles(1)       40      0       -90; 
                    angles(2)-90  0       100     0; 
                    angles(3)+90  0       100     0 ];
                
            rot = robot.dh2fk_forPlot(dh);
        end
        
        %Takes the current joint space and returns the transformation
        %matrix relative to the zero position for the current position
        function T = measured_cp(robot)
            curr_pos = robot.measured_js(1,0);
            curr_pos = curr_pos(1,:);
            T = robot.fk3001(curr_pos);
        end
        
        %Takes the current setpoint and returns the transformation
        %matrix required to get there from the zero position
        function T = setpoint_cp(robot)
            T = robot.fk3001(robot.setpoint_js());
        end
        
       %Takes the current goal setpoint and returns the transformation
        %matrix required to get there from the zero position
        function T=goal_cp(robot)
            T = robot.fk3001(robot.goal_js());
        end   
        
        %inverse kinematics function
        %pos is 3x1 position vector 
        function T = ik3001(robot, pos)
            try
                a1 = 100;
                a2 = 100;
                d1 = 40;
                xc = pos(1);
                yc = pos(2);
                zc = pos(3) - 55;

                s = zc - d1;
                r = sqrt(xc^2 + yc^2);

                D3 = (a1^2 + a2^2 - (r^2 + s^2))/(2*a1*a2);
                C3 = sqrt(1 - D3^2);

                theta3(1) = -(atan2d(C3,D3) - 90); %hardcoded this negative sign
                theta3(2) = -(atan2d(-C3,D3) - 90);
                %chose which one to use

                alpha = atan2d(s,r);
                D2 = (a1^2 + r^2 + s^2 - a2^2)/(2*a1* sqrt(r^2+s^2));
                C2 = sqrt(1 - D2^2);

                beta(1) = atan2d(C2,D2);
                beta(2) = atan2d(-C2,D2);
                %chose which one to use

                theta2(1) = 90-(alpha+beta(1));
                theta2(2) = 90-(alpha+beta(2));

                theta1(1) = atan2d(yc,xc);
                theta1(2) = atan2d(-yc,xc);
                %chose which one to use


                if (theta1(1) >= 0 & yc >= 0) | (theta1(1) <= 0 & yc <= 0)
                    T = [theta1(1) theta2(1) theta3(1)];
                else
                    T = [theta1(2) theta2(2) theta3(2)];
                end

                jail = 0;
                jail = sqrt(xc^2 + yc^2) < 20;
                if(sum(isnan(T))>0)
                    msg = "Point outside of range";
                    error(msg)
                end
                if jail
                    msg = "Point would self intersect";
                    error(msg)
                end
            catch e
                fprintf(1,'The identifier was:\n%s',e.identifier);
                fprintf(1,'There was an error! The message was:\n%s',e.message);
            end
                
        end
        
        
        function M = jacob3001(robot, q)
            m1 = 100* (cosd(q(2)+q(3)) + sind(q(2)));
            m2 = 100* (-sind(q(2)+q(3)) + cosd(q(2)));
            m3 = 100* -sind(q(2)+q(3));
            
            M = [-sind(q(1))*m1,     cosd(q(1))*m2,     cosd(q(1))*m3;
                cosd(q(1))*m1,       sind(q(1))*m2,      sind(q(1))*m3;
                0,                  -m1,                100*cosd(q(3)-q(2));
                0,                  -sind(q(1)),        -sind(q(1));
                0,                  cosd(q(1)),         cosd(q(1));
                1,                  0,                  0];
            
            
        end
        
        function M = fdk3001(robot, qs)
           q = qs(1,:);
           jacob = robot.jacob3001(q);
           M = jacob*(qs(2,:)');
            
        end 
    end
end