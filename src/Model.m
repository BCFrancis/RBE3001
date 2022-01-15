classdef Model
    %Class to display models of the robot
    %   This class contains the different methods that can be called to
    %   create/visualize the 
    
    properties
        pp
    end
    
    methods
        
        function self = Model(robot)
            self.pp = robot;
        end
             
        % Plot the tip positions vs time (x, y, and z)
        function plot_tip(self, recorded_tip)
            hold offangular
            plot(recorded_tip(:, 4), recorded_tip(:, 1))
            hold on
            plot(recorded_tip(:, 4), recorded_tip(:, 2))
            plot(recorded_tip(:, 4), recorded_tip(:, 3))
            legend("x", "y", "z")
            xlabel("Time (s)")
            ylabel("Pos (mm)")
            title("Tip Positions vs Time")
            hold off
        end
        
        % Plot the tip velocity vs time (x, y, and z)
        function plot_velo(self, recorded_velo)
            hold off
            plot(recorded_velo(:, 4), recorded_velo(:, 1))
            hold on
            plot(recorded_velo(:, 4), recorded_velo(:, 2))
            plot(recorded_velo(:, 4), recorded_velo(:, 3))
            legend("x", "y", "z")
            xlabel("Time (s)")
            ylabel("Velocity (mm/s)")
            title("Tip Velocity vs Time")
            hold off
        end
        
        % Plot the magnitude of the velocity vs time
        function plot_velo_mag(self, recorded_velo)
            hold off
            mags = sqrt(sum(recorded_velo(:, 1:3).^2,2));
            plot(recorded_velo(:, 4), mags)
            hold on
            xlabel("Time (s)")
            ylabel("Velocity (mm/s)")
            title("Tip Velocity Magnitude vs Time")
            hold off
        end
        
        % Plot the tip acceleration vs time (x, y, and z)
        function plot_accel(self, recorded_accel)
            hold off
            plot(recorded_accel(:, 4), recorded_accel(:, 1))
            hold on
            plot(recorded_accel(:, 4), recorded_accel(:, 2))
            plot(recorded_accel(:, 4), recorded_accel(:, 3))
            legend("x", "y", "z")
            xlabel("Time (s)")
            ylabel("Acceleration (mm/s^2)")
            title("Tip Velocity vs Time")
            hold off
        end
        
        function plot_angles(self, recorded_angles)
            hold off
            plot(recorded_angles(:, 4), recorded_angles(:, 1))
            hold on
            plot(recorded_angles(:, 4), recorded_angles(:, 2))
            plot(recorded_angles(:, 4), recorded_angles(:, 3))
            legend("Base", "Elbow", "Wrist")
            xlabel("Time (s)")
            ylabel("Angle (deg)")
            title("Angles vs Time")
            hold off
        end
        
        %Creates a stick plot that can be used as a 3D representation of
        %the configuration of the arm from        
        function  plot_arm(self, euler_angles)
           %Turns vectors of each point into a 3d line that represents the
            %robot 
            plot = animatedline('MaximumNumPoints', 5, 'Marker', '.');
            clearpoints(plot);
          
            view(3);
            xlabel("X Axis");
            ylabel("Y Axis");
            zlabel("Z Axis");
            hold on
            grid on
            frame0 = animatedline('MaximumNumPoints', 6, 'Marker', 'd','Color','red');
            x0 = [0,30,0,0,0,0];
            y0 =[0,0,0,30,0,0];
            z0 =[0,0,0,0,0,30];
            addpoints(frame0,x0,y0,z0);
            frame1 = animatedline('MaximumNumPoints', 6, 'Marker', 'd','Color','blue');
            frame2 = animatedline('MaximumNumPoints', 6, 'Marker', 'd','Color','green');
            frame3 = animatedline('MaximumNumPoints', 6, 'Marker', 'd','Color','magenta');
            frame4 = animatedline('MaximumNumPoints', 6, 'Marker', 'd','Color','yellow');
            velocity = animatedline('MaximumNumPoints', 2, 'Marker', '^','Color','black');
            frames=[frame1,frame2,frame3,frame4];
            Xs = zeros(1,5);
            Ys = Xs;
            Zs = Xs;
            %transforms =  self.fk3001_forPlot(euler_angles);
            transforms =  self.pp.fk3001_forPlot(euler_angles);
            qs = self.pp.measured_js(1,1);
            velocities = self.pp.fdk3001(qs);
            for i=1:4
                point = transforms(:,:,i)*[0; 0; 0; 1];
                Xi = transforms(:,1,i);
                Yi= transforms(:,2,i);
                Zi= transforms(:,3,i);
                
                Xs(i+1)= point(1);
                Ys(i+1)= point(2);
                Zs(i+1)= point(3);
                xis = [point(1),point(1)+Xi(1)*30,point(1),point(1)+Yi(1)*30,point(1),point(1)+Zi(1)*30];
                yis =[point(2),point(2)+Xi(2)*30,point(2),point(2)+Yi(2)*30,point(2),point(2)+Zi(2)*30];
                zis =[point(3),point(3)+Xi(3)*30,point(3),point(3)+Yi(3)*30,point(3),point(3)+Zi(3)*30];
                addpoints(frames(i),xis,yis,zis);
                if i == 4
                   vvector = [ point(1),point(2),point(3); point(1)+velocities(1)/450,point(2)+velocities(2)/450,point(3)+velocities(3)/450];
                   addpoints(velocity,vvector(:,1),vvector(:,2),vvector(:,3));
                end
            end
            addpoints(plot,Xs,Ys,Zs);
            drawnow;
            clearpoints(plot);
            clearpoints(frame0);
            clearpoints(frame1);
            clearpoints(frame2);
            clearpoints(frame3);
            clearpoints(frame4);
            clearpoints(velocity);
            xlim([-200 ,200])
            ylim([-200,200])
            zlim([0,250])
            hold off
        end
                
    end
end

