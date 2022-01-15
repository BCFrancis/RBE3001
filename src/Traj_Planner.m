classdef Traj_Planner
    %Summary of this class goes here
    %class to create different types of trajectories

    properties
        points
    end
    
    methods
        function self  = Traj_Planner(points_array)
            %Construct an instance of this class
            %Takes array nx3 array of [x,y,z] values
            self.points = points_array;
        end
        
        function coefficients = cubic_traj(self,t0,tf,q0,qf,v0,vf)
            %   Creates a cubic trajectory for the given arguments (time,
            %   velocity and position
            M = [1,t0,t0^2,t0^3;
                    0,1,2*t0,3*(t0^2);
                    1,tf,tf^2,tf^3;
                    0,1,2*tf,3*tf^2];
           given = [q0,v0,qf,vf]';
           %coefficients = inv(M)*given;
           coefficients = M\given;
        end
        
        function angles = joint_planning(self,coeff,time,steps)
           t = time(2)-time(1);
           t = t/steps;
           angles = zeros(steps,1);
           for i=1:steps
               angles(i)=self.next_angle(coeff,t*i);
           end
        end
        
        function q = next_angle(self, coeff, time)
            q = coeff(1) + time*coeff(2) + time^2*coeff(3) + time^3*coeff(4);
        end
        
        function coefficients = quintic_traj(self,t0,tf,q0,qf,v0,vf,ac0,acf)
              %   Creates a quintic trajectory for the given arguments (time,
              %   velocity and position and acceleration
              M = [1,t0,t0^2,t0^3,t0^4,t0^5;
                    0,1,2*t0,3*(t0^2),4*(t0^3),5*(t0^4);
                    0,0,2,6*t0,12*(t0^2),20*(t0^3);
                    1,tf,tf^2,tf^3,tf^4,tf^5;
                    0,1,2*tf,3*(tf^2),4*(tf^3),5*(tf^4);
                    0,0,2,6*tf,12*(tf^2),20*(tf^3)];
              given = [q0,v0,ac0,qf,vf,acf]';
              coefficients = M\given;
        end
        
        function q = next_angle_quintic(self, coeff, time)
            q = coeff(1) + time*coeff(2) + time^2*coeff(3) + time^3*coeff(4) + time^4*coeff(5) + time^5*coeff(6);
        end
        
    end
end

