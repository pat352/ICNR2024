
classdef MyRobot < handle
% Laura Ferrante
    properties(SetAccess = public, GetAccess = public)
        DataIn = cell(0);
  
        s0;
        s;
        myrobot;
        maxemg1;
        maxemg2; % other param useful to save for analysis
    end
    properties(SetAccess = protected, GetAccess = public)
        DataOut = cell(0);      % Publish the handle to the result of processing in here
    end
    methods(Access = public)
        function obj = MyRobot()
            % Define the robot model
            % (more examples: https://github.com/petercorke/robotics-toolbox-matlab/tree/master/models)
            obj.s0 = [0 0 0]; % initial state (q,dq,ddq)
            obj.s = obj.s0; % current state
            
            % One-link robot, with a revolute joint.
            % Parameter values use SI units: meter,radians,kg...
            obj.myrobot = SerialLink([Revolute('d', 0, 'a', 1, 'alpha', 0, 'standard')],'gravity',[1 0 0],'name','My beautiful robot');
            links = obj.myrobot.links;
            links.m = 1;
            links.r = [0,-0.02, 0];
            
            obj.plot_robot(obj.s(1));
        end
        function initRobotState(obj)
            obj.s = obj.s0;
        end
        function [qout] = positionController(obj,qt,Kp,u_ext)
            %% PD controller for tracking a target position qt with proportional and derivative gains Kp and Kd. 
            %% u_ext may be an external torque simultating a force acting on the robot
            qt = deg2rad(qt);
            % get the robot state
            q = obj.s(1);
            dq = obj.s(2);
            % simplifying assumption: Kd obtained as function of Kp
            % (cridically damped system assumtion)
            Kd = 0.4*sqrt(Kp);
            % Compute compensation for coriolis effect and gravity
            tau_gc = obj.myrobot.gravload(q) + obj.myrobot.coriolis(q,dq)*dq;  % Coriolis contribution needed for multi joint robots
            % Control law. This may be regarded as a simplified control law
            % for a position-based impedance controller where the gains are
            % adjusted by the user and mimic the joint stiffness and
            % damping
            u = Kp*(qt - q) + Kd*(-dq) + tau_gc + u_ext;
          
            % Given the torque at the joint, apply it to the robot and obtain the next robot state
            [ddq] = obj.myrobot.accel(q,dq,u);

            dt = 0.04;
            [q,dq] = obj.eulerInteg(q,dq,ddq,dt); %adjust integration time based on running frequency of the framework

            % update the robot state
            obj.s = [q dq ddq];
            % Output in deg for interface
            qout = rad2deg(q);

            sprintf('Current position Error = %.3f',rad2deg(qt - q))
        end

    

        function [qout] = variablePosController(obj,qt,emg1,emg2,u_ext)
            %% INPUT: qt target joint position, emg1 emg activity of agonist muscle, emg2 emg activity of antagonist muscle, u_ext external torque
            % get the robot state
            q = obj.s(1);
            dq = obj.s(2);
            % get ref pos
            %dqt = a1*emg1 - a2*emg2;
            %qt = q + dqt;
            % compute index of stiffness
            coeff = [0.5 0.5 1 0.1];
            STI = coeff(1)*emg1 + coeff(2)*emg2;
            % tune the index of stiffness 
            Kp = coeff(3)*STI + coeff(4);
            % damping as fun of stiffness
            Kd = 0.4*sqrt(Kp);
            % Compute compensation for coriolis effect and gravity
            tau_gc = obj.myrobot.gravload(q) + obj.myrobot.coriolis(q,dq)*dq;
            % compute torques
            u = Kp*(qt - q) + Kd*(-dq) + tau_gc + u_ext;
            % Given the torque at the joint, apply it to the robot and obtain the next robot state
            [ddq] = obj.myrobot.accel(q,dq,u);
            dt = 0.04;
            [q,dq] = obj.eulerInteg(q,dq,ddq,dt); %adjust integration time based on running frequency of the framework

            % update the robot state
            obj.s = [q dq ddq];
            % Output in deg for interface
            qout = rad2deg(q);
        end
        %% Utility fuctions
        function [ pos,vel ] = eulerInteg(obj,q,dq,ddq,dt)
            % Euler integration method to compute the velocity and position of joints given the joint accelerations
            % Could you other methods such as Runge Kutta's
            q = q + dq*dt + 0.5*ddq*(dt^2);
            dq = dq + ddq*dt;
            pos = q';
            vel = dq';
        end
        function plot_robot(obj,q)
            % Visualize the robot given the joint position
            figure
            obj.myrobot.plot(q)
        end
    end    
end
