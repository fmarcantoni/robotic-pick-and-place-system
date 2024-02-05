% (c) 2023 Robotics Engineering Department, WPI Skeleton Robot class for
% OpenManipulator-X Robot for RBE 3001

classdef Robot < OM_X_arm
    % Many properties are abstracted into OM_X_arm and DX_XM430_W350.
    % classes Hopefully, you should only need what's in this class to
    % accomplish everything. But feel free to poke around!
    properties
        mDim; % Stores the robot link dimentions (mm)
        mOtherDim; % Stores extraneous second link dimensions (mm)
        links; % dimentions been selected for calculation 
        rotationAroundZ % Stores if the joints rotation around Z axis true as yes, false as no
        currentSetPoints; % Stores latest set point
        traj; % trajectory_Planner object
        fold;
        rest;
    end

    methods
        % Creates constants and connects via serial Super class constructor
        % called implicitly Add startup functionality here
        function self = Robot()
            % Change robot to position mode with torque enabled by default
            % Feel free to change this as desired
            self.writeMode('p');
            self.writeMotorState(true);
            % Robot Dimensions
            self.mDim = [96.326, 130.23, 124, 133.4]; % (mm)
            self.mOtherDim = [128, 24]; % (mm)
            self.links = [96.326, 128, 24, 124, 133.4];
            self.fold = [-90 -90 90 30];
            self.rest = [3 * 25 0 5 * 25 90];
            %Lab 3: Calling Class Traj_Planner.m
            self.traj = Traj_Planner();
        end

        % Sends the joints to the desired angles goals [1x4 double] -
        % angles (degrees) for each of the joints to go to
        function writeJoints(self, goals)
            goals = mod(round(goals .* DX_XM430_W350.TICKS_PER_DEG + DX_XM430_W350.TICK_POS_OFFSET), DX_XM430_W350.TICKS_PER_ROT);
            self.bulkReadWrite(DX_XM430_W350.POS_LEN, DX_XM430_W350.GOAL_POSITION, goals);
        end

        % Creates a time based profile (trapezoidal) based on the desired
        % times This will cause writePosition to take the desired number of
        % seconds to reach the setpoint. Set time to 0 to disable this
        % profile (be careful). time [double] - total profile time in s. If
        % 0, the profile will be disabled (be extra careful). acc_time
        % [double] - the total acceleration time (for ramp up and ramp down
        % individually, not combined) acc_time is an optional parameter. It
        % defaults to time/3.
      
        function writeTime(self, time, acc_time)
            if (~exist("acc_time", "var"))
                acc_time = time / 3;
            end

            time_ms = time * DX_XM430_W350.MS_PER_S;
            acc_time_ms = acc_time * DX_XM430_W350.MS_PER_S;

            disp("time")
            disp(time_ms)
            disp("acc time")
            disp(acc_time_ms)
            self.bulkReadWrite(DX_XM430_W350.PROF_ACC_LEN, DX_XM430_W350.PROF_ACC, acc_time_ms);
            self.bulkReadWrite(DX_XM430_W350.PROF_VEL_LEN, DX_XM430_W350.PROF_VEL, time_ms);
        end
        
        % Sets the gripper to be open or closed Feel free to change values
        % for open and closed positions as desired (they are in degrees)
        % open [boolean] - true to set the gripper to open, false to close
        function writeGripper(self, open)
            if open
                self.gripper.writePosition(-35);
            else
                self.gripper.writePosition(55);
            end
        end

        % Sets position holding for the joints on or off enable [boolean] -
        % true to enable torque to hold last set position for all joints,
        % false to disable
        function writeMotorState(self, enable)
            self.bulkReadWrite(DX_XM430_W350.TORQUE_ENABLE_LEN, DX_XM430_W350.TORQUE_ENABLE, enable);
        end

        % Supplies the joints with the desired currents currents [1x4
        % double] - currents (mA) for each of the joints to be supplied
        function writeCurrents(self, currents)
            currentInTicks = round(currents .* DX_XM430_W350.TICKS_PER_mA);
            self.bulkReadWrite(DX_XM430_W350.CURR_LEN, DX_XM430_W350.GOAL_CURRENT, currentInTicks);
        end

        % Change the operating mode for all joints:
        % https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/#operating-mode11
        % mode [string] - new operating mode for all joints "current":
        % Current Control Mode (writeCurrent) "velocity": Velocity Control
        % Mode (writeVelocity) "position": Position Control Mode
        % (writePosition) Other provided but not relevant/useful modes:
        % "ext position": Extended Position Control Mode "curr position":
        % Current-based Position Control Mode "pwm voltage": PWM Control
        % Mode
        function writeMode(self, mode)
            switch mode
                case {'current', 'c'} 
                    writeMode = DX_XM430_W350.CURR_CNTR_MD;
                case {'velocity', 'v'}
                    writeMode = DX_XM430_W350.VEL_CNTR_MD;
                case {'position', 'p'}
                    writeMode = DX_XM430_W350.POS_CNTR_MD;
                case {'ext position', 'ep'} % Not useful normally
                    writeMode = DX_XM430_W350.EXT_POS_CNTR_MD;
                case {'curr position', 'cp'} % Not useful normally
                    writeMode = DX_XM430_W350.CURR_POS_CNTR_MD;
                case {'pwm voltage', 'pwm'} % Not useful normally
                    writeMode = DX_XM430_W350.PWM_CNTR_MD;
                otherwise
                    error("setOperatingMode input cannot be '%s'. See implementation in DX_XM430_W350. class.", mode)
            end

            lastVelTimes = self.bulkReadWrite(DX_XM430_W350.PROF_VEL_LEN, DX_XM430_W350.PROF_VEL);
            lastAccTimes = self.bulkReadWrite(DX_XM430_W350.PROF_ACC_LEN, DX_XM430_W350.PROF_ACC);

            self.writeMotorState(false);
            self.bulkReadWrite(DX_XM430_W350.OPR_MODE_LEN, DX_XM430_W350.OPR_MODE, writeMode);
            self.writeTime(lastVelTimes(1) / 1000, lastAccTimes(1) / 1000);
            self.writeMotorState(true);
        end

        % Gets the current joint positions, velocities, and currents
        % readings [3x4 double] - The joints' positions, velocities, and
        % efforts (deg, deg/s, mA)
        function readings = getJointsReadings(self)
            readings = zeros(3,4);
            
            readings(1, :) = (self.bulkReadWrite(DX_XM430_W350.POS_LEN, DX_XM430_W350.CURR_POSITION) - DX_XM430_W350.TICK_POS_OFFSET) ./ DX_XM430_W350.TICKS_PER_DEG;
            readings(2, :) = self.bulkReadWrite(DX_XM430_W350.VEL_LEN, DX_XM430_W350.CURR_VELOCITY) ./ DX_XM430_W350.TICKS_PER_ANGVEL;
            readings(3, :) = self.bulkReadWrite(DX_XM430_W350.CURR_LEN, DX_XM430_W350.CURR_CURRENT) ./ DX_XM430_W350.TICKS_PER_mA;
        end

        % Sends the joints at the desired velocites vels [1x4 double] -
        % angular velocites (deg/s) for each of the joints to go at
        function writeVelocities(self, vels)
            vels = round(vels .* DX_XM430_W350.TICKS_PER_ANGVEL);
            self.bulkReadWrite(DX_XM430_W350.VEL_LEN, DX_XM430_W350.GOAL_VELOCITY, vels);
        end

        %% OUR CODE

        % Sends a 1x4 array of four joint values in degrees to the robot
        % bypassing interpolation values [1x4 double] - joint angles for
        % each of the joints to travel to
        function servo_jp(self, values)
            self.currentSetPoints = values;
            self.writeJoints(values);
        end
        

        % Sends a 1x4 array of four joint values in degrees to the robot
        % with an interpolation time in milliseconds values [1x4 double] -
        % joint angles for each of the joints to travel to time [double] -
        % interpolation time in milliseconds
        function interpolate_jp(self, values, time)
            self.bulkReadWrite(DX_XM430_W350.PROF_ACC_LEN, DX_XM430_W350.PROF_ACC, time/3);
            self.bulkReadWrite(DX_XM430_W350.PROF_VEL_LEN, DX_XM430_W350.PROF_VEL, time);
            self.servo_jp(values);
        end

        % Creates a 2x4 array of the current joint positions in the first
        % row ofand/or the current joint values as requested GETPOS
        % [boolean] - true to collect position values, false to return
        % zeros for position GETVEL [boolean] - true to collect velocity
        % values, false toreturn zeros for velocity
        function result = measured_js(self, GETPOS,GETVEL)
            result = zeros(2,4);
            error = [
                -0.0878906250000000	0.527343750000000	1.66992187500000	0.966796875000000;
                0	67.3260000000000	59.0820000000000	-32.9760000000000
            ];
            if GETPOS
                result(1, :) = (self.bulkReadWrite(DX_XM430_W350.POS_LEN, DX_XM430_W350.CURR_POSITION) - DX_XM430_W350.TICK_POS_OFFSET) ./ DX_XM430_W350.TICKS_PER_DEG;
            end

            if GETVEL
                result(2, :) = self.bulkReadWrite(DX_XM430_W350.VEL_LEN, DX_XM430_W350.CURR_VELOCITY) ./ DX_XM430_W350.TICKS_PER_ANGVEL;
            end

            result = result - error;
        end

        % Creates a 1x4 array of of current joint setpoints in degrees read
        % [1x4 interger] - current joint setpoint positions
        function read = setpoint_js(self)
             read(1, :) = (self.bulkReadWrite(DX_XM430_W350.POS_LEN, DX_XM430_W350.CURR_POSITION) - DX_XM430_W350.TICK_POS_OFFSET) ./ DX_XM430_W350.TICKS_PER_DEG;
        end
        
        % Creates a 1x4 array of target joint setpoints in degrees
        function goals = goal_js(self)
            goals = self.currentSetPoints;
        end
        

        function red = toRed(self,deg)
            red = deg/180 * pi;
        end

        function deg = toDeg(self,red)
            deg = red/pi * 180;
        end
%% FORWARD POSITION KINEMATICS

        % Creates a 4x4 rotation matrix representing the rotation about the
        % z axis theta [interger] - angle of rotation about the z axis
        function T = rotationZ(self,theta)
            radTheta = deg2rad(theta);
            T = [
                cos(radTheta) -sin(radTheta) 0 0;
                sin(radTheta) cos(radTheta) 0 0;
                0 0 1 0;
                0 0 0 1
            ];
        end
        
        % Creates a 4x4 rotation matrix representing the rotation about the
        % x axis theta [interger] - angle of rotation about the x axis
        function T = rotationX(self,theta)
            radTheta = deg2rad(theta);
            T = [
                1 0 0 0;
                0 cos(radTheta) -sin(radTheta) 0;
                0 sin(radTheta) cos(radTheta) 0;
                0 0 0 1;
            ];
        end

        % Creates a 4x4 homogeneous transformation matrix from one frame to
        % it's consecutive frame given the DH parameters for the transform

        % theta [interger] - value of theta DH parameter
        % d [interger] - value of the d DH parameter
        % a [interger] - value of the a DH parameter
        % alpha [interger] - value of alpha DH parameter
        function dhmat = dh2mat(self,theta,d,a,alpha)
            dhlinear = [1 0 0 a; 
                        0 1 0 0; 
                        0 0 1 d;
                        0 0 0 1];
            dhmat = self.rotationZ(theta) * dhlinear * self.rotationX(alpha);
        end

        % Creates n 4x4 homogenous transformation matricies representing
        % all the intermediate frame transformrations
        % dhInput [nx4 interger] - matrix holding DH parameters theta, d,
        % a, alpha in columns 1, 2, 3, and 4 respectively for each
        % intermediate transform in each row
        function DHtransformations = dh2fk(self, dhInput)
            dhmat = [1 0 0 0;
                     0 1 0 0;
                     0 0 1 0;
                     0 0 0 1];
            inputSize = size(dhInput);
            DHtransformations = zeros(4,4,1,inputSize(2));
            for dh = 1:size(dhInput)
                dhmat = dhmat * self.dh2mat(dhInput(dh,1),dhInput(dh,2),dhInput(dh,3),dhInput(dh,4)); %pass value of dh table in order
                DHtransformations(:,:,1,dh) = dhmat;
            end
        end

        % Creates 4 4x4 homogeneous transformation matricies representing
        % the transformations from frame 0-1, 0-2, 0-3, and 0-4 (full base
        % to end effector forward position kinematics matrix)
        % degrees [1x4 interger] - the joint values of all 4 joints
        function fkOutput = fk3001(self, degrees)
            offsetDegree = rad2deg(atan2(self.mOtherDim(1),self.mOtherDim(2)));
            % DH table
            dhinput = [
                degrees(1) self.mDim(1) 0 -90;
                degrees(2)-offsetDegree 0 self.mDim(2) 0;
                degrees(3)+offsetDegree 0 self.mDim(3) 0;
                degrees(4) 0 self.mDim(4) 0;
            ];
            fkOutput = self.dh2fk(dhinput);
        end

        % Feeds current joint values into fk3001 to return the 4 current
        % 4x4 homogenous transformation matricies
        function measured_cp = measured_cp(self)
            result_measured_js = self.measured_js(true,false);
            measured_cp = self.fk3001(result_measured_js);
        end

        % Feeds intermediate target joint values into fk3001 to return the 
        % 4 intermediate target 4x4 homogenous transformation matricies
        function current_setPoint = setpoint_cp(self)
            result_SetPoint = self.setpoint_js();
            current_setPoint = self.fk3001(result_SetPoint);
        end

        % Feeds target joint values into fk3001 to return the 4 target 4x4
        % homogeneous transformation matricies
        function cp_goal = goal_cp(self)
            result_Goal = self.goal_js();
            cp_goal = self.fk3001(result_Goal);
        end     

        %% INVERSE POSITION KINEMATICS AND TRAJECTORY PLANNING

        % Creates 1x4 matrix representing the joint values needed to
        % achieve a given end effector position
        % ee [1x4 interger] - the desired xyz coordinates and orientation
        % of the end effector
        function degrees_ik = ik3001(self,ee)
            % define constants
            offsetDegree = atan2(self.mOtherDim(2),self.mOtherDim(1));
            x = ee(1);
            y = ee(2);
            z = ee(3);
            a = deg2rad(ee(4));
            l1 = self.mDim(1);
            l2 = self.mDim(2);
            l3 = self.mDim(3);
            l4 = self.mDim(4);
            th1 = atan2(y,x);

            % find s and r
            s = z - l1;
            r = sqrt(x^2 + y^2);

            % adjust for offset
            ds = sin(a) * l4;
            dr = cos(a) * l4;
            s = s + ds;
            r = r - dr;
            
            % ik calculation
            w = sqrt(s^2 + r^2);
            b3 = atan2(s,r);
            cb1 = (l2^2 + l3^2 - (s^2 + r^2)) / (2 * l2 * l3);
            sb1 = sqrt(1-cb1^2);
            b1 = atan2(sb1,cb1);

            th3 = pi/2 + offsetDegree - b1;

            sb2 = sb1 * l3 / w;
            cb2 = sqrt(1-sb2^2);
            b2 = atan2(sb2,cb2);

            th2 = pi/2 - (b2 + b3 + offsetDegree);

            th4 = a - th2 - th3;

            degrees_ik = [rad2deg(th1) rad2deg(th2) rad2deg(th3) rad2deg(th4)]; 
         end

        function traj_record = run_trajectory(self, coefficents, tsum, Joint_Space, isCubic)
            dt = 0.0005;
            self.writeTime(dt,0);
            traj_record = zeros(6,0);
            trajIndex = 1;
            tic
            while toc < tsum
                t = toc;
                coefIndex = 1;
                degrees = zeros(1,4);
                orientations = zeros(1,4);
                if Joint_Space % jointSpace
                    % polynomials summed up
                    for coef = coefficents
                        coefT = coef';
                        if isCubic
                            degree = coefT(1) + coefT(2)*t + coefT(3)*t^2 + coefT(4)*t^3;
                        else
                            degree = coefT(1) + coefT(2)*t + coefT(3)*t^2 + coefT(4)*t^3 + coefT(5)*t^4 + coefT(6)*t^5;
                        end
                        degrees(coefIndex) = degree;
                        % Next data entery
                        coefIndex = coefIndex + 1;
                    end
                else % taskSpace
                    for coef = coefficents
                        coefT = coef';
                        if isCubic
                            orientation = coefT(1) + coefT(2)*t + coefT(3)*t^2 + coefT(4)*t^3;
                        else
                            orientation = coefT(1) + coefT(2)*t + coefT(3)*t^2 + coefT(4)*t^3 + coefT(5)*t^4 + coefT(6)*t^5;
                        end
                        orientations(coefIndex) = orientation;
                        coefIndex = coefIndex + 1;
                    end
                    degrees = self.ik3001(orientations);
                end
                self.servo_jp(degrees);
                pause(dt);
                traj_record(5,trajIndex) = t;
                traj_record(1:4,trajIndex) = degrees;
                trajIndex = trajIndex + 1;
%                 n = 350000;
%                 SJ = self.jacobian(degrees);
%                 SJ_det = det(SJ(1:3,1:3));
%                 traj_record(6,trajIndex) = SJ_det;
%                 if (SJ_det < n)
%                     disp("Error Singularity Reached: Try Again");
%                     self.interpolate_jp([0 -90 90 30],1000);
%                     pause(1);
%                     disp(SJ_det);
%                     break;
%                 else
%                     self.servo_jp(degrees);
%                     pause(dt);
%                     traj_record(5,trajIndex) = t;
%                     traj_record(1:4,trajIndex) = degrees;
%                     trajIndex = trajIndex + 1;
%                 end % end of if rank(SJ,n)
            end
        end % end traj_record

        %==========

        %lab4 start
        function J = jacobian(self, degrees)
            offsetDegree = atan2(self.mOtherDim(1),self.mOtherDim(2));
            
            dhinput = [
                deg2rad(degrees(1)) self.mDim(1) 0 -pi/2;
                deg2rad(degrees(2))-offsetDegree 0 self.mDim(2) 0;
                deg2rad(degrees(3))+offsetDegree 0 self.mDim(3) 0;
                deg2rad(degrees(4)) 0 self.mDim(4) 0;
            ];

            th1 = dhinput(1,1);
            a1 = dhinput(1,3);
            alp1 = dhinput(1,4);
            th2 = dhinput(2,1);
            d2 = dhinput(2,2);
            a2 = dhinput(2,3);
            alp2 = dhinput(2,4);
            th3 = dhinput(3,1);
            d3 = dhinput(3,2);
            a3 = dhinput(3,3);
            alp3 = dhinput(3,4);
            th4 = dhinput(4,1);
            d4 = dhinput(4,2);
            a4 = dhinput(4,3);                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            
            J = [
                d3*(cos(alp2)*sin(alp1)*cos(th1) - sin(alp2)*sin(th1)*sin(th2) + cos(alp1)*sin(alp2)*cos(th1)*cos(th2)) - d4*(sin(alp3)*sin(th3)*(cos(th2)*sin(th1) + cos(alp1)*cos(th1)*sin(th2)) - cos(alp3)*(cos(alp2)*sin(alp1)*cos(th1) - sin(alp2)*sin(th1)*sin(th2) + cos(alp1)*sin(alp2)*cos(th1)*cos(th2)) + sin(alp3)*cos(th3)*(sin(alp1)*sin(alp2)*cos(th1) + cos(alp2)*sin(th1)*sin(th2) - cos(alp1)*cos(alp2)*cos(th1)*cos(th2))) - a1*sin(th1) + a4*sin(th4)*(sin(alp3)*(cos(alp2)*sin(alp1)*cos(th1) - sin(alp2)*sin(th1)*sin(th2) + cos(alp1)*sin(alp2)*cos(th1)*cos(th2)) + cos(alp3)*sin(th3)*(cos(th2)*sin(th1) + cos(alp1)*cos(th1)*sin(th2)) + cos(alp3)*cos(th3)*(sin(alp1)*sin(alp2)*cos(th1) + cos(alp2)*sin(th1)*sin(th2) - cos(alp1)*cos(alp2)*cos(th1)*cos(th2))) + d2*sin(alp1)*cos(th1) + a3*sin(th3)*(sin(alp1)*sin(alp2)*cos(th1) + cos(alp2)*sin(th1)*sin(th2) - cos(alp1)*cos(alp2)*cos(th1)*cos(th2)) - a2*cos(th2)*sin(th1) + a4*cos(th4)*(sin(th3)*(sin(alp1)*sin(alp2)*cos(th1) + cos(alp2)*sin(th1)*sin(th2) - cos(alp1)*cos(alp2)*cos(th1)*cos(th2)) - cos(th3)*(cos(th2)*sin(th1) + cos(alp1)*cos(th1)*sin(th2))) - a3*cos(th3)*(cos(th2)*sin(th1) + cos(alp1)*cos(th1)*sin(th2)) - a2*cos(alp1)*cos(th1)*sin(th2), d4*(cos(alp3)*(sin(alp2)*cos(th1)*cos(th2) - cos(alp1)*sin(alp2)*sin(th1)*sin(th2)) - sin(alp3)*sin(th3)*(cos(th1)*sin(th2) + cos(alp1)*cos(th2)*sin(th1)) + sin(alp3)*cos(th3)*(cos(alp2)*cos(th1)*cos(th2) - cos(alp1)*cos(alp2)*sin(th1)*sin(th2))) + d3*(sin(alp2)*cos(th1)*cos(th2) - cos(alp1)*sin(alp2)*sin(th1)*sin(th2)) - a2*cos(th1)*sin(th2) - a3*sin(th3)*(cos(alp2)*cos(th1)*cos(th2) - cos(alp1)*cos(alp2)*sin(th1)*sin(th2)) + a4*sin(th4)*(sin(alp3)*(sin(alp2)*cos(th1)*cos(th2) - cos(alp1)*sin(alp2)*sin(th1)*sin(th2)) + cos(alp3)*sin(th3)*(cos(th1)*sin(th2) + cos(alp1)*cos(th2)*sin(th1)) - cos(alp3)*cos(th3)*(cos(alp2)*cos(th1)*cos(th2) - cos(alp1)*cos(alp2)*sin(th1)*sin(th2))) - a4*cos(th4)*(sin(th3)*(cos(alp2)*cos(th1)*cos(th2) - cos(alp1)*cos(alp2)*sin(th1)*sin(th2)) + cos(th3)*(cos(th1)*sin(th2) + cos(alp1)*cos(th2)*sin(th1))) - a3*cos(th3)*(cos(th1)*sin(th2) + cos(alp1)*cos(th2)*sin(th1)) - a2*cos(alp1)*cos(th2)*sin(th1), d4*(sin(alp3)*cos(th3)*(cos(th1)*cos(th2) - cos(alp1)*sin(th1)*sin(th2)) - sin(alp3)*sin(th3)*(cos(alp2)*cos(th1)*sin(th2) - sin(alp1)*sin(alp2)*sin(th1) + cos(alp1)*cos(alp2)*cos(th2)*sin(th1))) - a3*sin(th3)*(cos(th1)*cos(th2) - cos(alp1)*sin(th1)*sin(th2)) - a4*sin(th4)*(cos(alp3)*cos(th3)*(cos(th1)*cos(th2) - cos(alp1)*sin(th1)*sin(th2)) - cos(alp3)*sin(th3)*(cos(alp2)*cos(th1)*sin(th2) - sin(alp1)*sin(alp2)*sin(th1) + cos(alp1)*cos(alp2)*cos(th2)*sin(th1))) - a3*cos(th3)*(cos(alp2)*cos(th1)*sin(th2) - sin(alp1)*sin(alp2)*sin(th1) + cos(alp1)*cos(alp2)*cos(th2)*sin(th1)) - a4*cos(th4)*(cos(th3)*(cos(alp2)*cos(th1)*sin(th2) - sin(alp1)*sin(alp2)*sin(th1) + cos(alp1)*cos(alp2)*cos(th2)*sin(th1)) + sin(th3)*(cos(th1)*cos(th2) - cos(alp1)*sin(th1)*sin(th2))), a4*sin(th4)*(sin(th3)*(cos(alp2)*cos(th1)*sin(th2) - sin(alp1)*sin(alp2)*sin(th1) + cos(alp1)*cos(alp2)*cos(th2)*sin(th1)) - cos(th3)*(cos(th1)*cos(th2) - cos(alp1)*sin(th1)*sin(th2))) - a4*cos(th4)*(cos(alp3)*sin(th3)*(cos(th1)*cos(th2) - cos(alp1)*sin(th1)*sin(th2)) - sin(alp3)*(cos(alp2)*sin(alp1)*sin(th1) + sin(alp2)*cos(th1)*sin(th2) + cos(alp1)*sin(alp2)*cos(th2)*sin(th1)) + cos(alp3)*cos(th3)*(cos(alp2)*cos(th1)*sin(th2) - sin(alp1)*sin(alp2)*sin(th1) + cos(alp1)*cos(alp2)*cos(th2)*sin(th1)));
                a1*cos(th1) + d4*(cos(alp3)*(cos(alp2)*sin(alp1)*sin(th1) + sin(alp2)*cos(th1)*sin(th2) + cos(alp1)*sin(alp2)*cos(th2)*sin(th1)) + sin(alp3)*sin(th3)*(cos(th1)*cos(th2) - cos(alp1)*sin(th1)*sin(th2)) + sin(alp3)*cos(th3)*(cos(alp2)*cos(th1)*sin(th2) - sin(alp1)*sin(alp2)*sin(th1) + cos(alp1)*cos(alp2)*cos(th2)*sin(th1))) + d3*(cos(alp2)*sin(alp1)*sin(th1) + sin(alp2)*cos(th1)*sin(th2) + cos(alp1)*sin(alp2)*cos(th2)*sin(th1)) + a2*cos(th1)*cos(th2) - a4*sin(th4)*(cos(alp3)*sin(th3)*(cos(th1)*cos(th2) - cos(alp1)*sin(th1)*sin(th2)) - sin(alp3)*(cos(alp2)*sin(alp1)*sin(th1) + sin(alp2)*cos(th1)*sin(th2) + cos(alp1)*sin(alp2)*cos(th2)*sin(th1)) + cos(alp3)*cos(th3)*(cos(alp2)*cos(th1)*sin(th2) - sin(alp1)*sin(alp2)*sin(th1) + cos(alp1)*cos(alp2)*cos(th2)*sin(th1))) + d2*sin(alp1)*sin(th1) - a3*sin(th3)*(cos(alp2)*cos(th1)*sin(th2) - sin(alp1)*sin(alp2)*sin(th1) + cos(alp1)*cos(alp2)*cos(th2)*sin(th1)) - a4*cos(th4)*(sin(th3)*(cos(alp2)*cos(th1)*sin(th2) - sin(alp1)*sin(alp2)*sin(th1) + cos(alp1)*cos(alp2)*cos(th2)*sin(th1)) - cos(th3)*(cos(th1)*cos(th2) - cos(alp1)*sin(th1)*sin(th2))) + a3*cos(th3)*(cos(th1)*cos(th2) - cos(alp1)*sin(th1)*sin(th2)) - a2*cos(alp1)*sin(th1)*sin(th2), d4*(cos(alp3)*(sin(alp2)*cos(th2)*sin(th1) + cos(alp1)*sin(alp2)*cos(th1)*sin(th2)) - sin(alp3)*sin(th3)*(sin(th1)*sin(th2) - cos(alp1)*cos(th1)*cos(th2)) + sin(alp3)*cos(th3)*(cos(alp2)*cos(th2)*sin(th1) + cos(alp1)*cos(alp2)*cos(th1)*sin(th2))) + d3*(sin(alp2)*cos(th2)*sin(th1) + cos(alp1)*sin(alp2)*cos(th1)*sin(th2)) - a2*sin(th1)*sin(th2) - a3*sin(th3)*(cos(alp2)*cos(th2)*sin(th1) + cos(alp1)*cos(alp2)*cos(th1)*sin(th2)) + a4*sin(th4)*(sin(alp3)*(sin(alp2)*cos(th2)*sin(th1) + cos(alp1)*sin(alp2)*cos(th1)*sin(th2)) + cos(alp3)*sin(th3)*(sin(th1)*sin(th2) - cos(alp1)*cos(th1)*cos(th2)) - cos(alp3)*cos(th3)*(cos(alp2)*cos(th2)*sin(th1) + cos(alp1)*cos(alp2)*cos(th1)*sin(th2))) - a4*cos(th4)*(sin(th3)*(cos(alp2)*cos(th2)*sin(th1) + cos(alp1)*cos(alp2)*cos(th1)*sin(th2)) + cos(th3)*(sin(th1)*sin(th2) - cos(alp1)*cos(th1)*cos(th2))) - a3*cos(th3)*(sin(th1)*sin(th2) - cos(alp1)*cos(th1)*cos(th2)) + a2*cos(alp1)*cos(th1)*cos(th2), d4*(sin(alp3)*cos(th3)*(cos(th2)*sin(th1) + cos(alp1)*cos(th1)*sin(th2)) - sin(alp3)*sin(th3)*(sin(alp1)*sin(alp2)*cos(th1) + cos(alp2)*sin(th1)*sin(th2) - cos(alp1)*cos(alp2)*cos(th1)*cos(th2))) - a3*sin(th3)*(cos(th2)*sin(th1) + cos(alp1)*cos(th1)*sin(th2)) - a4*sin(th4)*(cos(alp3)*cos(th3)*(cos(th2)*sin(th1) + cos(alp1)*cos(th1)*sin(th2)) - cos(alp3)*sin(th3)*(sin(alp1)*sin(alp2)*cos(th1) + cos(alp2)*sin(th1)*sin(th2) - cos(alp1)*cos(alp2)*cos(th1)*cos(th2))) - a3*cos(th3)*(sin(alp1)*sin(alp2)*cos(th1) + cos(alp2)*sin(th1)*sin(th2) - cos(alp1)*cos(alp2)*cos(th1)*cos(th2)) - a4*cos(th4)*(cos(th3)*(sin(alp1)*sin(alp2)*cos(th1) + cos(alp2)*sin(th1)*sin(th2) - cos(alp1)*cos(alp2)*cos(th1)*cos(th2)) + sin(th3)*(cos(th2)*sin(th1) + cos(alp1)*cos(th1)*sin(th2))), a4*sin(th4)*(sin(th3)*(sin(alp1)*sin(alp2)*cos(th1) + cos(alp2)*sin(th1)*sin(th2) - cos(alp1)*cos(alp2)*cos(th1)*cos(th2)) - cos(th3)*(cos(th2)*sin(th1) + cos(alp1)*cos(th1)*sin(th2))) - a4*cos(th4)*(sin(alp3)*(cos(alp2)*sin(alp1)*cos(th1) - sin(alp2)*sin(th1)*sin(th2) + cos(alp1)*sin(alp2)*cos(th1)*cos(th2)) + cos(alp3)*sin(th3)*(cos(th2)*sin(th1) + cos(alp1)*cos(th1)*sin(th2)) + cos(alp3)*cos(th3)*(sin(alp1)*sin(alp2)*cos(th1) + cos(alp2)*sin(th1)*sin(th2) - cos(alp1)*cos(alp2)*cos(th1)*cos(th2)));
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         0,                                                                                                                                                                                                                                                                                                                                                                                                                                                            d4*(cos(alp3)*sin(alp1)*sin(alp2)*sin(th2) + sin(alp1)*sin(alp3)*cos(th2)*sin(th3) + cos(alp2)*sin(alp1)*sin(alp3)*cos(th3)*sin(th2)) - a4*sin(th4)*(cos(alp3)*sin(alp1)*cos(th2)*sin(th3) - sin(alp1)*sin(alp2)*sin(alp3)*sin(th2) + cos(alp2)*cos(alp3)*sin(alp1)*cos(th3)*sin(th2)) + a2*sin(alp1)*cos(th2) + a4*cos(th4)*(sin(alp1)*cos(th2)*cos(th3) - cos(alp2)*sin(alp1)*sin(th2)*sin(th3)) + a3*sin(alp1)*cos(th2)*cos(th3) + d3*sin(alp1)*sin(alp2)*sin(th2) - a3*cos(alp2)*sin(alp1)*sin(th2)*sin(th3),                                                                                                                                                                                                                                                                                                                             d4*(sin(alp3)*sin(th3)*(cos(alp1)*sin(alp2) + cos(alp2)*sin(alp1)*cos(th2)) + sin(alp1)*sin(alp3)*cos(th3)*sin(th2)) + a4*cos(th4)*(cos(th3)*(cos(alp1)*sin(alp2) + cos(alp2)*sin(alp1)*cos(th2)) - sin(alp1)*sin(th2)*sin(th3)) + a3*cos(th3)*(cos(alp1)*sin(alp2) + cos(alp2)*sin(alp1)*cos(th2)) - a4*sin(th4)*(cos(alp3)*sin(th3)*(cos(alp1)*sin(alp2) + cos(alp2)*sin(alp1)*cos(th2)) + cos(alp3)*sin(alp1)*cos(th3)*sin(th2)) - a3*sin(alp1)*sin(th2)*sin(th3),                                                                                                                                                                                                               a4*cos(th4)*(sin(alp3)*(cos(alp1)*cos(alp2) - sin(alp1)*sin(alp2)*cos(th2)) + cos(alp3)*cos(th3)*(cos(alp1)*sin(alp2) + cos(alp2)*sin(alp1)*cos(th2)) - cos(alp3)*sin(alp1)*sin(th2)*sin(th3)) - a4*sin(th4)*(sin(th3)*(cos(alp1)*sin(alp2) + cos(alp2)*sin(alp1)*cos(th2)) + sin(alp1)*cos(th3)*sin(th2));
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          sin(alp1)*sin(th1),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               cos(alp2)*sin(alp1)*sin(th1) + sin(alp2)*cos(th1)*sin(th2) + cos(alp1)*sin(alp2)*cos(th2)*sin(th1),                                                                                                                                                                                                          cos(alp3)*(cos(alp2)*sin(alp1)*sin(th1) + sin(alp2)*cos(th1)*sin(th2) + cos(alp1)*sin(alp2)*cos(th2)*sin(th1)) + sin(alp3)*sin(th3)*(cos(th1)*cos(th2) - cos(alp1)*sin(th1)*sin(th2)) + sin(alp3)*cos(th3)*(cos(alp2)*cos(th1)*sin(th2) - sin(alp1)*sin(alp2)*sin(th1) + cos(alp1)*cos(alp2)*cos(th2)*sin(th1));
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         -sin(alp1)*cos(th1),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               sin(alp2)*sin(th1)*sin(th2) - cos(alp2)*sin(alp1)*cos(th1) - cos(alp1)*sin(alp2)*cos(th1)*cos(th2),                                                                                                                                                                                                          sin(alp3)*sin(th3)*(cos(th2)*sin(th1) + cos(alp1)*cos(th1)*sin(th2)) - cos(alp3)*(cos(alp2)*sin(alp1)*cos(th1) - sin(alp2)*sin(th1)*sin(th2) + cos(alp1)*sin(alp2)*cos(th1)*cos(th2)) + sin(alp3)*cos(th3)*(sin(alp1)*sin(alp2)*cos(th1) + cos(alp2)*sin(th1)*sin(th2) - cos(alp1)*cos(alp2)*cos(th1)*cos(th2));
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         1,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   cos(alp1),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               cos(alp1)*cos(alp2) - sin(alp1)*sin(alp2)*cos(th2),                                                                                                                                                                                                                                                                                                                                         cos(alp3)*(cos(alp1)*cos(alp2) - sin(alp1)*sin(alp2)*cos(th2)) - sin(alp3)*cos(th3)*(cos(alp1)*sin(alp2) + cos(alp2)*sin(alp1)*cos(th2)) + sin(alp1)*sin(alp3)*sin(th2)*sin(th3)
            ];
        end
        
        function dp = fdk3001(self,q,dq)
            dq = deg2rad(dq);
            J = self.jacobian(q);
            dp = J*dq';
        end
        
        function result = velocity_control(self,p0,pf,t)
            result = zeros(5,0);
            resultIndex = 1;
            p0 = p0(1:3)';
            pf = pf(1:3)';
            q0 = self.setpoint_js;
            q0 = q0';
            dt = 0.0005;
            d = pf - p0;
            d = d(1:3);
            sd = sqrt(d(1)^2 + d(2)^2 + d(3)^2);
            v = sd / t;
            ud = d / sd;
            dp = ud * v;
            
            self.writeTime(dt,0);
            tic
            while toc < t
                time = toc;
                J = self.jacobian(q0');
                J = J(1:3,:);
                Ji = pinv(J);
                dq = Ji*dp;
                q = q0 + rad2deg(dq) * dt;
                self.servo_jp(q');
                pause(dt);
                result(1:4,resultIndex) = q;
                result(5,resultIndex) = time;
                resultIndex = resultIndex + 1;
                q0 = q;
            end
        end
        %lab4 end

        %lab5
        function output = cameraToRobot(x,y)
            worldPt = pointsToWorld(cam.getCameraInstrinsics, R, t, pos);
        disp(worldPt);
        R_0_checker = [ 0  1  0; 
                        1  0  0; 
                        0  0 -1
                    ];
        t_0_checker = [
                    113;
                    -70;
                    0
                  ];
        T_0_check = [R_0_checker, t_0_checker;zeros(1,3), 1];
        r_pos = inv(T_0_check) * [
                                    worldPt';
                                    0;
                                    1
                                ];
        end

        function gripperOpenPercent(self,percent)
            self.gripper.writePosition(55 - 155*percent);
        end

        function foldArm(self)
            self.servo_jp(self.fold);
        end

        function toZeroPos(self)
            self.servo_jp([0 0 0 0]);
        end
        %lab5 end
        function result = taskSpace_interpolation(self,p0,pf,t)
            coef1 = self.traj.quintic_traj(0,t,0,0,p0(1),pf(1),0,0);
            coef2 = self.traj.quintic_traj(0,t,0,0,p0(2),pf(2),0,0);
            coef3 = self.traj.quintic_traj(0,t,0,0,p0(3),pf(3),0,0);
            coef4 = self.traj.quintic_traj(0,t,0,0,p0(4),pf(4),0,0);
            coefs = [coef1 coef2 coef3 coef4];
            result = self.run_trajectory(coefs,t,false,false);
        end

        function taskSpacePoint = taskSpace_setPoint(self)
            point = self.setpoint_cp;
            taskSpacePoint = point(1:3,4,1,4)';
        end

        function restPoint(self)
            self.servo_jp(self.ik3001(self.rest));
        end
    end % end methods
end % end class 
