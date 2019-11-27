classdef soccerbot < handle
    properties
        robot = importrobot('../soccer_description/models/soccerbot_stl.urdf');
        robot_left_leg_subtree;
        robot_right_leg_subtree;
        configuration;
        pose = Geometry.transform;              % Pose of the center between the two legs position
        foot_center_to_floor;
        ik_left
        ik_right
        dh
    end
    
    methods
        function obj = soccerbot(position, foot_center_to_floor)
            obj.configuration = obj.robot.homeConfiguration;
            obj.robot_left_leg_subtree = obj.robot.subtree('left_hip_side');
            obj.robot_right_leg_subtree = obj.robot.subtree('right_hip_side');
            
            obj.ik_left = inverseKinematics('RigidBodyTree', obj.robot_left_leg_subtree);
            obj.ik_right = inverseKinematics('RigidBodyTree', obj.robot_right_leg_subtree);
            
            % Solve DH Table
            solveDHTable(obj);

            % Calculating hip and feet location
            hip_to_torso = obj.robot.getTransform(obj.robot.homeConfiguration, 'torso', 'left_hip_front');
            position(3) = position(3) + hip_to_torso(3,4);
            obj.pose.setPosition(position);
            
            obj.foot_center_to_floor = foot_center_to_floor; % Using foot box height and foot to thing parameter
            
            left_foot_position = obj.robot_left_leg_subtree.getTransform(obj.robot_left_leg_subtree.homeConfiguration, 'left_foot', 'torso');
            left_foot_position(3,4) = - position(3) + foot_center_to_floor;
            configSolL = obj.ik_left_foot(left_foot_position);
            
            right_foot_position = obj.robot_right_leg_subtree.getTransform(obj.robot_right_leg_subtree.homeConfiguration, 'right_foot', 'torso');
            right_foot_position(3,4) = - position(3) + foot_center_to_floor;
            configSolR = obj.ik_right_foot(right_foot_position);
            
            obj.configuration(3:8) = configSolL;
            obj.configuration(13:18) = configSolR;
            
            showdetails(obj.robot)
        end
        
        function footpath = getPath(obj, finishPosition)
            crotch = obj.pose.position;
            finishPositionCoordinate = finishPosition.position;
            finishPositionCoordinate(3) = crotch(3);
            finishPosition.setPosition(finishPositionCoordinate);
            
            footpath = Geometry.robotpath(obj.pose, finishPosition, ...
                obj.foot_center_to_floor);
        end
        
        function solveDHTable(obj)
            H3 = obj.robot.getTransform(obj.robot.homeConfiguration, 'left_thigh', 'left_calve');
            H4 = obj.robot.getTransform(obj.robot.homeConfiguration, 'left_calve', 'left_ankle');
            
            % Started from the foot to the top
            obj.dh = [0 -pi/2 0 0; ...
                  0 pi/2 0 0; ...
                  H3(3,4) 0 0 0; ...
                  H4(3,4) 0 0 0; ...
                  0 pi/2 0 0; ...
                  0 0 0 0];
        end
        
        function H = leftFootPose(obj)
            H = obj.robot.getTransform(obj.robot.homeConfiguration, 'torso', 'left_foot');
        end
        
        function H = rightFootPose(obj)
            H = obj.robot.getTransform(obj.robot.homeConfiguration, 'torso', 'right_foot');
        end
        
        function configSolL = ik_left_foot(obj, configuration)
            torso_to_hip = obj.robot.getTransform(obj.robot.homeConfiguration, 'left_hip_front', 'torso');
            configSolL = obj.robot_left_leg_subtree.homeConfiguration;
            
            configuration(1:3,4) = configuration(1:3,4) - torso_to_hip(1:3,4);
            invconf = inv(configuration);
            
            d3 = obj.dh(3,1);
            d4 = obj.dh(4,1);
            
            Xd = invconf(1,4);
            Yd = invconf(2,4);
            Zd = invconf(3,4);
            
            assert(norm([Xd, Yd, Zd]) <= d3 + d4, "IK Position Unreachable: Desired Distance: " + num2str(norm([Xd, Yd, Zd])) + ", Limited Distance: " + num2str(d3 + d4));
            
            theta6 = -atan2(Yd, Zd);            
            tmp1 = Zd / cos(theta6);
            tmp2 = Xd;
            D = (tmp1^2 + tmp2^2 - d3^2 - d4^2) / 2 / d3 / d4;
            tmp3 = atan2(D, -sqrt(1-D^2));
            theta4 = -wrapTo2Pi(tmp3 - pi/2);
            
            alp = atan2(tmp1, tmp2);
            beta = atan2(-d3 * cos(tmp3), d4 + d3 * sin(tmp3));
            theta5 = pi/2 - (alp - beta);
            
            H4 = Geometry.transform(obj.dh(4,1), obj.dh(4,2), obj.dh(4,3), theta4);
            H5 = Geometry.transform(obj.dh(5,1), obj.dh(5,2), obj.dh(5,3), theta5);
            H6 = Geometry.transform(obj.dh(6,1), obj.dh(6,2), obj.dh(6,3), theta6);
            
            H46 = H4.H * H5.H * H6.H;
            final_rotation = rotm2tform(eul2rotm([0,pi/2,pi]));
            H03 = (configuration * final_rotation) / (H46);
            assert(norm(H03(1:3,4)) - d3 < 0.03);

            angles = rotm2eul(inv(H03(1:3,1:3)), 'ZYX');
            theta3 = pi/2 - angles(1);
            theta1 = -angles(2);
            theta2 = (angles(3) + pi/2);            
        
            configSolL(1).JointPosition = theta1;
            configSolL(2).JointPosition = theta2;
            configSolL(3).JointPosition = theta3;
            configSolL(4).JointPosition = theta4;
            configSolL(5).JointPosition = theta5;
            configSolL(6).JointPosition = theta6;
        end
        
        function configSolR = ik_right_foot(obj, configuration)
            left_hip_to_right_hip = obj.robot.getTransform(obj.robot.homeConfiguration, 'left_hip_front', 'right_hip_front');

            configuration(1:3,4) = configuration(1:3,4) + left_hip_to_right_hip(1:3,4);
            configSolL = ik_left_foot(obj, configuration);
            
            configSolR = obj.robot_right_leg_subtree.homeConfiguration;
            configSolR(1).JointPosition = -configSolL(1).JointPosition;
            configSolR(2).JointPosition = configSolL(2).JointPosition;
            configSolR(3).JointPosition = configSolL(3).JointPosition;
            configSolR(4).JointPosition = configSolL(4).JointPosition;
            configSolR(5).JointPosition = configSolL(5).JointPosition;
            configSolR(6).JointPosition = configSolL(6).JointPosition;        
        end
        
        function stepPath(obj, t, robot_path)
            tstart = tic;
            crotch_position = robot_path.crotchPosition(t);
            tend1 = toc(tstart);
            [left_foot_position, right_foot_position] = robot_path.footPosition(t);
            tend2 = toc(tstart);
                        
            torso_to_right_foot = crotch_position \ right_foot_position;
            torso_to_left_foot = crotch_position \ left_foot_position;
        
%             weights_left = [0.25 0.25 0.25 1 1 1];
%             [configSolL1,err] = obj.ik_left('left_foot',torso_to_left_foot,weights_left,obj.robot_left_leg_subtree.homeConfiguration);
%             weights_right = [0.25 0.25 0.25 1 1 1];
%             [configSolR1,~] = obj.ik_right('right_foot',torso_to_right_foot,weights_right,obj.robot_right_leg_subtree.homeConfiguration);
%             assert(err.ExitFlag == 1)
            
            configSolL = obj.ik_left_foot(torso_to_left_foot);
            configSolR = obj.ik_right_foot(torso_to_right_foot);
            
            obj.configuration(3:8) = configSolL;
            obj.configuration(13:18) = configSolR;
            
            obj.pose = Geometry.transform(crotch_position);
            tend3 = toc(tstart);
            
            fprintf('Torso Time: %f, Foot Path Time: %f, IK Time: %f\n\n', tend1, tend2, tend3);
        end
        
        function angles = getAngles(obj)
            angles = [obj.configuration.JointPosition];
        end
        
        function show(obj)
            maprobot = robotics.RigidBodyTree;
            maplink = robotics.RigidBody('torso');
            maplink.Joint.setFixedTransform(obj.pose.H);
            maprobot.addBody(maplink, maprobot.BaseName);
            maprobot.addSubtree('torso', obj.robot);
            maprobot.show(obj.configuration);
            patch( [-3 -3 3 3 -3], [5 -5 -5 5 5], [0 0 0 0 0], 'green')
        end
    end
end

