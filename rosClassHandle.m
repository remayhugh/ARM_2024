classdef rosClassHandle < handle
% Class definition — Description of what is common to every instance of a
% class.

% Properties — Data storage for class instancee
    properties       
        %FlowChart
        % Robot
        % World                       = {};
        % 
        % % ROS Structures
        % ROSinfo
        % 
        % % Manipulator Information
        % base
        % RobotEndEffector
        % CurrentRobotJConfig
        % CurrentRobotTaskConfig
        % NextPart = 0;
        % HomeRobotTaskConfig 
        % PlacingPose
        % GraspPose
        % NumJoints
        % NumDetectionRuns        = 0;
        
        % services
        joint_state_sub         = rossubscriber("/joint_states");
        point                   = rosmessage('trajectory_msgs/JointTrajectoryPoint', 'DataFormat','struct');
        pose                    = rosmessage('gazebo_msgs/GetModelStateResponse', 'DataFormat','struct');
        res_client              = rossvcclient('/gazebo/reset_world', 'std_srvs/Empty', 'DataFormat', 'struct');
        TimeFromStart           = rosduration(1,'DataFormat','struct');
        trajPts                 = rosmessage('trajectory_msgs/JointTrajectoryPoint','DataFormat', 'struct');
        trajPtsVar              = rosmessage('trajectory_msgs/JointTrajectoryPoint');

        % Actions
        get_models_client       = rossvcclient('/gazebo/get_model_state','DataFormat','struct');
        grip_action_client      = rosactionclient('/gripper_controller/follow_joint_trajectory', ...
                                                    'control_msgs/FollowJointTrajectory',...
                                                    'DataFormat','struct');
        models                  = rosmessage('gazebo_msgs/GetWorldPropertiesResponse', 'DataFormat','struct');
        pick_traj_act_client    = rosactionclient('/pos_joint_traj_controller/follow_joint_trajectory',...
                                                   'control_msgs/FollowJointTrajectory', ...
                                                   'DataFormat', 'struct');
        %YOLO
        rgb_sub       = rossubscriber('/camera/rgb/image_raw');
        pt_cloud_sub  = rossubscriber('/camera/depth/points');

    end
    methods (Access = public)
    end
end