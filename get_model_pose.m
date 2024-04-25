function pos = get_model_pose(model_name,optns)
%--------------------------------------------------------------------------
% getModels
% This method will create an action client that talks to Gazebo's
% get_model_state action server to retrieve the pose of a given model_name wrt to the world.
%
% Inputs
% model_name (string): name of existing model in Gazebo
%
% Ouput
% models (gazebo_msgs/GetModelStateResponse): contains Pose and Twist
% structures
%--------------------------------------------------------------------------
    r = optns('rHandle');
    r = r{1};

% 02 Create model_client_msg
get_models_client_msg = rosmessage(r.get_models_client);

% 03 Populate message with model name
get_models_client_msg.ModelName = model_name;

% 04 Call client 
try
    [pos,status] = call(r.get_models_client,get_models_client_msg);
catch
    disp('Error - model could not be found')
    r.pose
end