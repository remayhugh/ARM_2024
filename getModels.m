function models = getModels(optns)
%-------------------------------------------------------------------------- 
% getModels
% This method will create an action client that talks to Gazebo's
% get_world_properties to get all models.
%
% Inputs
% None
%
% Output
% models (gazebo_msgs/GetWorldPropertiesResponse): has cell of ModelNames
%--------------------------------------------------------------------------
    r = optns('rHandle');
    r = r{1};

% 01 Create get_model_state action client
% 02 Create model_client_msg 
get_models_client_msg = rosmessage(r.get_models_client);

% 03 Call client 
try
    [models,~,~] = call(r.get_models_client,get_models_client_msg);
catch
    disp('Error - models not listed')
    r.models
    % models = rosmessage('gazebo_msgs/GetWorldPropertiesResponse', 'DataFormat','struct');
end