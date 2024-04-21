function [res,state] = doGrip(type,optns)
%--------------------------------------------------------------------------
% Tell gripper to either pick or place via the ros gripper action client
%
% Input: type (string) - 'pick' or 'place'
% Output: actoin result and state
%--------------------------------------------------------------------------

    %% Init
    r = optns('rHandle');
    r = r{1};
    % Create a gripper goal action message
    grip_msg = rosmessage(r.grip_action_client);

    %% Testing if setting FeedbackFcn to 0 minimizes the loss connection
    r.grip_action_client.FeedbackFcn = [];

    %% Set Grip Pos by default to pick / close gripper
    gripPos = 0.23; %0.517; % 0.23 for upright cans tends to slip. 

    if nargin==0
        type = 'pick';
    end

    % Modify it if place (i.e. open)
    if strcmp(type,'place')
        gripPos = 0;           
    end

    %% Pack gripper information intro ROS message
    grip_goal = packGripGoal_struct(gripPos,grip_msg,optns);%%%%%%%%

    %% Send action goal
    disp('Sending grip goal...');

    if waitForServer(r.grip_action_client)
        disp('Connected to action server. Sending goal...')
        [res,state,~] = sendGoalAndWait(r.grip_action_client,grip_goal);
    else
        % Re-attempt
        disp('First try failed... Trying again...');
        [res,state,~] = sendGoalAndWait(r.grip_action_client,grip_goal);
    end    

    %% Clear grip_action_client: checking to see if this minimizes ROS network connection errors
    clear r.grip_action_client;
end