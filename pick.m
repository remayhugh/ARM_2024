function grip_result = pick(strategy,mat_R_T_M,optns)
    %----------------------------------------------------------------------
    % pick 
    % Top-level function to executed a complete pick. 
    % 
    % 01 Calls moveTo to move to desired pose
    % 02 Calls doGrip to execute a grip
    %
    % Inputs
    % mat_R_T_M [4x4]: object pose wrt to base_link
    % mat_R_T_G  [4x4]: gripper pose wrt to base_link used as starting point in ctraj (optional)    
    % optns (dict): options 
    %
    % Outputs:
    % ret (bool): 0 indicates success, other failure.
    %----------------------------------------------------------------------
    
    %% 1. Init
    % z_offset = optns("z_offset");
    % z_offset = z_offset{1};
    grip_result                = -1;           % Init to failure number  
    
    %% 2. Move to desired location
    if strcmp(strategy,'topdown')
        
        % Hover over
        over_R_T_M = lift(mat_R_T_M, optns);
        traj_result = moveTo(over_R_T_M, optns);
        
        % Descend
        if ~traj_result
            traj_result = moveTo(mat_R_T_M,optns);
        else
            error('Trajectory failed with result %d', int2str(traj_result));            
        end

    elseif strcmpi(strategy,'direct')
        traj_result = moveTo(mat_R_T_M,optns);
    end


    %% 3. Pick if successfull (check structure of resultState). Otherwise...
    if ~traj_result
        [grip_result,~] = doGrip('pick',optns); 
        grip_result = grip_result.ErrorCode;
    else
        error('Grip command failed with result %d', int2str(grip_result));
    end
end