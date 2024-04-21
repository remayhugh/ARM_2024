function resetWorld
%--
% Calls /gazebo
r = rosClassHandle;
% res_client = rossvcclient('/gazebo/reset_world', 'std_srvs/Empty', 'DataFormat', 'struct');
disp('Resetting the world...');
ros_client_msg = rosmessage(r.res_client);
call(r.res_client, ros_client_msg);