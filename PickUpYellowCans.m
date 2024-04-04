% Pick and place yellow cans

%% 00 Connect to ROS 
clc
clear
rosshutdown;
pause(2);     
masterhostIP = "172.16.54.129";
rosinit(masterhostIP)

%% 02 Go Home
disp('Going home...');
goHome('qr');    % moves robot arm to a q-ready position

disp('Resetting the world...');
resetWorld;      % reset models through a gazebo service

pause(2)

%% 03 Get Pose
disp('Getting goal...')
type = 'gazebo'; % gazebo, ptcloud, cam, manual

% Via Gazebo
if strcmp(type,'gazebo')
    models = getModels;                         % Extract gazebo model list
    model_name = models.ModelNames{27};         % yCan1

    fprintf('Picking up model: %s \n',model_name);
    [mat_R_T_G, mat_R_T_M] = get_robot_object_pose_wrt_base_link(model_name);
end 


%% 04 Pick Model
strategy = 'topdown';
ret = pick(strategy, mat_R_T_M); 

%% 05 Place
if ~ret
    disp('Attempting to place yCan1...')
    greenBin = [-0.4, -0.45, 0.25, -pi/2, -pi 0];
    place_pose = set_manual_goal(greenBin);
    strategy = 'topdown';
    fprintf('Moving to bin...');
    ret = moveToBin(strategy,mat_R_T_M,place_pose);
end
%% Return to home
if ~ret
    ret = moveToQ('qr');
end
%% 03 Get Pose
disp('Getting goal...')
type = 'manual';

% Via Gazebo
if strcmp(type,'manual')
   goal = [0.4828, 0.6504, 0.1, pi/4, -pi 0];    %[px,py,pz, z y z]                  
    mat_R_T_M = set_manual_goal(goal);        % yCan2
end 

%% 04 Pick Model
strategy = 'topdown';
ret = pick(strategy, mat_R_T_M);

%% 05 Place
if ~ret
    disp('Attempting to place yCan2...')
    greenBin = [-0.4, -0.45, 0.25, -pi/2, -pi 0];
    place_pose = set_manual_goal(greenBin);
    strategy = 'topdown';
    fprintf('Moving to bin...');
    ret = moveToBin(strategy,mat_R_T_M,place_pose);
end

%% Return to home
if ~ret
    ret = moveToQ('qr');
end

pause (2)

%% 03 Get Pose
disp('Getting goal...')
type = 'gazebo'; 

% Via Gazebo
if strcmp(type,'gazebo')
    models = getModels;                         % Extract gazebo model list
    model_name = models.ModelNames{29};         % yCan3

    fprintf('Picking up model: %s \n',model_name);
    [mat_R_T_G, mat_R_T_M] = get_robot_object_pose_wrt_base_link(model_name);
end 

%% 04 Pick Model
strategy = 'topdown';
ret = pick(strategy, mat_R_T_M);

%% 05 Place
if ~ret
    disp('Attempting place...')
    greenBin = [-0.4, -0.45, 0.25, -pi/2, -pi 0];
    place_pose = set_manual_goal(greenBin);
    strategy = 'topdown';
    fprintf('Moving to bin...');
    ret = moveToBin(strategy,mat_R_T_M,place_pose);
end
%% Return to home
if ~ret
    ret = moveToQ('qr');
end

%% 03 Get Pose
disp('Getting goal...')
type = 'manual';

% Via Gazebo
if strcmp(type,'manual')
   goal = [0.4244,0.3669,0.10, pi/4, -pi 0];    %[px,py,pz, z y z]                  
    mat_R_T_M = set_manual_goal(goal);        % yCan4
end 

%% 04 Pick Model
strategy = 'topdown';
ret = pick(strategy, mat_R_T_M);

%% 05 Place
if ~ret
    disp('Attempting to place yCan4...')
    greenBin = [-0.4, -0.45, 0.25, -pi/2, -pi 0];
    place_pose = set_manual_goal(greenBin);
    strategy = 'topdown';
    fprintf('Moving to bin...');
    ret = moveToBin(strategy,mat_R_T_M,place_pose);
end
%% Return to home
if ~ret
    ret = moveToQ('qr');
end

