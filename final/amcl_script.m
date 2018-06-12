%% Localize TurtleBot Using Monte Carlo Localization

rosshutdown
clear

start = [2 2];
goal = [4 5];

ipaddress = '172.16.205.128';
rosinit(ipaddress);

% gazebo = ExampleHelperGazeboCommunicator();
% models = getSpawnedModels(gazebo);
% turtlebot = ExampleHelperGazeboSpawnedModel('mobile_base',gazebo);
% [position, orientation] = getState(turtlebot)

% Gera o Binary Occupancy Grid do espaco modelado do LaSER
map = createOccupancyGrid ('laser2.png');
%show(map);

% Gerador de caminho: Algoritmo A estrela
waypoints = wrapper_a_star(start, goal, map.GridSize);
% Converte os waypoints de coordenadas de grid para coordenadas x e y
waypoints = waypoints / map.Resolution;
% Calcula os angulos (em radianos)
tethas = atan (waypoints(:, 2) ./ waypoints(:, 1));
% Acrescenta no vetor de waypoints
waypoints = [waypoints tethas];
% Index of active waypoint
widx = 2;
% Set a distance metric for reaching a waypoint
distThreshold = 0.3;

% Cinematica inversa
linvels = sqrt((diff(waypoints(:, 1))).^2 + (diff(waypoints(:, 2))).^2);
angvels = diff(waypoints(:, 3));

% Setup the Laser Sensor Model and TurtleBot Motion Model
odometryModel = robotics.OdometryMotionModel;
odometryModel.Noise = [0.2 0.2 0.2 0.2];

% The sensor on TurtleBot is a simulated range finder converted from
% Kinect readings.
rangeFinderModel = robotics.LikelihoodFieldSensorModel;
rangeFinderModel.SensorLimits = [0.45 8];
rangeFinderModel.Map = map;

% Query the Transformation Tree (tf tree) in ROS.
tftree = rostf;
waitForTransform(tftree,'/base_link','/camera_depth_frame');
sensorTransform = getTransform(tftree,'/base_link', '/camera_depth_frame');

% Get the euler rotation angles.
laserQuat = [sensorTransform.Transform.Rotation.W sensorTransform.Transform.Rotation.X ...
    sensorTransform.Transform.Rotation.Y sensorTransform.Transform.Rotation.Z];
laserRotation = quat2eul(laserQuat, 'ZYX');

% Setup the |SensorPose|
rangeFinderModel.SensorPose = ...
    [sensorTransform.Transform.Translation.X sensorTransform.Transform.Translation.Y laserRotation(1)];

% Interface for Receiving Sensor Measurements From TurtleBot and Sending Velocity Commands to TurtleBot.
laserSub = rossubscriber('scan');
odomSub = rossubscriber('odom');

% Create ROS publisher for sending out velocity commands to TurtleBot.
[velPub,velMsg] = ...
    rospublisher('/mobile_base/commands/velocity','geometry_msgs/Twist');
pause(2); % Wait to ensure publisher and subscriber are registered

%% Initialize AMCL Algorithm
amclMCLObj = robotics.algs.internal.MonteCarloLocalization.empty;
%RecoveryAlphaSlow Recovery parameter
amclRecoveryAlphaSlow = 0;
%RecoveryAlphaFast Recovery parameter
amclRecoveryAlphaFast = 0;
%KLDError Maximum KLD distribution error
amclKLDError = 0.05;
%KLDZ KLD parameter
amclKLDZ = 0.99;

amclUseLidarScan = true;

% Assign the |MotionModel| and |SensorModel| properties in the |amcl| object.
amclMotionModel = odometryModel;
amclSensorModel = rangeFinderModel;

% The particle filter only updates the particles when the robot's movement exceeds
% the |UpdateThresholds|
amclUpdateThresholds = [0.2,0.2,0.2];
amclResamplingInterval = 1;

% Configure AMCL Object for Localization with Initial Pose Estimate.
amclParticleLimits = [500 5000];
amclGlobalLocalization = false;
amclInitialPose = ExampleHelperAMCLGazeboTruePose();
amclInitialCovariance = eye(3)*0.5;


% Setup Helper for Visualization and Driving TurtleBot.
visualizationHelper = myHelperAMCLVisualization(map);

% Setup
randomState = rng;
amclSeed = double(randomState.Seed);
amclMCLObj = robotics.algs.internal.MonteCarloLocalization(amclSeed);

% Initialize by assigning all data to internal object
amclMCLObj.setUpdateThresholds(amclUpdateThresholds(1), ...
    amclUpdateThresholds(2), amclUpdateThresholds(3));
amclMCLObj.setResamplingInterval(amclResamplingInterval);

% Set sensor model
robotics.algs.internal.AccessMCL.setSensorModel(amclMCLObj, amclSensorModel);

% Set motion model
amclMCLObj.setMotionModel(amclMotionModel.Noise);

% Initialize particle filter
amclMCLObj.initializePf(amclParticleLimits(1), ...
    amclParticleLimits(2), amclRecoveryAlphaSlow, ...
    amclRecoveryAlphaFast, amclKLDError, amclKLDZ);

if amclGlobalLocalization
  % Global initialization
  amclMCLObj.globalLocalization();
else
  % Initialize with pose and covariance
  amclMCLObj.setInitialPose(amclInitialPose, ...
    amclInitialCovariance);
end

i = 0;
%% Localization Procedure
while 1
    % Receive laser scan and odometry message.
    scanMsg = receive(laserSub);
    odompose = odomSub.LatestMessage;

    % Create lidarScan object to pass to the AMCL object.
    scan = lidarScan(scanMsg);

    % For sensors that are mounted upside down, you need to reverse the
    % order of scan angle readings using 'flip' function.

    % Compute robot's pose [x,y,yaw] from odometry message.
    odomQuat = [odompose.Pose.Pose.Orientation.W, odompose.Pose.Pose.Orientation.X, ...
        odompose.Pose.Pose.Orientation.Y, odompose.Pose.Pose.Orientation.Z];
    odomRotation = quat2eul(odomQuat);
    pose = [odompose.Pose.Pose.Position.X, odompose.Pose.Pose.Position.Y odomRotation(1)];

    % Update estimated robot's pose and covariance using new odometry and
    % sensor readings.
    %step Estimate robot pose using sensor data
    % Validate inputs
    if amclUseLidarScan
       % Only lidarScan input
       scan = robotics.internal.validation.validateLidarScan(...
            scan, 'step', 'scan');
    end
    validateattributes(pose, {'double'}, ...
       {'vector', 'numel', 3, 'real', 'finite'}, 'step', 'pose');

    % Combined predict, correct and resampling step
    numranges = length(scan.Ranges);
    amclMCLObj.update(numranges, scan.Ranges, scan.Angles, pose)

    % Get the outputs
    isUpdated = amclMCLObj.isUpdated;
    [estimatedPose, estimatedCovariance] = ...
         robotics.algs.internal.AccessMCL.getHypothesis(amclMCLObj);

    % Get x, y and theta from pose estimation
    x = estimatedPose(1);
    y = estimatedPose(2);
    theta = estimatedPose(3);  % In radians!

    % Calculate the distance to the current waypoint
    dist = sqrt((x - waypoints(widx, 1))^2 + (y - waypoints(widx, 2))^2);

    % Evaluate what to do next based on the distance to the waypoint.
    if (dist <= distThreshold)
        if (widx < (size(waypoints, 1) - 1))
            widx = widx + 1
        else
            break;
        end
    else
        % Populate the twist message
        velMsg.Linear.X = linvels(widx-1);
        velMsg.Angular.Z = angvels(widx-1);
        % Publish
        send(velPub,velMsg);
    end

    % Plot the robot's estimated pose, particles and laser scans on the map.
    if isUpdated
        plotStep(visualizationHelper, amclMCLObj, amclSensorModel, estimatedPose, scan, i)
        i = i + 1;
     end

end

velMsg.Linear.X = 0;
velMsg.Angular.Z = 0;
send(velPub,velMsg);

% Stop the TurtleBot and Shutdown ROS in MATLAB
rosshutdown
