%% Localiza o TurtleBot usando Localicao de Monte Carlo
% Exemplo original do MATLAB:
% TurtleBotMonteCarloLocalizationExample

% Define o IP da maquina virtual com o ROS e Gazebo
ipaddress = '172.16.134.130';
% Inicia a conexao do MATLAB com o ROS
rosinit(ipaddress);

%% Load the Map of the Simulation World
% Load a binary occupancy grid of the office environment in Gazebo
filePath = fullfile(fileparts(which('TurtleBotMonteCarloLocalizationExample')),'data','officemap.mat');
load(filePath);
show(map);

%% Setup the Laser Sensor Model and TurtleBot Motion Model
% TurtleBot can be modeled as a differential drive robot and its motion can
% be estimated using odometry data.
odometryModel = robotics.OdometryMotionModel;
odometryModel.Noise = [0.2 0.2 0.2 0.2];

%% The sensor on TurtleBot is a simulated range finder converted from
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

% Setup the |SensorPose|, which includes the translation along base_link's
% +X, +Y direction in meters and rotation angle along base_link's +Z axis
% in radians.
rangeFinderModel.SensorPose = ...
    [sensorTransform.Transform.Translation.X sensorTransform.Transform.Translation.Y laserRotation(1)];

%% Interface for Receiving Sensor Measurements From TurtleBot and Sending Velocity Commands to TurtleBot.
% Create ROS subscribers for retrieving sensor and odometry measurements
% from TurtleBot.
laserSub = rossubscriber('scan');
odomSub = rossubscriber('odom');

%%
% Create ROS publisher for sending out velocity commands to TurtleBot.
% TurtleBot subscribes to |'/mobile_base/commands/velocity'| for velocity
% commands.
[velPub,velMsg] = ...
    rospublisher('/mobile_base/commands/velocity','geometry_msgs/Twist');

%% Rotina AMCL
% Referencia: robotics.MonteCarloLocalization

amclUseLidarScan = true;

% Constantes
% RecoveryAlphaSlow Recovery parameter
amclRecoveryAlphaSlow = 0;
% RecoveryAlphaFast Recovery parameter
amclRecoveryAlphaFast = 0;
% KLDError Maximum KLD distribution error
amclKLDError = 0.05;
% KLDZ KLD parameter
amclKLDZ = 0.99;

amclPrivateParticleLimits = [500 5000];
amclPrivateInitialPose = [0 0 0];
amclPrivateInitialCovariance = eye(3);
amclPrivateResamplingInterval = 1;
amclPrivateUseLidarScan = false;
amclPrivateUpdateThresholds = [0.2 0.2 0.2];
amclPrivateGlobalLocalization = false;
amclSeed;

%% Inicializacao do algoritmo do AMCL
amclPrivateSensorModel = robotics.LikelihoodFieldSensorModel;
amclPrivateMotionModel = robotics.OdometryMotionModel;
MCLObj = robotics.algs.internal.MonteCarloLocalization.empty;

try
   [particles, weights] = robotics.algs.internal.AccessMCL.getParticles(MCLObj);
catch ex
   throw(ex);
end

%%
% Assign the |MotionModel| and |SensorModel| properties in the |amcl| algorithm.
amclMotionModel = odometryModel;
amclSensorModel = rangeFinderModel;

%%
% The particle filter only updates the particles when the robot's movement exceeds
% the |UpdateThresholds|, which defines minimum displacement in [x, y, yaw]
% to trigger filter update. This prevents too frequent updates due to
% sensor noise.
% Particle resampling happens after the |amclResamplingInterval| filter updates.
% Using larger numbers leads to slower particle depletion at the price of slower
% particle convergence as well.
amclUpdateThresholds = [0.2,0.2,0.2];
amclResamplingInterval = 1;

%% Configure AMCL algorithm for Localization with Initial Pose Estimate.
% |amclParticleLimits| defines the lower and upper bound on the number of
% particles that will be generated during the resampling process. Allowing more
% particles to be generated may improve the chance of converging to the
% true robot pose, but has an impact on computation speed and particles may
% take longer time or even fail to converge.
% Please refer to the 'KL-D Sampling' section in [1] for computing a
% reasonable bound value on the number of particles.
% Note that global localization may need significantly more particles compared
% to localization with an initial pose estimate.
% If the robot knows its initial pose with some uncertainty, such
% additional information can help AMCL localize robots faster with a less
% number of particles, i.e. you can use a smaller value of upper bound in
% |amclParticleLimits|.
%
% Now set |amclGlobalLocalization| to false and provide an estimated
% initial pose to AMCL. By doing so, AMCL holds the initial
% belief that robot's true pose follows a Gaussian distribution with a mean
% equal to |amclInitialPose| and a covariance matrix equal to
% |amclInitialCovariance|.
% Initial pose estimate should be obtained according to your setup. This
% example helper retrieves the robot's current true pose from Gazebo.
amclParticleLimits = [500 5000];
amclGlobalLocalization = false;
amclInitialPose = ExampleHelperAMCLGazeboTruePose();
amclInitialCovariance = eye(3)*0.5;

%% Setup Helper for Visualization and Driving TurtleBot.
% Setup ExampleHelperAMCLVisualization to plot the map and update robot's
% estimated pose, particles, and laser scan readings on the map.
visualizationHelper = ExampleHelperAMCLVisualization(map);

%%
% Robot motion is essential for the AMCL algorithm. In this example, we drive
% TurtleBot randomly using the ExampleHelperAMCLWanderer class, which
% drives the robot inside the environment while avoiding obstacles using the
% |<docid:robotics_ref.buv7g7y robotics.VectorFieldHistogram>| class.
wanderHelper = ExampleHelperAMCLWanderer(laserSub, sensorTransform, velPub, velMsg);

%% Localization Procedure
% The AMCL algorithm is updated with odometry and sensor readings at each
% time step when the robot is moving around.
% Please allow a few seconds before particles are initialized and plotted
% in the figure.
% In this example we will run |numUpdates| AMCL updates. If the robot
% doesn't converge to the correct robot pose, consider using a larger
% |numUpdates|.
numUpdates = 60;
i = 0;
while i < numUpdates
    % Receive laser scan and odometry message.
    scanMsg = receive(laserSub);
    odompose = odomSub.LatestMessage;

    % Create lidarScan object to pass to the AMCL algorithm.
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
    [isUpdated,estimatedPose, estimatedCovariance] = amcl(pose, scan);

    % Drive robot to next pose.
    wander(wanderHelper);

    % Plot the robot's estimated pose, particles and laser scans on the map.
    if isUpdated
        i = i + 1;
        plotStep(visualizationHelper, amcl, estimatedPose, scan, i)
    end

end


%% Stop the TurtleBot and Shutdown ROS in MATLAB
stop(wanderHelper);
rosshutdown

%% Configure AMCL algorithm for Global Localization.
% In case no initial robot pose estimate is available, AMCL will try to
% localize robot without knowing the robot's initial position. The
% algorithm initially assumes that the robot has equal probability in being
% anywhere in the office's free space and generates uniformly distributed
% particles inside such space. Thus Global localization requires
% significantly more particles compared to localization with initial pose
% estimate.
%
% To enable AMCL global localization feature, replace the code sections in
% *Configure AMCL object for localization with initial pose estimate* with
% the code in this section.
%%
%       amclGlobalLocalization = true;
%       amclParticleLimits = [500 50000];

displayEndOfDemoMessage(mfilename)
