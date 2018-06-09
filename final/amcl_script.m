%% Localize TurtleBot Using Monte Carlo Localization

rosshutdown
clear

ipaddress = '172.16.205.128';
rosinit(ipaddress);

%gazebo = ExampleHelperGazeboCommunicator();
%models = getSpawnedModels(gazebo);
%turtlebot = ExampleHelperGazeboSpawnedModel('mobile_base',gazebo);
%[position, orientation] = getState(turtlebot)

%% Gera o Binary Occupancy Grid do espaco modelado do LaSER

%% Load the Map of the Simulation World
map = createOccupancyGrid ('laser.png');

%filePath = fullfile(fileparts(which('TurtleBotMonteCarloLocalizationExample')),'data','officemap.mat');
%load(filePath);

%show(map);

%% Setup the Laser Sensor Model and TurtleBot Motion Model
odometryModel = robotics.OdometryMotionModel;
odometryModel.Noise = [0.2 0.2 0.2 0.2];

%%
% The sensor on TurtleBot is a simulated range finder converted from
% Kinect readings.
rangeFinderModel = robotics.LikelihoodFieldSensorModel;
rangeFinderModel.SensorLimits = [0.45 8];
rangeFinderModel.Map = map;

%%
% Set |rangeFinderModel.SensorPose| to the coordinate transform of the fixed
% camera with respect to the robot base. This is used to transform the
% laser readings from camera frame to the base frame of TurtleBot. Please
% refer to <docid:robotics_examples.example-ROSTransformationTreeExample>
% for details on coordinate transformations.
%
% Note that currently |SensorModel| is only compatible with sensors that are
% fixed on the robot's frame, which means the sensor transform is constant.

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

%%
% Assign the |MotionModel| and |SensorModel| properties in the |amcl| object.
amclMotionModel = odometryModel;
amclSensorModel = rangeFinderModel;

%%
% The particle filter only updates the particles when the robot's movement exceeds
% the |UpdateThresholds|, which defines minimum displacement in [x, y, yaw]
% to trigger filter update. This prevents too frequent updates due to
% sensor noise.
% Particle resampling happens after the |amcl.ResamplingInterval| filter updates.
% Using larger numbers leads to slower particle depletion at the price of slower
% particle convergence as well.
amclUpdateThresholds = [0.2,0.2,0.2];
amclResamplingInterval = 1;

%% Configure AMCL Object for Localization with Initial Pose Estimate.
% |amcl.ParticleLimits| defines the lower and upper bound on the number of
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
% |amcl.ParticleLimits|.
%
% Now set |amcl.GlobalLocalization| to false and provide an estimated
% initial pose to AMCL. By doing so, AMCL holds the initial
% belief that robot's true pose follows a Gaussian distribution with a mean
% equal to |amcl.InitialPose| and a covariance matrix equal to
% |amcl.InitialCovariance|.
% Initial pose estimate should be obtained according to your setup. This
% example helper retrieves the robot's current true pose from Gazebo.
%
% Please refer to section *Configure AMCL object for global localization*
% for an example on using global localization.

amclParticleLimits = [500 5000];
amclGlobalLocalization = false;
amclInitialPose = ExampleHelperAMCLGazeboTruePose();
amclInitialCovariance = eye(3)*0.5;


%% Setup Helper for Visualization and Driving TurtleBot.
% Setup ExampleHelperAMCLVisualization to plot the map and update robot's
% estimated pose, particles, and laser scan readings on the map.
visualizationHelper = myHelperAMCLVisualization(map);

%%
% Robot motion is essential for the AMCL algorithm. In this example, we drive
% TurtleBot randomly using the ExampleHelperAMCLWanderer class, which
% drives the robot inside the environment while avoiding obstacles using the
% |<docid:robotics_ref.buv7g7y robotics.VectorFieldHistogram>| class.
wanderHelper = ...
    ExampleHelperAMCLWanderer(laserSub, sensorTransform, velPub, velMsg);

    % Setup
   randomState = rng;
   amclSeed = double(randomState.Seed);
   amclMCLObj = robotics.algs.internal.MonteCarloLocalization(amclSeed);

   %initialize Initialize by assigning all data to internal object
   amclMCLObj.setUpdateThresholds(amclUpdateThresholds(1), ...
        amclUpdateThresholds(2), amclUpdateThresholds(3));
   amclMCLObj.setResamplingInterval(amclResamplingInterval);

   % Set sensor model
   robotics.algs.internal.AccessMCL.setSensorModel( ...
        amclMCLObj, amclSensorModel);

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

    % Drive robot to next pose.
    wander(wanderHelper);

    % Plot the robot's estimated pose, particles and laser scans on the map.
    if isUpdated
        i = i + 1;
        plotStep(visualizationHelper, amclMCLObj, amclSensorModel, estimatedPose, scan, i)
     end

end


%% Stop the TurtleBot and Shutdown ROS in MATLAB
stop(wanderHelper);
rosshutdown

%% Sample Results for AMCL Localization with Initial Pose Estimate
% _AMCL is a probabilistic algorithm, the simulation result on your
% computer may be slightly different from the sample run shown here._
%
% After first AMCL update, particles are generated by sampling Gaussian
% distribution with mean equal to |amcl.InitialPose| and covariance equal
% to |amcl.InitialCovariance|.
%
% <<initial_particle_distribution_local.png>>
%
%%
% After 8 updates, the particles start converging to areas with higher
% likelihood:
%
% <<converging_particles_local.png>>
%
%%
% After 60 updates, all particles should converge to the correct robot pose
% and the laser scans should closely align with the map outlines.
%
% <<robot_localized_local.png>>
%

%% Configure AMCL Object for Global Localization.
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
%       amcl.GlobalLocalization = true;
%       amcl.ParticleLimits = [500 50000];

%% Sample Results for AMCL Global Localization
% _AMCL is a probabilistic algorithm, the simulation result on your
% computer may be slightly different from the sample run shown here._
%
% After first AMCL update, particles are uniformly distributed inside the
% free office space:
%
% <<initial_particle_distribution_global.png>>
%
%%
% After 8 updates, the particles start converging to areas with higher
% likelihood:
%
% <<converging_particles_global.png>>
%
%%
% After 60 updates, all particles should converge to the correct robot pose
% and the laser scans should closely align with the map outlines.
%
% <<robot_localized_global.png>>

%% See Also
% <docid:robotics_ref.bu31hfz-1 robotics.MonteCarloLocalization>,
% <docid:robotics_examples.example-MappingWithKnownPosesExample Mapping With Known Poses>.

%% References
% [1] S. Thrun, W. Burgard and D. Fox, Probabilistic Robotics.
% Cambridge, MA: MIT Press, 2005.
