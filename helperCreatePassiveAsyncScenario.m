function ts = helperCreatePassiveAsyncScenario()
% This is a helper function for the example showing passive angle-only
% tracking with async sensors using GM-PHD tracker. It may be removed or
% modified in a future release.

% Copyright 2022 The MathWorks, Inc.

% Create scenario. UpdateRate = 0 allows the sensor to control the
% simulation time
ts = trackingScenario(UpdateRate=0,...
    StopTime=60);

% Create an ESM sensor
esm = fusionRadarSensor(1, 'No scanning', ...
    'DetectionMode', 'ESM', ...
    'FieldOfView',[180 1],...
    'HasElevation',true,...
    'MountingAngles',[90 0 0],...
    'HasNoise',false,...
    'HasFalseAlarms',true,...
    'HasINS',true,...
    'FalseAlarmRate',1e-3,...
    'AzimuthResolution',0.07,...
    'UpdateRate',1,...
    'ElevationResolution',1);

% Create a large jammer for each target
jammer = radarEmitter(1,'UpdateRate',3,'ScanMode','No scanning','FieldOfView',[360 180],'EIRP',100);

% Add targets

numTargets = 5;
% Targets in a straight line
yTgt = 5e2*ones(1,numTargets)*5;
xTgt = linspace(-800,800,numTargets)*5;

% Pick a velocity for each target in the plane.
velocity = [-5 5;
            -3 3;
            0 5;
            3 3;
            5 5]*2*5;

for i = 1:numTargets
    traj = kinematicTrajectory('Position',...
        [xTgt(i) yTgt(i) 0],'Velocity',[velocity(i,:) 0]);
    tgtPlat = platform(ts,'Trajectory',traj);
    tgtPlat.Emitters = {clone(jammer)};
    tgtPlat.Emitters{1}.EmitterIndex = i;
end

% Add sensing platforms
r = 8e3;
numSensors = 3;
theta = [-5*pi/28 0 5*pi/28] + pi/2;
xSen = r*cos(theta);
ySen = r*sin(theta)-8e3;
locationIDs = [3 2 1];

for i = 1:numSensors
    thisID = locationIDs(i);
    wp = repmat([xSen(thisID) ySen(thisID) 0],2,1);
    % Make platform appear at slightly different time stamp to simulate
    % async sensors 
    tStart = 1/3*i;
    tEnd = 60;
    toa = [tStart tEnd]; 
    traj = waypointTrajectory(wp,toa);
    senPlat = platform(ts,'Trajectory',traj);
    senPlat.Sensors = {clone(esm)};
    senPlat.Sensors{1}.SensorIndex = i;
    senPlat.Signatures{1} = rcsSignature('Azimuth',[-180 180],'Elevation',[-90;90],'Pattern',-realmax*ones(2));
end

end