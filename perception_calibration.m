%**********************************************************************
% FUNCTION FOR CALIBRATING THE EXTEROCEPTIVE SENSORS OF THE ROBOTS
% Apolo environment: 'Exteroceptive_sensors_calibration.xml'
%**********************************************************************

close all;
clear

%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Ultrasonic Sensor
%%%%%%%%%%%%%%%%%%%%%%%%%%
apoloPlaceMRobot('Marvin',[1.8,0,0],0)
% Initialize an empty array to store ultrasonic measurements
ultrasonicMeasures = [];

% Set the number of samples
numSamples = 1000;

% Acquire ultrasonic measurements and store them in the array
for i = 1:numSamples
    ultrasonicMeasures(end+1) = apoloGetUltrasonicSensor('uc0');
end

% Plot a histogram of the ultrasonic sensor measurements
% figure;
% hold on; 
% grid on; 
% plot(ultrasonicMeasures);

% Add a title to the histogram
% title('Plot of Ultrasonic Sensor Measurements');
% xlabel('Measures');
% ylabel('Distance (m)');

% Calculate the variance of the ultrasonic measurements
ultrasonicVar = var(ultrasonicMeasures);

%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Laser Points Cloud
%%%%%%%%%%%%%%%%%%%%%%%%%%
apoloPlaceMRobot('Marvin',[0,0,0],0)
% Initialize an empty array to store laser cloud data
laserCloud = [];

% Set the number of samples
numSamples = 1000;

% Acquire laser data and store it in the array
for i = 1:numSamples
    laserCloud(end+1, :) = apoloGetLaserData('LMS100');
end

% Calculate the variance for a specific laser beams
laserVarBeam = []
beams = [90 270 450]
for i = beams
    laserVarBeam(end+1) = var(laserCloud(:, i));
end

% Plot a histogram of the laser beam variances
% figure; 
% hold on; 
% grid on; 
% histogram(laserVarBeam);

%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Laser Landmarks
%%%%%%%%%%%%%%%%%%%%%%%%%%

% Set the number of samples
numSamples = 1000;

% Initialize cell array to store laser measures
laserMeasures = cell(1, numSamples);

% Acquire laser measures and store them in the cell array
for i = 1:numSamples
    laserMeasures{i} = apoloGetLaserLandMarks('LMS100');
end

% Variance calculation
laserAngle = zeros(1, numSamples);
laserDistance = zeros(1, numSamples);

for i = 1:numSamples
    laserAngle(i) = laserMeasures{i}.angle(1);
    laserDistance(i) = laserMeasures{i}.distance(1);
end

% Calculate variance
laserAngleVar = var(laserAngle);
laserDistanceVar = var(laserDistance);

% Calculate mean
laserAngleMean = mean(laserAngle);
laserDistanceMean = mean(laserDistance);

% Create Gaussian distributions
x = linspace(min(laserAngle), max(laserAngle), numSamples);
y = linspace(min(laserDistance), max(laserDistance), numSamples);
gaussAngle = exp(-(x - laserAngleMean).^2 / (2 * laserAngleVar));
gaussDistance = exp(-(y - laserDistanceMean).^2 / (2 * laserDistanceVar));

% Normalize Gaussian distributions
gaussAngle = gaussAngle / trapz(x, gaussAngle);
gaussDistance = gaussDistance / trapz(y, gaussDistance);

% Plot Gaussian distributions
figure;
subplot(2, 1, 1);
plot(x, gaussAngle);
title('Gaussian Distribution of Laser Angles');
xlabel('Angle (rad)');
ylabel('Probability');

subplot(2, 1, 2);
plot(y, gaussDistance);
title('Gaussian Distribution of Laser Distances');
xlabel('Distance (m)');
ylabel('Probability');

%% Saving data
save('sensors_error.mat',"ultrasonicVar","laserDistanceVar","laserAngleVar");
