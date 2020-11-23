function [ isOK ] = configGenerator( aCalName, aOutName, aRobotName, aBoardID, aDigital, aDirection)
% Summary of this function goes here
% Detailed explanation goes here

% Set isOK to false by default
% if configuration file generation is good then isOK = true
% otherwise isOK = false
isOK = false;

% sanity check for board id
if (length(aBoardID) ~= 2)
    disp('ERROR: board id size should be 2');
    cleanUp;
    return;
elseif (aBoardID(1) == aBoardID(2))
    disp('ERROR: board id should be different');
    cleanUp;
    return;
end
boardID = aBoardID; % save to local variable


% create temporary .m file
% the configuration file uses matlab syntax, save them to temp.m
copyfile(aCalName, 'calibration_file_temp.m');

% define some constants for mXXX.cal/pXXX.cal/eXXX.cal file
UPPER_LIMIT = 1;
LOWER_LIMIT = 2;

% constants for MTM configuration file (.cal)
MST_JNT_POS_GR_DOFS = 8;
MST_MOT_DOFS = 8;
MST_JNT_POS_DOFS = 7;

% constants for PSM configuration file (.cal)
SLV_JNT_POS_GR_DOFS = 7;
SLV_MOT_DOFS = 7;

% constants for ECM configuration file (.cal)
ECM_JNT_POS_GR_DOFS = 4;
ECM_MOT_DOFS = 4;

% load temp file, this will load all constants in ***.cal file
try
    calibration_file_temp;
catch err
    msgbox('ERROR: syntax error in calibration file - check Matlab console');
    msgbox(err.message);
    err.message
    cleanUp;
end % try/catch error loading cal file

% sanity check for RobotName/Type
% rType = robot type
CONST_MTM = 1;
CONST_PSM = 2;
CONST_ECM = 3;
if (strcmp(aRobotName(1:3), FileType(1:3)) == 0)
    disp(' ERROR: robot type and calibration file mismatch');
    cleanUp;
    return;
else
    if (strcmp(aRobotName(1:3), 'MTM'))
        rType = CONST_MTM;
    elseif (strcmp(aRobotName(1:3), 'PSM'))
        rType = CONST_PSM;
    elseif (strcmp(aRobotName(1:3), 'ECM'))
        rType = CONST_ECM;
    end
end

% assign numOfActuator & numOfJoint based on robot type
if (rType == CONST_MTM)
    numOfJoints = 7;
elseif (rType == CONST_PSM)
    numOfJoints = 7;
elseif (rType == CONST_ECM)
    numOfJoints = 4;
end


% Arms, all arrays use size 7 internally to simplify code until XML
% generation

% motor default current Unit: A
motorDefCur(CONST_MTM,:) = [0.67 0.67 0.67 0.67 0.59 0.59 0.407];
motorDefCur(CONST_PSM,:) = [1.34 1.34 0.67 0.67 0.67 0.67 0.670];
motorDefCur(CONST_ECM,:) = [0.943 0.943 0.67 0.59 0.0 0.0 0.0];

% motor torque const  Unit: Nm/A
% NOTE: no motor on axis 8, set value to 1
motorTor(CONST_MTM,:) = [0.0438 0.0438 0.0438 0.0438 0.00495 0.00495 0.00339];
motorTor(CONST_PSM,:) = [0.0438 0.0438 0.0438 0.0438 0.0438 0.0438 0.0438];
motorTor(CONST_ECM,:) = [0.1190 0.1190 0.0438 0.00495 1.0 1.0 1.0];

% Gear ratio
% NOTE: gear ratio for axis 8 is set to 1
gearRatio(CONST_MTM,:) = [63.41 49.88 59.73 10.53 33.16 33.16 16.58];
gearRatio(CONST_PSM,:) = [56.50 56.50 336.6 11.71 11.71 11.71 11.71];
gearRatio(CONST_ECM,:) = [240 240 2748.55 300.15    1.0   1.0   1.0];

% Encoder counts per turn (quadrature encoder)
% NOTE: no encoder for last axis
encCPT(CONST_MTM,:) = [ 4000  4000  4000 4000   64   64   64];
encCPT(CONST_PSM,:) = [14400 14400 14400 4000 4000 4000 4000];
encCPT(CONST_ECM,:) = [ 4000  4000   640   64    1    1    1];

% Pitch
% 1 for revolute, mm/deg for prismatic
pitch(CONST_MTM,:) = [1 1 1 1 1 1 1];
pitch(CONST_PSM,:) = [1 1 17.4533 1 1 1 1];
pitch(CONST_ECM,:) = [1 1 17.4533 1 1 1 1];

% Brake constants for ECM
hasBrake = [1 1 1 0];
brakeMaxCurrent =      [0.25 0.22 0.90];
brakeReleaseCurrent =  [0.25 0.22 0.90];
brakeReleaseTime =     [2.00 2.00 2.00];
brakeReleasedCurrent = [0.08 0.07 0.20];
brakeEngagedCurrent =  [0.0 0.0 0.0];

% Actuator Type (Prismatic/Revolute)
if (rType == CONST_MTM)
    actuatorType = {'Revolute', 'Revolute', 'Revolute', 'Revolute', ...
                    'Revolute', 'Revolute', 'Revolute'};
elseif (rType == CONST_PSM)
    actuatorType = {'Revolute', 'Revolute', 'Prismatic', 'Revolute', ...
                    'Revolute', 'Revolute', 'Revolute'};
elseif (rType == CONST_ECM)
    actuatorType = {'Revolute', 'Revolute', 'Prismatic', 'Revolute', ...
                    'Null',     'Null',     'Null'};
end



% ==== POT =======
% raw value from Intuitive Surgical Inc mXXXX.cal file
potGain = motor.pot_input_gain;
potOffset = motor.pot_input_offset;

% pot to encoder consistency check, for MTMs, last two joints are not used
if (rType == CONST_MTM)
    potToleranceLatency = [0.01 0.01 0.01 0.01 0.01 0.01 0.00];
    potToleranceDistance = [5.0 5.0 5.0 5.0 5.0 5.0 0.0];
    potToleranceUnit = {'deg', 'deg', 'deg', 'deg', ...
                        'deg', 'deg', 'deg'};
elseif (rType == CONST_PSM)
    potToleranceLatency = [0.01 0.01 0.01 0.01 0.01 0.01 0.01];
    potToleranceDistance = [5.0 5.0 5.0 5.0 5.0 5.0 5.0];
    potToleranceUnit = {'deg', 'deg', 'mm', 'deg', ...
                        'deg', 'deg', 'deg'};
elseif (rType == CONST_ECM)
    potToleranceLatency = [0.01 0.01 0.01 0.01];
    potToleranceDistance = [5.0 5.0 5.0 5.0];
    potToleranceUnit = {'deg', 'deg', 'mm', 'deg'};
end

%%
% =============================================
% Compute XML values
% =============================================
% === Drive =======
% Direction
% The linear amp drives +/- 6.25 Amps current, which is controlled by a DAC with 16 bits resolution.
% So the conversion from amp to bits is 2^16/(6.25 * 2) = 5242.88
driveDirection = aDirection;
AmpsToBitsScale = driveDirection(1:numOfJoints) .* 5242.8800;
AmpsToBitsOffset = ones(1, numOfJoints) .* (2^15);

BrakeAmpsToBitsScale = 5242.8800;
BrakeAmpsToBitsOffset = 2^15;

BitsToFbAmpsScale = driveDirection(1:numOfJoints) .* 0.000190738;
BitsToFbAmpsOffset = driveDirection(1:numOfJoints) .* (-6.25);

BrakeBitsToFbAmpsScale = 0.000190738;
BrakeBitsToFbAmpsOffset = -6.25;

NmToAmps = ones(1, numOfJoints) ./ gearRatio(rType, 1:numOfJoints) ./ motorTor(rType, 1:numOfJoints);
MaxCurrent = motorDefCur(rType, 1:numOfJoints);

% === Encoder ======
% EncPos = (360.0 * EncCounts / encCPT) / gearRatio * pitch
BitsToPosSIScale = driveDirection(1:numOfJoints) .* 360 ./ encCPT(rType, 1:numOfJoints) .* pitch(rType, 1:numOfJoints) ./ gearRatio(rType, 1:numOfJoints);

% AmpIO buff = buff + MIDRANGE_VEL
% Velocity = deltaPos / deltaTime
% deltaPos = 360 / (encCPT/4) / gearRatio * pitch  || not quadratic
% deltaTime = timeCounter * 1 / 768000 (Unit: sec)
% Fast clock 49.152 MHz / 2^6 = 768 kHz
% Velocity = 360 * 768000 / (encCPT / 4.0) / gearRatio * pitch / timeCounter
%          = BitsToDeltaPosSIScale / timeCounter

% BitsToDeltaPosSI = driveDirection(1:numOfJoints) .* 360.0 .* 768000 ./ (encCPT(rType, 1:numOfJoints) ./ 4.0) ./ gearRatio(rType, 1:numOfJoints) .* pitch(rType, 1:numOfJoints);
% BitsToDeltaT = ones(1, numOfJoints) * -1;
% CountsPerTurn = encCPT(rType, 1:numOfJoints);

% === AnalogIn =====
%  Intuitive system
%    1. 12 bit ADC
%    2. 0-4.096 V (Typical)
%    3. Unit: Radian
%
%  JHU QLA board
%    1. 16 bit ADC
%    2. 0-4.5 V
%    3. Unit: Radian
BitsToVolts = ones(1, numOfJoints) * (4.5 / 2^16);  % 16 bits ADC with 4.5 V ref
VoltsToPosSIScale = potGain(1:numOfJoints) * (2^12 / 4.5) * 180.0 / pi .* pitch(rType, 1:numOfJoints);
VoltsToPosSIOffset = potOffset(1:numOfJoints) * 180.0 / pi .* pitch(rType, 1:numOfJoints);

% special case for MTM last joint (Hall effect sensor)
if (rType == CONST_MTM)
    VoltsToPosSIScale(8) = -23.1788;
    VoltsToPosSIOffset(8) = 91.4238;
end


% === Coupling ====
if (rType == CONST_MTM)
    ActuatorToJointPosition = [ ...
        1.00  0.00   0.00 0.00 0.00 0.00 0.00; ...
        0.00  1.00   0.00 0.00 0.00 0.00 0.00; ...
        0.00 -1.00   1.00 0.00 0.00 0.00 0.00; ...
        0.00  0.6697 -0.6697 1.00 0.00 0.00 0.00; ...
        0.00  0.00   0.00 0.00 1.00 0.00 0.00; ...
        0.00  0.00   0.00 0.00 0.00 1.00 0.00; ...
        0.00  0.00   0.00 0.00 0.00 0.00 1.00  ...
        ];
end

%% Create XML file(s)

% ==============================
% Generate XML file
% Reference:
% Simple XML Node Creation
% http://blogs.mathworks.com/community/2010/09/13/simple-xml-node-creation/
% ==============================

fileName = aOutName;

docNode = com.mathworks.xml.XMLUtils.createDocument('Config');
Config = docNode.getDocumentElement;
Config.setAttribute('Version','4');

% ------------- Robot ----------------
Robot = docNode.createElement('Robot');
Robot.setAttribute('Name', aRobotName);
Robot.setAttribute('NumOfActuator', num2str(numOfJoints));
Robot.setAttribute('NumOfJoint', num2str(numOfJoints));
Robot.setAttribute('SN', num2str(serial_number));
Config.appendChild(Robot);

% Acutator array
for i = 1:numOfJoints
    Actuator = docNode.createElement('Actuator');
    Actuator.setAttribute('ActuatorID', num2str(i-1));
    % set to boardID1 & boardID2
    Actuator.setAttribute('BoardID', num2str(boardID( idivide(i-1, int32(4))+1 )));
    Actuator.setAttribute('AxisID', num2str(mod(i-1,4)));
    Actuator.setAttribute('Type', actuatorType{i});
    Robot.appendChild(Actuator);

    % Drive
    Drive = docNode.createElement('Drive');
    Actuator.appendChild(Drive);

    X_Amps2Bits = docNode.createElement('AmpsToBits');
    X_Amps2Bits.setAttribute('Scale', num2str(AmpsToBitsScale(i), '%5.2f'));
    X_Amps2Bits.setAttribute('Offset', num2str(AmpsToBitsOffset(i), '%5.0f'));
    Drive.appendChild(X_Amps2Bits);
    X_BitsToFeedbackAmps = docNode.createElement('BitsToFeedbackAmps');
    X_BitsToFeedbackAmps.setAttribute('Scale', num2str(BitsToFbAmpsScale(i), '%5.9f'));
    X_BitsToFeedbackAmps.setAttribute('Offset', num2str(BitsToFbAmpsOffset(i), '%5.2f'));
    Drive.appendChild(X_BitsToFeedbackAmps);
    X_NmToAmps = docNode.createElement('NmToAmps');
    X_NmToAmps.setAttribute('Scale', num2str(NmToAmps(i), '%5.6f'));
    Drive.appendChild(X_NmToAmps);
    X_MaxCurrent = docNode.createElement('MaxCurrent');
    X_MaxCurrent.setAttribute('Value', num2str(MaxCurrent(i), '%5.3f'));
    X_MaxCurrent.setAttribute('Unit', 'A');
    Drive.appendChild(X_MaxCurrent);

    % Brake
    if ((rType == CONST_ECM) && (hasBrake(i) == 1))
        Brake = docNode.createElement('AnalogBrake');
        Brake.setAttribute('BoardID', num2str(boardID(2)));
        Brake.setAttribute('AxisID', num2str(i-1));
        Actuator.appendChild(Brake);
        X_BrakeAmps2Bits = docNode.createElement('AmpsToBits');
        X_BrakeAmps2Bits.setAttribute('Scale', num2str(BrakeAmpsToBitsScale, '%5.2f'));
        X_BrakeAmps2Bits.setAttribute('Offset', num2str(BrakeAmpsToBitsOffset, '%5.0f'));
        Brake.appendChild(X_BrakeAmps2Bits);
        X_BrakeBitsToFeedbackAmps = docNode.createElement('BitsToFeedbackAmps');
        X_BrakeBitsToFeedbackAmps.setAttribute('Scale', num2str(BrakeBitsToFbAmpsScale, '%5.9f'));
        X_BrakeBitsToFeedbackAmps.setAttribute('Offset', num2str(BrakeBitsToFbAmpsOffset, '%5.2f'));
        Brake.appendChild(X_BrakeBitsToFeedbackAmps);
        X_BrakeMaxCurrent = docNode.createElement('MaxCurrent');
        X_BrakeMaxCurrent.setAttribute('Value', num2str(brakeMaxCurrent(i), '%5.3f'));
        X_BrakeMaxCurrent.setAttribute('Unit', 'A');
        Brake.appendChild(X_BrakeMaxCurrent);
        X_BrakeReleaseCurrent = docNode.createElement('ReleaseCurrent');
        X_BrakeReleaseCurrent.setAttribute('Unit', 'A');
        X_BrakeReleaseCurrent.setAttribute('Value', num2str(brakeReleaseCurrent(i), '%5.3f'));
        Brake.appendChild(X_BrakeReleaseCurrent);
        X_BrakeReleaseTime = docNode.createElement('ReleaseTime');
        X_BrakeReleaseTime.setAttribute('Value', num2str(brakeReleaseTime(i), '%5.3f'));
        Brake.appendChild(X_BrakeReleaseTime);
        X_BrakeReleasedCurrent = docNode.createElement('ReleasedCurrent');
        X_BrakeReleasedCurrent.setAttribut    % Actuator.setAttribute('Pos1', 'ENC');
        X_BrakeReleasedCurrent.setAttribute('Value', num2str(brakeReleasedCurrent(i), '%5.3f'));
        Brake.appendChild(X_BrakeReleasedCurrent);
        X_BrakeEngagedCurrent = docNode.createElement('EngagedCurrent');
        X_BrakeEngagedCurrent.setAttribute('Unit', 'A');
        X_BrakeEngagedCurrent.setAttribute('Value', num2str(brakeEngagedCurrent(i), '%5.3f'));
        Brake.appendChild(X_BrakeEngagedCurrent);
    end

    % Encoder
    Enc = docNode.createElement('Encoder');
    Actuator.appendChild(Enc);

    X_BitsToPosSI = docNode.createElement('BitsToPosSI');
    X_BitsToPosSI.setAttribute('Scale', num2str(BitsToPosSIScale(i), '%5.8f'));
    Enc.appendChild(X_BitsToPosSI);

    % AnalogIn
    AnalogIn = docNode.createElement('AnalogIn');
    Actuator.appendChild(AnalogIn);

    X_BitsToVolts = docNode.createElement('BitsToVolts');
    % BitsToVolts
    X_BitsToVolts.setAttribute('Scale', num2str(BitsToVolts(i), 6));
    X_BitsToVolts.setAttribute('Offset', '0');
    AnalogIn.appendChild(X_BitsToVolts);
    X_VoltsToPosSI = docNode.createElement('VoltsToPosSI');
    X_VoltsToPosSI.setAttribute('Scale', num2str(VoltsToPosSIScale(i), '%5.6f'));
    X_VoltsToPosSI.setAttribute('Offset', num2str(VoltsToPosSIOffset(i), '%5.6f'));
    AnalogIn.appendChild(X_VoltsToPosSI);
end

% ---------- Potentiometers ---------
Potentiometers = docNode.createElement('Potentiometers');
if (rType == CONST_MTM)
    Potentiometers.setAttribute('Position', 'Joints');
elseif (rType == CONST_PSM)
    Potentiometers.setAttribute('Position', 'Actuators');
elseif (rType == CONST_ECM)
    Potentiometers.setAttribute('Position', 'Actuators');
end

for i = 1:numOfJoints
    Tolerance = docNode.createElement('Tolerance');
    Tolerance.setAttribute('Axis', num2str(i-1));
    Tolerance.setAttribute('Distance', num2str(potToleranceDistance(i), '%5.2f'));
    Tolerance.setAttribute('Latency', num2str(potToleranceLatency(i), '%5.2f'));
    Tolerance.setAttribute('Unit', potToleranceUnit(i));
    Potentiometers.appendChild(Tolerance);
end

Robot.appendChild(Potentiometers);

% ----------- Coupling ---------------
if (rType == CONST_MTM)
    X_Coupling = docNode.createElement('Coupling');
    X_Coupling.setAttribute('Value', num2str(1));
    Robot.appendChild(X_Coupling);

    % Coupling/ActuatorToJointPosition
    X_ActuatorToJointPosition = docNode.createElement('ActuatorToJointPosition');
    X_Coupling.appendChild(X_ActuatorToJointPosition);

    for i = 1:size(ActuatorToJointPosition, 1)
        Row = docNode.createElement('Row');
        Row.setAttribute('Val', vector2str(ActuatorToJointPosition(i,:)));
        X_ActuatorToJointPosition.appendChild(Row);
    end
end

% ---------- DigitalIn ---------------
% 2 boards
% read from GUI
for b = 1:2
    for i = 1:12
        if ~strcmp(aDigital{i,2,b},'unused')
            DigitalIn = docNode.createElement('DigitalIn');
            DigitalIn.setAttribute('BitID', num2str(i-1));
            DigitalIn.setAttribute('Name', aDigital{i,2,b});
            DigitalIn.setAttribute('BoardID', num2str(boardID(b)));
            DigitalIn.setAttribute('Pressed', aDigital{i,3,b});
            DigitalIn.setAttribute('Trigger', aDigital{i,4,b});
            DigitalIn.setAttribute('Debounce', num2str(aDigital{i,5,b}));
            Config.appendChild(DigitalIn);
        end
    end
end

% ---------- DallasChip ---------------
if (rType == CONST_PSM)
    DallasChip = docNode.createElement('DallasChip');
    DallasChip.setAttribute('BoardID', num2str(boardID(2)));
    DallasChip.setAttribute('Name', strcat(aRobotName, '-Dallas'));
    Config.appendChild(DallasChip);
end

% generate xml file
mainFileName = [fileName '-' num2str(serial_number) '.xml'];
xmlwrite(mainFileName, docNode);

% generate xml file for gripper
if (rType == CONST_MTM)
    % this code is pretty much a copy/paste of loop above
    gripperDocNode = com.mathworks.xml.XMLUtils.createDocument('Config');
    Config = gripperDocNode.getDocumentElement;
    Config.setAttribute('Version','4');

    % ------------- Robot ----------------
    Robot = gripperDocNode.createElement('Robot');
    Robot.setAttribute('Name', strcat(aRobotName, '-Gripper'));
    Robot.setAttribute('NumOfActuator', '1');
    Robot.setAttribute('NumOfJoint', '1');
    Robot.setAttribute('SN', num2str(serial_number));
    Robot.setAttribute('Type', 'io-only');
    Config.appendChild(Robot);

    % Single pseudo-actuator
    i = 8; % 8th "actuator"
    Actuator = gripperDocNode.createElement('Actuator');
    Actuator.setAttribute('ActuatorID', '0');
    % set to boardID1 & boardID2
    Actuator.setAttribute('BoardID', num2str(boardID( idivide(i-1, int32(4))+1 )));
    Actuator.setAttribute('AxisID', num2str(mod(i-1,4)));
    Actuator.setAttribute('Type', 'Revolute');
    Robot.appendChild(Actuator);

    % Drive
    Drive = gripperDocNode.createElement('Drive');
    Actuator.appendChild(Drive);

    X_Amps2Bits = gripperDocNode.createElement('AmpsToBits');
    X_Amps2Bits.setAttribute('Scale', num2str(AmpsToBitsScale(1), '%5.2f'));
    X_Amps2Bits.setAttribute('Offset', num2str(AmpsToBitsOffset(1), '%5.0f'));
    Drive.appendChild(X_Amps2Bits);
    X_BitsToFeedbackAmps = gripperDocNode.createElement('BitsToFeedbackAmps');
    X_BitsToFeedbackAmps.setAttribute('Scale', num2str(BitsToFbAmpsScale(1), '%5.9f'));
    X_BitsToFeedbackAmps.setAttribute('Offset', num2str(BitsToFbAmpsOffset(1), '%5.2f'));
    Drive.appendChild(X_BitsToFeedbackAmps);
    X_NmToAmps = gripperDocNode.createElement('NmToAmps');
    X_NmToAmps.setAttribute('Scale', num2str(NmToAmps(1), '%5.6f'));
    Drive.appendChild(X_NmToAmps);
    X_MaxCurrent = gripperDocNode.createElement('MaxCurrent');
    X_MaxCurrent.setAttribute('Value', '0.0');
    X_MaxCurrent.setAttribute('Unit', 'A');
    Drive.appendChild(X_MaxCurrent);

    % Encoder
    Enc = gripperDocNode.createElement('Encoder');
    Actuator.appendChild(Enc);
    X_BitsToPosSI = gripperDocNode.createElement('BitsToPosSI');
    X_BitsToPosSI.setAttribute('Scale', '1.0');
    Enc.appendChild(X_BitsToPosSI);

    % AnalogIn
    AnalogIn = gripperDocNode.createElement('AnalogIn');
    Actuator.appendChild(AnalogIn);
    X_BitsToVolts = gripperDocNode.createElement('BitsToVolts');
    % BitsToVolts
    X_BitsToVolts.setAttribute('Scale', num2str(BitsToVolts(1), 6));
    X_BitsToVolts.setAttribute('Offset', '0');
    AnalogIn.appendChild(X_BitsToVolts);
    X_VoltsToPosSI = gripperDocNode.createElement('VoltsToPosSI');
    X_VoltsToPosSI.setAttribute('Scale', num2str(VoltsToPosSIScale(i), '%5.6f'));
    X_VoltsToPosSI.setAttribute('Offset', num2str(VoltsToPosSIOffset(i), '%5.6f'));
    AnalogIn.appendChild(X_VoltsToPosSI);

    % generate xml file
    gripperFileName = [fileName '-gripper-' num2str(serial_number) '.xml'];
    xmlwrite(gripperFileName, gripperDocNode);
end

isOK = true;

%%
% delete temporary .m file
cleanUp;

end  % configGenerator


function cleanUp
if (exist('calibration_file_temp.m', 'file') == 2)
    delete('calibration_file_temp.m');
end
end  % cleanUp

function outStr = vector2str(inVector)
    outStr = '';
    for i = 1:length(inVector)
        if (i == 1)
            outStr = [outStr, num2str(inVector(i),  '%6.4f')];
        else
            outStr = [outStr, ' ', num2str(inVector(i),  '%6.4f')];
        end
    end
%     disp(outStr);
end  % vector2str
