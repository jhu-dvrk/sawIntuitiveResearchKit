function [ isOK ] = configGenerator( aCalName, aOutName, aRobotName, aBoardID, aDigital, aDirection)
%UNTITLED6 Summary of this function goes here
%   Detailed explanation goes here

% TODOs
%  1. verify all settings

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
copyfile(aCalName, 'temp.m');

% define some constants for mXXX.cal/pXXX.cal/eXXX.cal file
UPPER_LIMIT = 1; LOWER_LIMIT = 2;

% constants for master configuration file
MST_JNT_POS_GR_DOFS = 8;
MST_MOT_DOFS = 8;
MST_JNT_POS_DOFS = 7;

% constants for slave configuration file
SLV_JNT_POS_GR_DOFS = 7;
SLV_MOT_DOFS = 7;

% constants for ECM configuration file
ECM_JNT_POS_GR_DOFS = 4;
ECM_MOT_DOFS = 4;

% load temp file, this will load all constants in ***.cal file
temp;

% sanity check for RobotName/Type
% rType = robot type
CONST_MST = 1;
CONST_SLV = 2;
CONST_ECM = 3;
if (strcmp(aRobotName(1:3), FileType(1:3)) == 0)
    disp(' ERROR: robot type and calibration file mismatch');
    cleanUp;
    return;
else
    if (strcmp(aRobotName(1:3), 'MTM'))
        rType = CONST_MST;
    elseif (strcmp(aRobotName(1:3), 'PSM'))
        rType = CONST_SLV;
    elseif (strcmp(aRobotName(1:3), 'ECM'))
        rType = CONST_ECM;
    end
end

% assign numOfActuator & numOfJoint based on robot type
if (rType == CONST_MST)
    numOfActuator = MST_MOT_DOFS;
    numOfJoint = MST_JNT_POS_GR_DOFS;
elseif (rType == CONST_SLV)
    numOfActuator = SLV_MOT_DOFS;
    numOfJoint = SLV_JNT_POS_GR_DOFS;
elseif (rType == CONST_ECM)
    numOfActuator = ECM_MOT_DOFS;
    numOfJoint = ECM_JNT_POS_GR_DOFS;
end


% master

% motor constants Unit: V
motorVol(CONST_MST,:) = [24 24 24 24  9  9  6 0]; % motor const for master
motorVol(CONST_SLV,:) = [24 24 24 24 24 24 24 0]; % motor const for slave
motorVol(CONST_ECM,:) = [48 48 24 9   0  0  0 0]; % motor const for camera arm

% motor default current Unit: A
motorDefCur(CONST_MST,:) = [0.67 0.67 0.67 0.67 0.59 0.59 0.407 0.0];
motorDefCur(CONST_SLV,:) = [1.34 1.34 0.67 0.67 0.67 0.67 0.670 0.0];
motorDefCur(CONST_ECM,:) = [0.943 0.943 0.67 0.59 0.0 0.0 0.0 0.0];

% motor max current Unit: A
motorMaxCur(CONST_MST,:) = [0.67 0.67 0.67 0.92 0.75 0.59 0.407 0.0];
motorMaxCur(CONST_SLV,:) = [2.01 2.01 1.005 1.005 1.005 1.005 1.005 0.0];
motorMaxCur(CONST_ECM,:) = [1.4145 1.4145 1.005 0.885 0.0 0.0 0.0 0.0];

% motor torque const  Unit: Nm/A
% NOTE: no motor on axis 8, set value to 1 
motorTor(CONST_MST,:) = [0.0438 0.0438 0.0438 0.0438 0.00495 0.00495 0.00339 1.0];
motorTor(CONST_SLV,:) = [0.0438 0.0438 0.0438 0.0438 0.0438 0.0438 0.0438 1.0];
motorTor(CONST_ECM,:) = [0.1190 0.1190 0.0438 0.00495 1.0 1.0 1.0 1.0];

% Gear ratio
% NOTE: gear ratio for axis 8 is set to 1 
gearRatio(CONST_MST,:) = [63.41 49.88 59.73 10.53 33.16 33.16 16.58 1.0];
gearRatio(CONST_SLV,:) = [56.50 56.50 336.6 11.71 11.71 11.71 11.71 1.0];
gearRatio(CONST_ECM,:) = [240 240 2748.55 300.15    1.0   1.0   1.0 1.0];

% Encoder counts per turn (quadrature encoder)
% NOTE: no encoder for last axis
encCPT(CONST_MST,:) = [ 4000  4000  4000 4000   64   64   64 1];
encCPT(CONST_SLV,:) = [14400 14400 14400 4000 4000 4000 4000 1];
encCPT(CONST_ECM,:) = [ 4000  4000   640   64    1    1    1 1];

% Pitch
% 1 for revolute, mm/deg for prismatic
pitch(CONST_MST,:) = [1 1 1 1 1 1 1 1];
pitch(CONST_SLV,:) = [1 1 17.4533 1 1 1 1 1];
pitch(CONST_ECM,:) = [1 1 1 1 1 1 1 1];

% Actuator Type (Prismatic/Revolute)

if (rType == CONST_MST)
    actuatorType = {'Revolute', 'Revolute', 'Revolute', 'Revolute', ...
                    'Revolute', 'Revolute', 'Revolute', 'Revolute'};
elseif (rType == CONST_SLV)
    actuatorType = {'Revolute', 'Revolute', 'Prismatic', 'Revolute', ...
                    'Revolute', 'Revolute', 'Revolute', 'Revolute'};
elseif (rType == CONST_ECM)
    actuatorType = {'Revolute', 'Revolute', 'Prismatic', 'Revolute', ...
                    'Revolute', 'Revolute', 'Prismatic', 'Revolute'};  

end



% ==== POT =======
% raw value from Intuitive Surgical Inc mXXXX.cal file
% NOTE:
%  Intuitive system
%    1. 12 bit ADC
%    2. 0-5 V (Typical)
%    3. Unit: Radian
%
%  JHU QLA board
%    1. 16 bit ADC
%    2. 0-4.5 V
%    3. Unit: Radian
jointUpper = joint.signal_range(UPPER_LIMIT,:);
jointLower = joint.signal_range(LOWER_LIMIT,:);
potGain = motor.pot_input_gain;
potOffset = motor.pot_input_offset;


%%
% =============================================
% Compute XML values
% =============================================
% === Drive =======
% Direction
driveDirection = aDirection;
AmpsToBitsScale = driveDirection(1:numOfActuator) .* 5242.8800;
AmpsToBitsOffset = ones(1, numOfActuator) .* (2^15);

BrakeAmpsToBitsScale = ones(1, numOfActuator) .* 5242.8800;
BrakeAmpsToBitsOffset = ones(1, numOfActuator) .* (2^15);

BitsToFbAmpsScale = driveDirection(1:numOfActuator) .* 0.000190738;
BitsToFbAmpsOffset = driveDirection(1:numOfActuator) .* (-6.25);

NmToAmps = ones(1,numOfActuator) ./ gearRatio(rType,1:numOfActuator) ./ motorTor(rType,1:numOfActuator);
MaxCurrent = motorDefCur(rType,1:numOfActuator);

% === Encoder ======
% EncPos = (360.0 * EncCounts / encCPT) / gearRatio * pitch 
BitsToPosSIScale = driveDirection(1:numOfActuator) .* 360 ./ encCPT(rType,1:numOfActuator) .* pitch(rType,1:numOfActuator) ./ gearRatio(rType,1:numOfActuator);

% AmpIO buff = buff + MIDRANGE_VEL
% Velocity = deltaPos / deltaTime
% deltaPos = 360 / (encCPT/4) / gearRatio * pitch  || not quadratic
% deltaTime = timeCounter * 1 / 768000 (Unit: sec)
% Fast clock 49.152 MHz / 2^6 = 768 kHz
% Velocity = 360 * 768000 / (encCPT / 4.0) / gearRatio * pitch / timeCounter
%          = BitsToDeltaPosSIScale / timeCounter

BitsToDeltaPosSI = driveDirection(1:numOfActuator) .* 360.0 .* 768000 ./ (encCPT(rType,1:numOfActuator) ./ 4.0) ./ gearRatio(rType,1:numOfActuator) .* pitch(rType,1:numOfActuator);
BitsToDeltaT = ones(1,numOfActuator) * -1;
CountsPerTurn = encCPT(rType,1:numOfActuator);

% === AnalogIn =====
BitsToVolts = ones(1,numOfActuator) * 0.0000686656;
VoltsToPosSIScale = potGain * 2^12 / (4.5 - 0.0) * 180.0 / pi .* pitch(rType,1:numOfActuator);
VoltsToPosSIOffset = potOffset * 180.0 / pi .* pitch(rType,1:numOfActuator);

% special case for master last joint (Hall effect sensor)
if (rType == CONST_MST)
    VoltsToPosSIScale(8) = -23.1788;
    VoltsToPosSIOffset(8) = 91.4238;
end


% === Coupling ====
if (rType == CONST_MST)
    ActuatorToJointPosition = [ ...
        1.00  0.00   0.00 0.00 0.00 0.00 0.00 0.00; ...
        0.00  1.00   0.00 0.00 0.00 0.00 0.00 0.00; ...
        0.00 -1.00   1.00 0.00 0.00 0.00 0.00 0.00; ...
        0.00  0.6697 -0.6697 1.00 0.00 0.00 0.00 0.00; ...
        0.00  0.00   0.00 0.00 1.00 0.00 0.00 0.00; ...
        0.00  0.00   0.00 0.00 0.00 1.00 0.00 0.00; ...
        0.00  0.00   0.00 0.00 0.00 0.00 1.00 0.00; ...
        0.00  0.00   0.00 0.00 0.00 0.00 0.00 1.00  ...
        ];
elseif (rType == CONST_SLV)
    ActuatorToJointPosition = [ ...
        1.00  0.00   0.00  0.00 0.00 0.00 0.00; ...
        0.00  1.00   0.00  0.00 0.00 0.00 0.00; ...
        0.00  0.00   1.00  0.00 0.00 0.00 0.00; ...
        0.00  0.00   0.00 -1.5632 0.00 0.00 0.00; ...
        0.00  0.00   0.00  0.00 1.0186 0.00 0.00;  ...
        0.00  0.00   0.00  0.00 -0.8306 0.6089 0.6089; ...
        0.00  0.00   0.00 0.00 0.00 -1.2177 1.2177; ...
        ];
elseif (rType == CONST_ECM)
    ActuatorToJointPosition = [ ...
        1.00  0.00   0.00  0.00 ; ...
        0.00  1.00   0.00  0.00; ...
        0.00  0.00   1.00  0.00 ; ...
        0.00  0.00   0.00  1.00 ; ...
        ];
end


JointToActuatorPosition = inv(ActuatorToJointPosition);
JointToActuatorTorque = ActuatorToJointPosition';
ActuatorToJointTorque = inv(JointToActuatorTorque);


%% Create XML file

% ==============================
% Generate XML file
% Reference:
% Simple XML Node Creation
% http://blogs.mathworks.com/community/2010/09/13/simple-xml-node-creation/
% ==============================

fileName = aOutName;

docNode = com.mathworks.xml.XMLUtils.createDocument('Config');
Config = docNode.getDocumentElement;

% ------------- Robot ----------------
Robot = docNode.createElement('Robot');
Robot.setAttribute('Name', aRobotName);
Robot.setAttribute('NumOfActuator', num2str(numOfActuator));
Robot.setAttribute('NumOfJoint', num2str(numOfJoint));
Robot.setAttribute('SN', num2str(serial_number));
Config.appendChild(Robot);

% Acutator array
for i = 1:numOfActuator
    Actuator = docNode.createElement('Actuator');
    Actuator.setAttribute('ActuatorID', num2str(i-1));
    % set to boardID1 & boardID2
    Actuator.setAttribute('BoardID', num2str(boardID( idivide(i-1, int32(4))+1 )));
    Actuator.setAttribute('AxisID', num2str(mod(i-1,4)));
    Actuator.setAttribute('Type', actuatorType{i});
    % Actuator.setAttribute('Pos1', 'ENC');
    % Actuator.setAttribute('Pos2', 'POT');
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
    if (rType == CONST_ECM)
        Brake = docNode.createElement('AnalogBrake');
        Brake.setAttribute('BoardID', num2str(boardID(2)));
        Brake.setAttribute('AxisID', num2str(i-1));
        Actuator.appendChild(Brake);
        X_BrakeAmps2Bits = docNode.createElement('AmpsToBits');
        X_BrakeAmps2Bits.setAttribute('Scale', num2str(BrakeAmpsToBitsScale(i), '%5.2f'));
        X_BrakeAmps2Bits.setAttribute('Offset', num2str(BrakeAmpsToBitsOffset(i), '%5.0f'));
        Brake.appendChild(X_BrakeAmps2Bits);
        X_BrakeBitsToFeedbackAmps = docNode.createElement('BitsToFeedbackAmps');
        X_BrakeBitsToFeedbackAmps.setAttribute('Scale', num2str(BitsToFbAmpsScale(i), '%5.9f'));
        X_BrakeBitsToFeedbackAmps.setAttribute('Offset', num2str(BitsToFbAmpsOffset(i), '%5.2f'));
        Brake.appendChild(X_BrakeBitsToFeedbackAmps);
    end
    
    % Encoder
    Enc = docNode.createElement('Encoder');
    Actuator.appendChild(Enc);
    
    X_BitsToPosSI = docNode.createElement('BitsToPosSI');
    X_BitsToPosSI.setAttribute('Scale', num2str(BitsToPosSIScale(i), '%5.8f'));
    X_BitsToPosSI.setAttribute('Offset', '0');
    Enc.appendChild(X_BitsToPosSI);
    X_BitsToDeltaPosSI = docNode.createElement('BitsToDeltaPosSI');
    X_BitsToDeltaPosSI.setAttribute('Scale', num2str(BitsToDeltaPosSI(i), '%5.2f'));
    X_BitsToDeltaPosSI.setAttribute('Offset', '0');
    Enc.appendChild(X_BitsToDeltaPosSI);
    X_BitsToDeltaT = docNode.createElement('BitsToDeltaT');
    X_BitsToDeltaT.setAttribute('Scale', num2str(BitsToDeltaT(i), '%5.3f'));
    X_BitsToDeltaT.setAttribute('Offset', '0');
    Enc.appendChild(X_BitsToDeltaT);
    X_CountsPerTurn = docNode.createElement('CountsPerTurn');
    X_CountsPerTurn.setAttribute('Value', num2str(CountsPerTurn(i), '%5.0f'));
    Enc.appendChild(X_CountsPerTurn);
    
    % AnalogIn
    AnaglogIn = docNode.createElement('AnalogIn');
    Actuator.appendChild(AnaglogIn);
    
    X_BitsToVolts = docNode.createElement('BitsToVolts');
    % BitsToVolts
    X_BitsToVolts.setAttribute('Scale', num2str(BitsToVolts(i), 6));
    X_BitsToVolts.setAttribute('Offset', '0');
    AnaglogIn.appendChild(X_BitsToVolts);
    X_VoltsToPosSI = docNode.createElement('VoltsToPosSI');
    X_VoltsToPosSI.setAttribute('Scale', num2str(VoltsToPosSIScale(i), '%5.6f'));
    X_VoltsToPosSI.setAttribute('Offset', num2str(VoltsToPosSIOffset(i), '%5.6f'));
    AnaglogIn.appendChild(X_VoltsToPosSI);
end

% ---------- Potentiometers ---------
Potentiometers = docNode.createElement('Potentiometers');
if (rType == CONST_MST)
    Potentiometers.setAttribute('Position', 'Joints');
elseif (rType == CONST_SLV)
    Potentiometers.setAttribute('Position', 'Actuators');
elseif (rType == CONST_ECM)
    Potentiometers.setAttribute('Position', 'Actuators');
end

Robot.appendChild(Potentiometers);

% ----------- Coupling ---------------
X_Coupling = docNode.createElement('Coupling');
X_Coupling.setAttribute('Value', num2str(1));
Robot.appendChild(X_Coupling);

% 1 Coupling/ActuatorToJointPosition
X_ActuatorToJointPosition = docNode.createElement('ActuatorToJointPosition');
X_Coupling.appendChild(X_ActuatorToJointPosition);

for i = 1:size(ActuatorToJointPosition, 1)
    Row = docNode.createElement('Row');
    Row.setAttribute('Val', vector2str(ActuatorToJointPosition(i,:)));
    X_ActuatorToJointPosition.appendChild(Row);
end


% 2 Coupling/JointToActuatorPosition
X_JointToActuatorPosition = docNode.createElement('JointToActuatorPosition');
X_Coupling.appendChild(X_JointToActuatorPosition);

for i = 1:size(JointToActuatorPosition, 1)
    Row = docNode.createElement('Row');
    Row.setAttribute('Val', vector2str(JointToActuatorPosition(i,:)));
    X_JointToActuatorPosition.appendChild(Row);
end


% 3 Coupling/ActuatorToJointTorque
X_ActuatorToJointTorque = docNode.createElement('ActuatorToJointTorque');
X_Coupling.appendChild(X_ActuatorToJointTorque);

for i = 1:size(ActuatorToJointTorque, 1)
    Row = docNode.createElement('Row');
    Row.setAttribute('Val', vector2str(ActuatorToJointTorque(i,:)));
    X_ActuatorToJointTorque.appendChild(Row);
end


% 4 Coupling/JointToActuatorTorque
X_JointToActuatorTorque = docNode.createElement('JointToActuatorTorque');
X_Coupling.appendChild(X_JointToActuatorTorque);

for i = 1:size(JointToActuatorTorque, 1)
    Row = docNode.createElement('Row');
    Row.setAttribute('Val', vector2str(JointToActuatorTorque(i,:)));
    X_JointToActuatorTorque.appendChild(Row);
end


% ---------- DigitalIn ---------------
Config.appendChild(docNode.createComment('Digital Input Configuration'));
% 2 boards
% read from GUI
for b = 1:2
    for i = 1:12
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

% generate xml file
fileName = [fileName '-' num2str(serial_number) '.xml'];
xmlwrite(fileName,docNode);

isOK = true;

%%
% delete temporary .m file
cleanUp;

end  % configGenerator


function cleanUp
if (exist('temp.m', 'file') == 2)
    delete('temp.m');
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


