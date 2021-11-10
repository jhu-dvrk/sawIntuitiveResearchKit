#!/usr/bin/env python3

import argparse
import json
import math
import xml.etree.ElementTree as ET
from enum import Enum

import numpy as np

import calParser


class RobotType(Enum):
    MTM = 0
    PSM = 1
    ECM = 2

    def fromTypeName(robotTypeName):
        if robotTypeName[0:3] == "MTM":
            return RobotType.MTM
        elif robotTypeName[0:3] == "PSM":
            return RobotType.PSM
        elif robotTypeName[0:3] == "ECM":
            return RobotType.ECM
        else:
            return None


class OutputFormat(Enum):
    JSON = 0
    XML = 1


# Empty base class for all class we want to serialize to JSON/XML
class Serializable:
    def toDict(self):
        raise NotImplementedError("Please implement toDict()")


# Represents the various unit conversions e.g. NmToAmps, AmpsToBits, etc.
class Conversion(Serializable):
    def __init__(self, scale, offset, units=None):
        self.scale = scale
        self.offset = offset
        self.units = units

    def toDict(self):
        dict = {}
        if self.offset:
            dict["Offset"] = self.offset
        if self.scale:
            dict["Scale"] = self.scale
        if self.units:
            dict["Unit"] = self.units
        return dict


# Dimensioned value
class UnitValue(Serializable):
    def __init__(self, value, units):
        self.value = value
        self.units = units

    def toDict(self):
        dict = {}
        if self.units:
            dict["Unit"] = self.units
        if self.value:
            dict["Value"] = self.value
        return dict


# dVRK board ID conventions
def getBoardIDs(robotTypeName):
    boardIDs = None
    if robotTypeName == "MTML":
        boardIDs = [0, 1]
    elif robotTypeName == "MTMR":
        boardIDs = [2, 3]
    elif robotTypeName == "ECM":
        boardIDs = [4, 5]
    elif robotTypeName == "PSM1":
        boardIDs = [6, 7]
    elif robotTypeName == "PSM2":
        boardIDs = [8, 9]
    elif robotTypeName == "PSM3":
        boardIDs = [10, 11]

    return boardIDs


# creates appropriate DigitalInput configs based on robot type
def generateDigitalInputs(robotTypeName):
    robotType = RobotType.fromTypeName(robotTypeName)
    boardIDs = getBoardIDs(robotTypeName)
    digitalInputBitIDs = None
    if robotType == RobotType.MTM:
        digitalInputBitIDs = []
    elif robotType == RobotType.PSM:
        digitalInputBitIDs = [
            (boardIDs[0], 0),
            (boardIDs[0], 2),
            (boardIDs[1], 7),
            (boardIDs[1], 10),
        ]
    elif robotType == RobotType.ECM:
        digitalInputBitIDs = [(boardIDs[0], 0), (boardIDs[0], 2)]

    digitalInputs = []
    for boardID, bitID in digitalInputBitIDs:
        digitalInput = DigitalInput(robotTypeName, bitID, boardID)
        digitalInputs.append(digitalInput)

    return digitalInputs


# Root config object
class Config(Serializable):
    def __init__(self, calData, versionID, robotTypeName):
        self.versionID = versionID
        serialNumber = calData["serial_number"]
        self.robot = Robot(calData, robotTypeName, serialNumber)
        self.digitalInputs = generateDigitalInputs(robotTypeName)

        robotType = RobotType.fromTypeName(robotTypeName)
        self.dallasChip = (
            DallasChip(robotTypeName) if robotType == RobotType.PSM else None
        )

    def toDict(self):
        dict = {
            "Version": self.versionID,
            "Robot": self.robot,
        }

        if self.dallasChip != None:
            dict["DallasChip"] = self.dallasChip

        if len(self.digitalInputs) > 0:
            dict["DigitalIns"] = self.digitalInputs

        return dict


class Robot(Serializable):
    def __init__(self, calData, robotTypeName, serialNumber):
        self.name = robotTypeName
        self.serialNumber = serialNumber

        self.type = RobotType.fromTypeName(robotTypeName)
        numberOfActuators = 4 if self.type == RobotType.ECM else 7

        driveDirections = None  # motor drive directions by actuator
        if robotTypeName == "MTML":
            driveDirections = [-1, 1, 1, 1, -1, 1, -1, 1]
        elif robotTypeName == "MTMR":
            driveDirections = [-1, 1, 1, 1, 1, 1, -1, 1]
        elif robotTypeName[0:3] == "PSM":
            driveDirections = [-1, -1, 1, -1, -1, 1, 1, 1]
        elif robotTypeName == "ECM":
            driveDirections = [1, 1, -1, 1]

        self.actuators = []
        boardIDs = getBoardIDs(robotTypeName)
        for idx in range(numberOfActuators):
            driveDirection = driveDirections[idx]
            actuator = Actuator(calData, self.type, idx, driveDirection, boardIDs)
            self.actuators.append(actuator)

        self.potentiometers = Potentiometers(self.type, numberOfActuators)

        self.coupling = Coupling() if self.type == RobotType.MTM else None

    def toDict(self):
        dict = {
            "Name": self.name,
            "NumOfActuator": len(self.actuators),
            "NumOfJoint": len(self.actuators),
            "SN": self.serialNumber,
            "Actuators": self.actuators,
            "Potentiometers": self.potentiometers,
        }

        if self.coupling != None:
            dict["Coupling"] = self.coupling

        return dict


class Actuator(Serializable):
    def __init__(self, calData, robotType, id, driveDirection, boardIDs):
        self.id = id
        self.actuatorType = (
            "Prismatic" if (robotType != RobotType.MTM and id == 2) else "Revolute"
        )
        self.boardID = boardIDs[0] if id < 4 else boardIDs[1]
        self.axisID = id % 4

        CPT = None  # Encoder counts per turn (quadrature encoder), NOTE: no encoder for last axis
        gearRatio = None  # NOTE: gear ratio for axis 8 is set to 1
        pitch = None  # 1 for revolute, mm/deg for prismatic
        motorMaxCurrent = None  # Sustained max current, amps
        motorTorque = None  # units are Nm/A

        # Lookup constants based on robot type and actuator ID
        if robotType == RobotType.MTM:
            CPT = [4000, 4000, 4000, 4000, 64, 64, 64][id]
            gearRatio = [63.41, 49.88, 59.73, 10.53, 33.16, 33.16, 16.58][id]
            pitch = 1
            motorMaxCurrent = [0.67, 0.67, 0.67, 0.67, 0.59, 0.59, 0.407][id]
            motorTorque = [0.0438, 0.0438, 0.0438, 0.0438, 0.00495, 0.00495, 0.00339][
                id
            ]
        elif robotType == RobotType.PSM:
            CPT = [14400, 14400, 14400, 4000, 4000, 4000, 4000][id]
            gearRatio = [56.50, 56.50, 336.6, 11.71, 11.71, 11.71, 11.71][id]
            pitch = [1, 1, 17.4533, 1, 1, 1, 1][id]
            motorMaxCurrent = [1.34, 1.34, 0.67, 0.67, 0.67, 0.67, 0.670][id]
            motorTorque = [0.0438, 0.0438, 0.0438, 0.0438, 0.0438, 0.0438, 0.0438][id]
        elif robotType == RobotType.ECM:
            CPT = [4000, 4000, 640, 64, 1, 1, 1][id]
            gearRatio = [240, 240, 2748.55, 300.15, 1.0, 1.0, 1.0][id]
            pitch = [1, 1, 17.4533, 1, 1, 1, 1][id]
            motorMaxCurrent = [0.943, 0.943, 0.67, 0.59, 0.0, 0.0, 0.0][id]
            motorTorque = [0.1190, 0.1190, 0.0438, 0.00495, 1.0, 1.0, 1.0][id]

        self.drive = Drive(driveDirection, gearRatio, motorTorque, motorMaxCurrent)
        self.encoder = Encoder(
            self.id, robotType, driveDirection, CPT, pitch, gearRatio
        )
        self.analogIn = AnalogIn(calData, robotType, self.id, pitch)

        # Actuators 0, 1, 2 on ECM have a brake
        hasBrake = robotType == RobotType.ECM and id <= 2
        self.brake = Brake(self.id, self.axisID, self.boardId) if hasBrake else None

    def toDict(self):
        dict = {
            "ActuatorID": self.id,
            "AxisID": self.axisID,
            "BoardID": self.boardID,
            "Type": self.actuatorType,
            "Drive": self.drive,
            "Encoder": self.encoder,
            "AnalogIn": self.analogIn,
        }

        if self.brake != None:
            dict["AnalogBrake"] = self.brake

        return dict


# === Drive =======
# Direction
# The linear amp drives +/- 6.25 Amps current, which is controlled by a DAC with 16 bits resolution.
# So the conversion from amp to bits is 2^16/(6.25 * 2) = 5242.88
class Drive(Serializable):
    def __init__(
        self,
        driveDirection,
        gearRatio,
        motorTorque,
        motorMaxCurrent,
    ):
        self.ampsToBits = Conversion(
            "{:5.2f}".format(driveDirection * 5242.8800), "{:5.0f}".format(2 ** 15)
        )
        self.bitsToFeedbackAmps = Conversion(
            "{:5.9f}".format(driveDirection * 0.000190738),
            "{:5.2f}".format(driveDirection * -6.25),
        )

        self.nmToAmps = Conversion(
            "{:5.6f}".format(1.0 / (gearRatio * motorTorque)), None
        )
        self.maxCurrent = UnitValue("{:5.3f}".format(motorMaxCurrent), "A")

    def toDict(self):
        return {
            "AmpsToBits": self.ampsToBits,
            "BitsToFeedbackAmps": self.bitsToFeedbackAmps,
            "NmToAmps": self.nmToAmps,
            "MaxCurrent": self.maxCurrent,
        }


# Brake config for first three actuators of ECM
class Brake(Serializable):
    def __init__(
        self,
        actuatorID,
        axisID,
        boardID,
    ):
        self.axisID = axisID
        self.boardID = boardID
        self.ampsToBits = Conversion("5242.8800", 2 ** 15)
        self.bitsToFeedbackAmps = Conversion("0.000190738", "-6.25")
        self.maxCurrent = [0.25, 0.22, 0.90][actuatorID]
        self.releaseCurrent = [0.25, 0.22, 0.90][actuatorID]
        self.releaseTime = [2.00, 2.00, 2.00][actuatorID]
        self.releasedCurrent = [0.08, 0.07, 0.20][actuatorID]
        self.engagedCurrent = [0.0, 0.0, 0.0][actuatorID]

    def toDict(self):
        return {
            "AxisID": self.axisID,
            "BoardID": self.boardID,
            "AmpsToBits": self.ampsToBits,
            "BitsToFeedbackAmps": self.bitsToFeedbackAmps,
            "MaxCurrent": self.maxCurrent,
            "ReleaseCurrent": self.releaseCurrent,
            "ReleaseTime": self.releaseTime,
            "ReleasedCurrent": self.releasedCurrent,
            "EngagedCurret": self.engagedCurrent,
        }


class Encoder(Serializable):
    def __init__(self, actuatorID, robotType, driveDirection, CPT, pitch, gearRatio):
        # degrees except for third actuator of PSM/ECM
        potToleranceUnit = (
            "mm" if (robotType != RobotType.MTM and actuatorID == 2) else "deg"
        )

        encoderPos = driveDirection * (360 / CPT) * (pitch / gearRatio)
        encoderPos = "{:5.8f}".format(encoderPos)
        self.bitsToPosSI = Conversion(encoderPos, None, potToleranceUnit)

    def toDict(self):
        return {"BitsToPosSI": self.bitsToPosSI}


# === AnalogIn =====
#  Intuitive system
#    1. 12 bit ADC
#    2. 0-4.096 V (Typical)
#    3. Unit: Radian
#
#  JHU QLA board
#    1. 16 bit ADC
#    2. 0-4.5 V
#    3. Unit: Radian
class AnalogIn(Serializable):
    def __init__(self, calData, robotType, actuatorID, pitch):
        # 16 bits ADC with 4.5 V ref
        self.bitsToVolts = Conversion(4.5 / (2 ** 16), "0")
        potToleranceUnits = (
            "mm" if (robotType != RobotType.MTM and actuatorID == 2) else "deg"
        )

        potGain = calData["motor.pot_input_gain"][actuatorID]
        potOffset = calData["motor.pot_input_offset"][actuatorID]

        voltsToPosSIScale = potGain * (2 ** 12 / 4.5) * (180.0 / math.pi) * pitch
        voltsToPosSIOffset = potOffset * (180.0 / math.pi) * pitch

        # special case for MTM last joint (Hall effect sensor)
        if robotType == RobotType.MTM and actuatorID == 7:
            voltsToPosSIScale = -23.1788
            voltsToPosSIScale = 91.4238

        voltsToPosSIScale = "{:5.6f}".format(voltsToPosSIScale)
        voltsToPosSIOffset = "{:5.6f}".format(voltsToPosSIOffset)

        self.voltsToPosSI = Conversion(
            voltsToPosSIScale, voltsToPosSIOffset, potToleranceUnits
        )

    def toDict(self):
        return {
            "BitsToVolts": self.bitsToVolts,
            "VoltsToPosSI": self.voltsToPosSI,
        }


class Potentiometers(Serializable):
    def __init__(self, robotType, axisCount):
        if robotType == RobotType.MTM:
            self.position = "Joints"
        else:
            self.position = "Actuators"

        self.tolerances = []
        for axis in range(axisCount):
            tolerance = PotentiometerTolerance(axis, robotType)
            self.tolerances.append(tolerance)

    def toDict(self):
        return {
            "Position": self.position,
            "Tolerances": self.tolerances,
        }


class PotentiometerTolerance(Serializable):
    def __init__(self, axisID, robotType):
        self.axisID = axisID

        # pot to encoder consistency check, for MTMs, last two joints are not used
        self.units = "mm" if (robotType != RobotType.MTM and axisID == 2) else "deg"
        self.latency = None
        self.distance = None

        if robotType == RobotType.MTM:
            self.latency = [0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.00][axisID]
            self.distance = [5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 0.0][axisID]
        elif robotType == RobotType.PSM:
            self.latency = [0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01][axisID]
            self.distance = [5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0][axisID]
        elif robotType == RobotType.ECM:
            self.latency = [0.01, 0.01, 0.01, 0.01][axisID]
            self.distance = [5.0, 5.0, 5.0, 5.0][axisID]

    def toDict(self):
        return {
            "Axis": self.axisID,
            "Distance": "{:5.2f}".format(self.distance),
            "Latency": "{:5.2f}".format(self.latency),
            "Unit": self.units,
        }


class Coupling(Serializable):
    def __init__(self):
        self.actuatorToJointPositionMatrix = np.array(
            [
                [1.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00],
                [0.00, 1.00, 0.00, 0.00, 0.00, 0.00, 0.00],
                [0.00, -1.00, 1.00, 0.00, 0.00, 0.00, 0.00],
                [0.00, 0.6697, -0.6697, 1.00, 0.00, 0.00, 0.00],
                [0.00, 0.00, 0.00, 0.00, 1.00, 0.00, 0.00],
                [0.00, 0.00, 0.00, 0.00, 0.00, 1.00, 0.00],
                [0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 1.00],
            ]
        )

    def toDict(self):
        return {
            "Value": 1,
            "ActuatorToJointPosition": self.actuatorToJointPositionMatrix,
        }


class DigitalInput(Serializable):
    def __init__(self, robotTypeName, bitID, boardID):
        self.bitID = bitID
        self.boardID = boardID

        debounceTimes = {0: 0.2, 2: 0.2, 7: 1.5, 10: 1.5}
        self.debounceTime = debounceTimes[bitID]

        robotType = RobotType.fromTypeName(robotTypeName)
        if robotType == RobotType.PSM:
            digitalInputTypes = {
                0: "SUJCatch",
                2: "ManipClutch",
                7: "Tool",
                10: "Adapter",
            }
            self.name = robotTypeName + "-" + digitalInputTypes[bitID]
        elif robotType == RobotType.ECM:
            digitalInputTypes = {0: "ManipClutch", 2: "SUJCatch"}
            self.name = robotTypeName + "-" + digitalInputTypes[bitID]

    def toDict(self):
        return {
            "BitID": self.bitID,
            "Name": self.name,
            "BoardID": self.boardID,
            "Pressed": 1,
            "Trigger": "all",
            "Debounce": self.debounceTime,
        }


class DallasChip(Serializable):
    def __init__(self, robotTypeName):
        self.name = robotTypeName + "-Dallas"
        self.boardID = getBoardIDs(robotTypeName)[1]

    def toDict(self):
        return {
            "BoardID": self.boardID,
            "Name": self.name,
        }


def canSerialize(obj):
    return (
        isinstance(obj, Serializable)
        or isinstance(obj, list)
        or isinstance(obj, np.ndarray)
    )


# Recursively converts an object to XML
def toXML(name, object, parent=None):
    # Serialize 2D NumPy array into multiple <Row Val="x y z ..."/> nodes
    if isinstance(object, np.ndarray) and object.ndim == 2:
        matrixNode = ET.SubElement(parent, name)
        for row in object:
            serializedRow = " ".join(map(lambda x: "{:6.4f}".format(x), row.tolist()))
            rowNode = ET.SubElement(matrixNode, "Row", Val=serializedRow)
        return matrixNode

    data = object.toDict()

    attributes = {
        key: str(value) for (key, value) in data.items() if not canSerialize(value)
    }

    node = (
        ET.SubElement(parent, name, attributes)
        if parent != None
        else ET.Element(name, attributes)
    )

    children = {key: value for (key, value) in data.items() if canSerialize(value)}

    for key, value in children.items():
        # for XML, list values are represented as repeated nodes with
        if isinstance(value, list):
            # remove plural 's' if present
            nodeName = key if key[-1] != "s" else key[0:-1]
            for v in value:
                toXML(nodeName, v, node)
        else:
            toXML(key, value, node)

    return node


# Adds nice indentation to existing ElementTree XML object
def pretty_print_xml(node, parent=None, index=-1, depth=0):
    indent = "    "
    for i, subNode in enumerate(node):
        pretty_print_xml(subNode, node, i, depth + 1)

    if parent is not None:
        if index == 0:
            parent.text = "\n" + (indent * depth)
        else:
            parent[index - 1].tail = "\n" + (indent * depth)

        if index == len(parent) - 1:
            node.tail = "\n" + (indent * (depth - 1))


# Allows use of our Serializable objects with Python's builtin json library
def configSerializeJSON(obj):
    if isinstance(obj, Serializable):
        return obj.toDict()

    if isinstance(obj, np.ndarray):
        return obj.tolist()

    raise TypeError


def generateConfig(calFileName, robotName, outputFormat):
    # Array size constants that .cal file parser needs to know
    CALContext = {
        "UPPER_LIMIT": 1,
        "LOWER_LIMIT": 2,
        # constants for MTM .cal configuration file
        "MST_JNT_POS_GR_DOFS": 8,
        "MST_MOT_DOFS": 8,
        "MST_JNT_POS_DOFS": 7,
        # constants for PSM .cal configuration file
        "SLV_JNT_POS_GR_DOFS": 7,
        "SLV_MOT_DOFS": 7,
        # constants for ECM .cal configuration file
        "ECM_JNT_POS_GR_DOFS": 4,
        "ECM_MOT_DOFS": 4,
    }

    calData = calParser.parseCALFile(calFileName, CALContext)

    # sanity check robot type matches cal file
    assert (
        robotName[0:3] == calData["FileType"][0:3]
    ), "Robot hardware type doesn't match type from cal file"

    version = 4
    config = Config(calData, version, robotName)

    outputFileName = "sawRobotIO1394-" + robotName + "-" + str(calData["serial_number"])

    if outputFormat == OutputFormat.JSON:
        with open(outputFileName + ".json", "w") as f:
            json.dump(config, f, indent=4, default=configSerializeJSON)
    elif outputFormat == OutputFormat.XML:
        root = toXML("Config", config)
        pretty_print_xml(root)

        tree = ET.ElementTree(root)
        tree.write(outputFileName + ".xml", xml_declaration=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-a",
        "--arm",
        type=str,
        required=True,
        choices=["MTML", "MTMR", "PSM1", "PSM2", "PSM3", "ECM"],
        help="robot arm hardware type",
    )
    parser.add_argument("-c", "--cal", type=str, required=True, help="calibration file")
    parser.add_argument(
        "-f",
        "--format",
        type=str,
        default="XML",
        choices=["XML", "JSON"],
        help="calibration file",
    )

    args = parser.parse_args()  # argv[0] is executable name

    outputFormat = OutputFormat.XML if args.format == "XML" else OutputFormat.JSON
    generateConfig(args.cal, args.arm, outputFormat)
