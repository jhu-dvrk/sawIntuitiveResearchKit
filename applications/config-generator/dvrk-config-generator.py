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
    MTMGripper = 3

    def fromTypeName(robotTypeName):
        if robotTypeName[0:3] == "MTM":
            if robotTypeName.endswith("-Gripper"):
                return RobotType.MTMGripper
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
        if self.offset != None:
            dict["Offset"] = self.offset
        if self.scale != None:
            dict["Scale"] = self.scale
        if self.units != None:
            dict["Unit"] = self.units
        return dict


# Dimensioned value
class UnitValue(Serializable):
    def __init__(self, value, units):
        self.value = value
        self.units = units

    def toDict(self):
        dict = {}
        if self.units != None:
            dict["Unit"] = self.units
        if self.value != None:
            dict["Value"] = self.value
        return dict


# dVRK board ID conventions
def getBoardIDs(robotTypeName):
    # Need to allow for MTML-Gripper/MTMR-Gripper
    if robotTypeName.startswith("MTML"):
        return (0, 1)
    elif robotTypeName.startswith("MTMR"):
        return (2, 3)
    elif robotTypeName == "ECM":
        return (4, 5)
    elif robotTypeName == "PSM1":
        return (6, 7)
    elif robotTypeName == "PSM2":
        return (8, 9)
    elif robotTypeName == "PSM3":
        return (10, 11)
    else:
        raise ValueError("Unrecognized robot type: {}".format(robotTypeName))


# Root config object
class Config(Serializable):
    def __init__(self, calData, versionID, robotTypeName, robotGeneration):
        self.versionID = versionID
        serialNumber = calData["serial_number"]

        robotType = RobotType.fromTypeName(robotTypeName)
        isClassic = robotGeneration == "Classic"
        if robotType == RobotType.PSM:
            if isClassic:
                self.robot = ClassicPSM(calData, robotTypeName, serialNumber)
            else:
                self.robot = SiPSM(calData, robotTypeName, serialNumber)
        elif robotType == RobotType.ECM:
            if isClassic:
                self.robot = ClassicECM(calData, robotTypeName, serialNumber)
            else:
                raise ValueError("Si ECM is currently unsupported.")
        elif robotType == RobotType.MTM:
            self.robot = MTM(calData, robotTypeName, serialNumber)
        else:
            self.robot = MTMGripper(calData, robotTypeName, serialNumber)

        self.digitalInputs = list(self.robot.generateDigitalInputs())
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
    def __init__(self, robotTypeName, serialNumber, calData, numberOfActuators):
        self.name = robotTypeName
        self.type = RobotType.fromTypeName(robotTypeName)
        self.serialNumber = serialNumber
        self.calData = calData
        self.boardIDs = getBoardIDs(robotTypeName)
        self.numberOfActuators = numberOfActuators
        self.actuators = list(self.generateActuators())

    def driveDirection(self, index: int) -> int:
        raise NotImplementedError()

    def encoderCPT(self, index: int) -> int:
        raise NotImplementedError()

    def gearRatio(self, index: int) -> float:
        raise NotImplementedError()

    def pitch(self, index: int) -> float:
        raise NotImplementedError()

    def motorMaxCurrent(self, index: int) -> float:
        raise NotImplementedError()

    def motorTorque(self, index: int) -> float:
        raise NotImplementedError()

    def actuatorType(self, index: int) -> str:
        raise NotImplementedError()

    def potentiometerUnits(self, index: int) -> str:
        raise NotImplementedError()

    def potentiometerLatency(self, index: int) -> float:
        raise NotImplementedError()

    def potentiometerDistance(self, index: int) -> float:
        raise NotImplementedError()

    def generateDrives(self):
        for index in range(self.numberOfActuators):
            direction = self.driveDirection(index)
            gearRatio = self.gearRatio(index)
            motorTorque = self.motorTorque(index)
            maxCurrent = self.motorMaxCurrent(index)
            yield Drive(direction, gearRatio, motorTorque, maxCurrent)

    def generateEncoders(self):
        for index in range(self.numberOfActuators):
            units = self.potentiometerUnits(index)
            direction = self.driveDirection(index)
            encoderCPT = self.encoderCPT(index)
            pitch = self.pitch(index)
            gearRatio = self.gearRatio(index)
            yield Encoder(units, direction, encoderCPT, pitch, gearRatio)

    def generateAnalogIns(self):
        for index in range(self.numberOfActuators):
            units = self.potentiometerUnits(index)
            pitch = self.pitch(index)
            yield AnalogIn(self.calData, index, units, pitch)

    def generateBrakes(self):
        for index in range(self.numberOfActuators):
            yield None

    def generateDigitalPotentiometers(self):
        for index in range(self.numberOfActuators):
            yield None

    def generateActuators(self):
        drives = self.generateDrives()
        encoders = self.generateEncoders()
        analogIns = self.generateAnalogIns()
        brakes = self.generateBrakes()
        digitalPots = self.generateDigitalPotentiometers()
        data = zip(range(self.numberOfActuators), drives, encoders, analogIns, brakes, digitalPots)
        for index, drive, encoder, analogIn, brake, digitalPot in data:
            type = self.actuatorType(index)
            yield Actuator(
                index, index, type, self.boardIDs, drive, encoder, analogIn, brake, digitalPot
            )

    def generateDigitalInputs(self):
        # Creates generator that terminates without yielding anything
        yield from ()

    def toDict(self):
        dict = {
            "Name": self.name,
            "NumOfActuator": self.numberOfActuators,
            "NumOfJoint": self.numberOfActuators,
            "SN": self.serialNumber,
            "Actuators": self.actuators,
        }

        return dict


class ClassicPSM(Robot):
    def __init__(self, calData, robotTypeName, serialNumber):
        self.driveDirection = lambda index: [-1, -1, 1, -1, -1, 1, 1][index]
        self.encoderCPT = lambda index: [14400, 14400, 14400, 4000, 4000, 4000, 4000][index]
        self.gearRatio = lambda index: [56.50, 56.50, 336.6, 11.71, 11.71, 11.71, 11.71][index]
        self.pitch = lambda index: [1, 1, 17.4533, 1, 1, 1, 1][index]
        self.motorMaxCurrent = lambda index: [1.34, 1.34, 0.67, 0.67, 0.67, 0.67, 0.670][index]
        self.motorTorque = lambda index: [0.0438, 0.0438, 0.0438, 0.0438, 0.0438, 0.0438, 0.0438][index]
        self.actuatorType = lambda index: "Revolute" if index != 2 else "Prismatic"
        self.potentiometerUnits = lambda index: "deg" if index != 2 else "mm"
        self.potentiometerLatency = lambda index: 0.01
        self.potentiometerDistance = lambda index: 5.0

        super().__init__(robotTypeName, serialNumber, calData, 7)

        potentiometerTolerances = list(self.generatePotentiometerTolerances())
        self.potentiometers = Potentiometers("Actuators", potentiometerTolerances)

    def generatePotentiometerTolerances(self):
        for index in range(self.numberOfActuators):
            units = self.potentiometerUnits(index)
            latency = self.potentiometerLatency(index)
            distance = self.potentiometerDistance(index)
            yield PotentiometerTolerance(index, units, latency, distance)

    def generateDigitalInputs(self):
        digitalInputBitIDs = [
            (self.boardIDs[0], 0, "SUJClutch", 0.2),
            (self.boardIDs[0], 2, "ManipClutch", 0.2),
            (self.boardIDs[1], 7, "Tool", 1.5),
            (self.boardIDs[1], 10, "Adapter", 1.5),
        ]

        for boardID, bitID, inputType, debounceTime in digitalInputBitIDs:
            yield DigitalInput(self.name, inputType, bitID, boardID, 1, debounceTime)

    def toDict(self):
        dict = super().toDict()
        dict["Potentiometers"] = self.potentiometers
        return dict


class SiPSM(Robot):
    def __init__(self, calData, robotTypeName, serialNumber):
        self.driveDirection = lambda index: [1, -1, 1, -1, -1, 1, 1][index]
        self.encoderCPT = lambda index: [81920, 81920, 50000, 4000, 4000, 4000, 4000][index]
        self.gearRatio = lambda index: [83.3333, 85.000, 965.91, 13.813, 13.813, 13.813, 13.813][index]
        self.pitch = lambda index: [1, 1, 17.4533, 1, 1, 1, 1][index]
        self.motorMaxCurrent = lambda index: [3.4, 3.4, 1.1, 1.1, 1.1, 1.1, 1.1][index]
        self.motorTorque = lambda index: [0.0603, 0.0603, 0.0385, 0.0385, 0.0385, 0.0385, 0.0385][index]
        self.actuatorType = lambda index: "Revolute" if index != 2 else "Prismatic"
        self.potentiometerUnits = lambda index: "deg" if index != 2 else "mm"
        self.potentiometerLatency = lambda index: 0.01
        self.potentiometerDistance = lambda index: 10.0

        self.driveLinearAmpCurrent = 8.192
        self.digitalPotResolution = lambda index: 4095 # 2^12 - 1

        self.brakeMaxCurrent = lambda index: [0.2, 0.2, 0.3][index]
        self.brakeReleaseCurrent = lambda index: [0.15, 0.15, -0.25][index]
        self.brakeReleaseTime = lambda index: 0.5
        self.brakeReleasedCurrent = lambda index: [0.08, 0.08, -0.25][index]
        self.brakeEngagedCurrent = lambda index: 0.0
        self.brakeLinearAmpCurrent = 2.048 # 2^11/10^3

        super().__init__(robotTypeName, serialNumber, calData, 7)

        potentiometerTolerances = list(self.generatePotentiometerTolerances())
        self.potentiometers = Potentiometers("Actuators", potentiometerTolerances)

    def digitalPotMinimum(self, index):
        lower_limit_index = self.calData["LOWER_LIMIT"] - 1
        raw_pot_minimum = self.calData["motor"]["pot_limit_checks"][lower_limit_index][index]

        # Make minimum limit be positive
        resolution = self.digitalPotResolution(index)
        pot_minimum = (raw_pot_minimum + resolution) % resolution
        return int(pot_minimum)

    def generatePotentiometerTolerances(self):
        for index in range(self.numberOfActuators):
            units = self.potentiometerUnits(index)
            latency = self.potentiometerLatency(index)
            distance = self.potentiometerDistance(index)
            yield PotentiometerTolerance(index, units, latency, distance)

    def generateBrakes(self):
        for index in range(self.numberOfActuators):
            if index <= 2:
                maxCurrent = self.brakeMaxCurrent(index)
                releaseCurrent = self.brakeReleaseCurrent(index)
                releaseTime = self.brakeReleaseTime(index)
                releasedCurrent = self.brakeReleasedCurrent(index)
                engagedCurrent = self.brakeEngagedCurrent(index)
                yield AnalogBrake(
                    index,
                    self.boardIDs[1],
                    maxCurrent,
                    releaseCurrent,
                    releaseTime,
                    releasedCurrent,
                    engagedCurrent,
                    linearAmpCurrent=self.brakeLinearAmpCurrent
                )
            else:
                yield None

    def generateDrives(self):
        for index in range(self.numberOfActuators):
            direction = self.driveDirection(index)
            gearRatio = self.gearRatio(index)
            motorTorque = self.motorTorque(index)
            maxCurrent = self.motorMaxCurrent(index)
            yield Drive(
                direction,
                gearRatio,
                motorTorque,
                maxCurrent,
                linearAmpCurrent=self.driveLinearAmpCurrent,
            )

    def generateDigitalPotentiometers(self):
        for index in range(self.numberOfActuators):
            min =  self.digitalPotMinimum(index)
            resolution = self.digitalPotResolution(index)
            units = self.potentiometerUnits(index)
            yield DigitalPotentiometer(min, resolution, units)

    def generateAnalogIns(self):
        for index in range(self.numberOfActuators):
            yield None

    def generateDigitalInputs(self):
        digitalInputBitIDs = [
            (self.boardIDs[0], 0, "SUJClutch2", 0.2),
            (self.boardIDs[0], 4, "SUJClutch", 0.2),
            (self.boardIDs[1], 11, "Tool", 1.5),
            (self.boardIDs[1], 13, "ManipClutch", 0.01),
            (self.boardIDs[1], 17, "Adapter", 1.5),
        ]

        for boardID, bitID, inputType, debounceTime in digitalInputBitIDs:
            yield DigitalInput(self.name, inputType, bitID, boardID, 0, debounceTime)

    def toDict(self):
        dict = super().toDict()
        dict["Potentiometers"] = self.potentiometers
        return dict


class ClassicECM(Robot):
    def __init__(self, calData, robotTypeName, serialNumber):
        self.driveDirection = lambda index: [1, 1, -1, 1][index]
        self.encoderCPT = lambda index: [4000, 4000, 640, 64][index]
        self.gearRatio = lambda index: [240, 240, 2748.55, 300.15][index]
        self.pitch = lambda index: [1, 1, 17.4533, 1][index]
        self.motorMaxCurrent = lambda index: [0.943, 0.943, 0.67, 0.59][index]
        self.motorTorque = lambda index: [0.1190, 0.1190, 0.0438, 0.00495][index]
        self.actuatorType = lambda index: "Revolute" if index != 2 else "Prismatic"
        self.potentiometerUnits = lambda index: "deg" if index != 2 else "mm"
        self.potentiometerLatency = lambda index: 0.01
        self.potentiometerDistance = lambda index: 5.0

        self.brakeMaxCurrent = lambda index: [0.25, 0.22, 0.90][index]
        self.brakeReleaseCurrent = lambda index: [0.25, 0.22, 0.90][index]
        self.brakeReleaseTime = lambda index: 2.0
        self.brakeReleasedCurrent = lambda index: [0.08, 0.07, 0.20][index]
        self.brakeEngagedCurrent = lambda index: 0.0

        super().__init__(robotTypeName, serialNumber, calData, 4)

        potentiometerTolerances = list(self.generatePotentiometerTolerances())
        self.potentiometers = Potentiometers("Actuators", potentiometerTolerances)

    def generatePotentiometerTolerances(self):
        for index in range(self.numberOfActuators):
            units = self.potentiometerUnits(index)
            latency = self.potentiometerLatency(index)
            distance = self.potentiometerDistance(index)
            yield PotentiometerTolerance(index, units, latency, distance)

    def generateBrakes(self):
        for index in range(self.numberOfActuators):
            if index <= 2:
                maxCurrent = self.brakeMaxCurrent(index)
                releaseCurrent = self.brakeReleaseCurrent(index)
                releaseTime = self.brakeReleaseTime(index)
                releasedCurrent = self.brakeReleasedCurrent(index)
                engagedCurrent = self.brakeEngagedCurrent(index)
                yield AnalogBrake(
                    index,
                    self.boardIDs[1],
                    maxCurrent,
                    releaseCurrent,
                    releaseTime,
                    releasedCurrent,
                    engagedCurrent
                )
            else:
                yield None

    def generateDigitalInputs(self):
        digitalInputBitIDs = [
            (self.boardIDs[0], 0, "ManipClutch", 0.2),
            (self.boardIDs[0], 2, "SUJClutch", 0.2),
        ]

        for boardID, bitID, inputType, debounceTime in digitalInputBitIDs:
            yield DigitalInput(self.name, inputType, bitID, boardID, 1, debounceTime)

    def toDict(self):
        dict = super().toDict()
        dict["Potentiometers"] = self.potentiometers
        return dict


class MTM(Robot):
    def __init__(self, calData, robotTypeName, serialNumber):
        if robotTypeName.startswith("MTML"):
            driveDirections = [-1, 1, 1, 1, -1, 1, -1]
        elif robotTypeName.startswith("MTMR"):
            driveDirections = [-1, 1, 1, 1, 1, 1, -1]
        else:
            raise ValueError("Unsupport MTM type: {}".format(robotTypeName))

        self.driveDirection = lambda index: driveDirections[index]
        self.encoderCPT = lambda index: [4000, 4000, 4000, 4000, 64, 64, 64][index]
        self.gearRatio = lambda index: [63.41, 49.88, 59.73, 10.53, 33.16, 33.16, 16.58][index]
        self.pitch = lambda index: 1
        self.motorMaxCurrent = lambda index: [0.67, 0.67, 0.67, 0.67, 0.59, 0.59, 0.407][index]
        self.motorTorque = lambda index: [0.0438, 0.0438, 0.0438, 0.0438, 0.00495, 0.00495, 0.00339][index]
        self.actuatorType = lambda index: "Revolute"
        self.potentiometerUnits = lambda index: "deg"
        self.potentiometerLatency = lambda index: 0.01 if index <= 5 else 0.0
        self.potentiometerDistance = lambda index: 5.0 if index <= 5 else 0.0

        super().__init__(robotTypeName, serialNumber, calData, 7)

        potentiometerTolerances = list(self.generatePotentiometerTolerances())
        self.potentiometers = Potentiometers("Joints", potentiometerTolerances)
        self.coupling = MTMCoupling()

    def generatePotentiometerTolerances(self):
        for index in range(self.numberOfActuators):
            units = self.potentiometerUnits(index)
            latency = self.potentiometerLatency(index)
            distance = self.potentiometerDistance(index)
            yield PotentiometerTolerance(index, units, latency, distance)

    def toDict(self):
        dict = super().toDict()
        dict["Potentiometers"] = self.potentiometers
        dict["Coupling"] = self.coupling
        return dict


class MTMGripper(Robot):
    def __init__(self, calData, robotTypeName, serialNumber):
        self.ioType = "io-only"
        self.driveDirection = lambda index: -1
        self.encoderCPT = lambda index: 4000
        self.gearRatio = lambda index: 63.41
        self.pitch = lambda index: 1
        self.motorMaxCurrent = lambda index: 0.0
        self.motorTorque = lambda index: 0.0438
        self.actuatorType = lambda index: "Revolute"
        self.potentiometerUnits = lambda index: "deg"
        self.potentiometerLatency = lambda index: None
        self.potentiometerDistance = lambda index: None

        # Want driveDirection * (360 / CPT) * (pitch / gearRatio) = 360
        desiredEncoderScale = 360.0
        self.encoderCPT = (
            lambda index: self.driveDirection(0)
            * (360 / desiredEncoderScale)
            * (self.pitch(0) / self.gearRatio(0))
        )

        self.voltsToPosSIScale = -23.1788
        self.voltsToPosSIOffset = 91.4238

        super().__init__(robotTypeName, serialNumber, calData, 1)

    def generateAnalogIns(self):
        for index in range(self.numberOfActuators):
            units = self.potentiometerUnits(index)
            pitch = self.pitch(index)
            yield AnalogIn(
                self.calData,
                index,
                units,
                pitch,
                self.voltsToPosSIScale,
                self.voltsToPosSIOffset,
            )

    def generateActuators(self):
        drives = self.generateDrives()
        encoders = self.generateEncoders()
        analogIns = self.generateAnalogIns()
        brakes = self.generateBrakes()
        data = zip(drives, encoders, analogIns, brakes)
        index = 7  # MTM gripper is treated as if it was 8th actuator of MTM

        for drive, encoder, analogIn, brake in data:
            type = self.actuatorType(index)
            id = self.boardIDs[1]
            yield Actuator(
                id, index, type, self.boardIDs, drive, encoder, analogIn, brake
            )

    def toDict(self):
        dict = super().toDict()
        dict["Type"] = self.ioType

        return dict


class Drive(Serializable):
    def __init__(
        self,
        driveDirection,
        gearRatio,
        motorTorque,
        motorMaxCurrent,
        DACresolution=16,
        linearAmpCurrent=6.25,
    ):
        ampsToBitsScale = driveDirection * (2**(DACresolution-1))/linearAmpCurrent
        self.ampsToBits = Conversion(
            "{:5.4f}".format(ampsToBitsScale), "{:5.0f}".format(2**(DACresolution-1))
        )

        bitsToAmpsScale = 1.0 / ampsToBitsScale
        self.bitsToFeedbackAmps = Conversion(
            "{:5.9f}".format(bitsToAmpsScale),
            "{:5.4f}".format(driveDirection * -linearAmpCurrent),
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


class AnalogBrake(Serializable):
    def __init__(
        self,
        axisID,
        boardID,
        maxCurrent,
        releaseCurrent,
        releaseTime,
        releasedCurrent,
        engagedCurrent,
        DACresolution=16,
        linearAmpCurrent=6.25,
    ):
        self.axisID = axisID
        self.boardID = boardID

        ampsToBitsScale = (2**(DACresolution-1))/linearAmpCurrent
        bitsToAmpsScale = 1.0 / ampsToBitsScale
        self.ampsToBits = Conversion(ampsToBitsScale, 2 ** (DACresolution-1))
        self.bitsToFeedbackAmps = Conversion(bitsToAmpsScale, -linearAmpCurrent)

        self.maxCurrent = UnitValue(maxCurrent, "A")
        self.releaseCurrent = UnitValue(releaseCurrent, "A")
        self.releaseTime = UnitValue(releaseTime, "s")
        self.releasedCurrent = UnitValue(releasedCurrent, "A")
        self.engagedCurrent = UnitValue(engagedCurrent, "A")

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
            "EngagedCurrent": self.engagedCurrent,
        }


class Encoder(Serializable):
    def __init__(
        self, potentiometerUnits, driveDirection, CPT, pitch, gearRatio
    ):
        encoderPos = driveDirection * (360 / CPT) * (pitch / gearRatio)
        encoderPos = "{:5.8f}".format(encoderPos)
        self.bitsToPosSI = Conversion(encoderPos, None, potentiometerUnits)

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
    def __init__(
        self,
        calData,
        actuatorIndex,
        potentiometerUnits,
        pitch,
        voltsToPosSIScale=None,
        voltsToPosSIOffset=None,
    ):
        # 16 bits ADC with 4.5 V ref
        self.bitsToVolts = Conversion(4.5 / (2 ** 16), "0")

        potGain = calData["motor"]["pot_input_gain"][actuatorIndex]
        potOffset = calData["motor"]["pot_input_offset"][actuatorIndex]

        if voltsToPosSIScale is None:
            voltsToPosSIScale = potGain * (2 ** 12 / 4.5) * (180.0 / math.pi) * pitch

        if voltsToPosSIOffset is None:
            voltsToPosSIOffset = potOffset * (180.0 / math.pi) * pitch

        voltsToPosSIScale = "{:5.6f}".format(voltsToPosSIScale)
        voltsToPosSIOffset = "{:5.6f}".format(voltsToPosSIOffset)

        self.voltsToPosSI = Conversion(
            voltsToPosSIScale, voltsToPosSIOffset, potentiometerUnits
        )

    def toDict(self):
        return {
            "BitsToVolts": self.bitsToVolts,
            "VoltsToPosSI": self.voltsToPosSI,
        }


class Potentiometers(Serializable):
    def __init__(self, position, tolerances):
        self.position = position
        self.tolerances = tolerances

    def toDict(self):
        return {
            "Position": self.position,
            "Tolerances": self.tolerances,
        }


class PotentiometerTolerance(Serializable):
    def __init__(self, axisID, units, latency, distance):
        self.axisID = axisID
        self.units = units
        self.latency = latency
        self.distance = distance

    def toDict(self):
        return {
            "Axis": self.axisID,
            "Distance": "{:5.2f}".format(self.distance),
            "Latency": "{:5.2f}".format(self.latency),
            "Unit": self.units,
        }

class Potentiometer(Serializable):
    def __init__(self, min, resolution):
        self.min = min
        self.resolution = resolution

    def toDict(self):
        return {
            "Min": self.min,
            "Resolution": self.resolution,
        }

class DigitalPotentiometer(Serializable):
    def __init__(self, min, resolution, units):
        self.pot = Potentiometer(min, resolution)
        self.potentiometerToPositionSI = Conversion(0.0, 0.0, units)

    def toDict(self):
        return {
            "Pot": self.pot,
            "PotToPosSI": self.potentiometerToPositionSI,
        }


class MTMCoupling(Serializable):
    def __init__(self):
        self.actuatorToJointPositionMatrix = [
            [1.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00],
            [0.00, 1.00, 0.00, 0.00, 0.00, 0.00, 0.00],
            [0.00, -1.00, 1.00, 0.00, 0.00, 0.00, 0.00],
            [0.00, 0.6697, -0.6697, 1.00, 0.00, 0.00, 0.00],
            [0.00, 0.00, 0.00, 0.00, 1.00, 0.00, 0.00],
            [0.00, 0.00, 0.00, 0.00, 0.00, 1.00, 0.00],
            [0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 1.00],
        ]

    def toDict(self):
        return {
            "Value": 1,
            "ActuatorToJointPosition": self.actuatorToJointPositionMatrix,
        }


class DigitalInput(Serializable):
    def __init__(self, robotTypeName: str, type: str, bitID: int, boardID: int, pressed: int, debounceTime: float):
        self.bitID = bitID
        self.boardID = boardID
        self.name = "{}-{}".format(robotTypeName, type)
        self.pressed = pressed
        self.trigger = "all"
        self.debounceTime = debounceTime

    def toDict(self):
        return {
            "BitID": self.bitID,
            "BoardID": self.boardID,
            "Debounce": self.debounceTime,
            "Name": self.name,
            "Pressed": self.pressed,
            "Trigger": self.trigger,
        }


class DallasChip(Serializable):
    def __init__(self, robotTypeName):
        self.name = "{}-Dallas".format(robotTypeName)
        self.boardID = getBoardIDs(robotTypeName)[1]

    def toDict(self):
        return {
            "BoardID": self.boardID,
            "Name": self.name,
        }


class Actuator(Serializable):
    """
    type: "Prismatic" or "Revolute"
    CPT: Encoder counts per turn (quadrature encoder), NOTE: no encoder for last axis
    gearRatio: NOTE: gear ratio for axis 8 is set to 1
    pitch: degrees for revolute, mm for prismatic
    motorMaxCurrent: Sustained max current, units are amps
    motorMaxTorque: units are Nm/A
    """

    def __init__(
        self,
        id: int,
        index: int,
        type: str,
        boardIDs,
        drive: Drive,
        encoder: Encoder,
        analogIn: AnalogIn = None,
        brake: AnalogBrake = None,
        digitalPotentiometer = None,
    ):
        self.id = id
        self.boardID = boardIDs[0] if index < 4 else boardIDs[1]
        self.axisID = index % 4
        self.type = type
        self.drive = drive
        self.encoder = encoder
        self.analogIn = analogIn
        self.brake = brake
        self.digitalPotentiometer = digitalPotentiometer

    def toDict(self):
        dict = {
            "ActuatorID": self.id,
            "AxisID": self.axisID,
            "BoardID": self.boardID,
            "Type": self.type,
            "Drive": self.drive,
            "Encoder": self.encoder,
        }

        if self.brake is not None:
            dict["AnalogBrake"] = self.brake

        if self.analogIn is not None:
            dict["AnalogIn"] = self.analogIn

        if self.digitalPotentiometer is not None:
            dict["DigitalPot"] = self.digitalPotentiometer

        return dict


def array_like(obj):
    return isinstance(obj, list) or isinstance(obj, np.ndarray)


def canSerialize(obj):
    return isinstance(obj, Serializable) or array_like(obj)


# Serialize 1D or 2D Numpy arrays/Python lists
def arrayToXML(name, object, parent):
    # Serialize 1D array into repeated child nodes
    if len(np.shape(object)) == 1:
        # remove plural 's' if present
        name = name if name[-1] != "s" else name[0:-1]
        for element in object:
            toXML(name, element, parent)
    # Serialize 2D array into multiple <Row Val="x y z ..."/> nodes
    else:
        matrixNode = ET.SubElement(parent, name)
        for row in object:
            row = row.tolist() if isinstance(row, np.ndarray) else row
            serializedRow = " ".join(map(lambda x: "{:6.4f}".format(x), row))
            ET.SubElement(matrixNode, "Row", Val=serializedRow)
        return matrixNode


# Convert instance of 'Serializable' to XML
def serializableToXML(name, object, parent):
    data = object.toDict()

    attributes = {
        key: str(value) for (key, value) in data.items() if not canSerialize(value)
    }

    node = (
        ET.SubElement(parent, name, attributes)
        if parent != None
        else ET.Element(name, attributes)
    )

    for key, value in data.items():
        if canSerialize(value):
            toXML(key, value, node)

    return node


# Recursively converts an object to XML
def toXML(name, object, parent=None):
    # Serialize NumPy arrays and Python lists
    if array_like(object):
        arrayToXML(name, object, parent)
    elif isinstance(object, Serializable):
        return serializableToXML(name, object, parent)
    else:
        raise ValueError(
            "Can only serialize instances of Serializable and array-like types"
        )


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

    raise TypeError


def saveConfigFile(fileName, config, format):
    fileName += ".json" if format == OutputFormat.JSON else ".xml"

    if format == OutputFormat.JSON:
        with open(fileName, "w") as f:
            json.dump(config, f, indent=4, default=configSerializeJSON)
    elif format == OutputFormat.XML:
        root = toXML("Config", config)
        pretty_print_xml(root)

        tree = ET.ElementTree(root)
        tree.write(fileName, xml_declaration=True)

    print("Generated config file {}".format(fileName))


def generateConfig(calFileName, robotTypeName, generation, outputFormat):
    # Array index/dimension constants that .cal file parser needs to know
    constants = {
        "UPPER_LIMIT": 1,
        "LOWER_LIMIT": 2,
        # constants for MTM calibration file
        "MST_JNT_POS_GR_DOFS": 8,
        "MST_MOT_DOFS": 8,
        "MST_JNT_POS_DOFS": 7,
        # constants for PSM calibration file
        "SLV_JNT_POS_GR_DOFS": 7,
        "SLV_MOT_DOFS": 7,
        # constants for ECM calibration file
        "ECM_JNT_POS_GR_DOFS": 4,
        "ECM_MOT_DOFS": 4,
        # index constants for MTM Si calibration file
        "INDEX1": 1,
        "INDEX2": 2,
        "INDEX3": 3,
        "INDEX4": 4,
        "INDEX5": 5,
        "INDEX6": 6,
        "INDEX7": 7,
    }

    parser = calParser.CalParser(constants)
    calData = parser.parseFile(calFileName)

    # sanity check robot type matches cal file
    assert (
        robotTypeName[0:3] == calData["FileType"][0:3]
    ), "Robot hardware type doesn't match type from cal file"

    version = 4
    config = Config(calData, version, robotTypeName, generation)

    outputFileName = "sawRobotIO1394-" + robotTypeName + "-" + str(calData["serial_number"])
    saveConfigFile(outputFileName, config, outputFormat)

    if robotTypeName[0:3] == "MTM":
        gripperConfigFileName = (
            "sawRobotIO1394-" + robotTypeName + "-gripper-" + str(calData["serial_number"])
        )
        gripperConfig = Config(calData, version, robotTypeName + "-Gripper", generation)
        saveConfigFile(gripperConfigFileName, gripperConfig, outputFormat)


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
    parser.add_argument(
        "-g",
        "--generation",
        type=str,
        default="Classic",
        choices=["Classic", "Si"],
        help="robot arm hardware generation",
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

    args = parser.parse_args()

    outputFormat = OutputFormat.XML if args.format == "XML" else OutputFormat.JSON
    generateConfig(args.cal, args.arm, args.generation, outputFormat)
