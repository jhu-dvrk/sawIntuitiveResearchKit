#!/usr/bin/env python3

import argparse
import os
import datetime
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


def getActuatorsPerBoard(hardwareVersionName):
    if hardwareVersionName == "QLA1":
        return 4
    elif hardwareVersionName == "DQLA":
        return 8
    elif hardwareVersionName == "dRA1":
        return 10
    else:
        raise ValueError("Unrecognized controller type: {}".format(hardwareVersionName))


# Root config object
class Config(Serializable):
    def __init__(self, calData, versionID, robotTypeName, hardwareVersionName, serialNumber, robotGenerationName):
        self.versionID = versionID

        robotType = RobotType.fromTypeName(robotTypeName)
        isClassic = robotGenerationName == "Classic"
        if robotType == RobotType.PSM:
            if isClassic:
                self.robot = ClassicPSM(calData, robotTypeName, hardwareVersionName, serialNumber)
            else:
                self.robot = SiPSM(calData, robotTypeName, hardwareVersionName, serialNumber)
        elif robotType == RobotType.ECM:
            if isClassic:
                self.robot = ClassicECM(calData, robotTypeName, hardwareVersionName, serialNumber)
            else:
                self.robot = SiECM(calData, robotTypeName, hardwareVersionName, serialNumber)
        elif robotType == RobotType.MTM:
            self.robot = MTM(calData, robotTypeName, hardwareVersionName, serialNumber)
        else:
            self.robot = MTMGripper(calData, robotTypeName, hardwareVersionName, serialNumber)

        self.digitalInputs = list(self.robot.generateDigitalInputs())
        self.digitalOutputs = list(self.robot.generateDigitalOutputs())
        self.dallasChip = self.robot.generateDallasChip()

    def toDict(self):
        dict = {
            "Version": self.versionID,
            "Robot": self.robot,
        }

        if self.dallasChip != None:
            dict["DallasChip"] = self.dallasChip

        if len(self.digitalInputs) > 0:
            dict["DigitalIns"] = self.digitalInputs

        if len(self.digitalOutputs) > 0:
            dict["DigitalOuts"] = self.digitalOutputs

        return dict


class Robot(Serializable):
    def __init__(self, robotTypeName, hardwareVersionName, serialNumber, calData, numberOfActuators):
        self.name = robotTypeName
        self.type = RobotType.fromTypeName(robotTypeName)
        self.serialNumber = serialNumber
        self.hardwareVersion = hardwareVersionName
        self.calData = calData
        self.boardIDs = getBoardIDs(robotTypeName)
        self.numberOfActuators = numberOfActuators
        self.actuatorsPerBoard = getActuatorsPerBoard(hardwareVersionName)
        self.actuators = list(self.generateActuators())

    def driveDirection(self, index: int) -> int:
        raise NotImplementedError()

    def encoderDirection(self, index: int) -> int:
        return self.driveDirection(index)

    def encoderCPT(self, index: int) -> int:
        raise NotImplementedError()

    def gearRatio(self, index: int) -> float:
        raise NotImplementedError()

    def pitch(self, index: int) -> float:
        raise NotImplementedError()

    def velocitySource(self, index: int) -> str:
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
            direction = self.encoderDirection(index)
            encoderCPT = self.encoderCPT(index)
            gearRatio = self.gearRatio(index)
            pitch = self.pitch(index)
            velocitySource = self.velocitySource(index)
            yield Encoder(units, direction, encoderCPT, gearRatio, pitch, velocitySource)

    def generateAnalogIns(self):
        for index in range(self.numberOfActuators):
            units = self.potentiometerUnits(index)
            pitch = self.pitch(index)
            yield AnalogIn(self.calData, index, units, pitch)

    def generateBrakes(self):
        for index in range(self.numberOfActuators):
            yield None

    def generateActuators(self):
        drives = self.generateDrives()
        encoders = self.generateEncoders()
        analogIns = self.generateAnalogIns()
        brakes = self.generateBrakes()
        data = zip(range(self.numberOfActuators), drives, encoders, analogIns, brakes)
        for index, drive, encoder, analogIn, brake in data:
            type = self.actuatorType(index)
            boardID = self.boardIDs[index // self.actuatorsPerBoard]
            axisID = index % self.actuatorsPerBoard

            yield Actuator(
                index, type, boardID, axisID, drive, encoder, analogIn, brake
            )

    def generateDigitalInputs(self):
        # Creates generator that terminates without yielding anything
        yield from ()

    def generateDigitalOutputs(self):
        # Creates generator that terminates without yielding anything
        yield from ()

    def generateDallasChip(self):
        return None

    def toDict(self):
        dict = {
            "Name": self.name,
            "HardwareVersion": self.hardwareVersion,
            "NumOfActuator": self.numberOfActuators,
            "NumOfJoint": self.numberOfActuators,
            "SN": self.serialNumber,
            "Actuators": self.actuators,
        }

        return dict


class ClassicPSM(Robot):
    def __init__(self, calData, robotTypeName, hardwareVersionName, serialNumber):
        self.driveDirection = lambda index: [-1, -1, 1, -1, -1, 1, 1][index]
        self.encoderCPT = lambda index: [14400, 14400, 14400, 4000, 4000, 4000, 4000][index]
        self.gearRatio = lambda index: [56.50, 56.50, 336.6, 11.71, 11.71, 11.71, 11.71][index]
        self.pitch = lambda index: [1, 1, 17.4533, 1, 1, 1, 1][index]
        self.velocitySource = lambda index: 'FIRMWARE'
        self.motorMaxCurrent = lambda index: [1.34, 1.34, 0.67, 0.67, 0.67, 0.67, 0.670][index]
        self.motorTorque = lambda index: [0.0438, 0.0438, 0.0438, 0.0438, 0.0438, 0.0438, 0.0438][index]
        self.actuatorType = lambda index: "Revolute" if index != 2 else "Prismatic"
        self.potentiometerUnits = lambda index: "deg" if index != 2 else "mm"
        self.potentiometerLatency = lambda index: 0.01
        self.potentiometerDistance = lambda index: 5.0

        super().__init__(robotTypeName, hardwareVersionName, serialNumber, calData, 7)

        potentiometerTolerances = list(self.generatePotentiometerTolerances())
        self.potentiometers = Potentiometers(potentiometerTolerances)

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
            (self.boardIDs[1], 7, "Tool", 0.2),
            (self.boardIDs[1], 10, "Adapter", 0.2),
        ]

        for boardID, bitID, inputType, debounceTime in digitalInputBitIDs:
            yield DigitalInput(self.name, inputType, bitID, boardID, 1, debounceTime)

    def generateDallasChip(self):
        return DallasChip(self.boardIDs[1], self.name)

    def toDict(self):
        dict = super().toDict()
        dict["Potentiometers"] = self.potentiometers
        return dict


class SiPSM(Robot):
    def __init__(self, calData, robotTypeName, hardwareVersionName, serialNumber):
        self.driveDirection = lambda index: [-1, 1, 1, 1, 1, -1, -1][index]
        self.encoderDirection = lambda index: -self.driveDirection(index)
        self.encoderCPT = lambda index: [81920, 81920, 50000, 4000, 4000, 4000, 4000][index]
        self.gearRatio = lambda index: [83.3333, 85.000, 965.91, 13.813, 13.813, 13.813, 13.813][index]
        self.pitch = lambda index: [1, 1, 17.4533, 1, 1, 1, 1][index]
        self.velocitySource = lambda index: 'SOFTWARE' if index < 3 else 'FIRMWARE'
        self.motorMaxCurrent = lambda index: [3.4, 3.4, 1.1, 1.1, 1.1, 1.1, 1.1][index]
        self.motorTorque = lambda index: [0.0603, 0.0603, 0.0385, 0.0385, 0.0385, 0.0385, 0.0385][index]
        self.actuatorType = lambda index: "Revolute" if index != 2 else "Prismatic"
        self.potentiometerUnits = lambda index: "deg" if index != 2 else "mm"
        self.potentiometerLatency = lambda index: 0.01
        self.potentiometerDistance = lambda index: 1.0

        # # 2^13/10^3 or 2^11/10^3
        i_high = 65536 / 4800 / 2
        self.driveLinearAmpCurrent = lambda index: [i_high, i_high, 2.048, 2.048, 2.048, 2.048, 2.048][index]

        self.brakeMaxCurrent = lambda index: [0.2, 0.2, 0.3][index]
        self.brakeReleaseCurrent = lambda index: [0.15, 0.15, -0.25][index]
        self.brakeReleaseTime = lambda index: 0.5
        self.brakeReleasedCurrent = lambda index: [0.08, 0.08, -0.25][index]
        self.brakeEngagedCurrent = lambda index: 0.0
        self.brakeLinearAmpCurrent = lambda index: 2.048 # 2^11/10^3

        super().__init__(robotTypeName, hardwareVersionName, serialNumber, calData, 7)

        potentiometerTolerances = list(self.generatePotentiometerTolerances())
        lookupTable = "sawRobotIO1394-{}-{}-PotentiometerLookupTable.json".format(robotTypeName, serialNumber)
        self.potentiometers = Potentiometers(potentiometerTolerances, lookupTable=lookupTable)

    def generatePotentiometerTolerances(self):
        for index in range(self.numberOfActuators):
            units = self.potentiometerUnits(index)
            latency = self.potentiometerLatency(index)
            distance = self.potentiometerDistance(index)
            yield PotentiometerTolerance(index, units, latency, distance)

    def generateBrakes(self):
        # Only actuators 0-2 have brakes,
        # Axis IDs are 0-6 for actuators, and 7-9 for brakes
        for index in range(self.numberOfActuators):
            if index <= 2:
                maxCurrent = self.brakeMaxCurrent(index)
                releaseCurrent = self.brakeReleaseCurrent(index)
                releaseTime = self.brakeReleaseTime(index)
                releasedCurrent = self.brakeReleasedCurrent(index)
                engagedCurrent = self.brakeEngagedCurrent(index)
                axisID = 7 + index # Brake 7 corresponds to actuator 0, etc.
                direction = 1 if index != 2 else -1 # some values in the third brake are reversed
                yield AnalogBrake(
                    axisID,
                    self.boardIDs[0],
                    direction,
                    maxCurrent,
                    releaseCurrent,
                    releaseTime,
                    releasedCurrent,
                    engagedCurrent,
                    linearAmpCurrent=self.brakeLinearAmpCurrent(index)
                )
            else:
                yield None

    def generateDrives(self):
        for index in range(self.numberOfActuators):
            yield Drive(
                self.driveDirection(index),
                self.gearRatio(index),
                self.motorTorque(index),
                self.motorMaxCurrent(index),
                linearAmpCurrent=self.driveLinearAmpCurrent(index),
            )

    def generateAnalogIns(self):
        for index in range(self.numberOfActuators):
            yield None

    def generateDigitalInputs(self):
        digitalInputBitIDs = [
            (self.boardIDs[0], 1, "SUJClutch2", 0.01),
            (self.boardIDs[0], 4, "SUJClutch", 0.01),
            (self.boardIDs[0], 9, "Tool", 1.5),
            (self.boardIDs[0], 10, "Adapter", 0.5),
            (self.boardIDs[0], 13, "ManipClutch", 0.01),
        ]

        for boardID, bitID, inputType, debounceTime in digitalInputBitIDs:
            yield DigitalInput(self.name, inputType, bitID, boardID, 0, debounceTime)

    def generateDigitalOutputs(self):
        digitalOutputBitIDs = [
            (self.boardIDs[0], 0, "SUJBrake"),
        ]

        for boardID, bitID, outputType in digitalOutputBitIDs:
            yield DigitalOutput(self.name, outputType, bitID, boardID)

    def generateDallasChip(self):
        return DallasChip(self.boardIDs[0], self.name)

    def toDict(self):
        dict = super().toDict()
        dict["Potentiometers"] = self.potentiometers
        return dict


class ClassicECM(Robot):
    def __init__(self, calData, robotTypeName, hardwareVersionName, serialNumber):
        self.driveDirection = lambda index: [1, 1, -1, 1][index]
        self.encoderCPT = lambda index: [4000, 4000, 640, 64][index]
        self.gearRatio = lambda index: [240, 240, 2748.55, 300.15][index]
        self.pitch = lambda index: [1, 1, 17.4533, 1][index]
        self.velocitySource = lambda index: 'FIRMWARE'
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

        super().__init__(robotTypeName, hardwareVersionName, serialNumber, calData, 4)

        potentiometerTolerances = list(self.generatePotentiometerTolerances())
        self.potentiometers = Potentiometers(potentiometerTolerances)

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
                    1,
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


class SiECM(Robot):
    def __init__(self, calData, robotTypeName, hardwareVersionName, serialNumber):
        self.driveDirection = lambda index: [-1, 1, 1, -1][index]
        self.encoderDirection = lambda index: [1, -1, -1, -1][index]
        self.encoderCPT = lambda index: [81920, 81920, 640, 64][index]
        self.gearRatio = lambda index: [83.3333, 168.3333, 2748.6, 300.2][index]
        self.pitch = lambda index: [1, 1, -17.4533, 1][index]
        self.velocitySource = lambda index: 'SOFTWARE'
        self.motorMaxCurrent = lambda index: [3.4, 3.4, 0.670, 0.590][index]
        self.motorTorque = lambda index: [0.0603, 0.0603, 0.0385, 0.0385][index]
        self.actuatorType = lambda index: "Revolute" if index != 2 else "Prismatic"
        self.potentiometerUnits = lambda index: "deg" if index != 2 else "mm"
        self.potentiometerLatency = lambda index: 0.01
        self.potentiometerDistance = lambda index: [1.0, 1.0, 2.0, 3.0][index]

        # # 2^13/10^3 or 2^11/10^3
        i_high = 65536 / 4800 / 2
        self.driveLinearAmpCurrent = lambda index: [i_high, i_high, 2.048, 2.048][index]

        self.brakeMaxCurrent = lambda index: [0.2, 0.2, 1.0][index]
        self.brakeReleaseCurrent = lambda index: [0.15, 0.15, 1.0][index]
        self.brakeReleaseTime = lambda index: 0.5
        self.brakeReleasedCurrent = lambda index: [0.08, 0.08, 0.25][index]
        self.brakeEngagedCurrent = lambda index: 0.0
        self.brakeLinearAmpCurrent = lambda index: 2.048 # 2^11/10^3

        super().__init__(robotTypeName, hardwareVersionName, serialNumber, calData, 4)

        potentiometerTolerances = list(self.generatePotentiometerTolerances())
        lookupTable = "sawRobotIO1394-{}-{}-PotentiometerLookupTable.json".format(robotTypeName, serialNumber)
        self.potentiometers = Potentiometers(potentiometerTolerances, lookupTable=lookupTable)

    def generatePotentiometerTolerances(self):
        for index in range(self.numberOfActuators):
            units = self.potentiometerUnits(index)
            latency = self.potentiometerLatency(index)
            distance = self.potentiometerDistance(index)
            yield PotentiometerTolerance(index, units, latency, distance)

    def generateBrakes(self):
        # Only actuators 0-2 have brakes,
        # Axis IDs are 0-3 for actuators, and 7-9 for brakes
        for index in range(self.numberOfActuators):
            if index <= 2:
                maxCurrent = self.brakeMaxCurrent(index)
                releaseCurrent = self.brakeReleaseCurrent(index)
                releaseTime = self.brakeReleaseTime(index)
                releasedCurrent = self.brakeReleasedCurrent(index)
                engagedCurrent = self.brakeEngagedCurrent(index)
                axisID = 7 + index # Brake 7 corresponds to actuator 0, etc.
                direction = 1 if index != 2 else -1 # some values in the third brake are reversed
                yield AnalogBrake(
                    axisID,
                    self.boardIDs[0],
                    direction,
                    maxCurrent,
                    releaseCurrent,
                    releaseTime,
                    releasedCurrent,
                    engagedCurrent,
                    linearAmpCurrent=self.brakeLinearAmpCurrent(index)
                )
            else:
                yield None

    def generateDrives(self):
        for index in range(self.numberOfActuators):
            yield Drive(
                self.driveDirection(index),
                self.gearRatio(index),
                self.motorTorque(index),
                self.motorMaxCurrent(index),
                linearAmpCurrent=self.driveLinearAmpCurrent(index),
            )

    def generateAnalogIns(self):
        for index in range(self.numberOfActuators):
            yield None

    def generateDigitalInputs(self):
        digitalInputBitIDs = [
            (self.boardIDs[0], 4, "SUJClutch", 0.01),
            (self.boardIDs[0], 13, "ManipClutch", 0.01),
            (self.boardIDs[0], 16, "SUJClutch2", 0.01)
        ]

        for boardID, bitID, inputType, debounceTime in digitalInputBitIDs:
            yield DigitalInput(self.name, inputType, bitID, boardID, 0, debounceTime)

    def generateDigitalOutputs(self):
        digitalOutputBitIDs = [
            (self.boardIDs[0], 0, "SUJBrake"),
        ]

        for boardID, bitID, outputType in digitalOutputBitIDs:
            yield DigitalOutput(self.name, outputType, bitID, boardID)

    def generateDallasChip(self):
        return None

    def toDict(self):
        dict = super().toDict()
        dict["Potentiometers"] = self.potentiometers
        return dict


class MTM(Robot):
    def __init__(self, calData, robotTypeName, hardwareVersionName, serialNumber):
        if robotTypeName.startswith("MTML"):
            driveDirections = [-1, 1, 1, 1, -1, 1, -1]
        elif robotTypeName.startswith("MTMR"):
            driveDirections = [-1, 1, 1, 1, 1, 1, -1]
        else:
            raise ValueError("Unsupported MTM type: {}".format(robotTypeName))

        self.driveDirection = lambda index: driveDirections[index]
        self.encoderCPT = lambda index: [4000, 4000, 4000, 4000, 64, 64, 64][index]
        self.gearRatio = lambda index: [63.41, 49.88, 59.73, 10.53, 33.16, 33.16, 16.58][index]
        self.pitch = lambda index: 1
        self.velocitySource = lambda index: 'FIRMWARE'
        self.motorMaxCurrent = lambda index: [0.67, 0.67, 0.67, 0.67, 0.59, 0.59, 0.407][index]
        self.motorTorque = lambda index: [0.0438, 0.0438, 0.0438, 0.0438, 0.00495, 0.00495, 0.00339][index]
        self.actuatorType = lambda index: "Revolute"
        self.potentiometerUnits = lambda index: "deg"
        self.potentiometerLatency = lambda index: 0.01 if index <= 5 else 0.0
        self.potentiometerDistance = lambda index: 5.0 if index <= 5 else 0.0

        super().__init__(robotTypeName, hardwareVersionName, serialNumber, calData, 7)

        potentiometerTolerances = list(self.generatePotentiometerTolerances())
        self.potentiometers = Potentiometers(potentiometerTolerances, couplingMatrix=MTMCoupling())

    def generatePotentiometerTolerances(self):
        for index in range(self.numberOfActuators):
            units = self.potentiometerUnits(index)
            latency = self.potentiometerLatency(index)
            distance = self.potentiometerDistance(index)
            yield PotentiometerTolerance(index, units, latency, distance)

    def toDict(self):
        dict = super().toDict()
        dict["Potentiometers"] = self.potentiometers
        return dict


class MTMGripper(Robot):
    def __init__(self, calData, robotTypeName, hardwareVersionName, serialNumber):
        self.ioType = "io-only"
        self.driveDirection = lambda index: -1
        self.encoderCPT = lambda index: 4000
        self.gearRatio = lambda index: 63.41
        self.pitch = lambda index: 1
        self.velocitySource = lambda index: 'SOFTWARE'
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
        super().__init__(robotTypeName, hardwareVersionName, serialNumber, calData, 1)

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
            boardID = self.boardIDs[index // self.actuatorsPerBoard]
            axisID = index % self.actuatorsPerBoard
            yield Actuator(
                0, type, boardID, axisID, drive, encoder, analogIn, brake
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
            "{:12.9f}".format(bitsToAmpsScale),
            "{:5.4f}".format(driveDirection * -linearAmpCurrent),
        )

        self.nmToAmps = Conversion(
            "{:7.10f}".format(1.0 / (gearRatio * motorTorque)), None
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
        direction,
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
        self.ampsToBits = Conversion(ampsToBitsScale, direction * 2 ** (DACresolution-1))
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
            self, potentiometerUnits, direction, CPT, gearRatio, pitch, velocitySource
    ):
        encoderPos = direction * (360 / CPT) * (pitch / gearRatio)
        encoderPos = "{:10.15f}".format(encoderPos)
        self.bitsToPosSI = Conversion(encoderPos, None, potentiometerUnits)
        self.velocitySource = velocitySource

    def toDict(self):
        return {
            "BitsToPosSI": self.bitsToPosSI,
            "VelocitySource": self.velocitySource,
        }


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

        voltsToPosSIScale = "{:10.8f}".format(voltsToPosSIScale)
        voltsToPosSIOffset = "{:10.8f}".format(voltsToPosSIOffset)

        self.voltsToPosSI = Conversion(
            voltsToPosSIScale, voltsToPosSIOffset, potentiometerUnits
        )

    def toDict(self):
        return {
            "BitsToVolts": self.bitsToVolts,
            "VoltsToPosSI": self.voltsToPosSI,
        }


class Potentiometers(Serializable):
    def __init__(self, tolerances, lookupTable=None, couplingMatrix=None):
        self.tolerances = tolerances
        self.lookupTable = lookupTable
        self.coupling = couplingMatrix

    def toDict(self):
        dict = {
            "Tolerances": self.tolerances,
        }

        if self.coupling is not None:
            dict["JointToActuatorPosition"] = self.coupling.jointToActuatorPosition

        if self.lookupTable is not None:
            dict["LookupTable"] = self.lookupTable

        return dict


class PotentiometerTolerance(Serializable):
    def __init__(self, axisID, units, latency, distance):
        self.axisID = axisID
        self.units = units
        self.latency = latency
        self.distance = distance

    def toDict(self):
        return {
            "Axis": self.axisID,
            "Distance": self.distance,
            "Latency": self.latency,
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


class MTMCoupling(Serializable):
    def __init__(self):
        self.jointToActuatorPosition = [
            [1.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00],
            [0.00, 1.00, 0.00, 0.00, 0.00, 0.00, 0.00],
            [0.00, 1.00, 1.00, 0.00, 0.00, 0.00, 0.00],
            [0.00, 0.00, 0.6697, 1.00, 0.00, 0.00, 0.00],
            [0.00, 0.00, 0.00, 0.00, 1.00, 0.00, 0.00],
            [0.00, 0.00, 0.00, 0.00, 0.00, 1.00, 0.00],
            [0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 1.00],
        ]

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


class DigitalOutput(Serializable):
    def __init__(self, robotTypeName: str, type: str, bitID: int, boardID: int):
        self.bitID = bitID
        self.boardID = boardID
        self.name = "{}-{}".format(robotTypeName, type)

    def toDict(self):
        return {
            "BitID": self.bitID,
            "BoardID": self.boardID,
            "Name": self.name,
        }


class DallasChip(Serializable):
    def __init__(self, boardID, robotTypeName):
        self.name = "{}-Dallas".format(robotTypeName)
        self.boardID = boardID

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
        type: str,
        boardID: int,
        axisID: int,
        drive: Drive,
        encoder: Encoder,
        analogIn: AnalogIn = None,
        brake: AnalogBrake = None,
    ):
        self.id = id
        self.boardID = boardID
        self.axisID = axisID
        self.type = type
        self.drive = drive
        self.encoder = encoder
        self.analogIn = analogIn
        self.brake = brake

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
    indent = "  "
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

    if os.path.exists(fileName):
        backup =  fileName + datetime.datetime.now().strftime("-backup-%Y-%m-%d_%H:%M:%S")
        os.rename(fileName, backup)
        print("Existing IO config file has been renamed {}".format(backup))

    if format == OutputFormat.JSON:
        with open(fileName, "w") as f:
            json.dump(config, f, indent=4, default=configSerializeJSON)
    elif format == OutputFormat.XML:
        root = toXML("Config", config)
        pretty_print_xml(root)

        tree = ET.ElementTree(root)
        tree.write(fileName, xml_declaration=True)

    print("Generated IO config file {}".format(fileName))


def generateConfig(calFileName, robotTypeName, hardwareVersionName, serialNumber, generationName, outputFormat):
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
    calData = parser.parseFile(calFileName) if generationName == "Classic" else None

    # sanity check robot type matches cal file
    if calData is not None:
        assert (
            robotTypeName[0:3] == calData["FileType"][0:3]
        ), "Robot hardware type doesn't match type from cal file"

    version = 5
    serialNumber = str(serialNumber or calData["serial_number"])
    config = Config(calData, version, robotTypeName, hardwareVersionName, serialNumber, generationName)

    outputFileName = "sawRobotIO1394-" + robotTypeName + "-" + serialNumber
    saveConfigFile(outputFileName, config, outputFormat)

    if robotTypeName[0:3] == "MTM":
        gripperConfigFileName = (
            "sawRobotIO1394-" + robotTypeName + "-gripper-" + serialNumber
        )
        gripperConfig = Config(calData, version, robotTypeName + "-Gripper", hardwareVersionName, serialNumber, generationName)
        saveConfigFile(gripperConfigFileName, gripperConfig, outputFormat)

    return serialNumber


def generateArmConfig(robotTypeName, hardwareVersionName, serialNumber, generationName):
    fileName = "{}-{}.json".format(robotTypeName, serialNumber)
    if os.path.exists(fileName):
        backup =  fileName + datetime.datetime.now().strftime("-backup-%Y-%m-%d_%H:%M:%S")
        os.rename(fileName, backup)
        print("Existing arm config file has been renamed {}".format(backup))
    kinematic = '    "kinematic": "kinematic/';
    if robotTypeName.startswith("PSM"):
        kinematic += "psm"
    elif robotTypeName == "MTML":
        kinematic += "mtml"
    elif robotTypeName == "MTMR":
        kinematic += "mtmr"
    elif robotTypeName == "ECM":
        kinematic += "ecm"
    else:
        raise ValueError("Unrecognized robot type: {}".format(robotTypeName))
    if generationName == "Si":
        kinematic += "-si"
    kinematic += '.json"\n'

    with open(fileName, "w") as f:
        f.write("{\n")
        f.write(kinematic)
        f.write('    , "generation": "' + generationName + '"\n')
        if robotTypeName.startswith("PSM"):
            f.write('    // , "tool-detection": "MANUAL"\n')
            f.write('    , "tool-detection": "AUTOMATIC"\n')
        f.write("}\n")
    print("Generated arm config file {}".format(fileName))


def generateConsoleConfig(robotTypeName, hardwareVersionName, serialNumber, generationName):
    fileName = "console-{}.json".format(robotTypeName)
    if os.path.exists(fileName):
        backup =  fileName + datetime.datetime.now().strftime("-backup-%Y-%m-%d_%H:%M:%S")
        os.rename(fileName, backup)
        print("Existing console file has been renamed {}".format(backup))
    type = ""
    if robotTypeName.startswith("PSM"):
        type = "PSM"
    elif robotTypeName.startswith("MTM"):
        type = "MTM"
    elif robotTypeName == "ECM":
        type = robotTypeName
    else:
        raise ValueError("Unrecognized robot type: {}".format(robotTypeName))

    with open(fileName, "w") as f:
        f.write('{\n')
        f.write('    "arms":\n')
        f.write('    [\n')
        f.write('        {\n')
        f.write('            "name": "' + robotTypeName + '",\n')
        f.write('            "type": "' + type + '",\n')
        f.write('            "serial": "' + str(serialNumber) + '"\n')
        f.write('        }\n')
        f.write('    ]\n')
        f.write('}\n')
    print("Generated console file {}".format(fileName))


def main():
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument(
        "-a",
        "--arm",
        type = str,
        required = True,
        choices = ["MTML", "MTMR", "PSM1", "PSM2", "PSM3", "ECM"],
        help = "robot arm hardware type",
        default = argparse.SUPPRESS,
    )
    parser.add_argument(
        "-g",
        "--generation",
        type = str,
        required = True,
        choices = ["Classic", "Si"],
        help = "robot arm hardware generation",
        default = argparse.SUPPRESS,
    )
    parser.add_argument(
        "-H",
        "--hardware-version",
        type = str,
        required = True,
        choices = ["QLA1", "DQLA", "dRA1"],
        help = "hardware version.  Note that QLA1 and DQLA are the only two options for a Classic arm",
        default = argparse.SUPPRESS,
    )
    parser.add_argument(
        "-c",
        "--cal",
        type = str,
        help = "calibration file",
        default = None
    )
    parser.add_argument(
        "-s",
        "--serial",
        type = int,
        help = "serial number (for Si arms)",
        default = None
    )
    parser.add_argument(
        "-f",
        "--format",
        type = str,
        default = "XML",
        choices = ["XML", "JSON"],
        help = "format used for the calibration file (future use)",
    )

    args = parser.parse_args()
    if args.generation == "Classic" and args.cal is None:
        print("ERROR: must specify calibration file ('--cal') for classic generation arms")
        return

    if args.generation == "Classic" and args.hardware_version == "dRA1":
        print("ERROR: generation 'Classic' is not supported with hardware version 'dRA1'")
        return

    if args.generation == "Si" and args.serial is None:
        print("ERROR: must specify serial number ('--serial') for Si generation arms")
        return

    if args.generation == "Si" and args.hardware_version != "dRA1":
        print("ERROR: generation `Si` requires hardware version `dRA1`")
        return

    if args.arm[0:3] == "MTM" and args.generation == "Si":
        print("ERROR: Si MTMs are not supported (yet)")
        return

    outputFormat = OutputFormat.XML if args.format == "XML" else OutputFormat.JSON
    # sawRobotIO config file
    actualSerial = generateConfig(args.cal, args.arm, args.hardware_version, args.serial, args.generation, outputFormat)
    # sawIntuitiveResearchKit arm and console config files
    generateArmConfig(args.arm, args.hardware_version, actualSerial, args.generation)
    generateConsoleConfig(args.arm, args.hardware_version, actualSerial, args.generation)

if __name__ == "__main__":
    main()
