#!/usr/bin/env python3

# Author(s):  Brendan Burkhart, Anton Deguet
#
# (C) Copyright 2023-2025 Johns Hopkins University (JHU), All Rights Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

import argparse
import os
import datetime
import json
import math
from enum import Enum

import numpy as np

import calParser

# conversions to SI
deg = math.pi / 180.0
mm = 0.001

class RobotType(Enum):
    MTM = 0
    PSM = 1
    ECM = 2
    MTMGripper = 3

    def fromTypeName(robotTypeName):
        if robotTypeName[0:3] == "MTM":
            if robotTypeName.endswith("_gripper"):
                return RobotType.MTMGripper
            return RobotType.MTM
        elif robotTypeName[0:3] == "PSM":
            return RobotType.PSM
        elif robotTypeName[0:3] == "ECM":
            return RobotType.ECM
        else:
            return None


# Empty base class for all class we want to serialize to JSON/XML
class Serializable:
    def toDict(self):
        raise NotImplementedError("Please implement toDict()")


# Represents the various conversions e.g. EffortToCurrent, AmpsToBits, etc.
class Conversion(Serializable):
    def __init__(self, scale, offset):
        self.scale = scale
        self.offset = offset

    def toDict(self):
        dict = {}
        if self.offset != None:
            dict["offset"] = float(self.offset)
        if self.scale != None:
            dict["scale"] = float(self.scale)
        return dict

# Represents limits
class Limits(Serializable):
    def __init__(self, lower, upper):
        self.lower = lower
        self.upper = upper

    def toDict(self):
        dict = {}
        if self.lower != None:
            dict["lower"] = self.lower
        if self.upper != None:
            dict["upper"] = self.upper
        return dict


# dVRK board Id conventions
def getBoardIds(robotTypeName):
    # Need to allow for MTML_gripper/MTMR_gripper
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


def getActuatorsPerBoard(hardwareVersion):
    if hardwareVersion == "QLA1":
        return 4
    elif hardwareVersion == "DQLA":
        return 8
    elif hardwareVersion == "dRA1":
        return 10
    else:
        raise ValueError("Unrecognized controller type: {}".format(hardwareVersion))


# Root config object
class Config(Serializable):
    def __init__(self, calData, versionId, robotTypeName, hardwareVersion, serialNumber, robotGenerationName):
        self.versionId = versionId

        robotType = RobotType.fromTypeName(robotTypeName)
        isClassic = robotGenerationName == "Classic"
        if robotType == RobotType.PSM:
            if isClassic:
                self.robot = ClassicPSM(calData, robotTypeName, hardwareVersion, serialNumber)
            else:
                self.robot = SiPSM(calData, robotTypeName, hardwareVersion, serialNumber)
        elif robotType == RobotType.ECM:
            if isClassic:
                self.robot = ClassicECM(calData, robotTypeName, hardwareVersion, serialNumber)
            else:
                self.robot = SiECM(calData, robotTypeName, hardwareVersion, serialNumber)
        elif robotType == RobotType.MTM:
            self.robot = MTM(calData, robotTypeName, hardwareVersion, serialNumber)
        else:
            self.robot = MTMGripper(calData, robotTypeName, hardwareVersion, serialNumber)

        self.digitalInputs = list(self.robot.generateDigitalInputs())
        self.digitalOutputs = list(self.robot.generateDigitalOutputs())
        self.dallasChip = self.robot.generateDallasChip()

    def toDict(self):
        dict = {
            "$id:" : "saw-robot-io.schema.json",
            "$version": self.versionId,
            "robots": [ self.robot ],
        }

        if self.dallasChip != None:
            dict["dallas_chips"] = self.dallasChip,

        if len(self.digitalInputs) > 0:
            dict["digital_inputs"] = self.digitalInputs

        if len(self.digitalOutputs) > 0:
            dict["digital_outputs"] = self.digitalOutputs

        return dict


class Robot(Serializable):
    def __init__(self, robotTypeName, hardwareVersion, serialNumber, calData, numberOfActuators, numberOfBrakes):
        self.name = robotTypeName
        self.type = RobotType.fromTypeName(robotTypeName)
        self.serialNumber = serialNumber
        self.hardwareVersion = hardwareVersion
        self.calData = calData
        self.boardIds = getBoardIds(robotTypeName)
        self.numberOfActuators = numberOfActuators
        self.numberOfBrakes = numberOfBrakes
        self.actuatorsPerBoard = getActuatorsPerBoard(hardwareVersion)
        self.actuators = list(self.generateActuators())
        self.brakes = list(self.generateBrakes())

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
            direction = self.encoderDirection(index)
            encoderCPT = self.encoderCPT(index)
            gearRatio = self.gearRatio(index)
            pitch = self.pitch(index)
            velocitySource = self.velocitySource(index)
            positionLimitsSoftLower = self.positionLimitsSoftLower(index)
            positionLimitsSoftUpper = self.positionLimitsSoftUpper(index)
            yield Encoder(direction, encoderCPT, gearRatio, pitch, velocitySource,
                          positionLimitsSoftLower, positionLimitsSoftUpper)

    def generatePotentiometers(self):
        for index in range(self.numberOfActuators):
            pitch = self.pitch(index)
            yield Potentiometer(self.calData, index, pitch)

    def generateBrakes(self):
        for index in range(self.numberOfBrakes):
            yield None

    def generateActuators(self):
        drives = self.generateDrives()
        encoders = self.generateEncoders()
        potentiometers = self.generatePotentiometers()
        data = zip(range(self.numberOfActuators), drives, encoders, potentiometers)
        for index, drive, encoder, potentiometer in data:
            type = self.actuatorType(index)
            boardId = self.boardIds[index // self.actuatorsPerBoard]
            axisId = index % self.actuatorsPerBoard

            yield Actuator(
                index, type, boardId, axisId, drive, encoder, potentiometer
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
            "name": self.name,
            "hardware_version": self.hardwareVersion,
            "number_of_actuators": self.numberOfActuators,
            "number_of_brakes": self.numberOfBrakes,
            "serial_number": self.serialNumber,
            "actuators": self.actuators,
            "brakes": self.brakes,
        }

        return dict


class ClassicPSM(Robot):
    def __init__(self, calData, robotTypeName, hardwareVersion, serialNumber):
        self.driveDirection = lambda index: [-1, -1, 1, -1, -1, 1, 1][index]
        self.encoderCPT = lambda index: [14400, 14400, 14400, 4000, 4000, 4000, 4000][index]
        self.gearRatio = lambda index: [56.50, 56.50, 336.6, 11.71, 11.71, 11.71, 11.71][index]
        self.pitch = lambda index: 1
        self.velocitySource = lambda index: 'FIRMWARE'
        self.positionLimitsSoftLower = lambda index: [-91.0 * deg, -53.0 * deg,   0.0 * mm, -174.0 * deg, -174.0 * deg, -174.0 * deg, -174.0 * deg][index]
        self.positionLimitsSoftUpper = lambda index: [ 91.0 * deg,  53.0 * deg, 240.0 * mm,  174.0 * deg,  174.0 * deg,  174.0 * deg,  174.0 * deg][index]
        self.motorMaxCurrent = lambda index: [1.34, 1.34, 0.67, 0.67, 0.67, 0.67, 0.670][index]
        self.motorTorque = lambda index: [0.0438, 0.0438, 0.0438, 0.0438, 0.0438, 0.0438, 0.0438][index]
        self.actuatorType = lambda index: "REVOLUTE" if index != 2 else "PRISMATIC"
        self.potentiometerLatency = lambda index: 0.01
        self.potentiometerDistance = lambda index: [5.0 * deg, 5.0 * deg, 5.0 * mm, 5.0 * deg, 5.0 * deg, 5.0 * deg, 5.0 * deg][index]

        super().__init__(robotTypeName, hardwareVersion, serialNumber, calData, 7, 0)

        potentiometerTolerances = list(self.generatePotentiometerTolerances())
        self.potentiometers = Potentiometers('ANALOG', potentiometerTolerances)

    def generatePotentiometerTolerances(self):
        for index in range(self.numberOfActuators):
            latency = self.potentiometerLatency(index)
            distance = self.potentiometerDistance(index)
            yield PotentiometerTolerance(index, latency, distance)

    def generateDigitalInputs(self):
        if self.hardwareVersion == "QLA1":
            digitalInputBitIds = [
                (self.boardIds[0], 0, "SUJ_clutch", 0.2),
                (self.boardIds[0], 2, "arm_clutch", 0.2),
                (self.boardIds[1], 7, "tool", 0.2),
                (self.boardIds[1], 10, "adapter", 0.2),
            ]
        else: # DQLA, add 16 for second board
            digitalInputBitIds = [
                (self.boardIds[0], 0, "SUJ_clutch", 0.2),
                (self.boardIds[0], 2, "arm_clutch", 0.2),
                (self.boardIds[0], 23, "tool", 0.2),
                (self.boardIds[0], 26, "adapter", 0.2),
            ]

        for boardId, bitId, inputType, debounceTime in digitalInputBitIds:
            yield DigitalInput(self.name, inputType, bitId, boardId, True, debounceTime)

    def generateDallasChip(self):
        if self.hardwareVersion == "QLA1":
            return DallasChip(self.boardIds[1], self.name)
        else:
            return DallasChip(self.boardIds[0], self.name)

    def toDict(self):
        dict = super().toDict()
        dict["potentiometers"] = self.potentiometers
        return dict


class SiPSM(Robot):
    def __init__(self, calData, robotTypeName, hardwareVersion, serialNumber):
        self.driveDirection = lambda index: [-1, 1, 1, 1, 1, -1, -1][index]
        self.encoderDirection = lambda index: -self.driveDirection(index)
        self.encoderCPT = lambda index: [81920, 81920, 50000, 4000, 4000, 4000, 4000][index]
        self.gearRatio = lambda index: [83.3333, 85.000, 965.91, 13.813, 13.813, 13.813, 13.813][index]
        self.pitch = lambda index: 1
        self.velocitySource = lambda index: 'SOFTWARE' if index < 3 else 'FIRMWARE'
        self.positionLimitsSoftLower = lambda index: [-170.0 * deg, -73.0 * deg,   0.0 * mm, -172.0 * deg, -172.0 * deg, -172.0 * deg, -172.0 * deg][index]
        self.positionLimitsSoftUpper = lambda index: [ 170.0 * deg,  75.0 * deg, 290.0 * mm,  172.0 * deg,  172.0 * deg,  172.0 * deg,  172.0 * deg][index]
        self.motorMaxCurrent = lambda index: [3.4, 3.4, 1.1, 1.1, 1.1, 1.1, 1.1][index]
        self.motorTorque = lambda index: [0.0603, 0.0603, 0.0385, 0.0385, 0.0385, 0.0385, 0.0385][index]
        self.actuatorType = lambda index: "REVOLUTE" if index != 2 else "PRISMATIC"
        self.potentiometerLatency = lambda index: 0.01
        self.potentiometerDistance =  lambda index: [1.0 * deg, 1.0 * deg, 2.0 * mm, 3.0 * deg, 3.0 * deg, 3.0 * deg, 3.0 * deg][index]

        # # 2^13/10^3 or 2^11/10^3
        i_high = 65536 / 4800 / 2
        self.driveLinearAmpCurrent = lambda index: [i_high, i_high, 2.048, 2.048, 2.048, 2.048, 2.048][index]

        self.brakeMaxCurrent = lambda index: [0.2, 0.2, 0.3][index]
        self.brakeReleaseCurrent = lambda index: [0.15, 0.15, -0.25][index]
        self.brakeReleaseTime = lambda index: 0.5
        self.brakeReleasedCurrent = lambda index: [0.08, 0.08, -0.25][index]
        self.brakeEngagedCurrent = lambda index: 0.0
        self.brakeLinearAmpCurrent = lambda index: 2.048 # 2^11/10^3

        super().__init__(robotTypeName, hardwareVersion, serialNumber, calData, 7, 3)

        potentiometerTolerances = list(self.generatePotentiometerTolerances())
        lookupTable = "sawRobotIO1394-{}-{}-PotentiometerLookupTable.json".format(robotTypeName, serialNumber)
        self.potentiometers = Potentiometers('DIGITAL', potentiometerTolerances, lookupTable=lookupTable)

    def generatePotentiometerTolerances(self):
        for index in range(self.numberOfActuators):
            latency = self.potentiometerLatency(index)
            distance = self.potentiometerDistance(index)
            yield PotentiometerTolerance(index, latency, distance)

    def generateBrakes(self):
        # Axis Ids are 0-6 for actuators, and 7-9 for brakes
        for index in range(self.numberOfBrakes):
            maxCurrent = self.brakeMaxCurrent(index)
            releaseCurrent = self.brakeReleaseCurrent(index)
            releaseTime = self.brakeReleaseTime(index)
            releasedCurrent = self.brakeReleasedCurrent(index)
            engagedCurrent = self.brakeEngagedCurrent(index)
            axisId = 7 + index # Brake 7 corresponds to actuator 0, etc.
            direction = 1 if index != 2 else -1 # some values in the third brake are reversed
            yield Brake(
                axisId,
                self.boardIds[0],
                direction,
                maxCurrent,
                releaseCurrent,
                releaseTime,
                releasedCurrent,
                engagedCurrent,
                linearAmpCurrent=self.brakeLinearAmpCurrent(index)
            )

    def generateDrives(self):
        for index in range(self.numberOfActuators):
            yield Drive(
                self.driveDirection(index),
                self.gearRatio(index),
                self.motorTorque(index),
                self.motorMaxCurrent(index),
                linearAmpCurrent=self.driveLinearAmpCurrent(index),
            )

    def generatePotentiometers(self):
        for index in range(self.numberOfActuators):
            yield None

    def generateDigitalInputs(self):
        digitalInputBitIds = [
            (self.boardIds[0], 1, "SUJ_clutch_2", 0.01),
            (self.boardIds[0], 4, "SUJ_clutch", 0.01),
            (self.boardIds[0], 9, "tool", 1.5),
            (self.boardIds[0], 10, "adapter", 1.6), # At least as high as tool so adapter is not detected before tool
            (self.boardIds[0], 13, "arm_clutch", 0.01),
        ]

        for boardId, bitId, inputType, debounceTime in digitalInputBitIds:
            yield DigitalInput(self.name, inputType, bitId, boardId, False, debounceTime)

    def generateDigitalOutputs(self):
        digitalOutputBitIds = [
            (self.boardIds[0], 0, "SUJBrake"),
        ]

        for boardId, bitId, outputType in digitalOutputBitIds:
            yield DigitalOutput(self.name, outputType, bitId, boardId)

    def generateDallasChip(self):
        return DallasChip(self.boardIds[0], self.name)

    def toDict(self):
        dict = super().toDict()
        dict["potentiometers"] = self.potentiometers
        return dict


class ClassicECM(Robot):
    def __init__(self, calData, robotTypeName, hardwareVersion, serialNumber):
        self.driveDirection = lambda index: [1, 1, -1, 1][index]
        self.encoderCPT = lambda index: [4000, 4000, 640, 64][index]
        self.gearRatio = lambda index: [240, 240, 2748.55, 300.15][index]
        self.pitch = lambda index: 1
        self.velocitySource = lambda index: 'FIRMWARE'
        self.positionLimitsSoftLower = lambda index: [-90.0 * deg, -45.0 * deg,   0.0 * mm, -89.0 * deg][index]
        self.positionLimitsSoftUpper = lambda index: [ 90.0 * deg,  64.0 * deg, 255.0 * mm,  89.0 * deg][index]
        self.motorMaxCurrent = lambda index: [0.943, 0.943, 0.67, 0.59][index]
        self.motorTorque = lambda index: [0.1190, 0.1190, 0.0438, 0.00495][index]
        self.actuatorType = lambda index: "REVOLUTE" if index != 2 else "PRISMATIC"
        self.potentiometerLatency = lambda index: 0.01
        self.potentiometerDistance = lambda index: [5.0 * deg, 5.0 * deg, 5.0 * mm, 5.0 * deg][index]

        self.brakeMaxCurrent = lambda index: [0.25, 0.22, 0.90][index]
        self.brakeReleaseCurrent = lambda index: [0.25, 0.22, 0.90][index]
        self.brakeReleaseTime = lambda index: 2.0
        self.brakeReleasedCurrent = lambda index: [0.08, 0.07, 0.20][index]
        self.brakeEngagedCurrent = lambda index: 0.0

        super().__init__(robotTypeName, hardwareVersion, serialNumber, calData, 4, 3)

        potentiometerTolerances = list(self.generatePotentiometerTolerances())
        self.potentiometers = Potentiometers('ANALOG', potentiometerTolerances)

    def generatePotentiometerTolerances(self):
        for index in range(self.numberOfActuators):
            latency = self.potentiometerLatency(index)
            distance = self.potentiometerDistance(index)
            yield PotentiometerTolerance(index, latency, distance)

    def generateBrakes(self):
        for index in range(self.numberOfBrakes):
            maxCurrent = self.brakeMaxCurrent(index)
            releaseCurrent = self.brakeReleaseCurrent(index)
            releaseTime = self.brakeReleaseTime(index)
            releasedCurrent = self.brakeReleasedCurrent(index)
            engagedCurrent = self.brakeEngagedCurrent(index)
            yield Brake(
                (index + 4) % self.actuatorsPerBoard,
                self.boardIds[(index + 4) // self.actuatorsPerBoard],
                1,
                maxCurrent,
                releaseCurrent,
                releaseTime,
                releasedCurrent,
                engagedCurrent
            )

    def generateDigitalInputs(self):
        digitalInputBitIds = [
            (self.boardIds[0], 0, "arm_clutch", 0.2),
            (self.boardIds[0], 2, "SUJ_clutch", 0.2),
        ]

        for boardId, bitId, inputType, debounceTime in digitalInputBitIds:
            yield DigitalInput(self.name, inputType, bitId, boardId, True, debounceTime)

    def toDict(self):
        dict = super().toDict()
        dict["potentiometers"] = self.potentiometers
        return dict


class SiECM(Robot):
    def __init__(self, calData, robotTypeName, hardwareVersion, serialNumber):
        self.driveDirection = lambda index: [-1, 1, 1, -1][index]
        self.encoderDirection = lambda index: [1, -1, -1, -1][index]
        self.encoderCPT = lambda index: [81920, 81920, 640, 64][index]
        self.gearRatio = lambda index: [83.3333, 168.3333, 2748.6, 300.2][index]
        self.pitch = lambda index: [1, 1, -1, 1][index]
        self.velocitySource = lambda index: 'SOFTWARE'
        self.positionLimitsSoftLower = lambda index: [-170.0 * deg, -64.0 * deg,   0.0 * mm, -89.0 * deg][index]
        self.positionLimitsSoftUpper = lambda index: [ 170.0 * deg,  60.0 * deg, 260.0 * mm,  89.0 * deg][index]
        self.motorMaxCurrent = lambda index: [3.4, 3.4, 0.670, 0.590][index]
        self.motorTorque = lambda index: [0.0603, 0.0603, 0.0385, 0.0385][index]
        self.actuatorType = lambda index: "REVOLUTE" if index != 2 else "PRISMATIC"
        self.potentiometerLatency = lambda index: 0.01
        self.potentiometerDistance = lambda index: [1.0 * deg, 1.0 * deg, 2.0 * mm, 3.0 * deg][index]

        # # 2^13/10^3 or 2^11/10^3
        i_high = 65536 / 4800 / 2
        self.driveLinearAmpCurrent = lambda index: [i_high, i_high, 2.048, 2.048][index]

        self.brakeMaxCurrent = lambda index: [0.2, 0.2, 1.0][index]
        self.brakeReleaseCurrent = lambda index: [0.15, 0.15, 1.0][index]
        self.brakeReleaseTime = lambda index: 0.5
        self.brakeReleasedCurrent = lambda index: [0.08, 0.08, 0.25][index]
        self.brakeEngagedCurrent = lambda index: 0.0
        self.brakeLinearAmpCurrent = lambda index: 2.048 # 2^11/10^3

        super().__init__(robotTypeName, hardwareVersion, serialNumber, calData, 4, 3)

        potentiometerTolerances = list(self.generatePotentiometerTolerances())
        lookupTable = "sawRobotIO1394-{}-{}-PotentiometerLookupTable.json".format(robotTypeName, serialNumber)
        self.potentiometers = Potentiometers('DIGITAL', potentiometerTolerances, lookupTable=lookupTable)

    def generatePotentiometerTolerances(self):
        for index in range(self.numberOfActuators):
            latency = self.potentiometerLatency(index)
            distance = self.potentiometerDistance(index)
            yield PotentiometerTolerance(index, latency, distance)

    def generateBrakes(self):
        # Axis Ids are 0-3 for actuators, and 7-9 for brakes
        for index in range(self.numberOfBrakes):
            maxCurrent = self.brakeMaxCurrent(index)
            releaseCurrent = self.brakeReleaseCurrent(index)
            releaseTime = self.brakeReleaseTime(index)
            releasedCurrent = self.brakeReleasedCurrent(index)
            engagedCurrent = self.brakeEngagedCurrent(index)
            axisId = 7 + index # Brake 7 corresponds to actuator 0, etc.
            direction = 1 if index != 2 else -1 # some values in the third brake are reversed
            yield Brake(
                axisId,
                self.boardIds[0],
                direction,
                maxCurrent,
                releaseCurrent,
                releaseTime,
                releasedCurrent,
                engagedCurrent,
                linearAmpCurrent=self.brakeLinearAmpCurrent(index)
            )

    def generateDrives(self):
        for index in range(self.numberOfActuators):
            yield Drive(
                self.driveDirection(index),
                self.gearRatio(index),
                self.motorTorque(index),
                self.motorMaxCurrent(index),
                linearAmpCurrent=self.driveLinearAmpCurrent(index),
            )

    def generatePotentiometers(self):
        for index in range(self.numberOfActuators):
            yield None

    def generateDigitalInputs(self):
        digitalInputBitIds = [
            (self.boardIds[0], 4, "SUJ_clutch", 0.01),
            (self.boardIds[0], 13, "arm_clutch", 0.01),
            (self.boardIds[0], 16, "SUJ_clutch_2", 0.01)
        ]

        for boardId, bitId, inputType, debounceTime in digitalInputBitIds:
            yield DigitalInput(self.name, inputType, bitId, boardId, False, debounceTime)

    def generateDigitalOutputs(self):
        digitalOutputBitIds = [
            (self.boardIds[0], 0, "SUJBrake"),
        ]

        for boardId, bitId, outputType in digitalOutputBitIds:
            yield DigitalOutput(self.name, outputType, bitId, boardId)

    def generateDallasChip(self):
        return None

    def toDict(self):
        dict = super().toDict()
        dict["potentiometers"] = self.potentiometers
        return dict


class MTM(Robot):
    def __init__(self, calData, robotTypeName, hardwareVersion, serialNumber):
        if robotTypeName.startswith("MTML"):
            driveDirections = [-1, 1, 1, 1, -1, 1, -1]
            positionLimitsSoftLower = [-40.0 * deg, -18.0 * deg, -15.0 * deg, -240.0 * deg, -95.0 * deg, -45.0 * deg, -475.0 * deg]
            positionLimitsSoftUpper = [ 65.0 * deg,  65.0 * deg,  42.0 * deg,  120.0 * deg, 185.0 * deg,  45.0 * deg,  445.0 * deg]
        elif robotTypeName.startswith("MTMR"):
            driveDirections = [-1, 1, 1, 1, 1, 1, -1]
            positionLimitsSoftLower = [-65.0 * deg, -18.0 * deg, -15.0 * deg, -120.0 * deg, -95.0 * deg, -45.0 * deg, -475.0 * deg]
            positionLimitsSoftUpper = [ 40.0 * deg,  65.0 * deg,  42.0 * deg,  240.0 * deg, 185.0 * deg,  45.0 * deg,  445.0 * deg]
        else:
            raise ValueError("Unsupported MTM type: {}".format(robotTypeName))

        self.driveDirection = lambda index: driveDirections[index]
        self.encoderCPT = lambda index: [4000, 4000, 4000, 4000, 64, 64, 64][index]
        self.gearRatio = lambda index: [63.41, 49.88, 59.73, 10.53, 33.16, 33.16, 16.58][index]
        self.pitch = lambda index: 1
        self.velocitySource = lambda index: 'FIRMWARE'
        self.positionLimitsSoftLower = lambda index: positionLimitsSoftLower[index]
        self.positionLimitsSoftUpper = lambda index: positionLimitsSoftUpper[index]

        self.motorMaxCurrent = lambda index: [0.67, 0.67, 0.67, 0.67, 0.59, 0.59, 0.407][index]
        self.motorTorque = lambda index: [0.0438, 0.0438, 0.0438, 0.0438, 0.00495, 0.00495, 0.00339][index]
        self.actuatorType = lambda index: "REVOLUTE"
        self.potentiometerLatency = lambda index: 0.01 if index <= 5 else 0.0 # in seconds
        self.potentiometerDistance = lambda index: (5.0 * deg) if index <= 5 else 0.0

        super().__init__(robotTypeName, hardwareVersion, serialNumber, calData, 7, 3)

        potentiometerTolerances = list(self.generatePotentiometerTolerances())
        self.potentiometers = Potentiometers('ANALOG', potentiometerTolerances, coupling = MTMCoupling())

    def generatePotentiometerTolerances(self):
        for index in range(self.numberOfActuators):
            latency = self.potentiometerLatency(index)
            distance = self.potentiometerDistance(index)
            yield PotentiometerTolerance(index, latency, distance)

    def toDict(self):
        dict = super().toDict()
        dict["potentiometers"] = self.potentiometers
        return dict


class MTMGripper(Robot):
    def __init__(self, calData, robotTypeName, hardwareVersion, serialNumber):
        self.ioType = "io-only"
        self.driveDirection = lambda index: -1
        self.encoderCPT = lambda index: 4000
        self.gearRatio = lambda index: 63.41
        self.pitch = lambda index: 1
        self.velocitySource = lambda index: 'SOFTWARE'
        self.positionLimitsSoftLower = lambda index: 0.0
        self.positionLimitsSoftUpper = lambda index: 0.0
        self.motorMaxCurrent = lambda index: 0.0
        self.motorTorque = lambda index: 0.0438
        self.actuatorType = lambda index: "REVOLUTE"
        self.potentiometerLatency = lambda index: None
        self.potentiometerDistance = lambda index: None

        # Want driveDirection * (360 / CPT) * (pitch / gearRatio) = 360
        desiredEncoderScale = 360.0
        self.encoderCPT = (
            lambda index: self.driveDirection(0)
            * (360 / desiredEncoderScale)
            * (self.pitch(0) / self.gearRatio(0))
        )

        self.voltsToPosScale = -100.0
        self.voltsToPosOffset = 300.0
        super().__init__(robotTypeName, hardwareVersion, serialNumber, calData, 1, 0)

    def generatePotentiometers(self):
        for index in range(self.numberOfActuators):
            pitch = self.pitch(index)
            yield Potentiometer(
                self.calData,
                index,
                pitch,
                self.voltsToPosScale,
                self.voltsToPosOffset,
            )

    def generateActuators(self):
        drives = self.generateDrives()
        encoders = self.generateEncoders()
        potentiometers = self.generatePotentiometers()
        data = zip(drives, encoders, potentiometers)
        index = 7  # MTM gripper is treated as if it was 8th actuator of MTM

        for drive, encoder, potentiometer in data:
            type = self.actuatorType(index)
            boardId = self.boardIds[index // self.actuatorsPerBoard]
            axisId = index % self.actuatorsPerBoard
            yield Actuator(
                0, type, boardId, axisId, drive, encoder, potentiometer
            )

    def toDict(self):
        dict = super().toDict()
        dict["type"] = self.ioType

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
        self.maxCurrent = motorMaxCurrent

    def toDict(self):
        return {
            "current_to_bits": self.ampsToBits,
            "bits_to_current": self.bitsToFeedbackAmps,
            "effort_to_current": self.nmToAmps,
            "maximum_current": self.maxCurrent,
        }


class Brake(Serializable):
    def __init__(
        self,
        axisId,
        boardId,
        direction,
        maxCurrent,
        releaseCurrent,
        releaseTime,
        releasedCurrent,
        engagedCurrent,
        DACresolution=16,
        linearAmpCurrent=6.25,
    ):
        self.axisId = axisId
        self.boardId = boardId

        ampsToBitsScale = (2**(DACresolution-1))/linearAmpCurrent
        bitsToAmpsScale = 1.0 / ampsToBitsScale
        self.ampsToBits = Conversion(ampsToBitsScale, direction * 2 ** (DACresolution-1))
        self.bitsToFeedbackAmps = Conversion(bitsToAmpsScale, -linearAmpCurrent)

        self.maxCurrent = maxCurrent
        self.releaseCurrent = releaseCurrent
        self.releaseTime = releaseTime
        self.releasedCurrent = releasedCurrent
        self.engagedCurrent = engagedCurrent

    def toDict(self):
        return {
            "axis_id": self.axisId,
            "board_id": self.boardId,
            "drive" : {
                "current_to_bits": self.ampsToBits,
                "bits_to_current": self.bitsToFeedbackAmps,
                "maximum_current": self.maxCurrent,
            },
            "release_current": self.releaseCurrent,
            "release_time": self.releaseTime,
            "released_current": self.releasedCurrent,
            "engaged_current": self.engagedCurrent,
        }


class Encoder(Serializable):
    def __init__(
            self, direction, CPT, gearRatio, pitch, velocitySource,
            positionLimitsSoftLower, positionLimitsSoftUpper
    ):
        encoderPos = direction * (2.0 * math.pi / CPT) * (pitch / gearRatio)
        encoderPos = "{:10.15f}".format(encoderPos)
        self.bitsToPos = Conversion(encoderPos, None)
        self.velocitySource = velocitySource
        self.positionLimitsSoft = Limits(positionLimitsSoftLower,
                                         positionLimitsSoftUpper)

    def toDict(self):
        return {
            "bits_to_position": self.bitsToPos,
            "velocity_source": self.velocitySource,
            "position_limits_soft": self.positionLimitsSoft,
        }


# === Potentiometer =====
#  Intuitive system
#    1. 12 bit ADC
#    2. 0-4.096 V (Typical)
#    3. Unit: Radian
#
#  JHU QLA board
#    1. 16 bit ADC
#    2. 0-4.5 V
#    3. Unit: Radian
class Potentiometer(Serializable):
    def __init__(
        self,
        calData,
        actuatorIndex,
        pitch,
        voltsToPosScale = None,
        voltsToPosOffset = None,
    ):
        # 16 bits ADC with 4.5 V ref
        self.bitsToVolts = Conversion(4.5 / (2 ** 16), "0")

        potGain = calData["motor"]["pot_input_gain"][actuatorIndex]
        potOffset = calData["motor"]["pot_input_offset"][actuatorIndex]

        if voltsToPosScale is None:
            voltsToPosScale = potGain * (2 ** 12 / 4.5) * pitch

        if voltsToPosOffset is None:
            voltsToPosOffset = potOffset * pitch

        voltsToPosScale = "{:10.8f}".format(voltsToPosScale)
        voltsToPosOffset = "{:10.8f}".format(voltsToPosOffset)

        self.voltsToPos = Conversion(
            voltsToPosScale, voltsToPosOffset
        )

    def toDict(self):
        return {
            "bits_to_voltage": self.bitsToVolts,
            "voltage_to_position": self.voltsToPos,
        }


class Potentiometers(Serializable):
    def __init__(self, pot_type, tolerances, lookupTable = None, coupling = None):
        self.pot_type = pot_type
        self.tolerances = tolerances
        self.lookupTable = lookupTable
        self.coupling = coupling

    def toDict(self):
        dict = {
            "potentiometers_type": self.pot_type,
            "tolerances": self.tolerances,
        }

        if self.coupling is not None:
            dict["coupling"] = self.coupling

        if self.lookupTable is not None:
            dict["lookup_table_file"] = self.lookupTable

        return dict


class PotentiometerTolerance(Serializable):
    def __init__(self, axisId, latency, distance):
        self.axisId = axisId
        self.latency = latency
        self.distance = distance

    def toDict(self):
        return {
            "axis_id": self.axisId,
            "distance": self.distance,
            "latency": self.latency,
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

    def toDict(self):
        return {
            "JointToActuatorPosition": self.jointToActuatorPosition,
        }


class DigitalInput(Serializable):
    def __init__(self, robotTypeName: str, type: str, bitId: int, boardId: int, pressed: int, debounceTime: float):
        self.bitId = bitId
        self.boardId = boardId
        self.name = "{}_{}".format(robotTypeName, type)
        self.pressed = pressed
        self.trigger_when_pressed = True
        self.trigger_when_released = True
        self.debounceTime = debounceTime

    def toDict(self):
        return {
            "bit_id": self.bitId,
            "board_id": self.boardId,
            "debounce_time": self.debounceTime,
            "name": self.name,
            "pressed_value": self.pressed,
            "trigger_when_pressed": self.trigger_when_pressed,
            "trigger_when_released": self.trigger_when_released,
        }


class DigitalOutput(Serializable):
    def __init__(self, robotTypeName: str, type: str, bitId: int, boardId: int):
        self.bitId = bitId
        self.boardId = boardId
        self.name = "{}_{}".format(robotTypeName, type)

    def toDict(self):
        return {
            "bit_id": self.bitId,
            "board_id": self.boardId,
            "name": self.name,
        }


class DallasChip(Serializable):
    def __init__(self, boardId, robotTypeName):
        self.name = "{}_dallas".format(robotTypeName)
        self.boardId = boardId

    def toDict(self):
        return {
            "board_id": self.boardId,
            "name": self.name,
        }


class Actuator(Serializable):
    """
    type: "PRISMATIC" or "REVOLUTE"
    CPT: Encoder counts per turn (quadrature encoder), NOTE: no encoder for last axis
    gearRatio: NOTE: gear ratio for axis 8 is set to 1
    pitch: ?
    motorMaxCurrent: Sustained max current, units are amps
    motorMaxTorque: units are Nm/A
    """

    def __init__(
        self,
        id: int,
        type: str,
        boardId: int,
        axisId: int,
        drive: Drive,
        encoder: Encoder,
        potentiometer: Potentiometer = None,
    ):
        self.id = id
        self.boardId = boardId
        self.axisId = axisId
        self.type = type
        self.drive = drive
        self.encoder = encoder
        self.potentiometer = potentiometer

    def toDict(self):
        dict = {
            "actuator_id": self.id,
            "axis_id": self.axisId,
            "board_id": self.boardId,
            "joint_type": self.type,
            "drive": self.drive,
            "encoder": self.encoder,
        }

        if self.potentiometer is not None:
            dict["potentiometer"] = self.potentiometer

        return dict


def array_like(obj):
    return isinstance(obj, list) or isinstance(obj, np.ndarray)


def canSerialize(obj):
    return isinstance(obj, Serializable) or array_like(obj)


# Allows use of our Serializable objects with Python's builtin json library
def configSerializeJSON(obj):
    if isinstance(obj, Serializable):
        return obj.toDict()

    raise TypeError


def saveConfigFile(fileName, config):
    fileName += ".json"

    if os.path.exists(fileName):
        backup =  fileName + datetime.datetime.now().strftime("-backup-%Y-%m-%d_%H:%M:%S")
        os.rename(fileName, backup)
        print("Existing IO config file has been renamed {}".format(backup))

    with open(fileName, "w") as f:
        json.dump(config, f, indent=4, default=configSerializeJSON)

    print("Generated IO config file {}".format(fileName))


def generateConfig(calFileName, robotTypeName, hardwareVersion, serialNumber, generation):
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
    calData = parser.parseFile(calFileName) if generation == "Classic" else None

    # sanity check robot type matches cal file
    if calData is not None:
        assert (
            robotTypeName[0:3] == calData["FileType"][0:3]
        ), "Robot hardware type doesn't match type from cal file"

    version = 6
    serialNumber = str(serialNumber or calData["serial_number"])
    config = Config(calData, version, robotTypeName, hardwareVersion, serialNumber, generation)

    outputFileName = "sawRobotIO1394-" + robotTypeName + "-" + serialNumber
    saveConfigFile(outputFileName, config)

    if robotTypeName[0:3] == "MTM":
        gripperConfigFileName = (
            "sawRobotIO1394-" + robotTypeName + "-gripper-" + serialNumber
        )
        gripperConfig = Config(calData, version, robotTypeName + "_gripper", hardwareVersion, serialNumber, generation)
        saveConfigFile(gripperConfigFileName, gripperConfig)

    return serialNumber


def generateArmConfig(robotTypeName, hardwareVersion, serialNumber, generation):
    fileName = "{}-{}.json".format(robotTypeName, serialNumber)
    if os.path.exists(fileName):
        backup =  fileName + datetime.datetime.now().strftime("-backup-%Y-%m-%d_%H:%M:%S")
        os.rename(fileName, backup)
        print("Existing arm config file has been renamed {}".format(backup))
    kinematic = '    "kinematic": "kinematic/';
    if robotTypeName.startswith('PSM'):
        kinematic += 'PSM'
    elif robotTypeName == 'MTML':
        kinematic += 'MTML'
    elif robotTypeName == 'MTMR':
        kinematic += 'MTMR'
    elif robotTypeName == 'ECM':
        kinematic += 'ECM'
    else:
        raise ValueError('Unrecognized robot type: {}'.format(robotTypeName))
    if generation == 'Si':
        kinematic += '_Si'
    kinematic += '.json",\n'

    mounting_pitch = 0.0
    with open(fileName, "w") as f:
        f.write('{\n')
        f.write('    // see https://dvrk.readthedocs.io\n')
        f.write('    "$id": "",\n')
        f.write('    "$version": "1",\n')
        f.write(kinematic)
        f.write('    "generation": "' + generation + '"\n')
        if robotTypeName.startswith("PSM"):
            f.write('    // , "tool_detection": "MANUAL"\n')
            f.write('    // , "tool_detection": "FIXED"\n')
            f.write('    , "tool_detection": "AUTOMATIC"\n')
            if generation == 'Si':
                if robotTypeName == 'PSM3':
                    mounting_pitch = -15.0
                else:
                    mounting_pitch = -45
        elif robotTypeName.startswith("ECM"):
            f.write('    // , "endoscope": "Classic_SD_STRAIGHT"\n')
            f.write('    // , "endoscope": "Classic_SD_UP"\n')
            f.write('    // , "endoscope": "Classic_SD_DOWN"\n')
            if generation == "Si":
                f.write('    , "endoscope": "Si_HD_STRAIGHT" // or UP or DOWN\n')
                mounting_pitch = -70.0
            else:
                f.write('    , "endoscope": "Classic_HD_STRAIGHT" // or UP or DOWN\n')
                mounting_pitch = -45
        elif robotTypeName.startswith("MTM"):
            f.write('    // , "gravity_compensation": "gc-' + robotTypeName + '-' + serialNumber + '.json"\n')
        if mounting_pitch != 0.0:
            f.write(f'    , "mounting_pitch": {(mounting_pitch * math.pi / 180.0):.5f} // {mounting_pitch} for {generation} {robotTypeName} on SUJ\n')
        f.write("}\n")

    print('Generated arm config file {}'.format(fileName))


def generateSystemConfig(robotTypeName, hardwareVersion, serialNumber, generation):
    fileName = "system-{}.json".format(robotTypeName)
    if os.path.exists(fileName):
        backup =  fileName + datetime.datetime.now().strftime("-backup-%Y-%m-%d_%H:%M:%S")
        os.rename(fileName, backup)
        print("Existing system file has been renamed {}".format(backup))
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
        f.write('    // see https://dvrk.readthedocs.io\n')
        f.write('    "$id": "dvrk-system.schema.json",\n')
        f.write('    "$version": "1",\n')
        f.write('    "IOs":\n')
        f.write('    [\n')
        f.write('        {\n')
        f.write('            "name": "IO1",\n')
        f.write('            "port": "fw" // or udpfw or udp\n')
        f.write('        }\n')
        f.write('    ]\n')
        f.write('    ,\n')
        f.write('    "arms":\n')
        f.write('    [\n')
        f.write('        {\n')
        f.write('            "name": "' + robotTypeName + '",\n')
        f.write('            "type": "' + type + '",\n')
        f.write('            "serial": "' + str(serialNumber) + '",\n')
        f.write('            "IO": "IO1" // defined in IOs\n')
        f.write('        }\n')
        f.write('    ]\n')
        f.write('}\n')
    print('Generated system file {}'.format(fileName))


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
        type = str,
        help = "serial number (for Si arms)",
        default = None
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

    # sawRobotIO config file
    actualSerial = generateConfig(args.cal, args.arm, args.hardware_version, args.serial, args.generation)
    # sawIntuitiveResearchKit arm and system config files
    generateArmConfig(args.arm, args.hardware_version, actualSerial, args.generation)
    generateSystemConfig(args.arm, args.hardware_version, actualSerial, args.generation)

if __name__ == "__main__":
    main()
