
"use strict";

let Compass = require('./Compass.js');
let RawImu = require('./RawImu.js');
let ControllerState = require('./ControllerState.js');
let RawMagnetic = require('./RawMagnetic.js');
let RC = require('./RC.js');
let HeightCommand = require('./HeightCommand.js');
let VelocityZCommand = require('./VelocityZCommand.js');
let MotorCommand = require('./MotorCommand.js');
let HeadingCommand = require('./HeadingCommand.js');
let MotorPWM = require('./MotorPWM.js');
let ThrustCommand = require('./ThrustCommand.js');
let PositionXYCommand = require('./PositionXYCommand.js');
let AttitudeCommand = require('./AttitudeCommand.js');
let Altimeter = require('./Altimeter.js');
let MotorStatus = require('./MotorStatus.js');
let RawRC = require('./RawRC.js');
let Supply = require('./Supply.js');
let YawrateCommand = require('./YawrateCommand.js');
let ServoCommand = require('./ServoCommand.js');
let RuddersCommand = require('./RuddersCommand.js');
let VelocityXYCommand = require('./VelocityXYCommand.js');
let TakeoffActionResult = require('./TakeoffActionResult.js');
let PoseGoal = require('./PoseGoal.js');
let TakeoffActionGoal = require('./TakeoffActionGoal.js');
let PoseAction = require('./PoseAction.js');
let TakeoffActionFeedback = require('./TakeoffActionFeedback.js');
let PoseActionResult = require('./PoseActionResult.js');
let LandingActionFeedback = require('./LandingActionFeedback.js');
let PoseFeedback = require('./PoseFeedback.js');
let LandingAction = require('./LandingAction.js');
let TakeoffAction = require('./TakeoffAction.js');
let TakeoffResult = require('./TakeoffResult.js');
let LandingResult = require('./LandingResult.js');
let LandingGoal = require('./LandingGoal.js');
let PoseActionGoal = require('./PoseActionGoal.js');
let PoseActionFeedback = require('./PoseActionFeedback.js');
let TakeoffGoal = require('./TakeoffGoal.js');
let LandingActionResult = require('./LandingActionResult.js');
let PoseResult = require('./PoseResult.js');
let LandingFeedback = require('./LandingFeedback.js');
let TakeoffFeedback = require('./TakeoffFeedback.js');
let LandingActionGoal = require('./LandingActionGoal.js');

module.exports = {
  Compass: Compass,
  RawImu: RawImu,
  ControllerState: ControllerState,
  RawMagnetic: RawMagnetic,
  RC: RC,
  HeightCommand: HeightCommand,
  VelocityZCommand: VelocityZCommand,
  MotorCommand: MotorCommand,
  HeadingCommand: HeadingCommand,
  MotorPWM: MotorPWM,
  ThrustCommand: ThrustCommand,
  PositionXYCommand: PositionXYCommand,
  AttitudeCommand: AttitudeCommand,
  Altimeter: Altimeter,
  MotorStatus: MotorStatus,
  RawRC: RawRC,
  Supply: Supply,
  YawrateCommand: YawrateCommand,
  ServoCommand: ServoCommand,
  RuddersCommand: RuddersCommand,
  VelocityXYCommand: VelocityXYCommand,
  TakeoffActionResult: TakeoffActionResult,
  PoseGoal: PoseGoal,
  TakeoffActionGoal: TakeoffActionGoal,
  PoseAction: PoseAction,
  TakeoffActionFeedback: TakeoffActionFeedback,
  PoseActionResult: PoseActionResult,
  LandingActionFeedback: LandingActionFeedback,
  PoseFeedback: PoseFeedback,
  LandingAction: LandingAction,
  TakeoffAction: TakeoffAction,
  TakeoffResult: TakeoffResult,
  LandingResult: LandingResult,
  LandingGoal: LandingGoal,
  PoseActionGoal: PoseActionGoal,
  PoseActionFeedback: PoseActionFeedback,
  TakeoffGoal: TakeoffGoal,
  LandingActionResult: LandingActionResult,
  PoseResult: PoseResult,
  LandingFeedback: LandingFeedback,
  TakeoffFeedback: TakeoffFeedback,
  LandingActionGoal: LandingActionGoal,
};
