
"use strict";

let LaserStatus = require('./LaserStatus.js');
let Cartesian_Euler_pose = require('./Cartesian_Euler_pose.js');
let StringStamped = require('./StringStamped.js');
let Pose2DArray = require('./Pose2DArray.js');
let ElevatorStatus = require('./ElevatorStatus.js');
let BatteryStatus = require('./BatteryStatus.js');
let BatteryDockingStatusStamped = require('./BatteryDockingStatusStamped.js');
let Axis = require('./Axis.js');
let BatteryStatusStamped = require('./BatteryStatusStamped.js');
let LaserMode = require('./LaserMode.js');
let OdomManualCalibrationStatusStamped = require('./OdomManualCalibrationStatusStamped.js');
let BatteryDockingStatus = require('./BatteryDockingStatus.js');
let OdomCalibrationStatusStamped = require('./OdomCalibrationStatusStamped.js');
let QueryAlarm = require('./QueryAlarm.js');
let PresenceSensorArray = require('./PresenceSensorArray.js');
let MotorsStatusDifferential = require('./MotorsStatusDifferential.js');
let MotorsStatus = require('./MotorsStatus.js');
let Registers = require('./Registers.js');
let Register = require('./Register.js');
let SubState = require('./SubState.js');
let Interfaces = require('./Interfaces.js');
let MotorHeadingOffset = require('./MotorHeadingOffset.js');
let MotorStatus = require('./MotorStatus.js');
let Pose2DStamped = require('./Pose2DStamped.js');
let ReturnMessage = require('./ReturnMessage.js');
let Alarms = require('./Alarms.js');
let named_inputs_outputs = require('./named_inputs_outputs.js');
let OdomCalibrationStatus = require('./OdomCalibrationStatus.js');
let MotorPID = require('./MotorPID.js');
let inputs_outputs = require('./inputs_outputs.js');
let InverterStatus = require('./InverterStatus.js');
let encoders = require('./encoders.js');
let PresenceSensor = require('./PresenceSensor.js');
let alarmmonitor = require('./alarmmonitor.js');
let SafetyModuleStatus = require('./SafetyModuleStatus.js');
let named_input_output = require('./named_input_output.js');
let OdomManualCalibrationStatus = require('./OdomManualCalibrationStatus.js');
let RobotnikMotorsStatus = require('./RobotnikMotorsStatus.js');
let Data = require('./Data.js');
let State = require('./State.js');
let alarmsmonitor = require('./alarmsmonitor.js');
let BoolArray = require('./BoolArray.js');
let ElevatorAction = require('./ElevatorAction.js');
let AlarmSensor = require('./AlarmSensor.js');
let ptz = require('./ptz.js');
let StringArray = require('./StringArray.js');
let SetElevatorActionFeedback = require('./SetElevatorActionFeedback.js');
let SetElevatorAction = require('./SetElevatorAction.js');
let SetElevatorActionGoal = require('./SetElevatorActionGoal.js');
let SetElevatorActionResult = require('./SetElevatorActionResult.js');
let SetElevatorGoal = require('./SetElevatorGoal.js');
let SetElevatorFeedback = require('./SetElevatorFeedback.js');
let SetElevatorResult = require('./SetElevatorResult.js');

module.exports = {
  LaserStatus: LaserStatus,
  Cartesian_Euler_pose: Cartesian_Euler_pose,
  StringStamped: StringStamped,
  Pose2DArray: Pose2DArray,
  ElevatorStatus: ElevatorStatus,
  BatteryStatus: BatteryStatus,
  BatteryDockingStatusStamped: BatteryDockingStatusStamped,
  Axis: Axis,
  BatteryStatusStamped: BatteryStatusStamped,
  LaserMode: LaserMode,
  OdomManualCalibrationStatusStamped: OdomManualCalibrationStatusStamped,
  BatteryDockingStatus: BatteryDockingStatus,
  OdomCalibrationStatusStamped: OdomCalibrationStatusStamped,
  QueryAlarm: QueryAlarm,
  PresenceSensorArray: PresenceSensorArray,
  MotorsStatusDifferential: MotorsStatusDifferential,
  MotorsStatus: MotorsStatus,
  Registers: Registers,
  Register: Register,
  SubState: SubState,
  Interfaces: Interfaces,
  MotorHeadingOffset: MotorHeadingOffset,
  MotorStatus: MotorStatus,
  Pose2DStamped: Pose2DStamped,
  ReturnMessage: ReturnMessage,
  Alarms: Alarms,
  named_inputs_outputs: named_inputs_outputs,
  OdomCalibrationStatus: OdomCalibrationStatus,
  MotorPID: MotorPID,
  inputs_outputs: inputs_outputs,
  InverterStatus: InverterStatus,
  encoders: encoders,
  PresenceSensor: PresenceSensor,
  alarmmonitor: alarmmonitor,
  SafetyModuleStatus: SafetyModuleStatus,
  named_input_output: named_input_output,
  OdomManualCalibrationStatus: OdomManualCalibrationStatus,
  RobotnikMotorsStatus: RobotnikMotorsStatus,
  Data: Data,
  State: State,
  alarmsmonitor: alarmsmonitor,
  BoolArray: BoolArray,
  ElevatorAction: ElevatorAction,
  AlarmSensor: AlarmSensor,
  ptz: ptz,
  StringArray: StringArray,
  SetElevatorActionFeedback: SetElevatorActionFeedback,
  SetElevatorAction: SetElevatorAction,
  SetElevatorActionGoal: SetElevatorActionGoal,
  SetElevatorActionResult: SetElevatorActionResult,
  SetElevatorGoal: SetElevatorGoal,
  SetElevatorFeedback: SetElevatorFeedback,
  SetElevatorResult: SetElevatorResult,
};
