
"use strict";

let SetMotorMode = require('./SetMotorMode.js')
let get_digital_input = require('./get_digital_input.js')
let SetEncoderTurns = require('./SetEncoderTurns.js')
let SetMotorPID = require('./SetMotorPID.js')
let SetTransform = require('./SetTransform.js')
let SetElevator = require('./SetElevator.js')
let set_analog_output = require('./set_analog_output.js')
let ack_alarm = require('./ack_alarm.js')
let set_named_digital_output = require('./set_named_digital_output.js')
let set_modbus_register = require('./set_modbus_register.js')
let SetByte = require('./SetByte.js')
let set_float_value = require('./set_float_value.js')
let set_odometry = require('./set_odometry.js')
let set_CartesianEuler_pose = require('./set_CartesianEuler_pose.js')
let get_alarms = require('./get_alarms.js')
let GetMotorsHeadingOffset = require('./GetMotorsHeadingOffset.js')
let SetString = require('./SetString.js')
let SetNamedDigitalOutput = require('./SetNamedDigitalOutput.js')
let set_mode = require('./set_mode.js')
let get_mode = require('./get_mode.js')
let SetBuzzer = require('./SetBuzzer.js')
let ResetFromSubState = require('./ResetFromSubState.js')
let set_digital_output = require('./set_digital_output.js')
let GetBool = require('./GetBool.js')
let set_height = require('./set_height.js')
let SetLaserMode = require('./SetLaserMode.js')
let InsertTask = require('./InsertTask.js')
let QueryAlarms = require('./QueryAlarms.js')
let GetPOI = require('./GetPOI.js')
let get_modbus_register = require('./get_modbus_register.js')
let set_ptz = require('./set_ptz.js')
let GetPTZ = require('./GetPTZ.js')
let axis_record = require('./axis_record.js')
let SetMotorStatus = require('./SetMotorStatus.js')
let enable_disable = require('./enable_disable.js')
let home = require('./home.js')

module.exports = {
  SetMotorMode: SetMotorMode,
  get_digital_input: get_digital_input,
  SetEncoderTurns: SetEncoderTurns,
  SetMotorPID: SetMotorPID,
  SetTransform: SetTransform,
  SetElevator: SetElevator,
  set_analog_output: set_analog_output,
  ack_alarm: ack_alarm,
  set_named_digital_output: set_named_digital_output,
  set_modbus_register: set_modbus_register,
  SetByte: SetByte,
  set_float_value: set_float_value,
  set_odometry: set_odometry,
  set_CartesianEuler_pose: set_CartesianEuler_pose,
  get_alarms: get_alarms,
  GetMotorsHeadingOffset: GetMotorsHeadingOffset,
  SetString: SetString,
  SetNamedDigitalOutput: SetNamedDigitalOutput,
  set_mode: set_mode,
  get_mode: get_mode,
  SetBuzzer: SetBuzzer,
  ResetFromSubState: ResetFromSubState,
  set_digital_output: set_digital_output,
  GetBool: GetBool,
  set_height: set_height,
  SetLaserMode: SetLaserMode,
  InsertTask: InsertTask,
  QueryAlarms: QueryAlarms,
  GetPOI: GetPOI,
  get_modbus_register: get_modbus_register,
  set_ptz: set_ptz,
  GetPTZ: GetPTZ,
  axis_record: axis_record,
  SetMotorStatus: SetMotorStatus,
  enable_disable: enable_disable,
  home: home,
};
