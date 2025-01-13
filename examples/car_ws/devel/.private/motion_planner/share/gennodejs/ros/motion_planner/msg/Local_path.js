// Auto-generated. Do not edit!

// (in-package motion_planner.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let State = require('./State.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class Local_path {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.ref_state = null;
      this.goal_pose = null;
      this.goal_reached = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('ref_state')) {
        this.ref_state = initObj.ref_state
      }
      else {
        this.ref_state = [];
      }
      if (initObj.hasOwnProperty('goal_pose')) {
        this.goal_pose = initObj.goal_pose
      }
      else {
        this.goal_pose = [];
      }
      if (initObj.hasOwnProperty('goal_reached')) {
        this.goal_reached = initObj.goal_reached
      }
      else {
        this.goal_reached = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Local_path
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [ref_state]
    // Serialize the length for message field [ref_state]
    bufferOffset = _serializer.uint32(obj.ref_state.length, buffer, bufferOffset);
    obj.ref_state.forEach((val) => {
      bufferOffset = State.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [goal_pose]
    bufferOffset = _arraySerializer.float32(obj.goal_pose, buffer, bufferOffset, null);
    // Serialize message field [goal_reached]
    bufferOffset = _serializer.bool(obj.goal_reached, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Local_path
    let len;
    let data = new Local_path(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [ref_state]
    // Deserialize array length for message field [ref_state]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.ref_state = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.ref_state[i] = State.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [goal_pose]
    data.goal_pose = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [goal_reached]
    data.goal_reached = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    object.ref_state.forEach((val) => {
      length += State.getMessageSize(val);
    });
    length += 4 * object.goal_pose.length;
    return length + 9;
  }

  static datatype() {
    // Returns string type for a message object
    return 'motion_planner/Local_path';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c9e6cb19725f08d09811528c99d2a8f5';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header 
    State[] ref_state
    float32[] goal_pose
    bool goal_reached
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    string frame_id
    
    ================================================================================
    MSG: motion_planner/State
    float32[] data
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Local_path(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.ref_state !== undefined) {
      resolved.ref_state = new Array(msg.ref_state.length);
      for (let i = 0; i < resolved.ref_state.length; ++i) {
        resolved.ref_state[i] = State.Resolve(msg.ref_state[i]);
      }
    }
    else {
      resolved.ref_state = []
    }

    if (msg.goal_pose !== undefined) {
      resolved.goal_pose = msg.goal_pose;
    }
    else {
      resolved.goal_pose = []
    }

    if (msg.goal_reached !== undefined) {
      resolved.goal_reached = msg.goal_reached;
    }
    else {
      resolved.goal_reached = false
    }

    return resolved;
    }
};

module.exports = Local_path;
