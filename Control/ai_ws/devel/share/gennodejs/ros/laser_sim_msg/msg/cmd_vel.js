// Auto-generated. Do not edit!

// (in-package laser_sim_msg.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class cmd_vel {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.steering_vel = null;
      this.accel_vel = null;
    }
    else {
      if (initObj.hasOwnProperty('steering_vel')) {
        this.steering_vel = initObj.steering_vel
      }
      else {
        this.steering_vel = 0;
      }
      if (initObj.hasOwnProperty('accel_vel')) {
        this.accel_vel = initObj.accel_vel
      }
      else {
        this.accel_vel = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type cmd_vel
    // Serialize message field [steering_vel]
    bufferOffset = _serializer.int32(obj.steering_vel, buffer, bufferOffset);
    // Serialize message field [accel_vel]
    bufferOffset = _serializer.int32(obj.accel_vel, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type cmd_vel
    let len;
    let data = new cmd_vel(null);
    // Deserialize message field [steering_vel]
    data.steering_vel = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [accel_vel]
    data.accel_vel = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'laser_sim_msg/cmd_vel';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'cc66927a56ea8ece1366e10349e9d514';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 steering_vel
    int32 accel_vel
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new cmd_vel(null);
    if (msg.steering_vel !== undefined) {
      resolved.steering_vel = msg.steering_vel;
    }
    else {
      resolved.steering_vel = 0
    }

    if (msg.accel_vel !== undefined) {
      resolved.accel_vel = msg.accel_vel;
    }
    else {
      resolved.accel_vel = 0
    }

    return resolved;
    }
};

module.exports = cmd_vel;
