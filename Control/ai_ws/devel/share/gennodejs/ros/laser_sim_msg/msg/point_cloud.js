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

class point_cloud {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.ranges = null;
      this.size = null;
      this.channel = null;
    }
    else {
      if (initObj.hasOwnProperty('ranges')) {
        this.ranges = initObj.ranges
      }
      else {
        this.ranges = [];
      }
      if (initObj.hasOwnProperty('size')) {
        this.size = initObj.size
      }
      else {
        this.size = 0;
      }
      if (initObj.hasOwnProperty('channel')) {
        this.channel = initObj.channel
      }
      else {
        this.channel = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type point_cloud
    // Serialize message field [ranges]
    bufferOffset = _arraySerializer.float32(obj.ranges, buffer, bufferOffset, null);
    // Serialize message field [size]
    bufferOffset = _serializer.uint32(obj.size, buffer, bufferOffset);
    // Serialize message field [channel]
    bufferOffset = _serializer.uint32(obj.channel, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type point_cloud
    let len;
    let data = new point_cloud(null);
    // Deserialize message field [ranges]
    data.ranges = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [size]
    data.size = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [channel]
    data.channel = _deserializer.uint32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 4 * object.ranges.length;
    return length + 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'laser_sim_msg/point_cloud';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '8c9f5b31aed2b325faeea7a6697b7f64';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32[] ranges
    uint32 size
    uint32 channel
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new point_cloud(null);
    if (msg.ranges !== undefined) {
      resolved.ranges = msg.ranges;
    }
    else {
      resolved.ranges = []
    }

    if (msg.size !== undefined) {
      resolved.size = msg.size;
    }
    else {
      resolved.size = 0
    }

    if (msg.channel !== undefined) {
      resolved.channel = msg.channel;
    }
    else {
      resolved.channel = 0
    }

    return resolved;
    }
};

module.exports = point_cloud;
