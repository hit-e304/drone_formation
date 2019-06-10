// Auto-generated. Do not edit!

// (in-package opencvtest.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class img_pro_info {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.find_obs_flag = null;
      this.dis = null;
      this.pos_left = null;
      this.pos_right = null;
      this.x_pos = null;
      this.y_pos = null;
    }
    else {
      if (initObj.hasOwnProperty('find_obs_flag')) {
        this.find_obs_flag = initObj.find_obs_flag
      }
      else {
        this.find_obs_flag = false;
      }
      if (initObj.hasOwnProperty('dis')) {
        this.dis = initObj.dis
      }
      else {
        this.dis = 0.0;
      }
      if (initObj.hasOwnProperty('pos_left')) {
        this.pos_left = initObj.pos_left
      }
      else {
        this.pos_left = 0;
      }
      if (initObj.hasOwnProperty('pos_right')) {
        this.pos_right = initObj.pos_right
      }
      else {
        this.pos_right = 0;
      }
      if (initObj.hasOwnProperty('x_pos')) {
        this.x_pos = initObj.x_pos
      }
      else {
        this.x_pos = 0;
      }
      if (initObj.hasOwnProperty('y_pos')) {
        this.y_pos = initObj.y_pos
      }
      else {
        this.y_pos = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type img_pro_info
    // Serialize message field [find_obs_flag]
    bufferOffset = _serializer.bool(obj.find_obs_flag, buffer, bufferOffset);
    // Serialize message field [dis]
    bufferOffset = _serializer.float64(obj.dis, buffer, bufferOffset);
    // Serialize message field [pos_left]
    bufferOffset = _serializer.int32(obj.pos_left, buffer, bufferOffset);
    // Serialize message field [pos_right]
    bufferOffset = _serializer.int32(obj.pos_right, buffer, bufferOffset);
    // Serialize message field [x_pos]
    bufferOffset = _serializer.int32(obj.x_pos, buffer, bufferOffset);
    // Serialize message field [y_pos]
    bufferOffset = _serializer.int32(obj.y_pos, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type img_pro_info
    let len;
    let data = new img_pro_info(null);
    // Deserialize message field [find_obs_flag]
    data.find_obs_flag = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [dis]
    data.dis = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [pos_left]
    data.pos_left = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [pos_right]
    data.pos_right = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [x_pos]
    data.x_pos = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [y_pos]
    data.y_pos = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 25;
  }

  static datatype() {
    // Returns string type for a message object
    return 'opencvtest/img_pro_info';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '254e05080fb643551976301900a3b655';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
        bool find_obs_flag
        float64 dis
        int32 pos_left
        int32 pos_right
        int32 x_pos
        int32 y_pos
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new img_pro_info(null);
    if (msg.find_obs_flag !== undefined) {
      resolved.find_obs_flag = msg.find_obs_flag;
    }
    else {
      resolved.find_obs_flag = false
    }

    if (msg.dis !== undefined) {
      resolved.dis = msg.dis;
    }
    else {
      resolved.dis = 0.0
    }

    if (msg.pos_left !== undefined) {
      resolved.pos_left = msg.pos_left;
    }
    else {
      resolved.pos_left = 0
    }

    if (msg.pos_right !== undefined) {
      resolved.pos_right = msg.pos_right;
    }
    else {
      resolved.pos_right = 0
    }

    if (msg.x_pos !== undefined) {
      resolved.x_pos = msg.x_pos;
    }
    else {
      resolved.x_pos = 0
    }

    if (msg.y_pos !== undefined) {
      resolved.y_pos = msg.y_pos;
    }
    else {
      resolved.y_pos = 0
    }

    return resolved;
    }
};

module.exports = img_pro_info;
