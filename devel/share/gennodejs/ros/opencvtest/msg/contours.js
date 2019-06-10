// Auto-generated. Do not edit!

// (in-package opencvtest.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let img_pro_info = require('./img_pro_info.js');

//-----------------------------------------------------------

class contours {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.pro_info = null;
    }
    else {
      if (initObj.hasOwnProperty('pro_info')) {
        this.pro_info = initObj.pro_info
      }
      else {
        this.pro_info = new img_pro_info();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type contours
    // Serialize message field [pro_info]
    bufferOffset = img_pro_info.serialize(obj.pro_info, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type contours
    let len;
    let data = new contours(null);
    // Deserialize message field [pro_info]
    data.pro_info = img_pro_info.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 25;
  }

  static datatype() {
    // Returns string type for a message object
    return 'opencvtest/contours';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '80338e53bc763a94f85a1a487d6d2c70';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    img_pro_info pro_info
    ================================================================================
    MSG: opencvtest/img_pro_info
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
    const resolved = new contours(null);
    if (msg.pro_info !== undefined) {
      resolved.pro_info = img_pro_info.Resolve(msg.pro_info)
    }
    else {
      resolved.pro_info = new img_pro_info()
    }

    return resolved;
    }
};

module.exports = contours;
