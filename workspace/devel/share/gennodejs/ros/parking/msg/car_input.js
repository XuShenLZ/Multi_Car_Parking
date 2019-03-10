// Auto-generated. Do not edit!

// (in-package parking.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class car_input {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.delta = null;
      this.acc = null;
    }
    else {
      if (initObj.hasOwnProperty('delta')) {
        this.delta = initObj.delta
      }
      else {
        this.delta = [];
      }
      if (initObj.hasOwnProperty('acc')) {
        this.acc = initObj.acc
      }
      else {
        this.acc = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type car_input
    // Serialize message field [delta]
    bufferOffset = _arraySerializer.float64(obj.delta, buffer, bufferOffset, null);
    // Serialize message field [acc]
    bufferOffset = _arraySerializer.float64(obj.acc, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type car_input
    let len;
    let data = new car_input(null);
    // Deserialize message field [delta]
    data.delta = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [acc]
    data.acc = _arrayDeserializer.float64(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 8 * object.delta.length;
    length += 8 * object.acc.length;
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'parking/car_input';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'abdc68e560ea7f4923b164eecfe49022';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # The input used in vehicle control
    # Steering angle and acceleration
    float64[] delta
    float64[] acc
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new car_input(null);
    if (msg.delta !== undefined) {
      resolved.delta = msg.delta;
    }
    else {
      resolved.delta = []
    }

    if (msg.acc !== undefined) {
      resolved.acc = msg.acc;
    }
    else {
      resolved.acc = []
    }

    return resolved;
    }
};

module.exports = car_input;
