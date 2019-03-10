// Auto-generated. Do not edit!

// (in-package parking.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

let car_state = require('../msg/car_state.js');
let car_input = require('../msg/car_input.js');

//-----------------------------------------------------------

class maneuverRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.Start_lane = null;
      this.End_spot = null;
      this.End_pose = null;
    }
    else {
      if (initObj.hasOwnProperty('Start_lane')) {
        this.Start_lane = initObj.Start_lane
      }
      else {
        this.Start_lane = '';
      }
      if (initObj.hasOwnProperty('End_spot')) {
        this.End_spot = initObj.End_spot
      }
      else {
        this.End_spot = '';
      }
      if (initObj.hasOwnProperty('End_pose')) {
        this.End_pose = initObj.End_pose
      }
      else {
        this.End_pose = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type maneuverRequest
    // Serialize message field [Start_lane]
    bufferOffset = _serializer.string(obj.Start_lane, buffer, bufferOffset);
    // Serialize message field [End_spot]
    bufferOffset = _serializer.string(obj.End_spot, buffer, bufferOffset);
    // Serialize message field [End_pose]
    bufferOffset = _serializer.string(obj.End_pose, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type maneuverRequest
    let len;
    let data = new maneuverRequest(null);
    // Deserialize message field [Start_lane]
    data.Start_lane = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [End_spot]
    data.End_spot = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [End_pose]
    data.End_pose = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.Start_lane.length;
    length += object.End_spot.length;
    length += object.End_pose.length;
    return length + 12;
  }

  static datatype() {
    // Returns string type for a service object
    return 'parking/maneuverRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a95a3873c578d430d5049d20303c042d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string Start_lane
    string End_spot
    string End_pose
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new maneuverRequest(null);
    if (msg.Start_lane !== undefined) {
      resolved.Start_lane = msg.Start_lane;
    }
    else {
      resolved.Start_lane = ''
    }

    if (msg.End_spot !== undefined) {
      resolved.End_spot = msg.End_spot;
    }
    else {
      resolved.End_spot = ''
    }

    if (msg.End_pose !== undefined) {
      resolved.End_pose = msg.End_pose;
    }
    else {
      resolved.End_pose = ''
    }

    return resolved;
    }
};

class maneuverResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.succeed = null;
      this.path = null;
      this.input = null;
      this.dt = null;
    }
    else {
      if (initObj.hasOwnProperty('succeed')) {
        this.succeed = initObj.succeed
      }
      else {
        this.succeed = false;
      }
      if (initObj.hasOwnProperty('path')) {
        this.path = initObj.path
      }
      else {
        this.path = new car_state();
      }
      if (initObj.hasOwnProperty('input')) {
        this.input = initObj.input
      }
      else {
        this.input = new car_input();
      }
      if (initObj.hasOwnProperty('dt')) {
        this.dt = initObj.dt
      }
      else {
        this.dt = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type maneuverResponse
    // Serialize message field [succeed]
    bufferOffset = _serializer.bool(obj.succeed, buffer, bufferOffset);
    // Serialize message field [path]
    bufferOffset = car_state.serialize(obj.path, buffer, bufferOffset);
    // Serialize message field [input]
    bufferOffset = car_input.serialize(obj.input, buffer, bufferOffset);
    // Serialize message field [dt]
    bufferOffset = _arraySerializer.float64(obj.dt, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type maneuverResponse
    let len;
    let data = new maneuverResponse(null);
    // Deserialize message field [succeed]
    data.succeed = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [path]
    data.path = car_state.deserialize(buffer, bufferOffset);
    // Deserialize message field [input]
    data.input = car_input.deserialize(buffer, bufferOffset);
    // Deserialize message field [dt]
    data.dt = _arrayDeserializer.float64(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += car_state.getMessageSize(object.path);
    length += car_input.getMessageSize(object.input);
    length += 8 * object.dt.length;
    return length + 5;
  }

  static datatype() {
    // Returns string type for a service object
    return 'parking/maneuverResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '50633944400a8fb4ab22c5885e422031';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool      succeed
    car_state path
    car_input input
    float64[] dt
    
    ================================================================================
    MSG: parking/car_state
    # The states used in vehicle control
    # x, y, heading and speed
    float64[] x
    float64[] y
    float64[] psi
    float64[] v
    ================================================================================
    MSG: parking/car_input
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
    const resolved = new maneuverResponse(null);
    if (msg.succeed !== undefined) {
      resolved.succeed = msg.succeed;
    }
    else {
      resolved.succeed = false
    }

    if (msg.path !== undefined) {
      resolved.path = car_state.Resolve(msg.path)
    }
    else {
      resolved.path = new car_state()
    }

    if (msg.input !== undefined) {
      resolved.input = car_input.Resolve(msg.input)
    }
    else {
      resolved.input = new car_input()
    }

    if (msg.dt !== undefined) {
      resolved.dt = msg.dt;
    }
    else {
      resolved.dt = []
    }

    return resolved;
    }
};

module.exports = {
  Request: maneuverRequest,
  Response: maneuverResponse,
  md5sum() { return '946fe324ce1d28d42fa3b8f242a28b5c'; },
  datatype() { return 'parking/maneuver'; }
};
