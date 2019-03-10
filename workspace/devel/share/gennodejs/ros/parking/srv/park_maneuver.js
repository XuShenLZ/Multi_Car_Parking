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


//-----------------------------------------------------------

class park_maneuverRequest {
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
    // Serializes a message object of type park_maneuverRequest
    // Serialize message field [Start_lane]
    bufferOffset = _serializer.string(obj.Start_lane, buffer, bufferOffset);
    // Serialize message field [End_spot]
    bufferOffset = _serializer.string(obj.End_spot, buffer, bufferOffset);
    // Serialize message field [End_pose]
    bufferOffset = _serializer.string(obj.End_pose, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type park_maneuverRequest
    let len;
    let data = new park_maneuverRequest(null);
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
    return 'parking/park_maneuverRequest';
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
    const resolved = new park_maneuverRequest(null);
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

class park_maneuverResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.path = null;
      this.input = null;
    }
    else {
      if (initObj.hasOwnProperty('path')) {
        this.path = initObj.path
      }
      else {
        this.path = [];
      }
      if (initObj.hasOwnProperty('input')) {
        this.input = initObj.input
      }
      else {
        this.input = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type park_maneuverResponse
    // Serialize message field [path]
    bufferOffset = _arraySerializer.float64(obj.path, buffer, bufferOffset, null);
    // Serialize message field [input]
    bufferOffset = _arraySerializer.float64(obj.input, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type park_maneuverResponse
    let len;
    let data = new park_maneuverResponse(null);
    // Deserialize message field [path]
    data.path = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [input]
    data.input = _arrayDeserializer.float64(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 8 * object.path.length;
    length += 8 * object.input.length;
    return length + 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'parking/park_maneuverResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '7466c0bf710479ba4de50e5ac3b9ae45';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64[] path
    float64[] input
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new park_maneuverResponse(null);
    if (msg.path !== undefined) {
      resolved.path = msg.path;
    }
    else {
      resolved.path = []
    }

    if (msg.input !== undefined) {
      resolved.input = msg.input;
    }
    else {
      resolved.input = []
    }

    return resolved;
    }
};

module.exports = {
  Request: park_maneuverRequest,
  Response: park_maneuverResponse,
  md5sum() { return 'a22294d83207313cb1c3be5a262be07d'; },
  datatype() { return 'parking/park_maneuver'; }
};
