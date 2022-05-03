/**
 * Created by Il Yeup, Ahn in KETI on 2017-02-25.
 */

/**
 * Copyright (c) 2018, OCEAN
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products derived from this software without specific prior written permission.
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

    // for TAS
var moment = require('moment');
var fs = require('fs');
const dgram = require("dgram");

var mavlink = require('./mavlibrary/mavlink.js');
var {mavlink20, MAVLink20Processor} = require('./mavlibrary/mavlink20.js');

let HOST = '127.0.0.1';
let PORT1 = 14550; // output: SITL --> GCS
let PORT2 = 14552; // input : GCS --> SITL

global.sitlUDP = null;
global.sitlUDP2 = null;

exports.ready = function tas_ready() {
    if ((my_drone_type === 'pixhawk') || (my_drone_type === 'ardupilot') || (my_drone_type === 'px4')) {
        mavPortOpening();
    } else {
    }
};

var aggr_content = {};

function send_aggr_to_Mobius(topic, content_each, gap) {
    if (aggr_content.hasOwnProperty(topic)) {
        var timestamp = moment().format('YYYY-MM-DDTHH:mm:ssSSS');
        aggr_content[topic][timestamp] = content_each;
    } else {
        aggr_content[topic] = {};
        timestamp = moment().format('YYYY-MM-DDTHH:mm:ssSSS');
        aggr_content[topic][timestamp] = content_each;

        setTimeout(function () {
            sh_adn.crtci(topic + '?rcn=0', 0, aggr_content[topic], null, function () {
            });

            delete aggr_content[topic];
        }, gap, topic);
    }
}

exports.noti = function (path_arr, cinObj, socket) {
    var cin = {};
    cin.ctname = path_arr[path_arr.length - 2];
    cin.con = (cinObj.con != null) ? cinObj.con : cinObj.content;

    if (cin.con == '') {
        console.log('---- is not cin message');
    } else {
        socket.write(JSON.stringify(cin));
    }
};

exports.gcs_noti_handler = function (message) {
    if (sitlUDP2 != null) {
        sitlUDP2.send(message, 0, message.length, PORT2, HOST,
            function (err) {
                if (err) {
                    console.log('UDP message send error', err);
                    return;
                }
            }
        );
    } else {

    }
};
sitlUDP2 = dgram.createSocket('udp4');

function mavPortOpening() {
    if (sitlUDP === null) {
        sitlUDP = dgram.createSocket('udp4');
        sitlUDP.bind(PORT1, HOST);

        sitlUDP.on('listening', mavPortOpen);
        sitlUDP.on('message', mavPortData);
        sitlUDP.on('close', mavPortClose);
        sitlUDP.on('error', mavPortError);
    }
}

function mavPortOpen() {
    console.log('UDP socket connect to ' + sitlUDP.address().address + ':' + sitlUDP.address().port);
}

function mavPortClose() {
    console.log('mavPort closed.');

    setTimeout(mavPortOpening, 2000);
}

function mavPortError(error) {
    console.log('[mavPort error]: ' + error.message);

    setTimeout(mavPortOpening, 2000);
}

var mavStrFromDrone = '';
var mavStrFromDroneLength = 0;
var mavVersion = 'unknown';
var mavVersionCheckFlag = false;

function mavPortData(data) {
    mavStrFromDrone += data.toString('hex').toLowerCase();
    // console.log(mavStrFromDrone)

    while (mavStrFromDrone.length > 20) {
        var stx = mavStrFromDrone.substr(0, 2);
        if (stx === 'fe') {
            var len = parseInt(mavStrFromDrone.substr(2, 2), 16);
            var mavLength = (6 * 2) + (len * 2) + (2 * 2);

            if ((mavStrFromDrone.length) >= mavLength) {
                var mavPacket = mavStrFromDrone.substr(0, mavLength);

                mqtt_client.publish(my_cnt_name, Buffer.from(mavPacket, 'hex'));
                // send_aggr_to_Mobius(my_cnt_name, mavPacket, 2000);
                setTimeout(parseMavFromDrone, 0, mavPacket);

                mavStrFromDrone = mavStrFromDrone.substr(mavLength);
                mavStrFromDroneLength = 0;
            } else {
                break;
            }
        } else if (stx === 'fd') {
            len = parseInt(mavStrFromDrone.substr(2, 2), 16);
            mavLength = (10 * 2) + (len * 2) + (2 * 2);

            if (mavStrFromDrone.length >= mavLength) {
                mavPacket = mavStrFromDrone.substr(0, mavLength);

                mqtt_client.publish(my_cnt_name, Buffer.from(mavPacket, 'hex'));
                // send_aggr_to_Mobius(my_cnt_name, mavPacket, 2000);
                setTimeout(parseMavFromDrone, 0, mavPacket);

                mavStrFromDrone = mavStrFromDrone.substr(mavLength);
                mavStrFromDroneLength = 0;
            } else {
                break;
            }
        } else {
            mavStrFromDrone = mavStrFromDrone.substr(2);
        }
    }
}

var fc = {};
try {
    fc = JSON.parse(fs.readFileSync('fc_data_model.json', 'utf8'));
} catch (e) {
    fc.heartbeat = {};
    fc.heartbeat.type = 2;
    fc.heartbeat.autopilot = 3;
    fc.heartbeat.base_mode = 0;
    fc.heartbeat.custom_mode = 0;
    fc.heartbeat.system_status = 0;
    fc.heartbeat.mavlink_version = 1;

    fc.attitude = {};
    fc.attitude.time_boot_ms = 123456789;
    fc.attitude.roll = 0.0;
    fc.attitude.pitch = 0.0;
    fc.attitude.yaw = 0.0;
    fc.attitude.rollspeed = 0.0;
    fc.attitude.pitchspeed = 0.0;
    fc.attitude.yawspeed = 0.0;

    fc.global_position_int = {};
    fc.global_position_int.time_boot_ms = 123456789;
    fc.global_position_int.lat = 0;
    fc.global_position_int.lon = 0;
    fc.global_position_int.alt = 0;
    fc.global_position_int.vx = 0;
    fc.global_position_int.vy = 0;
    fc.global_position_int.vz = 0;
    fc.global_position_int.hdg = 65535;

    fc.battery_status = {};
    fc.battery_status.id = 0;
    fc.battery_status.battery_function = 0;
    fc.battery_status.type = 3;
    fc.battery_status.temperature = 32767;
    fc.battery_status.voltages = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
    fc.battery_status.current_battery = -1;
    fc.battery_status.current_consumed = -1;
    fc.battery_status.battery_remaining = -1;
    fc.battery_status.time_remaining = 0;
    fc.battery_status.charge_state = 0;

    fs.writeFileSync('fc_data_model.json', JSON.stringify(fc, null, 4), 'utf8');
}

var flag_base_mode = 0;

function parseMavFromDrone(mavPacket) {
    try {
        var ver = mavPacket.substr(0, 2);
        if (ver == 'fd') {
            var sysid = mavPacket.substr(10, 2).toLowerCase();
            var msgid = mavPacket.substr(18, 2) + mavPacket.substr(16, 2) + mavPacket.substr(14, 2);
        } else {
            sysid = mavPacket.substr(6, 2).toLowerCase();
            msgid = mavPacket.substr(10, 2).toLowerCase();
        }

        var sys_id = parseInt(sysid, 16);
        var msg_id = parseInt(msgid, 16);

        var mavlinkParserv2 = new MAVLink20Processor(null/*logger*/, sys_id, 0);

        if (msg_id == mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT) { // #33
            if (ver == 'fd') {
                var base_offset = 20;
                var time_boot_ms = mavlinkParserv2.parseBuffer(Buffer.from(mavPacket, 'hex'))[0].time_boot_ms;
                base_offset += 8;
                var lat = mavlinkParserv2.parseBuffer(Buffer.from(mavPacket, 'hex'))[0].lat;
                base_offset += 8;
                var lon = mavlinkParserv2.parseBuffer(Buffer.from(mavPacket, 'hex'))[0].lon;
                base_offset += 8;
                var alt = mavlinkParserv2.parseBuffer(Buffer.from(mavPacket, 'hex'))[0].alt;
                base_offset += 8;
                var relative_alt = mavlinkParserv2.parseBuffer(Buffer.from(mavPacket, 'hex'))[0].relative_alt;

                fc.global_position_int.time_boot_ms = time_boot_ms;
                fc.global_position_int.lat = lat;
                fc.global_position_int.lon = lon;
                fc.global_position_int.alt = alt;
                fc.global_position_int.relative_alt = relative_alt;
            } else {
                base_offset = 12;
                time_boot_ms = mavPacket.substr(base_offset, 8).toLowerCase();
                base_offset += 8;
                lat = mavPacket.substr(base_offset, 8).toLowerCase();
                base_offset += 8;
                lon = mavPacket.substr(base_offset, 8).toLowerCase();
                base_offset += 8;
                alt = mavPacket.substr(base_offset, 8).toLowerCase();
                base_offset += 8;
                relative_alt = mavPacket.substr(base_offset, 8).toLowerCase();

                fc.global_position_int.time_boot_ms = Buffer.from(time_boot_ms, 'hex').readUInt32LE(0);
                fc.global_position_int.lat = Buffer.from(lat, 'hex').readInt32LE(0);
                fc.global_position_int.lon = Buffer.from(lon, 'hex').readInt32LE(0);
                fc.global_position_int.alt = Buffer.from(alt, 'hex').readInt32LE(0);
                fc.global_position_int.relative_alt = Buffer.from(relative_alt, 'hex').readInt32LE(0);
            }

        } else if (msg_id == mavlink.MAVLINK_MSG_ID_HEARTBEAT) { // #00 : HEARTBEAT
            if (ver == 'fd') {
                base_offset = 20;
                var custom_mode = mavlinkParserv2.parseBuffer(Buffer.from(mavPacket, 'hex'))[0].custom_mode;
                base_offset += 8;
                var type = mavlinkParserv2.parseBuffer(Buffer.from(mavPacket, 'hex'))[0].type;
                base_offset += 2;
                var autopilot = mavlinkParserv2.parseBuffer(Buffer.from(mavPacket, 'hex'))[0].autopilot;
                base_offset += 2;
                var base_mode = mavlinkParserv2.parseBuffer(Buffer.from(mavPacket, 'hex'))[0].base_mode;
                base_offset += 2;
                var system_status = mavlinkParserv2.parseBuffer(Buffer.from(mavPacket, 'hex'))[0].system_status;
                base_offset += 2;
                var mavlink_version = mavlinkParserv2.parseBuffer(Buffer.from(mavPacket, 'hex'))[0].mavlink_version;

                fc.heartbeat.type = type;
                fc.heartbeat.autopilot = autopilot;
                fc.heartbeat.base_mode = base_mode;
                fc.heartbeat.custom_mode = custom_mode;
                fc.heartbeat.system_status = system_status;
                fc.heartbeat.mavlink_version = mavlink_version;
            } else {
                base_offset = 12;
                custom_mode = mavPacket.substr(base_offset, 8).toLowerCase();
                base_offset += 8;
                type = mavPacket.substr(base_offset, 2).toLowerCase();
                base_offset += 2;
                autopilot = mavPacket.substr(base_offset, 2).toLowerCase();
                base_offset += 2;
                base_mode = mavPacket.substr(base_offset, 2).toLowerCase();
                base_offset += 2;
                system_status = mavPacket.substr(base_offset, 2).toLowerCase();
                base_offset += 2;
                mavlink_version = mavPacket.substr(base_offset, 2).toLowerCase();

                fc.heartbeat.type = Buffer.from(type, 'hex').readUInt8(0);
                fc.heartbeat.autopilot = Buffer.from(autopilot, 'hex').readUInt8(0);
                fc.heartbeat.base_mode = Buffer.from(base_mode, 'hex').readUInt8(0);
                fc.heartbeat.custom_mode = Buffer.from(custom_mode, 'hex').readUInt32LE(0);
                fc.heartbeat.system_status = Buffer.from(system_status, 'hex').readUInt8(0);
                fc.heartbeat.mavlink_version = Buffer.from(mavlink_version, 'hex').readUInt8(0);
            }

            if (fc.heartbeat.base_mode & 0x80) {
                if (flag_base_mode == 3) {
                    flag_base_mode++;
                    my_sortie_name = moment().format('YYYY_MM_DD_T_HH_mm');
                    my_cnt_name = my_parent_cnt_name + '/' + my_sortie_name;
                    sh_adn.crtct(my_parent_cnt_name + '?rcn=0', my_sortie_name, 0, function (rsc, res_body, count) {
                    });

                    for (var idx in mission_parent) {
                        if (mission_parent.hasOwnProperty(idx)) {
                            setTimeout(createMissionContainer, 10, idx);
                        }
                    }
                } else {
                    flag_base_mode++;
                    if (flag_base_mode > 16) {
                        flag_base_mode = 16;
                    }
                }
            } else {
                flag_base_mode = 0;
                my_sortie_name = 'disarm';
                my_cnt_name = my_parent_cnt_name + '/' + my_sortie_name;
            }
        }
    } catch (e) {
        console.log('[parseMavFromDrone Error]\n', mavPacket, '\n', e);
    }
}

function createMissionContainer(idx) {
    var mission_parent_path = mission_parent[idx];
    sh_adn.crtct(mission_parent_path + '?rcn=0', my_sortie_name, 0, function (rsc, res_body, count) {
    });
}
