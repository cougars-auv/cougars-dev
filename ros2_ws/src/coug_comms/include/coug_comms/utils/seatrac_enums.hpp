// Copyright (c) 2026 BYU FROST Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef _DEF_SEATRAC_DRIVER_SEATRAC_ENUMS_H_
#define _DEF_SEATRAC_DRIVER_SEATRAC_ENUMS_H_

#include <cstdint>
#include <iostream>
#include <sstream>

namespace coug_comms::utils {

/**
 * AMSGTYPE_E : Acoustic Message Type.
 *
 * The AMSGTYPE_E enumeration is used to specify a type of acoustic message,
 * and determines how the message is processed and which responses are
 * generated from beacons.
 */
enum AMSGTYPE_E : uint8_t {
  MSG_OWAY = 0x0,     // Indicates an acoustic message is sent One-Way, and
                      // does not require a response. One-Way messages may
                      // also be broadcast to all beacons if required.  No
                      // USBL information is sent.
  MSG_OWAYU = 0x1,    // Indicates an acoustic message is sent One-Way, and
                      // does not require a response. One-Way messages may
                      // also be broadcast to all beacons if required.
                      // Additionally, the message is sent with USBL acoustic
                      // information allowing an incoming bearing to be
                      // determined by USBL receivers, although range cannot
                      // be provided.
  MSG_REQ = 0x2,      // Indicates an acoustic message is sent as a Request
                      // type.  This requires the receiver to generate and
                      // return a Response (MSG_RESP) message.  No USBL
                      // information is requested.
  MSG_RESP = 0x3,     // Indicate an acoustic message is sent as a Response to
                      // a previous Request message (MSG_REQ).  No USBL
                      // information is returned.
  MSG_REQU = 0x4,     // Indicates an acoustic message is sent as a Request
                      // type.  This requires the receiver to generate and
                      // return a Response (MSG_RESP) message.  Additionally,
                      // the Response message should be returned with USBL
                      // acoustic information allowing a position fix to be
                      // computed by USBL receivers through the range and
                      // incoming signal angle.
  MSG_RESPU = 0x5,    // Indicate an acoustic message is sent as a Response to
                      // a previous Request message (MSG_REQ).  Additionally,
                      // the message is sent with USBL acoustic information
                      // allowing the position of the sender to be determined
                      // through the range and incoming signal angle.
  MSG_REQX = 0x6,     // Indicates an acoustic message is sent as a Request
                      // type.  This requires the receiver to generate and
                      // return a Response (MSG_RESP) message.  Additionally,
                      // the Response message should be returned with extended
                      // Depth and USBL acoustic information allowing a more
                      // accurate position fix to be computed by USBL
                      // receivers through the range, remote depth and
                      // incoming signal angle.
  MSG_RESPX = 0x7,    // Indicate an acoustic message is sent as a Response to
                      // a previous Request message (MSG_REQ).  Additionally,
                      // the message is sent with extended depth and USBL
                      // acoustic information allowing a more accurate
                      // position of the sender to be determined through the
                      // range, remote depth and incoming signal angle.
  MSG_UNKNOWN = 0xFF  // This value is NEVER used to specify a message type
                      // when sending Acoustic Messages. However, on occasions
                      // certain structures need to specify "No Message Type"
                      // (for example see ACOFIX_T), and this value is used as
                      // an OUTPUT ONLY to indicate this.
};

/**
 * BID_E : Beacon Identification Code
 *
 * Beacon Identification (BID) Codes are used to identify a specific beacon
 * that should receive acoustic messages, or identify which beacon was the
 * source (sender) of a message. Valid values are in the range from 0 to 15 and
 * are typically send and stored as a UINT8.
 */
enum BID_E : uint8_t {
  BEACON_ALL = 0x0,  // When used as an address for sending acoustic
                     // messages to, the value of 0x00 indicates "broadcast
                     // to all".  When used as an identifier of a sender of
                     // a message, the value of 0x00 should be interpreted
                     // as unknown or invalid (reserved).
  BEACON_ID_1 = 0x1,
  BEACON_ID_2 = 0x2,
  BEACON_ID_3 = 0x3,
  BEACON_ID_4 = 0x4,
  BEACON_ID_5 = 0x5,
  BEACON_ID_6 = 0x6,
  BEACON_ID_7 = 0x7,
  BEACON_ID_8 = 0x8,
  BEACON_ID_9 = 0x9,
  BEACON_ID_10 = 0xa,
  BEACON_ID_11 = 0xb,
  BEACON_ID_12 = 0xc,
  BEACON_ID_13 = 0xd,
  BEACON_ID_14 = 0xe,
  BEACON_ID_15 = 0xf,
};

/**
 * CID_E : Command Identification Codes
 *
 * Command Identification (CID) Codes are an enumeration (or defined set of
 * constants) stored/transmitted in a UINT8 type at the start of Command and
 * Response messages after the synchronisation character, with the purpose of
 * identifying the message function and its payload.
 */
enum CID_E : uint8_t {
  // System Messages
  CID_SYS_ALIVE = 0x01,        // Command sent to receive a simple alive
                               // message from the beacon.
  CID_SYS_INFO = 0x02,         // Command sent to receive hardware & firmware
                               // identification information.
  CID_SYS_REBOOT = 0x03,       // Command sent to soft reboot the beacon.
  CID_SYS_ENGINEERING = 0x04,  // Command sent to perform engineering actions.

  // Firmware Programming Messages
  CID_PROG_INIT = 0x0D,    // Command sent to initialise a firmware
                           // programming sequence.
  CID_PROG_BLOCK = 0x0E,   // Command sent to transfer a firmware programming
                           // block.
  CID_PROG_UPDATE = 0x0F,  // Command sent to update the firmware once program
                           // transfer has completed.

  // Status Messages
  CID_STATUS = 0x10,          // Command sent to request the current system
                              // status (AHRS, Depth, Temp, etc).
  CID_STATUS_CFG_GET = 0x11,  // Command sent to retrieve the configuration of
                              // the status system (message content and
                              // auto-output interval).
  CID_STATUS_CFG_SET = 0x12,  // Command sent to set the configuration of the
                              // status system (message content and
                              // auto-output interval).

  // Settings Messages
  CID_SETTINGS_GET = 0x15,    // Command sent to retrieve the working settings
                              // in use on the beacon.
  CID_SETTINGS_SET = 0x16,    // Command sent to set the working settings and
                              // apply them. They are NOT saved to permanent
                              // memory until CID_ SETTINGS_SAVE is issued.
                              // The device will need to be rebooted after
                              // this to apply some of the changes.
  CID_SETTINGS_LOAD = 0x17,   // Command sent to load the working settings
                              // from permanent storage and apply them. Not
                              // all settings can be loaded and applied as
                              // they only affect the device on start-up.
  CID_SETTINGS_SAVE = 0x18,   // Command sent to save the working settings
                              // into permanent storage.
  CID_SETTINGS_RESET = 0x19,  // Command sent to restore the working settings
                              // to defaults, store them into permanent memory
                              // and apply them.

  // Calibration messages
  CID_CAL_ACTION = 0x20,    // Command sent to perform specific calibration
                            // actions.
  CID_AHRS_CAL_GET = 0x21,  // Command sent to retrieve the current AHRS
                            // calibration.
  CID_AHRS_CAL_SET = 0x22,  // Command sent to set the contents of the current
                            // AHRS calibration (and store to memory)

  // Acoustic Transceiver Messages
  CID_XCVR_ANALYSE = 0x30,       // Command sent to instruct the receiver to
                                 // perform a noise analysis and report the
                                 // results.
  CID_XCVR_TX_MSG = 0x31,        // Message sent when the transceiver
                                 // transmits a message.
  CID_XCVR_RX_ERR = 0x32,        // Message sent when the transceiver receiver
                                 // encounters an error.
  CID_XCVR_RX_MSG = 0x33,        // Message sent when the transceiver receives
                                 // a message (not requiring a response).
  CID_XCVR_RX_REQ = 0x34,        // Message sent when the transceiver receives
                                 // a request (requiring a response).
  CID_XCVR_RX_RESP = 0x35,       // Message sent when the transceiver receives
                                 // a response (to a transmitted request).
  CID_XCVR_RX_UNHANDLED = 0x37,  // Message sent when a message has been
                                 // received but not handled by the protocol
                                 // stack.
  CID_XCVR_USBL = 0x38,          // Message sent when a USBL signal is decoded
                                 // into an angular bearing.
  CID_XCVR_FIX = 0x39,           // Message sent when the transceiver gets a
                                 // position/range fix on a beacon from a
                                 // request/response.
  CID_XCVR_STATUS = 0x3A,        // Message sent to query the current
                                 // transceiver state.

  // PING Protocol Messages
  CID_PING_SEND = 0x40,   // Command sent to transmit a PING message.
  CID_PING_REQ = 0x41,    // Message sent when a PING request is received.
  CID_PING_RESP = 0x42,   // Message sent when a PING response is received, or
                          // timeout occurs, with the echo response data.
  CID_PING_ERROR = 0x43,  // Message sent when a PING response error/timeout
                          // occurs.

  // ECHO Protocol Messages
  CID_ECHO_SEND = 0x48,   // Command sent to transmit an ECHO message.
  CID_ECHO_REQ = 0x49,    // Message sent when an ECHO request is received.
  CID_ECHO_RESP = 0x4A,   // Message sent when an ECHO response is received,
                          // or timeout occurs, with the echo response data.
  CID_ECHO_ERROR = 0x4B,  // Message sent when an ECHO response error/timeout
                          // occurs.

  // NAV Protocol Messages
  CID_NAV_QUERY_SEND = 0x50,      // Message sent to query navigation
                                  // information from a remote beacon.
  CID_NAV_QUERY_REQ = 0x51,       // Message sent from a beacon that receives
                                  // a NAV_QUERY.
  CID_NAV_QUERY_RESP = 0x52,      // Message generated when the beacon
                                  // received a response to a NAV_QUERY.
  CID_NAV_ERROR = 0x53,           // Message generated if there is a problem
                                  // with a NAV_QUERY - i.e. timeout etc.
  CID_NAV_QUEUE_SET = 0x58,       // Message sent to set the contents of the
                                  // packet data queue.
  CID_NAV_QUEUE_CLR = 0x59,       // Message sent to clear the contents of the
                                  // packet data queue.
  CID_NAV_QUEUE_STATUS = 0x5A,    // Message sent to obtain the current status
                                  // of the packet data queue.
  CID_NAV_STATUS_SEND = 0x5B,     // Message that is used to broadcast status
                                  // information from one beacon (typically
                                  // the USBL head) to others in the system.
                                  // This may include beacon positions, GPS
                                  // coordinates etc.
  CID_NAV_STATUS_RECEIVE = 0x5C,  // Message generated when a beacon receives
                                  // a NAV_STATUS message.

  // DAT Protocol Messages
  CID_DAT_SEND = 0x60,          // Message sent to transmit a datagram to
                                // another beacon
  CID_DAT_RECEIVE = 0x61,       // Message generated when a beacon receives a
                                // datagram.
  CID_DAT_ERROR = 0x63,         // Message generated when a beacon response
                                // error/timeout occurs for ACKs.
  CID_DAT_QUEUE_SET = 0x64,     // Message sent to set the contents of the
                                // packet data queue.
  CID_DAT_QUEUE_CLR = 0x65,     // Message sent to clear the contents of the
                                // packet data queue.
  CID_DAT_QUEUE_STATUS = 0x66,  // Message sent to obtain the current status
                                // of the packet data queue.
};

}  // namespace coug_comms::utils

inline std::ostream& operator<<(std::ostream& os, coug_comms::utils::AMSGTYPE_E msgType) {
  using namespace coug_comms::utils;
  switch (msgType) {
    default:
      os << "Unknown CID : " << (uint8_t)msgType;
      break;
    case MSG_OWAY:
      os << "MSG_OWAY";
      break;
    case MSG_OWAYU:
      os << "MSG_OWAYU";
      break;
    case MSG_REQ:
      os << "MSG_REQ";
      break;
    case MSG_RESP:
      os << "MSG_RESP";
      break;
    case MSG_REQU:
      os << "MSG_REQU";
      break;
    case MSG_RESPU:
      os << "MSG_RESPU";
      break;
    case MSG_REQX:
      os << "MSG_REQX";
      break;
    case MSG_RESPX:
      os << "MSG_RESPX";
      break;
    case MSG_UNKNOWN:
      os << "MSG_UNKNOWN";
      break;
  }
  return os;
}

inline std::ostream& operator<<(std::ostream& os, const coug_comms::utils::BID_E& beaconId) {
  if (beaconId == coug_comms::utils::BEACON_ALL)
    os << "BEACON_ALL";
  else if (beaconId <= 15)
    os << "BEACON_" << (int)beaconId;
  else
    os << "BEACON_UNKNOWN";
  return os;
}

inline std::ostream& operator<<(std::ostream& os, coug_comms::utils::CID_E cid) {
  using namespace coug_comms::utils;
  switch (cid) {
    default:
      os << "Unknown CID : " << (uint8_t)cid;
      break;
    case CID_SYS_ALIVE:
      os << "CID_SYS_ALIVE";
      break;
    case CID_SYS_INFO:
      os << "CID_SYS_INFO";
      break;
    case CID_SYS_REBOOT:
      os << "CID_SYS_REBOOT";
      break;
    case CID_SYS_ENGINEERING:
      os << "CID_SYS_ENGINEERING";
      break;
    case CID_PROG_INIT:
      os << "CID_PROG_INIT";
      break;
    case CID_PROG_BLOCK:
      os << "CID_PROG_BLOCK";
      break;
    case CID_PROG_UPDATE:
      os << "CID_PROG_UPDATE";
      break;
    case CID_STATUS:
      os << "CID_STATUS";
      break;
    case CID_STATUS_CFG_GET:
      os << "CID_STATUS_CFG_GET";
      break;
    case CID_STATUS_CFG_SET:
      os << "CID_STATUS_CFG_SET";
      break;
    case CID_SETTINGS_GET:
      os << "CID_SETTINGS_GET";
      break;
    case CID_SETTINGS_SET:
      os << "CID_SETTINGS_SET";
      break;
    case CID_SETTINGS_LOAD:
      os << "CID_SETTINGS_LOAD";
      break;
    case CID_SETTINGS_SAVE:
      os << "CID_SETTINGS_SAVE";
      break;
    case CID_SETTINGS_RESET:
      os << "CID_SETTINGS_RESET";
      break;
    case CID_CAL_ACTION:
      os << "CID_CAL_ACTION";
      break;
    case CID_AHRS_CAL_GET:
      os << "CID_AHRS_CAL_GET";
      break;
    case CID_AHRS_CAL_SET:
      os << "CID_AHRS_CAL_SET";
      break;
    case CID_XCVR_ANALYSE:
      os << "CID_XCVR_ANALYSE";
      break;
    case CID_XCVR_TX_MSG:
      os << "CID_XCVR_TX_MSG";
      break;
    case CID_XCVR_RX_ERR:
      os << "CID_XCVR_RX_ERR";
      break;
    case CID_XCVR_RX_MSG:
      os << "CID_XCVR_RX_MSG";
      break;
    case CID_XCVR_RX_REQ:
      os << "CID_XCVR_RX_REQ";
      break;
    case CID_XCVR_RX_RESP:
      os << "CID_XCVR_RX_RESP";
      break;
    case CID_XCVR_RX_UNHANDLED:
      os << "CID_XCVR_RX_UNHANDLED";
      break;
    case CID_XCVR_USBL:
      os << "CID_XCVR_USBL";
      break;
    case CID_XCVR_FIX:
      os << "CID_XCVR_FIX";
      break;
    case CID_XCVR_STATUS:
      os << "CID_XCVR_STATUS";
      break;
    case CID_PING_SEND:
      os << "CID_PING_SEND";
      break;
    case CID_PING_REQ:
      os << "CID_PING_REQ";
      break;
    case CID_PING_RESP:
      os << "CID_PING_RESP";
      break;
    case CID_PING_ERROR:
      os << "CID_PING_ERROR";
      break;
    case CID_ECHO_SEND:
      os << "CID_ECHO_SEND";
      break;
    case CID_ECHO_REQ:
      os << "CID_ECHO_REQ";
      break;
    case CID_ECHO_RESP:
      os << "CID_ECHO_RESP";
      break;
    case CID_ECHO_ERROR:
      os << "CID_ECHO_ERROR";
      break;
    case CID_NAV_QUERY_SEND:
      os << "CID_NAV_QUERY_SEND";
      break;
    case CID_NAV_QUERY_REQ:
      os << "CID_NAV_QUERY_REQ";
      break;
    case CID_NAV_QUERY_RESP:
      os << "CID_NAV_QUERY_RESP";
      break;
    case CID_NAV_ERROR:
      os << "CID_NAV_ERROR";
      break;
    case CID_NAV_QUEUE_SET:
      os << "CID_NAV_QUEUE_SET";
      break;
    case CID_NAV_QUEUE_CLR:
      os << "CID_NAV_QUEUE_CLR";
      break;
    case CID_NAV_QUEUE_STATUS:
      os << "CID_NAV_QUEUE_STATUS";
      break;
    case CID_NAV_STATUS_SEND:
      os << "CID_NAV_STATUS_SEND";
      break;
    case CID_NAV_STATUS_RECEIVE:
      os << "CID_NAV_STATUS_RECEIVE";
      break;
    case CID_DAT_SEND:
      os << "CID_DAT_SEND";
      break;
    case CID_DAT_RECEIVE:
      os << "CID_DAT_RECEIVE";
      break;
    case CID_DAT_ERROR:
      os << "CID_DAT_ERROR";
      break;
    case CID_DAT_QUEUE_SET:
      os << "CID_DAT_QUEUE_SET";
      break;
    case CID_DAT_QUEUE_CLR:
      os << "CID_DAT_QUEUE_CLR";
      break;
    case CID_DAT_QUEUE_STATUS:
      os << "CID_DAT_QUEUE_STATUS";
      break;
  }
  return os;
}

#endif  //_DEF_SEATRAC_DRIVER_SEATRAC_ENUMS_H_
