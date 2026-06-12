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

/**
 * @file seatrac_enums.hpp
 * @brief Shared enums for the Seatrac X150 driver.
 * @author Clayton Smith
 * @date Feb 2024
 */

#pragma once

#include <cstdint>

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

/**
 * CST_E : Command Status Codes
 *
 * Command Status (CST) Codes are an enumeration (or set of defined constants)
 * that are commonly used in Response  messages sent from the beacon to
 * indicate if a command executed successfully,  or if not, what type of error
 * occurred.  CST codes are always transmitted as a  UINT8 type.
 *
 * Different Response messages may only implement a subset of the constants
 * below, as appropriate for their function.
 */
enum CST_E : uint8_t {
  // General Status Codes
  CST_OK = 0x00,            // Returned if a command or operation is completed
                            // successful without error.
  CST_FAIL = 0x01,          // Returned if a command or operation cannot be
                            // completed.
  CST_EEPROM_ERROR = 0x03,  // Returned if an error occurs while reading or
                            // writing EEPROM data.

  // Command Processor Status Codes
  CST_CMD_PARAM_MISSING = 0x04,  // Returned if a command message is given
                                 // that does not have enough defined fields
                                 // for the specified CID code.
  CST_CMD_PARAM_INVALID = 0x05,  // Returned if a data field in a message does
                                 // not contain a valid or expected value.

  // Firmware Programming Status Codes
  CST_PROG_FLASH_ERROR = 0x0A,     // Returned if an error occurs while
                                   // writing data into the processors flash
                                   // memory.
  CST_PROG_FIRMWARE_ERROR = 0x0B,  // Returned if firmware cannot be
                                   // programmed due to incorrect firmware
                                   // credentials or signature.
  CST_PROG_SECTION_ERROR = 0x0C,   // Returned if the firmware cannot be
                                   // programmed into the specified memory
                                   // section.
  CST_PROG_LENGTH_ERROR = 0x0D,    // Returned if the firmware length is too
                                   // large to fit into the specified memory
                                   // section, or not what the current
                                   // operation is expecting.
  CST_PROG_DATA_ERROR = 0x0E,      // Returned if there is an error decoding
                                   // data in a firmware block.
  CST_PROG_CHECKSUM_ERROR = 0x0F,  // Returned if the specified checksum for
                                   // the firmware does not match the checksum
                                   // computed prior to performing the update.

  // Acoustic Transceiver Status Codes
  CST_XCVR_BUSY = 0x30,           // Returned if the transceiver cannot
                                  // perform a requested action as it is
                                  // currently busy (i.e. transmitting a
                                  // message).
  CST_XCVR_ID_REJECTED = 0x31,    // Returned if the received message did not
                                  // match the specified transceiver ID (and
                                  // wasn t a Sent-To-All), and the message
                                  // has been rejected.
  CST_XCVR_CSUM_ERROR = 0x32,     // Returned if received acoustic message's
                                  // checksum was invalid, and the message has
                                  // been rejected.
  CST_XCVR_LENGTH_ERROR = 0x33,   // Returned if an error occurred with
                                  // message framing, meaning the end of the
                                  // message has not been received within the
                                  // expected time.
  CST_XCVR_RESP_TIMEOUT = 0x34,   // Returned if the transceiver has sent a
                                  // request message to a beacon, but no
                                  // response has been returned within the
                                  // allotted waiting period.
  CST_XCVR_RESP_ERROR = 0x35,     // Returned if the transceiver has send a
                                  // request message to a beacon, but an error
                                  // occurred while receiving the response.
  CST_XCVR_RESP_WRONG = 0x36,     // Returned if the transceiver has sent a
                                  // request message to a beacon, but received
                                  // an unexpected response from another
                                  // beacon while waiting.
  CST_XCVR_PLOAD_ERROR = 0x37,    // Returned by protocol payload decoders, if
                                  // the payload can t be parsed correctly.
  CST_XCVR_STATE_STOPPED = 0x3A,  // Indicates the transceiver is in a stopped state.
  CST_XCVR_STATE_IDLE = 0x3B,     // Indicates the transceiver is in an idle
                                  // state waiting for reception or
                                  // transmission to start.
  CST_XCVR_STATE_TX = 0x3C,       // Indicates the transceiver is in a
                                  // transmitting states.
  CST_XCVR_STATE_REQ = 0x3D,      // Indicates the transceiver is in a
                                  // requesting state, having transmitted a
                                  // message and is waiting for a response to
                                  // be received.
  CST_XCVR_STATE_RX = 0x3E,       // Indicates the transceiver is in a
                                  // receiving state.
  CST_XCVR_STATE_RESP = 0x3F,     // Indicates the transceiver is in a
                                  // responding state, where a message is
                                  // being composed and the "response time"
                                  // period is being observed.

  // DEX Protocol Status Codes
  CST_DEX_SOCKET_ERROR = 0x70,       // Returned by the DEX protocol handler
                                     // if an error occurred trying to open,
                                     // close or access a specified socket ID.
  CST_DEX_RX_SYNC = 0x71,            // Returned by the DEX protocol handler
                                     // when receiver synchronisation has
                                     // occurred with the socket master and
                                     // data transfer is ready to commence.
  CST_DEX_RX_DATA = 0x72,            // Returned by the DEX protocol handler
                                     // when data has been received through a
                                     // socket.
  CST_DEX_RX_SEQ_ERROR = 0x73,       // Returned by the DEX protocol handler
                                     // when data transfer synchronisation has
                                     // been lost with the socket master.
  CST_DEX_RX_MSG_ERROR = 0x74,       // Returned by the DEX protocol handler
                                     // to indicate an unexpected acoustic
                                     // message type with the DEX protocol has
                                     // been received and cannot be processed.
  CST_DEX_REQ_ERROR = 0x75,          // Returned by the DEX protocol handler
                                     // to indicate a error has occurred while
                                     // responding to a request (i.e. lack of
                                     // data).
  CST_DEX_RESP_TMO_ERROR = 0x76,     // Returned by the DEX protocol handler
                                     // to indicate a timeout has occurred
                                     // while waiting for a response back from
                                     // a remote beacon with requested data.
  CST_DEX_RESP_MSG_ERROR = 0x77,     // Returned by the DEX protocol handler
                                     // to indicate an error has occurred
                                     // while receiving response back from a
                                     // remote beacon.
  CST_DEX_RESP_REMOTE_ERROR = 0x78,  // Returned by the DEX protocol handler
                                     // to indicate the remote beacon has
                                     // encountered an error and cannot return
                                     // the requested data or perform the
                                     // required operation.
};

}  // namespace coug_comms::utils
