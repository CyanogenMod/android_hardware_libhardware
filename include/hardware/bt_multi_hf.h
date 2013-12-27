/* Copyright (c) 2014, The Linux Foundation. All rights reserved.
 * Not a Contribution.
 *
 * Copyright (C) 2012 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef ANDROID_INCLUDE_BT_MULTI_HF_H
#define ANDROID_INCLUDE_BT_MULTI_HF_H

#include <hardware/bt_hf.h>

/** Callback for connection state change.
 *  state will have one of the values from BtHfConnectionState
 */
typedef void (* btmultihf_connection_state_callback)(bthf_connection_state_t state,
                                             bt_bdaddr_t *bd_addr);

/** Callback for audio connection state change.
 *  state will have one of the values from BtHfAudioState
 */
typedef void (* btmultihf_audio_state_callback)(bthf_audio_state_t state, bt_bdaddr_t *bd_addr);

/** Callback for VR connection state change.
 *  state will have one of the values from BtHfVRState
 */
typedef void (* btmultihf_vr_cmd_callback)(bthf_vr_state_t state, bt_bdaddr_t *bd_addr);

/** Callback for answer incoming call (ATA)
 */
typedef void (* btmultihf_answer_call_cmd_callback)(bt_bdaddr_t *bd_addr);

/** Callback for disconnect call (AT+CHUP)
 */
typedef void (* btmultihf_hangup_call_cmd_callback)(bt_bdaddr_t *bd_addr);

/** Callback for disconnect call (AT+CHUP)
 *  type will denote Speaker/Mic gain (BtHfVolumeControl).
 */
typedef void (* btmultihf_volume_cmd_callback)(bthf_volume_type_t type, int volume,
                                                        bt_bdaddr_t *bd_addr);

/** Callback for dialing an outgoing call
 *  If number is NULL, redial
 */
typedef void (* btmultihf_dial_call_cmd_callback)(char *number, bt_bdaddr_t *bd_addr);

/** Callback for sending DTMF tones
 *  tone contains the dtmf character to be sent
 */
typedef void (* btmultihf_dtmf_cmd_callback)(char tone, bt_bdaddr_t *bd_addr);

/** Callback for enabling/disabling noise reduction/echo cancellation
 *  value will be 1 to enable, 0 to disable
 */
typedef void (* btmultihf_nrec_cmd_callback)(bthf_nrec_t nrec, bt_bdaddr_t *bd_addr);

/** Callback for call hold handling (AT+CHLD)
 *  value will contain the call hold command (0, 1, 2, 3)
 */
typedef void (* btmultihf_chld_cmd_callback)(bthf_chld_type_t chld,
                                                  bt_bdaddr_t *bd_addr);

/** Callback for CNUM (subscriber number)
 */
typedef void (* btmultihf_cnum_cmd_callback)(bt_bdaddr_t *bd_addr);

/** Callback for indicators (CIND)
 */
typedef void (* btmultihf_cind_cmd_callback)(bt_bdaddr_t *bd_addr);

/** Callback for operator selection (COPS)
 */
typedef void (* btmultihf_cops_cmd_callback)(bt_bdaddr_t *bd_addr);

/** Callback for call list (AT+CLCC)
 */
typedef void (* btmultihf_clcc_cmd_callback) (bt_bdaddr_t *bd_addr);

/** Callback for unknown AT command recd from HF
 *  at_string will contain the unparsed AT string
 */
typedef void (* btmultihf_unknown_at_cmd_callback)(char *at_string,
                                               bt_bdaddr_t *bd_addr);

/** Callback for keypressed (HSP) event.
 */
typedef void (* btmultihf_key_pressed_cmd_callback)(bt_bdaddr_t *bd_addr);

/** Callback for Codec negotiation event.
*/
typedef void (* btmultihf_codec_negotiated_callback)(int codec_type, bt_bdaddr_t *bd_addr);

/** BT-Multi-HF callback structure. */
typedef struct {
    /** set to sizeof(BtMultiHfCallbacks) */
    size_t      size;
    btmultihf_connection_state_callback  connection_state_cb;
    btmultihf_audio_state_callback       audio_state_cb;
    btmultihf_vr_cmd_callback            vr_cmd_cb;
    btmultihf_answer_call_cmd_callback   answer_call_cmd_cb;
    btmultihf_hangup_call_cmd_callback   hangup_call_cmd_cb;
    btmultihf_volume_cmd_callback        volume_cmd_cb;
    btmultihf_dial_call_cmd_callback     dial_call_cmd_cb;
    btmultihf_dtmf_cmd_callback          dtmf_cmd_cb;
    btmultihf_nrec_cmd_callback          nrec_cmd_cb;
    btmultihf_chld_cmd_callback          chld_cmd_cb;
    btmultihf_cnum_cmd_callback          cnum_cmd_cb;
    btmultihf_cind_cmd_callback          cind_cmd_cb;
    btmultihf_cops_cmd_callback          cops_cmd_cb;
    btmultihf_clcc_cmd_callback          clcc_cmd_cb;
    btmultihf_unknown_at_cmd_callback    unknown_at_cmd_cb;
    btmultihf_key_pressed_cmd_callback   key_pressed_cmd_cb;
    btmultihf_codec_negotiated_callback  codec_negotiated_callback;
} btmultihf_callbacks_t;

/** Represents the standard BT-Multi-HF interface. */
typedef struct {

    /** set to sizeof(BtMultiHfInterface) */
    size_t          size;
    /**
     * Register the BtMultiHf callbacks
     */
    bt_status_t (*init)( btmultihf_callbacks_t* callbacks, int max_hf_clients );

    /** Set the feature bitmask */
    bt_status_t (*init_features)( int feature_bitmask );

    /** connect to headset */
    bt_status_t (*connect)( bt_bdaddr_t *bd_addr );

    /** dis-connect from headset */
    bt_status_t (*disconnect)( bt_bdaddr_t *bd_addr );

    /** create an audio connection */
    bt_status_t (*connect_audio)( bt_bdaddr_t *bd_addr );

    /** close the audio connection */
    bt_status_t (*disconnect_audio)( bt_bdaddr_t *bd_addr );

    /** start voice recognition */
    bt_status_t (*start_voice_recognition)( bt_bdaddr_t *bd_addr );

    /** stop voice recognition */
    bt_status_t (*stop_voice_recognition)( bt_bdaddr_t *bd_addr );

    /** volume control */
    bt_status_t (*volume_control) ( bthf_volume_type_t type, int volume,
                                                      bt_bdaddr_t *bd_addr );

    /** Combined device status change notification */
    bt_status_t (*device_status_notification)( bthf_network_state_t ntk_state,
             bthf_service_type_t svc_type, int signal, int batt_chg );

    /** Response for COPS command */
    bt_status_t (*cops_response)( const char *cops, bt_bdaddr_t *bd_addr );

    /** Response for CIND command */
    bt_status_t (*cind_response)( int svc, int num_active, int num_held,
                                  bthf_call_state_t call_setup_state, int signal,
                                  int roam, int batt_chg, bt_bdaddr_t *bd_addr );

    /** Pre-formatted AT response, typically in response to unknown AT cmd */
    bt_status_t (*formatted_at_response)( const char *rsp, bt_bdaddr_t *bd_addr );

    /** ok/error response
     *  ERROR (0)
     *  OK    (1)
     */
    bt_status_t (*at_response) ( bthf_at_response_t response_code, int error_code,
                                                       bt_bdaddr_t *bd_addr );

    /** response for CLCC command
     *  Can be iteratively called for each call index
     *  Call index of 0 will be treated as NULL termination (Completes response)
     */
    bt_status_t (*clcc_response) ( int index, bthf_call_direction_t dir,
                                bthf_call_state_t state, bthf_call_mode_t mode,
                                bthf_call_mpty_type_t mpty, const char *number,
                                bthf_call_addrtype_t type, bt_bdaddr_t *bd_addr );

    /** notify of a call state change
     *  Each update notifies
     *    1. Number of active/held/ringing calls
     *    2. call_state: This denotes the state change that triggered this msg
     *                   This will take one of the values from BtHfCallState
     *    3. number & type: valid only for incoming & waiting call
    */
    bt_status_t (*phone_state_change) ( int num_active, int num_held,
                        bthf_call_state_t call_setup_state, const char *number,
                        bthf_call_addrtype_t type );

    /** get remote supported features */
    int (*get_remote_features)(bt_bdaddr_t *bd_addr );

    /** Closes the interface. */
    void  (*cleanup)( void );
} btmultihf_interface_t;

#endif /* ANDROID_INCLUDE_BT_MULTI_HF_H */
