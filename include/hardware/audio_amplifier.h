/*
 * Copyright (C) 2015, The CyanogenMod Project
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

#ifndef CM_AUDIO_AMPLIFIER_INTERFACE_H
#define CM_AUDIO_AMPLIFIER_INTERFACE_H

#include <stdint.h>
#include <sys/cdefs.h>
#include <sys/types.h>

#include <hardware/hardware.h>

#include <system/audio.h>

__BEGIN_DECLS

#define AMPLIFIER_MODULE_API_VERSION_0_1 HARDWARE_MODULE_API_VERSION(0, 1)

#define AMPLIFIER_HARDWARE_MODULE_ID "audio_amplifier"

typedef struct amplifier_module {
    struct hw_module_t common;

    /**
     * Turn on amplifier hardware
     */
    int (*open)(struct amplifier_module *module);

    /**
     * Notify amplifier module of current input/output devices
     *
     * This function should handle both input and output devices.
     */
    void (*set_devices)(struct amplifier_module *module, int devices);

    /**
     * Notify amplifier module about current audio mode
     */
    int (*set_mode)(struct amplifier_module *module,
            struct audio_mode_t mode);

    /**
     * Notify amplifier module that an output stream has started
     */
    void (*stream_start)(struct amplifier_module *module,
            struct audio_stream_out *stream, bool offload);

    /**
     * Notify amplifier module that an output stream has stopped
     */
    void (*stream_standby)(struct amplifier_module *module,
            struct audio_stream_out *stream);

    /**
     * Shut down amplifier hardware
     */
    int (*close)(struct amplifier_module *module);
} amplifier_module_t;

__END_DECLS

#endif  // CM_AUDIO_AMPLIFIER_INTERFACE_H
