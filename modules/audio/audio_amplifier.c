/*
 * Copyright (C) 2015 The CyanogenMod Open Source Project
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

#define LOG_TAG "amplifier_default"
//#define LOG_NDEBUG 0

#include <stdint.h>
#include <sys/types.h>

#include <cutils/log.h>

#include <hardware/audio_amplifier.h>
#include <hardware/hardware.h>

static int amp_init(amplifier_module_t *module)
{
    return 0;
}

static int amp_set_devices(amplifier_module_t *module, int devices)
{
    return 0;
}

static int amp_set_mode(amplifier_module_t *module, audio_mode_t mode)
{
    return 0;
}

static int amp_stream_start(amplifier_module_t *module,
        struct audio_stream_out *stream, bool offload)
{
    return 0;
}

static int amp_stream_standby(amplifier_module_t *module,
        struct audio_stream_out *stream)
{
    return 0;
}

static int amp_shutdown(amplifier_module_t *module)
{
    return 0;
}

static struct hw_module_methods_t hal_module_methods = {
    .open = NULL,
};

amplifier_module_t HAL_MODULE_INFO_SYM = {
    .common = {
        .tag = HARDWARE_MODULE_TAG,
        .module_api_version = AMPLIFIER_MODULE_API_VERSION_0_1,
        .hal_api_version = HARDWARE_HAL_API_VERSION,
        .id = AMPLIFIER_HARDWARE_MODULE_ID,
        .name = "Default audio amplifier HAL",
        .author = "The CyanogenMod Open Source Project",
        .methods = &hal_module_methods,
    },
    .init = amp_init,
    .set_devices = amp_set_devices,
    .set_mode = amp_set_mode,
    .stream_start = amp_stream_start,
    .stream_standby = amp_stream_standby,
    .shutdown = amp_shutdown,
};
