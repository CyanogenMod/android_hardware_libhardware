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
#include <cutils/str_parms.h>

#include <hardware/audio_amplifier.h>
#include <hardware/hardware.h>

static int amp_set_input_devices(amplifier_device_t *device, uint32_t devices)
{
    return 0;
}

static int amp_set_output_devices(amplifier_device_t *device, uint32_t devices)
{
    return 0;
}

static int amp_enable_output_devices(amplifier_device_t *device,
        uint32_t devices, bool enable)
{
    return 0;
}

static int amp_enable_input_devices(amplifier_device_t *device,
        uint32_t devices, bool enable)
{
    return 0;
}

static int amp_set_mode(amplifier_device_t *device, audio_mode_t mode)
{
    return 0;
}

static int amp_output_stream_start(amplifier_device_t *device,
        struct audio_stream_out *stream, bool offload)
{
    return 0;
}

static int amp_input_stream_start(amplifier_device_t *device,
        struct audio_stream_in *stream)
{
    return 0;
}

static int amp_output_stream_standby(amplifier_device_t *device,
        struct audio_stream_out *stream)
{
    return 0;
}

static int amp_input_stream_standby(amplifier_device_t *device,
        struct audio_stream_in *stream)
{
    return 0;
}

static int amp_set_parameters(struct amplifier_device *device,
        struct str_parms *parms)
{
    return 0;
}

static int amp_dev_close(hw_device_t *device)
{
    if (device)
        free(device);

    return 0;
}

static int amp_module_open(const hw_module_t *module, const char *name,
        hw_device_t **device)
{
    if (strcmp(name, AMPLIFIER_HARDWARE_INTERFACE)) {
        ALOGE("%s:%d: %s does not match amplifier hardware interface name\n",
                __func__, __LINE__, name);
        return -ENODEV;
    }

    amplifier_device_t *amp_dev = calloc(1, sizeof(amplifier_device_t));
    if (!amp_dev) {
        ALOGE("%s:%d: Unable to allocate memory for amplifier device\n",
                __func__, __LINE__);
        return -ENOMEM;
    }

    amp_dev->common.tag = HARDWARE_DEVICE_TAG;
    amp_dev->common.module = (hw_module_t *) module;
    amp_dev->common.version = HARDWARE_DEVICE_API_VERSION(1, 0);
    amp_dev->common.close = amp_dev_close;

    amp_dev->set_input_devices = amp_set_input_devices;
    amp_dev->set_output_devices = amp_set_output_devices;
    amp_dev->enable_output_devices = amp_enable_output_devices;
    amp_dev->enable_input_devices = amp_enable_input_devices;
    amp_dev->set_mode = amp_set_mode;
    amp_dev->output_stream_start = amp_output_stream_start;
    amp_dev->input_stream_start = amp_input_stream_start;
    amp_dev->output_stream_standby = amp_output_stream_standby;
    amp_dev->input_stream_standby = amp_input_stream_standby;
    amp_dev->set_parameters = amp_set_parameters;

    *device = (hw_device_t *) amp_dev;

    return 0;
}

static struct hw_module_methods_t hal_module_methods = {
    .open = amp_module_open,
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
};
