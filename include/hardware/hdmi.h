/*
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

#ifndef ANDROID_HDMI_INTERFACE_H
#define ANDROID_HDMI_INTERFACE_H

#include <stdint.h>
#include <sys/cdefs.h>
#include <sys/types.h>

#include <hardware/hardware.h>

__BEGIN_DECLS

/**
 * The id of this module
 */
#define HDMI_HARDWARE_MODULE_ID "hdmi"

enum {
    HDMI_MODE_NONE = 0,
    HDMI_MODE_UI,
    HDMI_MODE_VIDEO,
};

struct hdmi_device_t {
    struct hw_device_t common;

    int (*connect)(struct hdmi_device_t* dev);

    int (*disconnect)(struct hdmi_device_t* dev);

    int (*clear) (struct hdmi_device_t* dev, int hdmiLayer);

    int (*blit)(struct hdmi_device_t* dev, int srcW, int srcH, int srcColorFormat,
                        uint32_t srcYAddr, uint32_t srcCbAddr, uint32_t srcCrAddr,
                        int dstX, int dstY,
                        int layer,
                        int num_of_hwc_layer);
};


__END_DECLS

#endif  // ANDROID_HDMI_INTERFACE_H

