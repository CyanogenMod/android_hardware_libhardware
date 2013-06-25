/*
 * Copyright (C) 2013 CyanogenMod Project
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

#ifndef ANDROID_IRDA_INTERFACE_H
#define ANDROID_IRDA_INTERFACE_H

#include <stdint.h>
#include <sys/cdefs.h>
#include <sys/types.h>

#include <hardware/hardware.h>

__BEGIN_DECLS

#define IRDA_HARDWARE_MODULE_ID "irda"

struct irda_device_t {
    struct hw_device_t common;

    /*
     * (*sendIrCode)() Send IR code
     *
     * @param ircode is the code to send
     * @param length is the length of the code string
     *
     */
    void (*send_ircode)(char* ircode, int length);
};

__END_DECLS

#endif // ANDROID_IRDA_INTERFACE_H
