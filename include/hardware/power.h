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

#ifndef ANDROID_INCLUDE_HARDWARE_POWER_H
#define ANDROID_INCLUDE_HARDWARE_POWER_H

#include <stdint.h>
#include <sys/cdefs.h>
#include <sys/types.h>

#include <hardware/hardware.h>

__BEGIN_DECLS

#define POWER_MODULE_API_VERSION_0_1  HARDWARE_MODULE_API_VERSION(0, 1)
#define POWER_MODULE_API_VERSION_0_2  HARDWARE_MODULE_API_VERSION(0, 2)

/**
 * The id of this module
 */
#define POWER_HARDWARE_MODULE_ID "power"

/**
* This definition is used by Camera HAL during  camcorder recording.
*
*/
#define HAS_MULTIMEDIA_HINTS

/*
 * Power hint identifiers passed to (*powerHint)
 */

typedef enum {
    POWER_HINT_VSYNC = 0x00000001,
    POWER_HINT_INTERACTION = 0x00000002,
    /* DO NOT USE POWER_HINT_VIDEO_ENCODE/_DECODE!  They will be removed in
     * KLP.
     */
    POWER_HINT_VIDEO_ENCODE = 0x00000003,
    POWER_HINT_VIDEO_DECODE = 0x00000004,
    POWER_HINT_LOW_POWER = 0x00000005,

    POWER_HINT_CPU_BOOST    = 0x00000010,
    POWER_HINT_AUDIO        = 0x00000020,
    POWER_HINT_SET_PROFILE  = 0x00000030

} power_hint_t;

/**
 * Every hardware module must have a data structure named HAL_MODULE_INFO_SYM
 * and the fields of this data structure must begin with hw_module_t
 * followed by module specific information.
 */
typedef struct power_module {
    struct hw_module_t common;

    /*
     * (*init)() performs power management setup actions at runtime
     * startup, such as to set default cpufreq parameters.  This is
     * called only by the Power HAL instance loaded by
     * PowerManagerService.
     */
    void (*init)(struct power_module *module);

    /*
     * (*setInteractive)() performs power management actions upon the
     * system entering interactive state (that is, the system is awake
     * and ready for interaction, often with UI devices such as
     * display and touchscreen enabled) or non-interactive state (the
     * system appears asleep, display usually turned off).  The
     * non-interactive state is usually entered after a period of
     * inactivity, in order to conserve battery power during
     * such inactive periods.
     *
     * Typical actions are to turn on or off devices and adjust
     * cpufreq parameters.  This function may also call the
     * appropriate interfaces to allow the kernel to suspend the
     * system to low-power sleep state when entering non-interactive
     * state, and to disallow low-power suspend when the system is in
     * interactive state.  When low-power suspend state is allowed, the
     * kernel may suspend the system whenever no wakelocks are held.
     *
     * on is non-zero when the system is transitioning to an
     * interactive / awake state, and zero when transitioning to a
     * non-interactive / asleep state.
     *
     * This function is called to enter non-interactive state after
     * turning off the screen (if present), and called to enter
     * interactive state prior to turning on the screen.
     */
    void (*setInteractive)(struct power_module *module, int on);

    /*
     * (*powerHint) is called to pass hints on power requirements, which
     * may result in adjustment of power/performance parameters of the
     * cpufreq governor and other controls.  The possible hints are:
     *
     * POWER_HINT_VSYNC
     *
     *     Foreground app has started or stopped requesting a VSYNC pulse
     *     from SurfaceFlinger.  If the app has started requesting VSYNC
     *     then CPU and GPU load is expected soon, and it may be appropriate
     *     to raise speeds of CPU, memory bus, etc.  The data parameter is
     *     non-zero to indicate VSYNC pulse is now requested, or zero for
     *     VSYNC pulse no longer requested.
     *
     * POWER_HINT_INTERACTION
     *
     *     User is interacting with the device, for example, touchscreen
     *     events are incoming.  CPU and GPU load may be expected soon,
     *     and it may be appropriate to raise speeds of CPU, memory bus,
     *     etc.  The data parameter is unused.
     *
     * POWER_HINT_LOW_POWER
     *
     *     Low power mode is activated or deactivated. Low power mode
     *     is intended to save battery at the cost of performance. The data
     *     parameter is non-zero when low power mode is activated, and zero
     *     when deactivated.
     *
     * POWER_HINT_CPU_BOOST
     *
     *     An operation is happening where it would be ideal for the CPU to
     *     be boosted for a specific duration. The data parameter is an
     *     integer value of the boost duration in microseconds.
     *
     * A particular platform may choose to ignore any hint.
     *
     * availability: version 0.2
     *
     */
    void (*powerHint)(struct power_module *module, power_hint_t hint,
                      void *data);
} power_module_t;

/**
 * System performance profiles, may be set per-app. System should
 * notify user when any profile other than BALANCED is in use.
 *
 * POWER_SAVE, BALANCED, and HIGH_PERFORMANCE are required for
 * the trivial implementation. BIAS profiles are optional.
 */
typedef enum {
    /*
     * Power saving mode, large performance degradation.
     * This mode should be linked with POWER_HINT_LOW_POWER.
     */
    PROFILE_POWER_SAVE          = 0x0,
    /*
     * Balanced profile, this is the default for normal operation
     */
    PROFILE_BALANCED            = 0x1,
    /*
     * Highest performance, highest power consumption.
     */
    PROFILE_HIGH_PERFORMANCE    = 0x2,
    /*
     * Bias towards low power consumption, some performance degradation.
     * Should not cause major usability issues as PROFILE_POWER_SAVE would.
     */
    PROFILE_BIAS_POWER          = 0x3,
    /*
     * Bias towards higher performance, with higher power consumption expected.
     * Should not cause severe battery drain as HIGH_PERFORMANCE would.
     */
    PROFILE_BIAS_PERFORMANCE    = 0x4,
    /*
     * Special low-power bias mode, implementation/device specific. May be
     * implemented as an additional bias step, or geared towards a specific
     * usage pattern such as video playback.
     */
    PROFILE_BIAS_POWER_2        = 0x5,
    /*
     * Special high-performance bias mode, implementation/device specific. May
     * be implemented as an additional bias step, or geared toward a specific
     * usage pattern such as gaming.
     */
    PROFILE_BIAS_PERFORMANCE_2  = 0x6

} power_profile_t;

__END_DECLS

#endif  // ANDROID_INCLUDE_HARDWARE_POWER_H
