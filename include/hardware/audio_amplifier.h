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

#include <hardware/audio.h>

#if defined(__cplusplus)
extern "C" {
#endif

#ifdef HAS_AUDIO_AMPLIFIER_IMPL
int amplifier_open(void);
void amplifier_set_devices(int devices);
int amplifier_set_mode(audio_mode_t mode);
void amplifier_stream_start(struct audio_stream_out *stream, bool offload);
void amplifier_stream_standby(struct audio_stream_out *stream);
int amplifier_close(void);
#else
static inline int amplifier_open(void)
{
    return 0;
}

static inline void amplifier_set_devices(int devices)
{
}

static inline int amplifier_set_mode(audio_mode_t mode)
{
    return 0;
}

static inline void amplifier_stream_start(struct audio_stream_out *stream,
        bool offload)
{
}

static inline void amplifier_stream_standby(struct audio_stream_out *stream)
{
}

static inline int amplifier_close(void)
{
    return 0;
}
#endif

#if defined(__cplusplus)
}
#endif
#endif
