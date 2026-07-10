/**
 * @file aaudio/aaudio.h AAudio audio driver for Android
 *
 * Copyright (C) 2024 Juha Heinanen
 */

#include <aaudio/AAudio.h>

void aaudio_close_stream(AAudioStream *stream);

/* Kallo SDK: AAudio output-device pinning. The player stream is (re)opened from
 * several paths (initial alloc, the disconnect→reopen restart, and the JNI
 * route-change rebind). Without an explicit device id AAudio binds the
 * USAGE_VOICE_COMMUNICATION output to the OS default comm device, which
 * auto-prefers a connected Bluetooth headset — so selecting Earpiece/Speaker on
 * the Kotlin side (setCommunicationDevice) flips the indicator but audio stays
 * in the buds. These pin the next open to a concrete AudioDeviceInfo.getId();
 * 0 (AAUDIO_UNSPECIFIED, the default) restores AAudio's own routing. */
void aaudio_set_output_device_id(int32_t id);
int32_t aaudio_get_output_device_id(void);

int aaudio_player_alloc(struct auplay_st **stp, const struct auplay *ap,
			struct auplay_prm *prm, const char *device,
			auplay_write_h *wh, void *arg);

int aaudio_recorder_alloc(struct ausrc_st **stp, const struct ausrc *as,
			  struct ausrc_prm *prm, const char *device,
			  ausrc_read_h *rh, ausrc_error_h *errh, void *arg);
