/**
 * @file aaudio/aaudio.c AAudio audio driver for Android
 *
 * Copyright (C) 2024 Juha Heinanen
 */

#include <re.h>
#include <re_atomic.h>
#include <rem.h>
#include <baresip.h>
#include "aaudio.h"


static struct auplay *auplay;
static struct ausrc *ausrc;

/* AAudio output device id to pin the player stream to on the next open. Default
 * 0 == AAUDIO_UNSPECIFIED (let AAudio pick the OS comm device). Written from the
 * JNI thread (route change / call-end reset) and read on whichever thread
 * (re)opens the player stream, so it is atomic; see aaudio.h for the rationale. */
static RE_ATOMIC int32_t aaudio_output_device_id;

/* Buffer size in framesPerBurst multiples for both streams; see aaudio.h. Same
 * threading story as the device id: written from JNI, read by whichever thread
 * opens a stream. 0 means "unset" and is read back as the default 2, so a
 * zero-initialised module behaves exactly as it did before this knob existed. */
static RE_ATOMIC int32_t aaudio_buffer_bursts;

#define AAUDIO_BUFFER_BURSTS_DEFAULT 2


void aaudio_set_output_device_id(int32_t id)
{
	re_atomic_rlx_set(&aaudio_output_device_id, id);
}


int32_t aaudio_get_output_device_id(void)
{
	return re_atomic_rlx(&aaudio_output_device_id);
}


void aaudio_set_buffer_bursts(int32_t bursts)
{
	re_atomic_rlx_set(&aaudio_buffer_bursts, bursts);
}


int32_t aaudio_get_buffer_bursts(void)
{
	int32_t b = re_atomic_rlx(&aaudio_buffer_bursts);

	return b > 0 ? b : AAUDIO_BUFFER_BURSTS_DEFAULT;
}


static int module_init(void)
{
	int err;

	err  = auplay_register(&auplay, baresip_auplayl(),
			       "aaudio", aaudio_player_alloc);
	err |= ausrc_register(&ausrc, baresip_ausrcl(),
			      "aaudio", aaudio_recorder_alloc);

	return err;
}


static int module_close(void)
{

	auplay = mem_deref(auplay);
	ausrc = mem_deref(ausrc);

	return 0;
}


EXPORT_SYM const struct mod_export DECL_EXPORTS(aaudio) = {
	"aaudio",
	"audio",
	module_init,
	module_close,
};
