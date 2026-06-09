/**
 * @file aaudio/player.c  AAudio audio driver for Android
 *
 * Copyright (C) 2024 Juha Heinanen
 * Copyright (C) 2024 Sebastian Reimers
 */

#include <re.h>
#include <rem.h>
#include <baresip.h>

#include "aaudio.h"


struct auplay_st {
	AAudioStream *playerStream;
	auplay_write_h *wh;
	void *arg;
	struct auplay_prm play_prm;
	size_t sampsz;

	/* Kallo SDK playout instrumentation. Measures what actually reaches the
	 * AAudio device at the pull point (dataCallback), so a silent downlink is
	 * visible directly instead of being inferred from RTP-receive counts.
	 * Logged once per ~1s: frames pulled, peak |sample| and the fraction of
	 * non-silent samples over the interval. A continuous call shows frames>0
	 * and a high active% every interval; a silent gap (aubuf underrun /
	 * jitter-buffer playout stall) drops active% and peak to ~0 for that
	 * interval even though dataCallback keeps being invoked. */
	uint64_t dbg_frames;     /**< frames pulled this interval        */
	uint64_t dbg_active;     /**< non-silent samples this interval   */
	int32_t  dbg_peak;       /**< peak |sample| this interval        */
	uint64_t dbg_total_fr;   /**< frames pulled since stream start   */
	uint64_t dbg_silent_iv;  /**< count of fully-silent intervals    */
	uint32_t dbg_iv;         /**< interval index (≈ seconds)         */
	uint64_t dbg_win;        /**< frames remaining in current window */
};

/* Sample magnitude below this (out of 32767) is treated as silence. PCMU
 * idle/comfort noise and zero-fill underruns sit well under this; speech /
 * a test tone sits far above it. */
#define KALLO_SILENCE_THRESH 64


static int open_player_stream(struct auplay_st *st);


static void auplay_destructor(void *arg)
{
	struct auplay_st *st = arg;

	info("aaudio: player: closing stream\n");
	aaudio_close_stream(st->playerStream);

	st->wh = NULL;
}


/**
 * For an output stream, this function should render and write from
 * userData numFrames of data in the streams current data format to
 * the audioData buffer.
 */
 static int dataCallback(AAudioStream *stream, void *userData,
			 void *audioData, int32_t numFrames) {
	(void)stream;
	struct auplay_st *st = userData;
	struct auframe af;

	auframe_init(&af, st->play_prm.fmt, audioData, numFrames,
		     st->play_prm.srate, st->play_prm.ch);

	st->wh(&af, st->arg);

	/* ── Kallo SDK playout instrumentation ──────────────────────────────
	 * Inspect the PCM the write handler just produced for the device. fmt is
	 * AUFMT_S16LE (enforced at alloc), ch==1. */
	const int16_t *s = (const int16_t *)audioData;
	for (int32_t i = 0; i < numFrames; i++) {
		int32_t v = s[i];
		if (v < 0)
			v = -v;
		if (v > st->dbg_peak)
			st->dbg_peak = v;
		if (v > KALLO_SILENCE_THRESH)
			++st->dbg_active;
	}
	st->dbg_frames   += (uint64_t)numFrames;
	st->dbg_total_fr += (uint64_t)numFrames;

	if (st->dbg_win <= (uint64_t)numFrames) {
		uint64_t fr  = st->dbg_frames ? st->dbg_frames : 1;
		unsigned act = (unsigned)((st->dbg_active * 100) / fr);
		bool silent  = (st->dbg_peak <= KALLO_SILENCE_THRESH);
		if (silent)
			++st->dbg_silent_iv;

		/* One greppable line per second. "SILENT" marks a gap. */
		info("aaudio: PLAYOUT t=%us frames=%llu peak=%d active=%u%% "
		     "total=%llu silent_ivs=%llu%s\n",
		     st->dbg_iv,
		     (unsigned long long)st->dbg_frames,
		     (int)st->dbg_peak, act,
		     (unsigned long long)st->dbg_total_fr,
		     (unsigned long long)st->dbg_silent_iv,
		     silent ? "  <<SILENT>>" : "");

		st->dbg_iv     += 1;
		st->dbg_frames  = 0;
		st->dbg_active  = 0;
		st->dbg_peak    = 0;
		st->dbg_win     = st->play_prm.srate; /* ~1s of frames */
	}
	else {
		st->dbg_win -= (uint64_t)numFrames;
	}

	return 0;
}


static void* restart_player_stream(void* data) {
	aaudio_result_t result;
	struct auplay_st *st = data;

	AAudioStream_close(st->playerStream);

	result = open_player_stream(st);
	if (result != AAUDIO_OK) {
		warning("aaudio: failed to open player stream\n");
		return NULL;
	}

	result = AAudioStream_requestStart(st->playerStream);
	if (result != AAUDIO_OK)
		warning("aaudio: player: failed to start stream\n");
	else
		info("aaudio: player: stream started\n");

	pthread_exit(NULL);
}


static void errorCallback(AAudioStream *stream, void *userData,
			  aaudio_result_t error) {
	struct auplay_st *st = userData;
	(void)error;
	pthread_t thread_id;
	int res;

	aaudio_stream_state_t streamState = AAudioStream_getState(stream);
	if (streamState == AAUDIO_STREAM_STATE_DISCONNECTED) {
		info("aaudio: player: stream disconnected\n");
		res = pthread_create(&thread_id, NULL, restart_player_stream,
				     (void *)st);
		if (res) {
			warning("aaudio: player: error creating thread: %d\n",
				res);
			return;
		}
		info("aaudio: player: created new thread (%u)\n", thread_id);
	}
}


static int open_player_stream(struct auplay_st *st) {

	AAudioStreamBuilder *builder;
	aaudio_result_t result;

	result = AAudio_createStreamBuilder(&builder);
	if (result != AAUDIO_OK) {
		warning("aaudio: player: failed to create stream builder: "
			"error %s\n", AAudio_convertResultToText(result));
		return result;
	}

	AAudioStreamBuilder_setDirection(builder, AAUDIO_DIRECTION_OUTPUT);
	AAudioStreamBuilder_setSharingMode(builder,
		AAUDIO_SHARING_MODE_SHARED);
	AAudioStreamBuilder_setSampleRate(builder, st->play_prm.srate);
	AAudioStreamBuilder_setChannelCount(builder, 1);
	AAudioStreamBuilder_setFormat(builder, AAUDIO_FORMAT_PCM_I16);
	AAudioStreamBuilder_setSessionId(builder, AAUDIO_SESSION_ID_ALLOCATE);
	AAudioStreamBuilder_setUsage(builder,
		AAUDIO_USAGE_VOICE_COMMUNICATION);
	AAudioStreamBuilder_setPerformanceMode(builder,
		AAUDIO_PERFORMANCE_MODE_LOW_LATENCY);
	AAudioStreamBuilder_setDataCallback(builder, &dataCallback, st);
	AAudioStreamBuilder_setErrorCallback(builder, &errorCallback, st);

	result = AAudioStreamBuilder_openStream(builder, &st->playerStream);
	if (result != AAUDIO_OK) {
		warning("aaudio: player: failed to open stream: error %s\n",
			AAudio_convertResultToText(result));
		return result;
	}

	info("aaudio: player: opened stream with direction %d, "
	     "sharing mode %d, sample rate %d, format %d, sessionId %d, "
	     "usage %d, performance mode %d\n",
	     AAudioStream_getDirection(st->playerStream),
	     AAudioStream_getSharingMode(st->playerStream),
	     AAudioStream_getSampleRate(st->playerStream),
	     AAudioStream_getFormat(st->playerStream),
	     AAudioStream_getSessionId(st->playerStream),
	     AAudioStream_getUsage(st->playerStream),
	     AAudioStream_getPerformanceMode(st->playerStream));

	AAudioStreamBuilder_delete(builder);

	AAudioStream_setBufferSizeInFrames(st->playerStream,
		AAudioStream_getFramesPerBurst(st->playerStream) * 2);

	return AAUDIO_OK;
}


int aaudio_player_alloc(struct auplay_st **stp, const struct auplay *ap,
	struct auplay_prm *prm, const char *dev, auplay_write_h *wh, void *arg)
{
	struct auplay_st *st;
	aaudio_result_t result;

	if (!stp || !ap || !prm || !wh)
		return EINVAL;

	info ("aadio: opening player (%u Hz, %d channels, device %s, "
		"ptime %u)\n", prm->srate, prm->ch, dev, prm->ptime);

	if (prm->fmt != AUFMT_S16LE) {
		warning("aaudio: player: unsupported sample format (%s)\n",
			aufmt_name((enum aufmt)prm->fmt));
		return ENOTSUP;
	}

	if (prm->ch != 1) {
		warning("aaudio: player: unsupported channel count (%u)\n",
			prm->ch);
		return ENOTSUP;
	}

	st = mem_zalloc(sizeof(*st), auplay_destructor);
	if (!st)
		return ENOMEM;

	st->play_prm = *prm;

	st->wh  = wh;
	st->arg = arg;

	result = open_player_stream(st);
	if (result != AAUDIO_OK)
		goto out;

	result = AAudioStream_requestStart(st->playerStream);
	if (result != AAUDIO_OK) {
		warning("aaudio: player: failed to start stream\n");
		goto out;
	}

	module_event("aaudio", "player sessionid", NULL, NULL, "%d",
		     AAudioStream_getSessionId(st->playerStream));

	info ("aaudio: player: stream started\n");

  out:
	if (result != AAUDIO_OK) {
		aaudio_close_stream(st->playerStream);
		mem_deref(st);
	}
	else
		*stp = st;

	return result;
}
