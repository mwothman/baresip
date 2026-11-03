/**
 * @file aaudio/player.c  AAudio audio driver for Android
 *
 * Copyright (C) 2024 Juha Heinanen
 * Copyright (C) 2024 Sebastian Reimers
 */

#include <re.h>
#include <re_atomic.h>
#include <rem.h>
#include <baresip.h>
#include <string.h>

#include "aaudio.h"


struct auplay_st {
	AAudioStream *playerStream;
	auplay_write_h *wh;
	void *arg;
	struct auplay_prm play_prm;
	size_t sampsz;

	/* Kallo SDK: set true by the destructor before the stream is closed.
	 * Closing an AAudio stream can itself fire errorCallback with state
	 * DISCONNECTED, which would otherwise spawn restart_player_stream() and
	 * reopen a fresh device stream on a detached thread just as this object
	 * is being freed — leaking a live AAudio output stream that holds the
	 * audio device into the next init() cycle (the "every other call has a
	 * dead/silent downlink" idempotency bug). Both the error callback and the
	 * restart thread bail out when this is set. */
	RE_ATOMIC bool closing;

	/* Kallo SDK (route-race fix): the disconnect->reopen restart runs on its
	 * own thread. It used to be fire-and-forget: nothing joined it, nothing
	 * stopped it racing the destructor (rebind / dispose / call end) over the
	 * same AAudioStream pointer, and it kept using the object after it was
	 * freed. Now: at most one restart thread per object; it is joinable and
	 * the destructor joins it BEFORE touching the stream. rmtx only guards
	 * the fields below and is never held across a blocking call. smtx serialises
	 * "open + start the stream" in alloc against the restart thread's
	 * close/open, so a disconnect that fires before alloc has finished cannot
	 * pull the stream out from under it. */
	mtx_t     rmtx;
	mtx_t     smtx;             /**< open/start vs. restart exclusion   */
	bool      rmtx_ok;
	bool      restart_run;      /**< restart_tid valid, not yet joined  */
	bool      restart_busy;     /**< a restart thread is running         */
	bool      restart_again;    /**< disconnect seen while it was busy   */
	pthread_t restart_tid;

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
	int32_t  dbg_xruns_last; /**< last sampled cumulative XRun count  */
};

/* Sample magnitude below this (out of 32767) is treated as silence. PCMU
 * idle/comfort noise and zero-fill underruns sit well under this; speech /
 * a test tone sits far above it. */
#define KALLO_SILENCE_THRESH 64


static int open_player_stream(struct auplay_st *st);


static void auplay_destructor(void *arg)
{
	struct auplay_st *st = arg;
	pthread_t tid;
	bool join = false;

	info("aaudio: player: closing stream\n");

	memset(&tid, 0, sizeof(tid));

	/* Prevent a close-induced DISCONNECT from resurrecting the stream, and
	 * take ownership of the restart thread (if any) so nobody else joins
	 * it. Once closing is set under rmtx no new restart thread can start. */
	if (st->rmtx_ok) {
		mtx_lock(&st->rmtx);
		re_atomic_rlx_set(&st->closing, true);
		join = st->restart_run;
		tid  = st->restart_tid;
		st->restart_run = false;
		mtx_unlock(&st->rmtx);
	}
	else {
		re_atomic_rlx_set(&st->closing, true);
	}

	/* Wait for an in-flight restart (close old / open new) to finish BEFORE
	 * touching the stream: this is what makes close, open and restart
	 * mutually exclusive. The restart thread takes no libre/baresip lock,
	 * so waiting here (under the caller's re lock) cannot deadlock. */
	if (join)
		pthread_join(tid, NULL);

	aaudio_close_stream(st->playerStream);
	st->playerStream = NULL;

	if (st->rmtx_ok) {
		mtx_destroy(&st->smtx);
		mtx_destroy(&st->rmtx);
	}

	st->wh = NULL;
}


/**
 * For an output stream, this function should render and write from
 * userData numFrames of data in the streams current data format to
 * the audioData buffer.
 */
 static int dataCallback(AAudioStream *stream, void *userData,
			 void *audioData, int32_t numFrames) {
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

		/* Kallo SDK audio-quality instrumentation: sample the device's
		 * own underrun counter on the same cadence. xruns is cumulative
		 * since the stream opened, so d_xruns (the per-interval delta)
		 * is the one to watch — a non-zero delta during the complaint
		 * means the AAudio buffer is starving, which sounds ROUGH or
		 * crackly. It staying 0 while audio still sounds bad points at
		 * the signal path (rate conversion / narrowband) instead. */
		int32_t xruns = AAudioStream_getXRunCount(stream);
		int32_t d_xruns = xruns - st->dbg_xruns_last;
		st->dbg_xruns_last = xruns;

		/* One greppable line per second. "SILENT" marks a gap. */
		info("aaudio: PLAYOUT t=%us frames=%llu peak=%d active=%u%% "
		     "total=%llu silent_ivs=%llu xruns=%d d_xruns=%d%s\n",
		     st->dbg_iv,
		     (unsigned long long)st->dbg_frames,
		     (int)st->dbg_peak, act,
		     (unsigned long long)st->dbg_total_fr,
		     (unsigned long long)st->dbg_silent_iv,
		     (int)xruns, (int)d_xruns,
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


static void do_restart_player(struct auplay_st *st)
{
	aaudio_result_t result;
	AAudioStream *old = st->playerStream;

	/* Object is being torn down - do not touch the stream; the destructor
	 * (which is waiting to join us) closes it. */
	if (re_atomic_rlx(&st->closing))
		return;

	st->playerStream = NULL;
	if (old)
		AAudioStream_close(old);

	/* Torn down while we were closing: stay closed, nothing to reopen. */
	if (re_atomic_rlx(&st->closing))
		return;

	result = open_player_stream(st);
	if (result != AAUDIO_OK) {
		warning("aaudio: failed to open player stream\n");
		st->playerStream = NULL;
		return;
	}

	result = AAudioStream_requestStart(st->playerStream);
	if (result != AAUDIO_OK)
		warning("aaudio: player: failed to start stream\n");
	else
		info("aaudio: player: stream started\n");
}


static void *restart_player_stream(void *data)
{
	struct auplay_st *st = data;
	int runs = 0;

	for (;;) {
		bool again;

		mtx_lock(&st->smtx);
		do_restart_player(st);
		mtx_unlock(&st->smtx);

		/* A disconnect that arrived while we were busy (the freshly
		 * opened stream dropped again) is folded in here, bounded, so
		 * it is neither lost nor turned into a second concurrent
		 * thread. restart_busy is cleared under the same lock
		 * errorCallback decides under, so exactly one of the two sees
		 * the event. */
		mtx_lock(&st->rmtx);
		again = st->restart_again && !re_atomic_rlx(&st->closing) &&
			++runs < 3;
		st->restart_again = false;
		if (!again)
			st->restart_busy = false;
		mtx_unlock(&st->rmtx);

		if (!again)
			break;
	}

	return NULL;
}


static void errorCallback(AAudioStream *stream, void *userData,
			  aaudio_result_t error) {
	struct auplay_st *st = userData;
	(void)error;
	int res;

	/* Ignore disconnects once the object is being destroyed. */
	if (re_atomic_rlx(&st->closing))
		return;

	aaudio_stream_state_t streamState = AAudioStream_getState(stream);
	if (streamState != AAUDIO_STREAM_STATE_DISCONNECTED)
		return;

	info("aaudio: player: stream disconnected\n");

	/* This runs on the stream's own callback thread, which
	 * AAudioStream_close() joins - so it must never wait for anything the
	 * closer holds. rmtx is only ever held for flag updates and for
	 * reaping a thread that has already finished, never across a
	 * close/open or a join of a live thread. */
	mtx_lock(&st->rmtx);
	if (re_atomic_rlx(&st->closing)) {
		mtx_unlock(&st->rmtx);
		return;
	}

	if (st->restart_busy) {
		/* The running restart thread re-checks this before it exits. */
		st->restart_again = true;
		mtx_unlock(&st->rmtx);
		return;
	}

	/* The previous restart thread has finished (restart_busy is cleared as
	 * its last locked act): reap it so its resources are released. */
	if (st->restart_run) {
		pthread_join(st->restart_tid, NULL);
		st->restart_run = false;
	}

	res = pthread_create(&st->restart_tid, NULL, restart_player_stream,
			     (void *)st);
	if (res) {
		warning("aaudio: player: error creating thread: %d\n", res);
	}
	else {
		st->restart_run  = true;
		st->restart_busy = true;
		info("aaudio: player: created restart thread\n");
	}
	mtx_unlock(&st->rmtx);
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

	/* Kallo SDK: pin the output to a concrete device when one was selected.
	 * Without this the USAGE_VOICE_COMMUNICATION stream binds to the OS default
	 * comm device, which auto-prefers a connected Bluetooth headset — so an
	 * Earpiece/Speaker selection on the Kotlin side (setCommunicationDevice)
	 * only flips the indicator while audio stays in the buds. The id is the
	 * Android AudioDeviceInfo.getId(); 0 (AAUDIO_UNSPECIFIED) restores AAudio's
	 * own routing. This also covers the restart_player_stream() disconnect→reopen
	 * path (same function), so the pin survives a mid-call reconnect. */
	int32_t want_id = aaudio_get_output_device_id();
	if (want_id != AAUDIO_UNSPECIFIED)
		AAudioStreamBuilder_setDeviceId(builder, want_id);

	result = AAudioStreamBuilder_openStream(builder, &st->playerStream);
	if (result != AAUDIO_OK && want_id != AAUDIO_UNSPECIFIED) {
		/* The pinned device id was rejected — e.g. AudioDeviceInfo.getId() did
		 * not map to a valid AAudio output device id on this OEM. Never hard-fail
		 * the open: retry unspecified so the stream still opens. The Kotlin-side
		 * setCommunicationDevice() stays in effect; the requested-vs-actual log
		 * below makes the divergence obvious. */
		warning("aaudio: player: openStream rejected deviceId=%d (%s); "
			"retrying unspecified\n", want_id,
			AAudio_convertResultToText(result));
		AAudioStreamBuilder_setDeviceId(builder, AAUDIO_UNSPECIFIED);
		want_id = AAUDIO_UNSPECIFIED;
		result = AAudioStreamBuilder_openStream(builder, &st->playerStream);
	}
	if (result != AAUDIO_OK) {
		warning("aaudio: player: failed to open stream: error %s\n",
			AAudio_convertResultToText(result));
		st->playerStream = NULL;
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

	/* Kallo SDK: verify the pin took. If requested != actual (and requested was
	 * not unspecified), AudioDeviceInfo.getId() and the AAudio device id space
	 * diverge on this device — audio may not land where selected even though the
	 * open "succeeded". This is the line to check on the on-device route test. */
	info("aaudio: player: deviceId pin requested=%d (0=unspecified) actual=%d\n",
	     want_id, AAudioStream_getDeviceId(st->playerStream));

	AAudioStreamBuilder_delete(builder);

	/* Kallo SDK (Lever B): buffer size in framesPerBurst multiples. Default 2
	 * is the historical value; a larger multiple trades playout latency for
	 * underrun headroom on the Bluetooth SCO path. See aaudio.h. */
	int32_t burst   = AAudioStream_getFramesPerBurst(st->playerStream);
	int32_t bursts  = aaudio_get_buffer_bursts();
	AAudioStream_setBufferSizeInFrames(st->playerStream, burst * bursts);

	/* Fresh stream — its XRun counter restarts at 0, so re-base the delta
	 * (this path also runs on the disconnect→reopen restart). */
	st->dbg_xruns_last = 0;

	/* ── Kallo SDK audio-quality instrumentation (STREAM-PARAMS) ─────────
	 * One greppable line stating what we ASKED AAudio for versus what it
	 * actually opened. The requested rate is the codec rate unless
	 * audio.srate_play decouples it (Lever A), and a requested/actual
	 * mismatch — or an 8000 Hz stream on a 48 kHz device — is the single
	 * fact that decides whether the framework is resampling underneath us. */
	info("aaudio: STREAM-PARAMS player req_srate=%u act_srate=%d "
	     "burst=%d bursts=%d bufsize=%d bufcap=%d perfmode=%d deviceId=%d\n",
	     st->play_prm.srate,
	     AAudioStream_getSampleRate(st->playerStream),
	     burst, bursts,
	     AAudioStream_getBufferSizeInFrames(st->playerStream),
	     AAudioStream_getBufferCapacityInFrames(st->playerStream),
	     AAudioStream_getPerformanceMode(st->playerStream),
	     AAudioStream_getDeviceId(st->playerStream));

	return AAUDIO_OK;
}


int aaudio_player_alloc(struct auplay_st **stp, const struct auplay *ap,
	struct auplay_prm *prm, const char *dev, auplay_write_h *wh, void *arg)
{
	struct auplay_st *st;
	aaudio_result_t result;
	int32_t sid = 0;

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

	if (mtx_init(&st->rmtx, mtx_plain) != thrd_success) {
		mem_deref(st);
		return ENOMEM;
	}
	if (mtx_init(&st->smtx, mtx_plain) != thrd_success) {
		mtx_destroy(&st->rmtx);
		mem_deref(st);
		return ENOMEM;
	}
	st->rmtx_ok = true;

	st->play_prm = *prm;

	st->wh  = wh;
	st->arg = arg;

	/* smtx: a disconnect that fires before we are done here must not let
	 * the restart thread close/replace the stream under us. */
	mtx_lock(&st->smtx);
	result = open_player_stream(st);
	if (result == AAUDIO_OK) {
		result = AAudioStream_requestStart(st->playerStream);
		if (result != AAUDIO_OK)
			warning("aaudio: player: failed to start stream\n");
		else
			sid = AAudioStream_getSessionId(st->playerStream);
	}
	mtx_unlock(&st->smtx);
	if (result != AAUDIO_OK)
		goto out;

	module_event("aaudio", "player sessionid", NULL, NULL, "%d", sid);

	info ("aaudio: player: stream started\n");

  out:
	if (result != AAUDIO_OK) {
		/* The destructor closes the stream; closing it here as well
		 * double-closed it on the requestStart-failure path. */
		mem_deref(st);
	}
	else
		*stp = st;

	return result;
}
