/**
 * @file aaudio/recorder.c  AAudio audio driver for Android
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


struct ausrc_st {
	AAudioStream *recorderStream;
	ausrc_read_h *rh;
	void *arg;
	struct ausrc_prm src_prm;
	ausrc_error_h *errh;
	void   *sampv;
	size_t  sampsz;
	size_t  sampc;
	uint64_t samps;

	/* Kallo SDK: see player.c. Set true by the destructor before closing the
	 * stream so a close-induced DISCONNECT cannot spawn a detached restart
	 * thread that reopens (and leaks) the input device into the next cycle. */
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
};


static int open_recorder_stream(struct ausrc_st *st);


static void ausrc_destructor(void *arg)
{
	struct ausrc_st *st = arg;
	pthread_t tid;
	bool join = false;

	info("aaudio: recorder: closing stream\n");

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

	aaudio_close_stream(st->recorderStream);
	st->recorderStream = NULL;

	if (st->rmtx_ok) {
		mtx_destroy(&st->smtx);
		mtx_destroy(&st->rmtx);
	}

	mem_deref(st->sampv);
	st->rh = NULL;
	st->errh = NULL;
}


/**
 * For an input stream, this function should read and process numFrames of
 * data from the audioData buffer. The data in the audioData buffer must not
 * be modified directly. Instead, it should be copied to another buffer
 * before doing any modification. Note that numFrames can vary unless
 * AAudioStreamBuilder_setFramesPerDataCallback() is called. Not currently
 * called.
 */
 static int dataCallback(AAudioStream *stream, void *userData,
			 void *audioData, int32_t numFrames) {
	(void)stream;
	struct ausrc_st *st = userData;
	struct auframe af;

	size_t sampc = 0;

	sampc = numFrames;
	if (sampc > st->sampc) {
		st->sampv = mem_realloc(st->sampv, st->sampsz * sampc);
		st->sampc = sampc;
	}

	if (!st->sampv)
		return ENOMEM;

	auframe_init(&af, st->src_prm.fmt, st->sampv, sampc,
		     st->src_prm.srate, st->src_prm.ch);

	memcpy(st->sampv, audioData, auframe_size(&af) * af.ch);

	af.timestamp = st->samps * AUDIO_TIMEBASE /
		       (st->src_prm.srate * st->src_prm.ch);
	st->samps += sampc;
	st->rh(&af, st->arg);

	return 0;
}


static void do_restart_recorder(struct ausrc_st *st)
{
	aaudio_result_t result;
	AAudioStream *old = st->recorderStream;

	/* Object is being torn down - do not touch the stream; the destructor
	 * (which is waiting to join us) closes it. */
	if (re_atomic_rlx(&st->closing))
		return;

	st->recorderStream = NULL;
	if (old)
		AAudioStream_close(old);

	/* Torn down while we were closing: stay closed, nothing to reopen. */
	if (re_atomic_rlx(&st->closing))
		return;

	result = open_recorder_stream(st);
	if (result != AAUDIO_OK) {
		warning("aaudio: failed to open recorder stream\n");
		st->recorderStream = NULL;
		return;
	}

	result = AAudioStream_requestStart(st->recorderStream);
	if (result != AAUDIO_OK)
		warning("aaudio: recorder: failed to start stream\n");
	else
		info("aaudio: recorder: stream started\n");
}


static void *restart_recorder_stream(void *data)
{
	struct ausrc_st *st = data;
	int runs = 0;

	for (;;) {
		bool again;

		mtx_lock(&st->smtx);
		do_restart_recorder(st);
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
	struct ausrc_st *st = userData;
	(void)error;
	int res;

	/* Ignore disconnects once the object is being destroyed. */
	if (re_atomic_rlx(&st->closing))
		return;

	aaudio_stream_state_t streamState = AAudioStream_getState(stream);
	if (streamState != AAUDIO_STREAM_STATE_DISCONNECTED)
		return;

	info("aaudio: recorder: stream disconnected\n");

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

	res = pthread_create(&st->restart_tid, NULL, restart_recorder_stream,
			     (void *)st);
	if (res) {
		warning("aaudio: recorder: error creating thread: %d\n", res);
	}
	else {
		st->restart_run  = true;
		st->restart_busy = true;
		info("aaudio: recorder: created restart thread\n");
	}
	mtx_unlock(&st->rmtx);
}


static int open_recorder_stream(struct ausrc_st *st) {

	AAudioStreamBuilder *builder;
	aaudio_result_t result;

	result = AAudio_createStreamBuilder(&builder);
	if (result != AAUDIO_OK) {
		warning("aaudio: recorder: failed to create stream builder: "
			"error %s\n", AAudio_convertResultToText(result));
		return result;
	}

	AAudioStreamBuilder_setDirection(builder, AAUDIO_DIRECTION_INPUT);
	AAudioStreamBuilder_setSharingMode(builder,
		AAUDIO_SHARING_MODE_SHARED);
	AAudioStreamBuilder_setSampleRate(builder, st->src_prm.srate);
	AAudioStreamBuilder_setChannelCount(builder, st->src_prm.ch);
	AAudioStreamBuilder_setFormat(builder, AAUDIO_FORMAT_PCM_I16);
	AAudioStreamBuilder_setSessionId(builder, AAUDIO_SESSION_ID_ALLOCATE);
	AAudioStreamBuilder_setUsage(builder,
		AAUDIO_USAGE_VOICE_COMMUNICATION);
	AAudioStreamBuilder_setPerformanceMode(builder,
		AAUDIO_PERFORMANCE_MODE_LOW_LATENCY);
	AAudioStreamBuilder_setInputPreset(builder,
		AAUDIO_INPUT_PRESET_VOICE_COMMUNICATION);
	AAudioStreamBuilder_setDataCallback(builder, &dataCallback, st);
	AAudioStreamBuilder_setErrorCallback(builder, &errorCallback, st);

	result = AAudioStreamBuilder_openStream(builder, &st->recorderStream);
	if (result != AAUDIO_OK) {
		warning("aaudio: recorder: failed to open stream: error %s\n",
			AAudio_convertResultToText(result));
		st->recorderStream = NULL;
		return result;
	}

	info("aaudio: recorder: opened stream with direction %d, "
	     "sharing mode %d, sample rate %d, format %d, sessionId %d, "
	     "input preset %d, usage %d, performance mode %d\n",
	     AAudioStream_getDirection(st->recorderStream),
	     AAudioStream_getSharingMode(st->recorderStream),
	     AAudioStream_getSampleRate(st->recorderStream),
	     AAudioStream_getFormat(st->recorderStream),
	     AAudioStream_getSessionId(st->recorderStream),
	     AAudioStream_getInputPreset(st->recorderStream),
	     AAudioStream_getUsage(st->recorderStream),
	     AAudioStream_getPerformanceMode(st->recorderStream));

	AAudioStreamBuilder_delete(builder);

	/* Kallo SDK (Lever B): shared burst multiplier, default 2 == historical
	 * behaviour. See aaudio.h. */
	int32_t burst  = AAudioStream_getFramesPerBurst(st->recorderStream);
	int32_t bursts = aaudio_get_buffer_bursts();
	AAudioStream_setBufferSizeInFrames(st->recorderStream, burst * bursts);
	int32_t bufferCapacity =
		AAudioStream_getBufferCapacityInFrames(st->recorderStream);
	int32_t bufferSize = AAudioStream_getBufferSizeInFrames(
		st->recorderStream);
	info("aaudio: recorder: buffer capacity: %d, buffer size: %d\n",
	     bufferCapacity,  bufferSize);

	/* Kallo SDK audio-quality instrumentation — uplink twin of the player's
	 * STREAM-PARAMS line. Requested rate is the codec rate unless
	 * audio.srate_src decouples it (Lever A). */
	info("aaudio: STREAM-PARAMS recorder req_srate=%u act_srate=%d "
	     "burst=%d bursts=%d bufsize=%d bufcap=%d perfmode=%d deviceId=%d\n",
	     st->src_prm.srate,
	     AAudioStream_getSampleRate(st->recorderStream),
	     burst, bursts, bufferSize, bufferCapacity,
	     AAudioStream_getPerformanceMode(st->recorderStream),
	     AAudioStream_getDeviceId(st->recorderStream));

	return AAUDIO_OK;
}


int aaudio_recorder_alloc(struct ausrc_st **stp, const struct ausrc *as,
			  struct ausrc_prm *prm, const char *dev,
			  ausrc_read_h *rh, ausrc_error_h *errh, void *arg)
{
	struct ausrc_st *st;
	aaudio_result_t result;
	int32_t sid = 0;

	if (!stp || !as || !prm || !rh)
		return EINVAL;

	info ("aaudio: recorder: opening recorder(%u Hz, %d channels,"
	      "device '%s')\n", prm->srate, prm->ch, dev);

	if (prm->fmt != AUFMT_S16LE) {
		warning("aaudio: recorder: unsupported sample format (%s)\n",
			aufmt_name((enum aufmt)prm->fmt));
		return ENOTSUP;
	}

	if (prm->ch != 1) {
		warning("aaudio: recorder: unsupported channel count (%u)\n",
			prm->ch);
		return ENOTSUP;
	}

	st = mem_zalloc(sizeof(*st), ausrc_destructor);
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

	st->src_prm = *prm;

	st->sampsz = aufmt_sample_size(prm->fmt);
	st->sampc  = prm->ptime * prm->ch * prm->srate / 1000;
	st->samps  = 0;
	st->sampv  = mem_zalloc(st->sampsz * st->sampc, NULL);
	if (!st->sampv) {
		result = ENOMEM;
		goto out;
	}

	st->rh  = rh;
	st->errh = errh;
	st->arg = arg;

	/* smtx: see player.c. */
	mtx_lock(&st->smtx);
	result = open_recorder_stream(st);
	if (result == AAUDIO_OK) {
		result = AAudioStream_requestStart(st->recorderStream);
		if (result != AAUDIO_OK)
			warning("aaudio: recorder: failed to start stream\n");
		else
			sid = AAudioStream_getSessionId(st->recorderStream);
	}
	mtx_unlock(&st->smtx);
	if (result != AAUDIO_OK)
		goto out;

	module_event("aaudio", "recorder sessionid", NULL, NULL, "%d", sid);

	info("aaudio: recorder: stream started\n");

  out:
	if (result != AAUDIO_OK) {
		/* The destructor closes the stream (see player.c). */
		mem_deref(st);
	}
	else
		*stp = st;

	return result;
}
