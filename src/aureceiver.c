/**
 * @file src/aureceiver.c  Audio stream receiver
 *
 * Copyright (C) 2023 Alfred E. Heggestad, Christian Spielberger
 */
#include <string.h>
#include <stdlib.h>
#include <re.h>
#include <re_atomic.h>
#include <rem.h>
#include <baresip.h>
#include "core.h"


/**
 * Audio receive pipeline
 *
 \verbatim

 Processing decoder pipeline:

       .--------.   .-------.   .--------.   .--------.
 |\    |        |   |       |   |        |   |        |
 | |<--| auplay |<--| aubuf |<--| aufilt |<--| decode |<--- RTP
 |/    |        |   |       |   |        |   |        |
       '--------'   '-------'   '--------'   '--------'

 \endverbatim
 */

enum {
	JITTER_EMA_COEFF   = 128,     /**< Jitter EMA coefficient            */
};


struct audio_recv {
	uint32_t srate;               /**< Decoder sample rate               */
	uint32_t ch;                  /**< Decoder channel number            */
	enum aufmt fmt;               /**< Decoder sample format             */
	const struct config_audio *cfg;  /**< Audio configuration            */
	struct audec_state *dec;      /**< Audio decoder state (optional)    */
	const struct aucodec *ac;     /**< Current audio decoder             */
	struct aubuf *aubuf;          /**< Audio buffer before auplay        */
	mtx_t *aubuf_mtx;             /**< Mutex for aubuf allocation        */
	uint32_t ssrc;                /**< Incoming synchronization source   */
	struct list filtl;            /**< Audio filters in decoding order   */
	void *sampv;                  /**< Sample buffer                     */
	size_t sampvsz;               /**< Sample buffer size                */
	uint64_t t;                   /**< Last auframe push time            */
	uint32_t ptime;               /**< Packet time for receiving [us]    */

	double level_last;            /**< Last audio level value [dBov]     */
	bool level_set;               /**< True if level_last is set         */
	struct timestamp_recv ts_recv;/**< Receive timestamp state           */
	uint8_t extmap_aulevel;       /**< ID Range 1-14 inclusive           */
	int pt;                       /**< Payload type of audio codec       */

	struct {
		uint64_t n_discard;   /**< Nbr of discarded packets          */
		RE_ATOMIC uint64_t latency;   /**< Latency in [ms]           */
		int32_t jitter;       /**< Auframe push jitter [us]          */
		int32_t dmax;         /**< Max deviation [us]                */
	} stats;

	mtx_t *mtx;

	const struct auplay *ap;      /**< Audio player module               */
	struct auplay_st *auplay;     /**< Audio player                      */
	struct auplay_prm auplay_prm; /**< Audio player parameters           */
	char *module;                 /**< Audio player module name          */
	char *device;                 /**< Audio player device name          */
	enum aufmt play_fmt;          /**< Sample format for audio playback  */
	bool done_first;              /**< First auplay write done flag      */

	/* Kallo SDK middle-link telemetry (Issue 1). The 0.1.7 trace had
	 * "rtprecv: DECODE" (jbuf, upstream of the aubuf) and "aaudio: PLAYOUT"
	 * (player, downstream of the aubuf) but nothing on the aubuf itself, so a
	 * "DECODE ok>0 / PLAYOUT peak=0" stall could not be localised between the
	 * two. Log, once per ~1s, frames pushed into the aubuf and its current
	 * fill: decode reaching the aubuf (push>0, fill_ms>0) while PLAYOUT stays
	 * silent means the player is bound to a different/stale buffer. */
	uint64_t dbg_push;            /**< auframes pushed this interval     */
	uint64_t dbg_push_tot;        /**< auframes pushed since alloc       */
	uint64_t dbg_next_log;        /**< next telemetry log time [jiffies] */
	uint32_t dbg_iv;              /**< interval index (≈ seconds)        */

	/* Kallo SDK 0.1.10 — receive-aubuf IDENTITY proof + self-heal.
	 *
	 * The decisive 0.1.9 field report (3CX/PCMU/UDP) was: the AAudio data
	 * callback keeps firing the whole call (PLAYOUT frames≈8004/s) yet outputs
	 * pure silence (peak=0) while the receive buffer fills to and pins at the
	 * 160 ms cap (AUBUF fill_ms=160) with DECODE/push fully healthy. A firing
	 * player + a buffer nothing drains can only mean the side that DRAINS and
	 * the side that FILLS are not looking at the same aubuf object — an
	 * identity divergence, not a contents problem (0.1.7–0.1.9 reasoned about
	 * contents and failed). The player binds `ar` (its arg) and reads
	 * `ar->aubuf`; the RTP-receive path pushes into `ar->aubuf`. To settle it
	 * in the field with zero ambiguity we log the ACTUAL pointers on BOTH
	 * sides (player read vs rx push) so the next trace can confirm they match.
	 *
	 * play_frames / play_peak are updated by the player pull (auplay_write_
	 * handler, an AAudio-thread context) and sampled by the rx push path to
	 * drive the self-heal guard: if the player is firing but outputs silence
	 * while the buffer is pinned at the cap for >~500 ms, request a player
	 * re-bind to the current receive aubuf. The request is consumed at the top
	 * of aurecv_receive (no ar->mtx held there) — never from inside the hot
	 * path under the lock — so it cannot deadlock or fight the normal path. */
	RE_ATOMIC uint64_t play_frames;  /**< frames the player has pulled       */
	RE_ATOMIC int32_t  play_peak;    /**< peak |sample| since last guard eval */
	RE_ATOMIC bool     heal_request; /**< deferred player-rebind request      */
	uint64_t play_frames_last;    /**< push-side snapshot of play_frames  */
	uint64_t heal_since;          /**< jiffies the stall was first seen   */
	uint64_t dbg_rd_next_log;     /**< next player-read ptr log [jiffies] */

	/* Kallo SDK 0.1.11 — self-heal DEBOUNCE + read-point instrumentation.
	 *
	 * 0.1.10's guard fixed the buffer identity but introduced a thrash
	 * regression: it judged a freshly opened AAudio player STALLED after only
	 * KALLO_HEAL_STALL_MS (500 ms) — shorter than the time the player needs to
	 * prime and report a real peak. So on every fresh call / resume the guard
	 * tripped ~600 ms after the player opened, tore it down and reopened it,
	 * tripped again on the next freshly opened (still priming) player, and
	 * looped forever (~40 reopens in 37 s in the field). With the player never
	 * allowed to run, it never drained the shared aubuf — fill stayed pinned at
	 * the 160 ms cap with peak=0: the SAME end symptom as the identity bug, via
	 * a different mechanism.
	 *
	 * Fix: (a) a grace window after ANY player open during which the guard must
	 * NOT trigger (play_open_at + KALLO_HEAL_GRACE_MS), so a priming player is
	 * never judged stalled; (b) a per-call retry cap (heal_attempts vs
	 * KALLO_HEAL_MAX_ATTEMPTS) so the guard can never become a continuous
	 * teardown loop — after N real stalls it logs and falls back; (c) the stall
	 * timer (heal_since) must still persist KALLO_HEAL_STALL_MS *after* the grace
	 * window before a rebind, and any healthy (non-silent / unpinned) window
	 * resets both the timer and the retry budget. Result: at most a handful of
	 * rebinds per genuinely stalled call, none while a player is priming. */
	RE_ATOMIC uint64_t play_open_at; /**< jiffies of last player open        */
	uint32_t heal_attempts;          /**< self-heal rebinds this call        */
	uint64_t dbg_rdpt_next_log;      /**< next aubuf-read-point log [jiffies] */
};

/* Sample magnitude (out of 32767) below which playout is treated as silence —
 * matches aaudio/player.c. PCMU comfort noise / zero-fill underruns sit under
 * it; speech / a test tone sits far above. */
#define KALLO_SILENCE_THRESH 64

/* Consecutive stall duration before the self-heal guard re-binds the player. */
#define KALLO_HEAL_STALL_MS 500

/* Kallo SDK 0.1.11 — grace window after any player open during which the
 * self-heal guard must NOT trigger. A freshly opened AAudio low-latency stream
 * needs time to prime and report a real peak (the field showed the first real
 * playout ~600 ms after open); judging it stalled before that caused the
 * 0.1.10 teardown loop. Must comfortably exceed the player prime time. */
#define KALLO_HEAL_GRACE_MS 1500

/* Max self-heal rebinds per call. Once exhausted the guard stops re-binding
 * (logs a fallback) instead of looping — it can never become a continuous
 * teardown loop. Reset on a healthy window and on a fresh leg (resume). */
#define KALLO_HEAL_MAX_ATTEMPTS 3


static void destructor(void *arg)
{
	struct audio_recv *ar = arg;

	mem_deref(ar->dec);
	mem_deref(ar->aubuf);
	mem_deref(ar->aubuf_mtx);
	mem_deref(ar->sampv);
	mem_deref(ar->mtx);
	list_flush(&ar->filtl);
	mem_deref(ar->module);
	mem_deref(ar->device);
}


static int aurecv_process_decfilt(struct audio_recv *ar, struct auframe *af)
{
	int err = 0;

	/* Process exactly one audio-frame in reverse list order */
	for (struct le *le = ar->filtl.tail; le; le = le->prev) {
		struct aufilt_dec_st *st = le->data;

		if (st->af && st->af->dech)
			err = st->af->dech(st, af);

		if (err)
			break;
	}

	return err;
}


static double aurecv_calc_seconds(const struct audio_recv *ar)
{
	uint64_t dur;
	double seconds;

	if (!ar->ac)
		return .0;

	dur = timestamp_duration(&ar->ts_recv);
	seconds = timestamp_calc_seconds(dur, ar->ac->crate);

	return seconds;
}


static int aurecv_alloc_aubuf(struct audio_recv *ar, uint32_t srate,
			      uint8_t ch)
{
	size_t min_sz;
	size_t max_sz;
	size_t sz;
	const struct config_audio *cfg = ar->cfg;
	int err;

	sz = aufmt_sample_size(cfg->play_fmt);
	min_sz = sz * au_calc_nsamp(srate, ch, cfg->buffer.min);
	max_sz = sz * au_calc_nsamp(srate, ch, cfg->buffer.max);

	debug("audio_recv: create audio buffer"
	      " [%u - %u ms]"
	      " [%zu - %zu bytes]\n",
	      (unsigned) cfg->buffer.min, (unsigned) cfg->buffer.max,
	      min_sz, max_sz);

	mtx_lock(ar->aubuf_mtx);
	err = aubuf_alloc(&ar->aubuf, min_sz, max_sz);
	if (err) {
		warning("audio_recv: aubuf alloc error (%m)\n", err);
		goto out;
	}

	/*
	 * Kallo SDK 0.1.13 — DO NOT call aubuf_set_id() here.
	 *
	 * Root cause of the residual 3CX downlink silence: aubuf_set_id() is
	 * defined ONLY in libre's aubuf TU (native/libre/rem/aubuf/aubuf.c),
	 * whose `struct aubuf` carries two extra leading pointers (`pool`, `id`)
	 * before `lock`, so its `id` field sits at byte offset 24. This buffer,
	 * however, is allocated by librem's aubuf_alloc() (the copy that wins the
	 * link order `rem` before `re` under --allow-multiple-definition), whose
	 * `struct aubuf` has no such fields — offset 24 is `wish_sz`. The
	 * cross-TU `ab->id = mem_ref(id)` therefore wrote the heap `id` pointer
	 * straight into the librem buffer's `wish_sz` (observed on-device as
	 * wish_sz≈1.3e19 = 0xB400…), and the read path re-armed fill_sz=wish_sz,
	 * corrupting both gate thresholds. With mode=fixed the gate flips to
	 * drain only once fill reaches wish_sz, but cur_sz caps at max_sz=2560
	 * while wish_sz≈1.3e19 — unreachable, so the playout aubuf stayed in
	 * `filling` forever and fed the player silence. The TX/mic aubuf never
	 * calls aubuf_set_id(), which is exactly why it worked.
	 *
	 * The id was a diagnostic label only; it is NOT what guarantees the
	 * 0.1.10 shared-buffer identity (that is the `ar->aubuf` pointer itself).
	 * Dropping the call leaves wish_sz = min_sz (320 B = 20 ms @ 8 kHz mono
	 * S16, ≤ max_sz), mirroring the working TX side. The GATE/AUBUF-READ
	 * instrumentation is retained.
	 */

	aubuf_set_mode(ar->aubuf, cfg->adaptive ?
		       AUBUF_ADAPTIVE : AUBUF_FIXED);
	aubuf_set_silence(ar->aubuf, cfg->silence);

out:
	mtx_unlock(ar->aubuf_mtx);

	return err;
}


static int aurecv_push_aubuf(struct audio_recv *ar, const struct auframe *af)
{
	int err;
	uint64_t bpms;

	if (!ar->aubuf) {
		err = aurecv_alloc_aubuf(ar, af->srate, af->ch);
		if (err)
			return err;
	}

#ifndef RELEASE
	int32_t d, da;
	uint64_t t;
	t = tmr_jiffies_usec();
	if (ar->t) {
		d = (int32_t) (int64_t) ((t - ar->t) - ar->ptime);
		da = abs(d);
		ar->stats.dmax = max(ar->stats.dmax, da);
		ar->stats.jitter += (da - ar->stats.jitter) / JITTER_EMA_COEFF;
	}

	ar->t = t;
#endif
	err = aubuf_write_auframe(ar->aubuf, af);
	if (err)
		return err;

	ar->srate = af->srate;
	ar->ch    = af->ch;
	ar->fmt   = af->fmt;

	bpms = (uint64_t)ar->srate * ar->ch * aufmt_sample_size(ar->fmt) /
	       1000;
	if (bpms)
		re_atomic_rlx_set(&ar->stats.latency,
				  aubuf_cur_size(ar->aubuf) / bpms);

	/* ── Kallo SDK aubuf middle-link telemetry (Issue 1) ────────────────────
	 * One greppable "aureceiver: AUBUF" line per ~1s, sitting between
	 * "rtprecv: DECODE" (jbuf) and "aaudio: PLAYOUT" (player). push>0 with
	 * fill_ms>0 while PLAYOUT peak stays 0 localises a hold/resume downlink
	 * stall to the aubuf↔player binding (the player is not draining THIS
	 * buffer); push>0 with fill_ms≈0 and PLAYOUT silent points downstream. */
	++ar->dbg_push;
	++ar->dbg_push_tot;
	{
		const uint64_t now = tmr_jiffies();
		uint32_t fill_ms = bpms ?
			(uint32_t)(aubuf_cur_size(ar->aubuf) / bpms) : 0;

		/* ── Kallo SDK 0.1.10 self-heal guard ───────────────────────────────
		 * Evaluated every push (rx side, ≈50/s) — cheap, and exactly when the
		 * buffer is being filled. If the player is FIRING (play_frames moving)
		 * yet outputs only silence (peak<=thresh) while the buffer is pinned
		 * near the cap, the draining side is not seeing what we fill — request
		 * a player re-bind. We sample then zero play_peak so it reflects only
		 * the ~20 ms since the previous push; a single non-silent window resets
		 * the stall timer, so a healthy call (or normal speech gaps) never
		 * trips it. The actual re-bind happens at the top of aurecv_receive,
		 * where ar->mtx is NOT held (calling aurecv_start_player here would
		 * re-lock ar->mtx and deadlock). Idempotent: the request is consumed
		 * once; if still broken it re-arms KALLO_HEAL_STALL_MS later. */
		const uint32_t cap_ms = ar->cfg->buffer.max;
		uint64_t pf = re_atomic_rlx(&ar->play_frames);
		int32_t  pk = re_atomic_rlx(&ar->play_peak);
		re_atomic_rlx_set(&ar->play_peak, 0);
		bool player_firing = (pf != ar->play_frames_last);
		ar->play_frames_last = pf;
		bool pinned = cap_ms && fill_ms >= (cap_ms * 7) / 8;
		bool silent = pk <= KALLO_SILENCE_THRESH;

		/* Kallo SDK 0.1.11 — debounce. Don't judge a player that is still
		 * priming: skip the whole eval while inside the grace window after
		 * the last open. Outside the window, only a stall that persists
		 * KALLO_HEAL_STALL_MS *and* a remaining retry budget triggers a
		 * rebind; any healthy window clears the timer and refills the budget,
		 * and an exhausted budget falls back (logs once) instead of looping. */
		uint64_t opened = re_atomic_rlx(&ar->play_open_at);
		bool in_grace = opened &&
			(int64_t)(now - opened) < KALLO_HEAL_GRACE_MS;
		bool stalling = player_firing && pinned && silent;

		if (in_grace) {
			/* priming — never trip, but don't reset the budget either */
			ar->heal_since = 0;
		}
		else if (stalling && ar->heal_attempts < KALLO_HEAL_MAX_ATTEMPTS) {
			if (!ar->heal_since)
				ar->heal_since = now;
			else if ((int64_t)(now - ar->heal_since) >=
				 KALLO_HEAL_STALL_MS) {
				++ar->heal_attempts;
				warning("aurecv: SELF-HEAL trigger ar=%p "
					"aubuf=%p fill_ms=%u peak=%d attempt=%u/%u"
					" — player firing but draining silence; "
					"requesting player re-bind\n",
					(void *)ar, (void *)ar->aubuf,
					fill_ms, pk, ar->heal_attempts,
					(unsigned)KALLO_HEAL_MAX_ATTEMPTS);
				re_atomic_rlx_set(&ar->heal_request, true);
				ar->heal_since = 0;
			}
		}
		else if (stalling) {
			/* budget exhausted (heal_attempts == MAX) — stop thrashing:
			 * the safety net stays disarmed (the last "attempt=N/N"
			 * trigger above is the marker) until a healthy window or a
			 * fresh leg (resume) refills the budget. Never loops. */
			ar->heal_since = 0;
		}
		else {
			/* healthy / not pinned — recovered: clear timer and
			 * refill the retry budget for the rest of the call */
			ar->heal_since    = 0;
			ar->heal_attempts = 0;
		}

		if (!ar->dbg_next_log)
			ar->dbg_next_log = now + 1000;
		else if ((int64_t)(now - ar->dbg_next_log) >= 0) {
			/* Greppable identity proof: the aubuf the rx fills.
			 * Must equal the "player read" pointer logged from the
			 * player pull side (auplay_write_handler). */
			info("aureceiver: AUBUF t=%us push=%llu fill_ms=%u "
			     "total_push=%llu  rx push ar=%p aubuf=%p\n",
			     ar->dbg_iv,
			     (unsigned long long)ar->dbg_push,
			     fill_ms,
			     (unsigned long long)ar->dbg_push_tot,
			     (void *)ar, (void *)ar->aubuf);
			ar->dbg_iv  += 1;
			ar->dbg_push = 0;
			ar->dbg_next_log = now + 1000;
		}
	}

	return 0;
}


static int aurecv_stream_decode(struct audio_recv *ar,
				const struct rtp_header *hdr,
				struct mbuf *mb, unsigned lostc, bool drop)
{
	struct auframe af;
	size_t sampc = ar->sampvsz / aufmt_sample_size(ar->fmt);
	bool marker = hdr->m;
	int err = 0;
	const struct aucodec *ac = ar->ac;

	/* No decoder set */
	if (!ac)
		return 0;

	/* TODO: PLC */
	if (lostc && ac->plch) {

		err = ac->plch(ar->dec,
				   ar->fmt, ar->sampv, &sampc,
				   mbuf_buf(mb), mbuf_get_left(mb));
		if (err) {
			warning("audio_recv: %s codec decode %zu bytes: %m\n",
				ac->name, mbuf_get_left(mb), err);
			goto out;
		}
	}
	else if (mbuf_get_left(mb)) {

		err = ac->dech(ar->dec,
				   ar->fmt, ar->sampv, &sampc,
				   marker, mbuf_buf(mb), mbuf_get_left(mb));
		if (err) {
			warning("audio_recv: %s codec decode %zu bytes: %m\n",
				ac->name, mbuf_get_left(mb), err);
			goto out;
		}
	}
	else {
		/* no PLC in the codec, might be done in filters below */
		sampc = 0;
	}

	auframe_init(&af, ar->fmt, ar->sampv, sampc, ac->srate, ac->ch);
	af.timestamp = ((uint64_t) hdr->ts) * AUDIO_TIMEBASE / ac->crate;

	if (drop) {
		aubuf_drop_auframe(ar->aubuf, &af);
		goto out;
	}

	err = aurecv_process_decfilt(ar, &af);
	if (err)
		goto out;

	err = aurecv_push_aubuf(ar, &af);
 out:
	return err;
}


void aurecv_reset(struct audio_recv *ar)
{
	if (!ar)
		return;

	mtx_lock(ar->mtx);
	ar->ts_recv.is_set = false;
	ar->ts_recv.num_wraps = 0;
	aubuf_flush(ar->aubuf);
	mtx_unlock(ar->mtx);
}

/* Handle incoming stream data from the network */
void aurecv_receive(struct audio_recv *ar, const struct rtp_header *hdr,
		    struct rtpext *extv, size_t extc,
		    struct mbuf *mb, unsigned lostc, bool *ignore)
{
	bool discard = false;
	bool drop = *ignore;
	int wrap;
	(void) lostc;

	if (!mb)
		return;

	/* Kallo SDK 0.1.10 self-heal: consume a deferred player-rebind request
	 * here — we are on the rx/decode thread and do NOT hold ar->mtx yet, so
	 * aurecv_start_player() (which locks ar->mtx via aurecv_codec) is safe.
	 * Re-binds the player to the CURRENT receive aubuf (single source of
	 * truth); recovers both the startup race and a lost hold/resume rebind. */
	if (re_atomic_rlx(&ar->heal_request)) {
		re_atomic_rlx_set(&ar->heal_request, false);
		aurecv_stop_auplay(ar);
		(void)aurecv_start_player(ar, baresip_auplayl());
		info("aurecv: SELF-HEAL re-bound player ar=%p aubuf=%p "
		     "auplay=%p\n",
		     (void *)ar, (void *)ar->aubuf, (void *)ar->auplay);
	}

	mtx_lock(ar->mtx);
	if (hdr->pt != ar->pt) {
		mtx_unlock(ar->mtx);
		*ignore = true;
		return;
	}

	*ignore = false;

	/* RFC 5285 -- A General Mechanism for RTP Header Extensions */
	const struct rtpext *ext = rtpext_find(extv, extc, ar->extmap_aulevel);
	if (ext) {
		ar->level_last = -(double)(ext->data[0] & 0x7f);
		ar->level_set = true;
	}

	/* Save timestamp for incoming RTP packets */

	if (!ar->ts_recv.is_set)
		timestamp_set(&ar->ts_recv, hdr->ts);

	wrap = timestamp_wrap(hdr->ts, ar->ts_recv.last);

	switch (wrap) {

	case -1:
		warning("audio_recv: rtp timestamp wraps backwards"
			" (delta = %d) -- discard\n",
			(int32_t)(ar->ts_recv.last - hdr->ts));
		discard = true;
		break;

	case 0:
		break;

	case 1:
		++ar->ts_recv.num_wraps;
		break;

	default:
		break;
	}

	ar->ts_recv.last = hdr->ts;

	if (discard) {
		++ar->stats.n_discard;
		goto out;
	}

	/* TODO:  what if lostc > 1 ?*/
	/* PLC should generate lostc frames here. Not only one.
	 * aubuf should replace PLC frames with late arriving real frames.
	 * It should use timestamp to decide if a frame should be replaced. */
/*        if (lostc)*/
/*                (void)aurecv_stream_decode(ar, hdr, mb, lostc, drop);*/

	(void)aurecv_stream_decode(ar, hdr, mb, 0, drop);

out:
	mtx_unlock(ar->mtx);
}


void aurecv_set_extmap(struct audio_recv *ar, uint8_t aulevel)
{
	if (!ar)
		return;

	mtx_lock(ar->mtx);
	ar->extmap_aulevel = aulevel;
	mtx_unlock(ar->mtx);
}


int aurecv_set_module(struct audio_recv *ar, const char *module)
{
	if (!ar)
		return EINVAL;

	ar->module = mem_deref(ar->module);
	return str_dup(&ar->module, module);
}


int aurecv_set_device(struct audio_recv *ar, const char *device)
{
	if (!ar)
		return EINVAL;

	ar->device = mem_deref(ar->device);
	return str_dup(&ar->device, device);
}


uint64_t aurecv_latency(const struct audio_recv *ar)
{
	if (!ar)
		return 0;

	return re_atomic_rlx(&ar->stats.latency);
}


int aurecv_alloc(struct audio_recv **aupp, const struct config_audio *cfg,
		 size_t sampc, uint32_t ptime)
{
	struct audio_recv *ar;
	int err;

	if (!aupp)
		return EINVAL;

	ar = mem_zalloc(sizeof(*ar), destructor);
	if (!ar)
		return ENOMEM;

	ar->cfg = cfg;
	ar->srate = cfg->srate_play;
	ar->ch    = cfg->channels_play;
	ar->fmt   = cfg->dec_fmt;
	ar->play_fmt = cfg->play_fmt;
	ar->sampvsz = sampc * aufmt_sample_size(ar->fmt);
	ar->sampv   = mem_zalloc(ar->sampvsz, NULL);
	ar->ptime   = ptime * 1000;
	ar->pt      = -1;
	if (!ar->sampv) {
		err = ENOMEM;
		goto out;
	}

	err  = mutex_alloc(&ar->mtx);
	err |= mutex_alloc(&ar->aubuf_mtx);

out:
	if (err)
		mem_deref(ar);
	else
		*aupp = ar;

	return err;
}


void aurecv_flush(struct audio_recv *ar)
{
	if (!ar)
		return;

	mtx_lock(ar->mtx);
	aubuf_flush(ar->aubuf);

	/* Reset audio filter chain */
	list_flush(&ar->filtl);
	mtx_unlock(ar->mtx);
}


/* Kallo SDK (Issue 1, hold/resume downlink — DETERMINISTIC rebind, 0.1.9).
 *
 * Reset the receive (downlink) chain in place across a hold→resume re-INVITE,
 * keeping a SINGLE STABLE aubuf object alive throughout the rebind.
 *
 * History: 0.1.6/0.1.7 reopened the player but reused the aubuf untouched
 * (stale anchor → silence). 0.1.8 then *dropped* the aubuf (`ar->aubuf =
 * mem_deref(...)` → NULL) so the next decoded frame would reallocate a pristine
 * one. But that opened a window: audio_update() reopens the AAudio player
 * (bound to `ar` via auplay_write_handler) while `ar->aubuf` is NULL, so the
 * fresh player's pull callback reads the NULL/none branch (silence) and only
 * "reconnects" once a later decoded frame happens to win the realloc race under
 * aubuf_mtx. On resume that lazy realloc raced the just-reopened player and
 * reconnected only intermittently — leaving the player reading silence for the
 * rest of the call (DECODE ok≈50/s, AUBUF fill_ms pinned at the cap because the
 * player never drained it, PLAYOUT peak=0).
 *
 * Fix: never let `ar->aubuf` go NULL across the rebind. The buffer object the
 * receiver pushes into and the buffer object the reopened player reads from are
 * THE SAME object, the whole time — so the decoder→aubuf→player consumer link
 * is re-pointed deterministically with no NULL window and no realloc race. We
 * only flush its contents and reset the playout/timestamp anchor so the new leg
 * re-anchors to wall-clock (the pristine-buffer benefit 0.1.8 wanted, without
 * the desync). Holds aubuf_mtx so the player's read handler (aurecv_read) never
 * races the flush. If no buffer exists yet (resume before any frame decoded),
 * leave it NULL — no player is draining it, and the next push allocates it.
 */
void aurecv_reset_aubuf(struct audio_recv *ar)
{
	if (!ar)
		return;

	mtx_lock(ar->aubuf_mtx);
	if (ar->aubuf)
		aubuf_flush(ar->aubuf);   /* clear contents + reset adaptive anchor */
	/* restart the middle-link telemetry for the new (post-resume) leg */
	ar->dbg_push     = 0;
	ar->dbg_next_log = 0;
	/* reset the self-heal stall timer/snapshot so the new leg starts clean,
	 * and refill the per-call retry budget for the resumed leg (0.1.11) */
	ar->heal_since        = 0;
	ar->heal_attempts     = 0;
	ar->play_frames_last  = re_atomic_rlx(&ar->play_frames);
	re_atomic_rlx_set(&ar->play_peak, 0);
	mtx_unlock(ar->aubuf_mtx);

	/* Re-anchor the receive timestamp / push-jitter state so the resumed
	 * stream re-establishes its playout timeline against wall-clock rather
	 * than carrying the pre-hold anchor forward. */
	mtx_lock(ar->mtx);
	ar->ts_recv.is_set   = false;
	ar->ts_recv.num_wraps = 0;
	ar->t = 0;
	mtx_unlock(ar->mtx);
}


int aurecv_decoder_set(struct audio_recv *ar,
		       const struct aucodec *ac, int pt, const char *params)
{
	int err = 0;

	if (!ar || !ac)
		return EINVAL;

	info("audio_recv: Set audio decoder: %s %uHz %dch\n",
	     ac->name, ac->srate, ac->ch);

	mtx_lock(ar->mtx);
	if (ac != ar->ac) {
		ar->ac = ac;
		ar->dec = mem_deref(ar->dec);
	}

	if (ac->decupdh) {
		err = ac->decupdh(&ar->dec, ac, params);
		if (err) {
			warning("audio_recv: alloc decoder: %m\n", err);
			goto out;
		}
	}

	ar->pt = pt;

out:
	mtx_unlock(ar->mtx);
	return err;
}


int aurecv_payload_type(const struct audio_recv *ar)
{
	if (!ar)
		return -1;

	return ar->pt;
}


int aurecv_filt_append(struct audio_recv *ar, struct aufilt_dec_st *decst)
{
	if (!ar || !decst)
		return EINVAL;

	mtx_lock(ar->mtx);
	list_append(&ar->filtl, &decst->le, decst);
	mtx_unlock(ar->mtx);

	return 0;
}


bool aurecv_filt_empty(const struct audio_recv *ar)
{
	bool empty;
	if (!ar)
		return false;

	mtx_lock(ar->mtx);
	empty = list_isempty(&ar->filtl);
	mtx_unlock(ar->mtx);

	return empty;
}


bool aurecv_level_set(const struct audio_recv *ar)
{
	bool set;
	if (!ar)
		return false;

	mtx_lock(ar->mtx);
	set = ar->level_set;
	mtx_unlock(ar->mtx);

	return set;
}


double aurecv_level(const struct audio_recv *ar)
{
	double v;
	if (!ar)
		return 0.0;

	mtx_lock(ar->mtx);
	v = ar->level_last;
	mtx_unlock(ar->mtx);

	return v;
}


const struct aucodec *aurecv_codec(const struct audio_recv *ar)
{
	const struct aucodec *ac;

	if (!ar)
		return NULL;

	mtx_lock(ar->mtx);
	ac = ar->ac;
	mtx_unlock(ar->mtx);
	return ac;
}


static void aurecv_read(struct audio_recv *ar, struct auframe *af)
{
	if (!ar)
		return;

	if (mtx_trylock(ar->aubuf_mtx) != thrd_success) {
		/* Kallo SDK 0.1.11: never hand the device a stale/garbage frame
		 * if we lose the trylock — emit silence and bail. (Prior code left
		 * af->sampv untouched, relying on the caller's buffer being zero.) */
		if (af && af->sampv)
			memset(af->sampv, 0, auframe_size(af));
		return;
	}

	/* ── Kallo SDK 0.1.11 read-point instrumentation ────────────────────────
	 * Measure the PCM the player ACTUALLY dequeues, at the aubuf read point,
	 * BEFORE it reaches AAudio — the 0.1.10 trace only had the peak at AAudio
	 * playout, which could not tell apart (a) aubuf_read returns zeros / the
	 * read pointer never advances, (b) it returns real samples but a convert
	 * stage zeroes them, (c) it returns real samples but the player drops
	 * them. We log, ~1×/s: the requested frame's srate/ch/fmt/sampc, the peak
	 * |sample| just read, and aubuf cur_sz before vs after the read (proving
	 * whether the read pointer advanced). Pair this with "aaudio: PLAYOUT
	 * peak": read-peak>0 here but PLAYOUT peak=0 ⇒ the player drops it
	 * downstream; read-peak=0 with cur_after≈cur_before ⇒ the read didn't
	 * advance (dequeue / playout-start gate); read-peak=0 with the buffer
	 * draining ⇒ the written region itself is silence (upstream). */
	size_t cur_before = ar->aubuf ? aubuf_cur_size(ar->aubuf) : 0;

	if (ar->aubuf)
		aubuf_read_auframe(ar->aubuf, af);
	else if (af && af->sampv)
		memset(af->sampv, 0, auframe_size(af));

	size_t cur_after = ar->aubuf ? aubuf_cur_size(ar->aubuf) : 0;

	const uint64_t now = tmr_jiffies();
	if (!ar->dbg_rdpt_next_log)
		ar->dbg_rdpt_next_log = now + 1000;
	else if ((int64_t)(now - ar->dbg_rdpt_next_log) >= 0) {
		int32_t pk = 0;
		if (af && af->fmt == AUFMT_S16LE && af->sampv) {
			const int16_t *s = af->sampv;
			for (size_t i = 0; i < af->sampc; i++) {
				int32_t v = s[i] < 0 ? -s[i] : s[i];
				if (v > pk)
					pk = v;
			}
		}
		info("aurecv: AUBUF-READ srate=%u ch=%u fmt=%s sampc=%zu "
		     "peak=%d cur_before=%zu cur_after=%zu drained=%lld "
		     "ar=%p aubuf=%p\n",
		     af ? af->srate : 0, af ? af->ch : 0,
		     af ? aufmt_name(af->fmt) : "?",
		     af ? af->sampc : 0, pk, cur_before, cur_after,
		     (long long)cur_before - (long long)cur_after,
		     (void *)ar, (void *)ar->aubuf);
		ar->dbg_rdpt_next_log = now + 1000;
	}

	mtx_unlock(ar->aubuf_mtx);
}


void aurecv_stop(struct audio_recv *ar)
{
	if (!ar)
		return;

	ar->auplay = mem_deref(ar->auplay);
	mtx_lock(ar->mtx);
	ar->ac = NULL;
	mtx_unlock(ar->mtx);
}


void aurecv_stop_auplay(struct audio_recv *ar)
{
	if (!ar)
		return;

	ar->auplay = mem_deref(ar->auplay);
}


static void check_plframe(struct auframe *af1, struct auframe *af2)
{
	if ((af1->srate && af1->srate != af2->srate) ||
	    (af1->ch    && af1->ch    != af2->ch   )) {
		warning("audio_recv: srate/ch of frame %u/%u vs "
			"player %u/%u. Use module auresamp!\n",
			af1->srate, af1->ch,
			af2->srate, af2->ch);
	}

	if (af1->fmt != af2->fmt) {
		warning("audio_recv: invalid sample formats (%s -> %s). "
			"%s\n",
			aufmt_name(af1->fmt), aufmt_name(af2->fmt),
			af1->fmt == AUFMT_S16LE ?
			"Use module auconv!" : "");
	}
}


/*
 * Write samples to Audio Player.
 *
 * @note This function has REAL-TIME properties
 *
 * @note The application is responsible for filling in silence in
 *       the case of underrun
 *
 * @note This function may be called from any thread
 *
 * @note The sample format is set in ar->play_fmt
 */
/* Kallo SDK 0.1.10: account what the player ACTUALLY pulled — drives the
 * self-heal guard and the identity proof. Runs in the player (AAudio) thread
 * context. Tracks liveness (play_frames) and peak |sample| over the window,
 * and emits one greppable "player read ar=%p aubuf=%p" line per ~1 s. That
 * pointer pair must equal the "rx push ar=%p aubuf=%p" line — if they ever
 * diverge, the firing player and the filled buffer are different objects
 * (the identity bug); if they match while downlink is silent, the fault is
 * not the binding. ar->aubuf is read here without aubuf_mtx only for logging
 * (a benign pointer read). */
static void aurecv_account_playout(struct audio_recv *ar,
				   const struct auframe *af)
{
	re_atomic_rlx_set(&ar->play_frames,
			  re_atomic_rlx(&ar->play_frames) + af->sampc);

	if (ar->play_fmt == AUFMT_S16LE && af->sampv) {
		const int16_t *s = af->sampv;
		int32_t pk = re_atomic_rlx(&ar->play_peak);
		for (size_t i = 0; i < af->sampc; i++) {
			int32_t v = s[i] < 0 ? -s[i] : s[i];
			if (v > pk)
				pk = v;
		}
		re_atomic_rlx_set(&ar->play_peak, pk);
	}

	const uint64_t now = tmr_jiffies();
	if (!ar->dbg_rd_next_log)
		ar->dbg_rd_next_log = now + 1000;
	else if ((int64_t)(now - ar->dbg_rd_next_log) >= 0) {
		info("aurecv: player read ar=%p aubuf=%p\n",
		     (void *)ar, (void *)ar->aubuf);
		ar->dbg_rd_next_log = now + 1000;
	}
}


static void auplay_write_handler(struct auframe *af, void *arg)
{
	struct audio_recv *ar = arg;

	if (!ar->done_first) {
		struct auframe afr;
		memset(&afr, 0, sizeof(afr));
		afr = *af;

		aurecv_read(ar, af);

		check_plframe(&afr, af);
		ar->done_first = true;
		aurecv_account_playout(ar, af);
		return;
	}

	aurecv_read(ar, af);
	aurecv_account_playout(ar, af);
}


int aurecv_start_player(struct audio_recv *ar, struct list *auplayl)
{
	const struct aucodec *ac = aurecv_codec(ar);
	uint32_t srate_dsp;
	uint32_t channels_dsp;
	int err = 0;

	if (!ac)
		return 0;

	srate_dsp    = ac->srate;
	channels_dsp = ac->ch;

	if (ar->cfg->srate_play && ar->cfg->srate_play != srate_dsp) {
		srate_dsp = ar->cfg->srate_play;
	}
	if (ar->cfg->channels_play && ar->cfg->channels_play != channels_dsp) {
		channels_dsp = ar->cfg->channels_play;
	}

	/* Start Audio Player */
	if (!ar->auplay && auplay_find(auplayl, NULL)) {

		struct auplay_prm prm;

		prm.srate      = srate_dsp;
		prm.ch         = channels_dsp;
		prm.ptime      = ar->ptime / 1000;
		prm.fmt        = ar->play_fmt;

		/* Kallo SDK 0.1.10 — startup-race fix. The AAudio player starts
		 * its data callback the instant auplay_alloc() returns (the
		 * module calls AAudioStream_requestStart internally), and that
		 * callback drains ar->aubuf. Previously ar->aubuf was created
		 * lazily by the FIRST decoded frame, which arrives only after RX
		 * is enabled — so the freshly opened player could fire against a
		 * NULL buffer (silence branch) and recovery depended on a later
		 * decode winning the realloc race under aubuf_mtx. On a fresh
		 * call that race could lose from t=0 (2/3 calls silent in the
		 * field). Pre-allocate the single, stable receive aubuf HERE,
		 * before the player can fire, using the decoder's rate/channels —
		 * the exact params the first push would use — so the player→aubuf
		 * bind is to a real, live object from the very first callback.
		 * Idempotent: a no-op if the buffer already exists (resume).
		 *
		 * Kallo SDK 0.1.19-dev — size it at srate_dsp/channels_dsp, NOT
		 * ac->srate/ac->ch. Those are the same value unless
		 * audio.srate_play decouples the device rate from the codec rate
		 * (the Android Bluetooth quality investigation), and with an
		 * override they differ by 6x: the frames that actually reach this
		 * buffer are post-decode-filter, i.e. already resampled to
		 * srate_play by auresamp (see aurecv_process_decfilt, which runs
		 * before aurecv_push_aubuf). Sizing at the codec rate would leave
		 * max_sz at 2560 B (160 ms @ 8 kHz) while each 20 ms frame is
		 * 1920 B @ 48 kHz — barely one frame — so aubuf_write_auframe
		 * would drop the oldest frame on nearly every push. Deriving both
		 * from srate_dsp keeps the buffer's duration constant in ms and
		 * leaves the no-override path byte-identical. */
		if (!ar->aubuf) {
			int aerr = aurecv_alloc_aubuf(ar, srate_dsp,
						      (uint8_t)channels_dsp);
			if (aerr) {
				warning("audio_recv: pre-alloc aubuf failed:"
					" %m\n", aerr);
			}
		}

		ar->auplay_prm = prm;
		err = auplay_alloc(&ar->auplay, auplayl,
				   ar->module,
				   &prm, ar->device,
				   auplay_write_handler, ar);
		if (err) {
			warning("audio_recv: start_player failed (%s.%s): "
				"%m\n",
				ar->module, ar->device, err);
			goto out;
		}

		ar->ap = auplay_find(auplayl, ar->module);

		/* Greppable identity proof at bind time: the player's write
		 * handler captured `ar` as its arg and will read ar->aubuf. This
		 * must be the SAME ar/aubuf the rx push path logs. */
		info("audio_recv: player started with sample format %s\n",
		     aufmt_name(ar->play_fmt));
		info("aurecv: player bind ar=%p aubuf=%p auplay=%p\n",
		     (void *)ar, (void *)ar->aubuf, (void *)ar->auplay);

		/* Kallo SDK 0.1.11 — open the self-heal grace window. The guard
		 * must not judge this freshly opened player until it has had
		 * KALLO_HEAL_GRACE_MS to prime. Snapshot play_frames and clear the
		 * stall timer so the new player starts from a clean liveness base.
		 * (heal_attempts is deliberately NOT reset here — a self-heal
		 * rebind opens a player too, and resetting would defeat the retry
		 * cap; it is refilled only by a healthy window or a fresh leg.) */
		re_atomic_rlx_set(&ar->play_open_at, tmr_jiffies());
		ar->play_frames_last = re_atomic_rlx(&ar->play_frames);
		ar->heal_since       = 0;
	}

out:

	return 0;
}


bool aurecv_started(const struct audio_recv *ar)
{
	bool ret;

	if (!ar || mtx_trylock(ar->aubuf_mtx) != thrd_success)
		return false;

	ret = aubuf_started(ar->aubuf);
	mtx_unlock(ar->aubuf_mtx);
	return ret;
}


bool aurecv_player_started(const struct audio_recv *ar)
{
	return ar ? ar->auplay != NULL : false;
}


int aurecv_debug(struct re_printf *pf, const struct audio_recv *ar)
{
	struct mbuf *mb;
	double bpms;
	int err;

	if (!ar)
		return 0;

	mb = mbuf_alloc(32);
	if (!mb) {
		err = ENOMEM;
		goto out;
	}

	mtx_lock(ar->mtx);
	bpms = (double)ar->srate * ar->ch * aufmt_sample_size(ar->fmt) /
	       1000.0;
	err  = mbuf_printf(mb,
			   " rx:   decode: %H %s\n",
			   aucodec_print, ar->ac,
			   aufmt_name(ar->fmt));
	mtx_lock(ar->aubuf_mtx);
	err |= mbuf_printf(mb, "       aubuf: %H"
			   " (cur %.2fms, max %.2fms)\n",
			   aubuf_debug, ar->aubuf,
			   aubuf_cur_size(ar->aubuf) / bpms,
			   aubuf_maxsz(ar->aubuf) / bpms);
	mtx_unlock(ar->aubuf_mtx);
#ifndef RELEASE
	err |= mbuf_printf(mb, "       SW jitter: %.2fms\n",
			   (double) ar->stats.jitter / 1000);
	err |= mbuf_printf(mb, "       deviation: %.2fms\n",
			   (double) ar->stats.dmax / 1000);
#endif
	err |= mbuf_printf(mb, "       n_discard: %llu\n",
			   ar->stats.n_discard);
	if (ar->level_set) {
		err |= mbuf_printf(mb, "       level %.3f dBov\n",
				   ar->level_last);
	}
	if (ar->ts_recv.is_set) {
		err |= mbuf_printf(mb, "       time = %.3f sec\n",
				   aurecv_calc_seconds(ar));
	}
	else {
		err |= mbuf_printf(mb, "       time = (not started)\n");
	}

	err |= mbuf_printf(mb, "       player: %s,%s %s\n",
			  ar->ap ? ar->ap->name : "none",
			  ar->device,
			  aufmt_name(ar->play_fmt));
	mtx_unlock(ar->mtx);

	if (err)
		goto out;

	err = re_hprintf(pf, "%b", mb->buf, mb->pos);
out:
	mem_deref(mb);
	return err;
}


int aurecv_print_pipeline(struct re_printf *pf, const struct audio_recv *ar)
{
	struct mbuf *mb;
	struct le *le;
	int err;

	if (!ar)
		return 0;

	mb = mbuf_alloc(32);
	if (!mb)
		return ENOMEM;

	err = re_hprintf(pf, "audio rx pipeline:  %10s",
			 ar->ap ? ar->ap->name : "(play)");
	err |= mbuf_printf(mb, " <--- aubuf");
	mtx_lock(ar->mtx);
	for (le = list_head(&ar->filtl); le; le = le->next) {
		struct aufilt_dec_st *st = le->data;

		if (st->af->dech)
			err |= mbuf_printf(mb, " <--- %s", st->af->name);
	}
	mtx_unlock(ar->mtx);

	err |= mbuf_printf(mb, " <--- %s",
			   ar->ac ? ar->ac->name : "(decoder)");

	if (err)
		goto out;

	err = re_hprintf(pf, "%b", mb->buf, mb->pos);
out:
	mem_deref(mb);
	return err;
}
