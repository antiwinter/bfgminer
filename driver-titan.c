/*
 * Copyright 2014 Vitalii Demianets
 * Copyright 2014 KnCMiner
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 3 of the License, or (at your option)
 * any later version.  See COPYING for more details.
 */

#include <fcntl.h>
#include <sys/ioctl.h>

#include "deviceapi.h"
#include "logging.h"
#include "miner.h"
#include "util.h"

#include "titan-asic.h"

#define	KNC_TITAN_DEFAULT_FREQUENCY	275

#define KNC_TITAN_HWERR_DISABLE_SECS	10

#define	KNC_POLL_INTERVAL_US		10000

/* Work queue pre-fill level.
 * Must be high enough to supply all ASICs with works after a flush */
#define WORK_QUEUE_PREFILL		10

/* Specify here minimum number of leading zeroes in hash */
#define	DEFAULT_DIFF_FILTERING_ZEROES	24
#define	DEFAULT_DIFF_FILTERING_FLOAT	(1. / ((double)(0x00000000FFFFFFFF >> DEFAULT_DIFF_FILTERING_ZEROES)))
#define	DEFAULT_DIFF_HASHES_PER_NONCE	(1 << DEFAULT_DIFF_FILTERING_ZEROES)

BFG_REGISTER_DRIVER(knc_titan_drv)

/* 3 - default number of threads per core */
static int opt_knc_threads_per_core = 3;

static const struct bfg_set_device_definition knc_titan_set_device_funcs[];

struct knc_titan_core {
	int asicno;
	int dieno; /* inside asic */
	int coreno; /* inside die */
	struct knc_titan_die *die;
	struct cgpu_info *proc;

	int hwerr_in_row;
	int hwerr_disable_time;
	struct timeval enable_at;
	struct timeval first_hwerr;

	struct nonce_report last_nonce;
};

struct knc_titan_die {
	int asicno;
	int dieno; /* inside asic */
	int cores;
	struct cgpu_info *first_proc;

	int freq;
};

struct knc_titan_info {
	void *ctx;
	struct cgpu_info *cgpu;
	int cores;
	struct knc_titan_die dies[KNC_TITAN_MAX_ASICS][KNC_TITAN_DIES_PER_ASIC];

	/* Per-ASIC data */
	bool need_flush[KNC_TITAN_MAX_ASICS];
	int next_slot[KNC_TITAN_MAX_ASICS];
	/* First slot after flush. If next_slot reaches this, then
	 * we need to re-flush all the cores to avoid duplicating slot numbers
	 * for different works */
	int first_slot[KNC_TITAN_MAX_ASICS];

	struct work *workqueue;
	int workqueue_size;
	int workqueue_max;
	int next_id;

	struct work *devicework;
};

static bool knc_titan_detect_one(const char *devpath)
{
	static struct cgpu_info *prev_cgpu = NULL;
	struct cgpu_info *cgpu;
	void *ctx;
	struct knc_titan_info *knc;
	int cores = 0, asic, die;
	struct knc_die_info die_info;
	char repr[6];

	cgpu = malloc(sizeof(*cgpu));
	if (unlikely(!cgpu))
		quit(1, "Failed to alloc cgpu_info");

	if (!prev_cgpu) {
		if (NULL == (ctx = knc_trnsp_new(NULL))) {
			free(cgpu);
			return false;
		}

		knc = calloc(1, sizeof(*knc));
		if (unlikely(!knc))
			quit(1, "Failed to alloc knc_titan_info");

		knc->ctx = ctx;
		knc->cgpu = cgpu;
		knc->workqueue_max = WORK_QUEUE_PREFILL;
	} else {
		knc = prev_cgpu->device_data;
		ctx = knc->ctx;
	}

	snprintf(repr, sizeof(repr), "%s %s", knc_titan_drv.name, devpath);
	asic = atoi(devpath);
	for (die = 0; die < KNC_TITAN_DIES_PER_ASIC; ++die) {
		die_info.cores = KNC_TITAN_CORES_PER_DIE; /* core hint */
		die_info.version = KNC_VERSION_TITAN;
		if (!knc_titan_get_info(repr, ctx, asic, die, &die_info))
			continue;
		if (0 < die_info.cores) {
			knc->dies[asic][die] = (struct knc_titan_die) {
				.asicno = asic,
				.dieno = die,
				.cores = die_info.cores,
				.first_proc = cgpu,
				.freq = KNC_TITAN_DEFAULT_FREQUENCY,
			};
			cores += die_info.cores;
		} else {
			knc->dies[asic][die] = (struct knc_titan_die) {
				.asicno = -INT_MAX,
				.dieno = -INT_MAX,
				.cores = 0,
				.first_proc = NULL,
			};
		}
	}
	if (0 == cores) {
		free(cgpu);
		if (!prev_cgpu) {
			free(knc);
			knc_trnsp_free(ctx);
		}
		return false;
	}

	applog(LOG_NOTICE, "%s: Found ASIC with %d cores", repr, cores);

	*cgpu = (struct cgpu_info) {
		.drv = &knc_titan_drv,
		.device_path = strdup(devpath),
		.set_device_funcs = knc_titan_set_device_funcs,
		.deven = DEV_ENABLED,
		.procs = cores,
		.threads = prev_cgpu ? 0 : 1,
		.extra_work_queue = -1,
		.device_data = knc,
	};
	const bool rv = add_cgpu_slave(cgpu, prev_cgpu);
	if (!prev_cgpu)
		cgpu->extra_work_queue += WORK_QUEUE_PREFILL;
	prev_cgpu = cgpu;
	return rv;
}

static int knc_titan_detect_auto(void)
{
	const int first = 0, last = KNC_TITAN_MAX_ASICS - 1;
	char devpath[256];
	int found = 0, i;

	for (i = first; i <= last; ++i) {
		sprintf(devpath, "%d", i);
		if (knc_titan_detect_one(devpath))
			++found;
	}

	return found;
}

static void knc_titan_detect(void)
{
	generic_detect(&knc_titan_drv, knc_titan_detect_one, knc_titan_detect_auto, GDF_REQUIRE_DNAME | GDF_DEFAULT_NOAUTO);
}

static void knc_titan_clean_flush(const char *repr, void * const ctx, int asic, int die)
{
	struct knc_report report;
	bool unused;
	knc_titan_set_work(repr, ctx, asic, die, 0xFFFF, 0, NULL, true, &unused, &report);
}

static bool knc_titan_init(struct thr_info * const thr)
{
	const int max_cores = KNC_TITAN_CORES_PER_ASIC;
	struct thr_info *mythr;
	struct cgpu_info * const cgpu = thr->cgpu, *proc;
	struct knc_titan_core *knccore;
	struct knc_titan_info *knc;
	int i, asic, die, core_base;
	int total_cores = 0;
	int asic_cores[KNC_TITAN_MAX_ASICS] = {0};

	for (proc = cgpu; proc; ) {
		proc->min_nonce_diff = DEFAULT_DIFF_FILTERING_FLOAT;
		if (proc->device != proc) {
			applog(LOG_WARNING, "%"PRIpreprv": Extra processor?", proc->proc_repr);
			proc = proc->next_proc;
			continue;
		}

		asic = atoi(proc->device_path);
		knc = proc->device_data;

		die = 0;
		core_base = 0;
		for (i = 0; i < max_cores; ++i) {
			while (i >= (core_base + knc->dies[asic][die].cores)) {
				core_base += knc->dies[asic][die].cores;
				if (++die >= KNC_TITAN_DIES_PER_ASIC)
					break;
			}
			if (die >= KNC_TITAN_DIES_PER_ASIC)
				break;

			mythr = proc->thr[0];
			mythr->cgpu_data = knccore = malloc(sizeof(*knccore));
			if (unlikely(!knccore))
				quit(1, "Failed to alloc knc_titan_core");
			*knccore = (struct knc_titan_core) {
				.asicno = asic,
				.dieno = die,
				.coreno = i - core_base,
				.die = &(knc->dies[asic][die]),
				.proc = proc,
				.hwerr_in_row = 0,
				.hwerr_disable_time = KNC_TITAN_HWERR_DISABLE_SECS,
			};
			timer_set_now(&knccore->enable_at);
			proc->device_data = knc;
			++total_cores;
			++(asic_cores[asic]);
			applog(LOG_DEBUG, "%s Allocated core %d:%d:%d", proc->device->dev_repr, asic, die, (i - core_base));

			if (0 == knccore->coreno) {
				knc->dies[asic][die].first_proc = proc;
				knc_titan_clean_flush(proc->device->dev_repr, knc->ctx, knccore->asicno, knccore->dieno);
			}

			proc = proc->next_proc;
			if ((!proc) || proc->device == proc)
				break;
		}

		knc->cores = total_cores;
	}

	cgpu_set_defaults(cgpu);
	if (0 >= total_cores)
		return false;

	/* Init nonce ranges for cores */
	double nonce_step = 4294967296.0 / total_cores;
	double nonce_f = 0.0;
	struct titan_setup_core_params setup_params = {
		.bad_address_mask = {0, 0},
		.bad_address_match = {0x3FF, 0x3FF},
		.difficulty = DEFAULT_DIFF_FILTERING_ZEROES - 1,
		.thread_enable = 0xFF,
		.thread_base_address = {0, 1, 2, 3, 4, 5, 6, 7},
		.lookup_gap_mask = {0x7, 0x7, 0x7, 0x7, 0x7, 0x7, 0x7, 0x7},
		.N_mask = {0, 0, 0, 0, 0, 0, 0, 0},
		.N_shift = {0, 0, 0, 0, 0, 0, 0, 0},
		.nonce_bottom = 0,
		.nonce_top = 0xFFFFFFFF,
	};
	fill_in_thread_params(opt_knc_threads_per_core, &setup_params);

	int prev_asic = -1;
	for (proc = cgpu; proc; proc = proc->next_proc) {
		knc = proc->device_data;
		mythr = proc->thr[0];
		knccore = mythr->cgpu_data;
		if (knccore->asicno != prev_asic) {
			int numcores = asic_cores[knccore->asicno];
			if (numcores < 1)
				numcores = 1;
			prev_asic = knccore->asicno;
			nonce_f = 0.0;
			nonce_step = 4294967296.0 / numcores;
			setup_params.nonce_top = 0xFFFFFFFF;
		}
		nonce_f += nonce_step;
		setup_params.nonce_bottom = setup_params.nonce_top + 1;
		if (NULL != proc->next_proc)
			setup_params.nonce_top = nonce_f;
		else
			setup_params.nonce_top = 0xFFFFFFFF;
		applog(LOG_DEBUG, "%s Setup core %d:%d:%d, nonces 0x%08X - 0x%08X", proc->device->dev_repr, knccore->asicno, knccore->dieno, knccore->coreno, setup_params.nonce_bottom, setup_params.nonce_top);
		knc_titan_setup_core_local(proc->device->dev_repr, knc->ctx, knccore->asicno, knccore->dieno, knccore->coreno, &setup_params);
	}

	for (asic = 0; asic < KNC_TITAN_MAX_ASICS; ++asic) {
		knc->next_slot[asic] = KNC_TITAN_MIN_WORK_SLOT_NUM;
		knc->first_slot[asic] = KNC_TITAN_MIN_WORK_SLOT_NUM;
		knc->need_flush[asic] = true;
	}
	timer_set_now(&thr->tv_poll);

	return true;
}

static bool knc_titan_prepare_work(struct thr_info *thr, struct work *work)
{
	struct cgpu_info * const cgpu = thr->cgpu;

	work->nonce_diff = cgpu->min_nonce_diff;
	return true;
}

static void knc_titan_set_queue_full(struct knc_titan_info * const knc)
{
	const bool full = (knc->workqueue_size >= knc->workqueue_max);
	struct cgpu_info *proc;

	for (proc = knc->cgpu; proc; proc = proc->next_proc) {
		struct thr_info * const thr = proc->thr[0];
		thr->queue_full = full;
	}
}

static void knc_titan_remove_local_queue(struct knc_titan_info * const knc, struct work * const work)
{
	DL_DELETE(knc->workqueue, work);
	free_work(work);
	--knc->workqueue_size;
}

static void knc_titan_prune_local_queue(struct thr_info *thr)
{
	struct cgpu_info * const cgpu = thr->cgpu;
	struct knc_titan_info * const knc = cgpu->device_data;
	struct work *work, *tmp;

	DL_FOREACH_SAFE(knc->workqueue, work, tmp) {
		if (stale_work(work, false))
			knc_titan_remove_local_queue(knc, work);
	}
	knc_titan_set_queue_full(knc);
}

static bool knc_titan_queue_append(struct thr_info * const thr, struct work * const work)
{
	struct cgpu_info * const cgpu = thr->cgpu;
	struct knc_titan_info * const knc = cgpu->device_data;

	if (knc->workqueue_size >= knc->workqueue_max) {
		knc_titan_prune_local_queue(thr);
		if (thr->queue_full)
			return false;
	}

	DL_APPEND(knc->workqueue, work);
	++knc->workqueue_size;

	knc_titan_set_queue_full(knc);
	if (thr->queue_full)
		knc_titan_prune_local_queue(thr);

	return true;
}

#define HASH_LAST_ADDED(head, out)  \
	(out = (head) ? (ELMT_FROM_HH((head)->hh.tbl, (head)->hh.tbl->tail)) : NULL)

static void knc_titan_queue_flush(struct thr_info * const thr)
{
	struct cgpu_info * const cgpu = thr->cgpu;
	struct knc_titan_info * const knc = cgpu->device_data;
	struct work *work, *tmp;

	if (knc->cgpu != cgpu)
		return;

	DL_FOREACH_SAFE(knc->workqueue, work, tmp){
		knc_titan_remove_local_queue(knc, work);
	}
	knc_titan_set_queue_full(knc);

	HASH_LAST_ADDED(knc->devicework, work);
	if (work && stale_work(work, true)) {
		int asic;
		for (asic = 0; asic < KNC_TITAN_MAX_ASICS; ++asic)
			knc->need_flush[asic] = true;
		timer_set_now(&thr->tv_poll);
	}
}

static void knc_titan_poll(struct thr_info * const thr)
{
	struct thr_info *mythr;
	struct cgpu_info * const cgpu = thr->cgpu, *proc;
	struct knc_titan_info * const knc = cgpu->device_data;
	struct knc_titan_core *knccore, *core1;
	struct work *work, *tmp;
	int workaccept = 0;
	unsigned long delay_usecs = KNC_POLL_INTERVAL_US;
	struct knc_report report;
	struct knc_die_info die_info;
	int asic;
	int die;
	int i, tmp_int;

	knc_titan_prune_local_queue(thr);

	for (asic = 0; asic < KNC_TITAN_MAX_ASICS; ++asic) {
		DL_FOREACH_SAFE(knc->workqueue, work, tmp) {
			bool work_accepted = false;
			bool need_replace;
			if (knc->first_slot[asic] > KNC_TITAN_MIN_WORK_SLOT_NUM)
				need_replace = ((knc->next_slot[asic] + 1) == knc->first_slot[asic]);
			else
				need_replace = (knc->next_slot[asic] == KNC_TITAN_MAX_WORK_SLOT_NUM);
			knccore = NULL;
			for (die = 0; die < KNC_TITAN_DIES_PER_ASIC; ++die) {
				if (0 >= knc->dies[asic][die].cores)
					continue;
				struct cgpu_info *first_proc = knc->dies[asic][die].first_proc;
				/* knccore is the core data of the first core in this asic */
				if (NULL == knccore)
					knccore = first_proc->thr[0]->cgpu_data;
				bool die_work_accepted = false;
				if (knc->need_flush[asic] || need_replace) {
					for (proc = first_proc; proc; proc = proc->next_proc) {
						mythr = proc->thr[0];
						core1 = mythr->cgpu_data;
						bool unused;
						if ((core1->dieno != die) || (core1->asicno != asic))
							break;
						if (knc_titan_set_work(proc->proc_repr, knc->ctx, asic, die, core1->coreno, knc->next_slot[asic], work, true, &unused, &report)) {
							core1->last_nonce.slot = report.nonce[0].slot;
							core1->last_nonce.nonce = report.nonce[0].nonce;
							die_work_accepted = true;
						}
					}
				} else {
					if (!knc_titan_set_work(first_proc->dev_repr, knc->ctx, asic, die, 0xFFFF, knc->next_slot[asic], work, false, &die_work_accepted, &report))
						die_work_accepted = false;
				}
				if (die_work_accepted)
					work_accepted = true;
			}
			if ((!work_accepted) || (NULL == knccore))
				break;
			bool was_flushed = false;
			if (knc->need_flush[asic] || need_replace) {
				struct work *work1, *tmp1;
				applog(LOG_NOTICE, "%s: Flushing stale works (%s)", knccore->proc->dev_repr,
				       knc->need_flush[asic] ? "New work" : "Slot collision");
				knc->need_flush[asic] = false;
				knc->first_slot[asic] = knc->next_slot[asic];
				HASH_ITER(hh, knc->devicework, work1, tmp1) {
					if (asic == ((work1->device_id >> 8) & 0xFF)) {
						HASH_DEL(knc->devicework, work1);
						free_work(work1);
					}
				}
				delay_usecs = 0;
				was_flushed = true;
			}
			--knc->workqueue_size;
			DL_DELETE(knc->workqueue, work);
			work->device_id = (asic << 8) | knc->next_slot[asic];
			HASH_ADD(hh, knc->devicework, device_id, sizeof(work->device_id), work);
			if (++(knc->next_slot[asic]) > KNC_TITAN_MAX_WORK_SLOT_NUM)
				knc->next_slot[asic] = KNC_TITAN_MIN_WORK_SLOT_NUM;
			++workaccept;
			/* If we know for sure that this work was urgent, then we don't need to hurry up
			 * with filling next slot, we have plenty of time until current work completes.
			 * So, better to proceed with other ASICs. */
			if (was_flushed)
				break;
		}
	}

	applog(LOG_DEBUG, "%s: %d jobs accepted to queue (max=%d)", knc_titan_drv.dname, workaccept, knc->workqueue_max);

	for (asic = 0; asic < KNC_TITAN_MAX_ASICS; ++asic) {
		for (die = 0; die < KNC_TITAN_DIES_PER_ASIC; ++die) {
			if (0 >= knc->dies[asic][die].cores)
				continue;
			die_info.cores = knc->dies[asic][die].cores; /* core hint */
			die_info.version = KNC_VERSION_TITAN;
			if (!knc_titan_get_info(cgpu->dev_repr, knc->ctx, asic, die, &die_info))
				continue;
			for (proc = knc->dies[asic][die].first_proc; proc; proc = proc->next_proc) {
				mythr = proc->thr[0];
				knccore = mythr->cgpu_data;
				if ((knccore->dieno != die) || (knccore->asicno != asic))
					break;
				if (!die_info.has_report[knccore->coreno])
					continue;
				if (!knc_titan_get_report(proc->proc_repr, knc->ctx, asic, die, knccore->coreno, &report))
					continue;
				for (i = 0; i < KNC_TITAN_NONCES_PER_REPORT; ++i) {
					if ((report.nonce[i].slot == knccore->last_nonce.slot) &&
					    (report.nonce[i].nonce == knccore->last_nonce.nonce))
						break;
					tmp_int = (asic << 8) | report.nonce[i].slot;
					HASH_FIND_INT(knc->devicework, &tmp_int, work);
					if (!work) {
						applog(LOG_WARNING, "%"PRIpreprv": Got nonce for unknown work in slot %u (asic %d)", proc->proc_repr, (unsigned)report.nonce[i].slot, asic);
						continue;
					}
					if (submit_nonce(mythr, work, report.nonce[i].nonce)) {
						hashes_done2(mythr, DEFAULT_DIFF_HASHES_PER_NONCE, NULL);
						knccore->hwerr_in_row = 0;
					}
				}
				knccore->last_nonce.slot = report.nonce[0].slot;
				knccore->last_nonce.nonce = report.nonce[0].nonce;
			}
		}
	}

	if (workaccept) {
		if (workaccept >= knc->workqueue_max) {
			knc->workqueue_max = workaccept;
			delay_usecs = 0;
		}
		knc_titan_set_queue_full(knc);
	}

	timer_set_delay_from_now(&thr->tv_poll, delay_usecs);
}

/*
 * specify settings / options via RPC or command line
 */

/* support for --set-device
 * must be set before probing the device
 */
static void knc_titan_set_clock_freq(struct cgpu_info * const device, int const val)
{
}

static const char *knc_titan_set_clock(struct cgpu_info * const device, const char * const option, const char * const setting, char * const replybuf, enum bfg_set_device_replytype * const success)
{
	knc_titan_set_clock_freq(device, atoi(setting));
	return NULL;
}

static const struct bfg_set_device_definition knc_titan_set_device_funcs[] = {
	{ "clock", knc_titan_set_clock, NULL },
	{ NULL },
};

/*
 * specify settings / options via TUI
 */

#ifdef HAVE_CURSES
static void knc_titan_tui_wlogprint_choices(struct cgpu_info * const proc)
{
	wlogprint("[C]lock speed ");
}

static const char *knc_titan_tui_handle_choice(struct cgpu_info * const proc, const int input)
{
	static char buf[0x100];  /* Static for replies */

	switch (input)
	{
		case 'c': case 'C':
		{
			sprintf(buf, "Set clock speed");
			char * const setting = curses_input(buf);

			knc_titan_set_clock_freq(proc->device, atoi(setting));

			return "Clock speed changed\n";
		}
	}
	return NULL;
}

static void knc_titan_wlogprint_status(struct cgpu_info * const proc)
{
	wlogprint("Clock speed: N/A\n");
}
#endif

struct device_drv knc_titan_drv =
{
	/* metadata */
	.dname = "titan",
	.name = "KNC",
	.supported_algos = POW_SCRYPT,
	.drv_detect = knc_titan_detect,

	.thread_init = knc_titan_init,

	/* specify mining type - queue */
	.minerloop = minerloop_queue,
	.queue_append = knc_titan_queue_append,
	.queue_flush = knc_titan_queue_flush,
	.poll = knc_titan_poll,
	.prepare_work = knc_titan_prepare_work,

	/* TUI support - e.g. setting clock via UI */
#ifdef HAVE_CURSES
	.proc_wlogprint_status = knc_titan_wlogprint_status,
	.proc_tui_wlogprint_choices = knc_titan_tui_wlogprint_choices,
	.proc_tui_handle_choice = knc_titan_tui_handle_choice,
#endif
};