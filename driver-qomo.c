/*
 * Driver for Intchains Qomolangma Protocol
 *
 * Copyright 2017, 2018 Intchains
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "config.h"

#include "miner.h"

#include <unistd.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

#ifdef HAVE_LINUX_SPI_SPIDEV_H
#include <linux/spi/spidev.h>
#else
#define SPI_MODE_0 0
#endif

#include "deviceapi.h"
#include "lowl-spi.h"
#include "util.h"

/* SYNC_MODE 1: write work in braodcast mode, all chips in the chain will do the
 *            same work (nonce is divided into 128 range for each chip)
 * SYNC_MODE 0: request a work for each chip separately from bfgminer
 */
#define SYNC_MODE 1

#define CHIP_NR_MAX 32
#define CHIP_CORES_NR 64
#define HASH_CYCLE  10000
#define DEFAULT_FREQ 10
#define DEFAULT_SPI_FREQ 400000
#define WORK_ID_NR_MAX 16

BFG_REGISTER_DRIVER(qomo_drv)

/* work_id pool */
struct qomo_wp {
    struct timeval stamp_tv[WORK_ID_NR_MAX];
    struct work * work[WORK_ID_NR_MAX];
    int cursor;
    struct timeval expire;
};

struct qomo_device {
    int sig0, sig1, sig2, sig3;
    char *spi_path, *iic_path;
    uint8_t prep[512];
    uint8_t payload[512];
    int prep_sz;
    int echo_loc;
    int slave_addr;
    struct spi_port spi;
    int id;
    int chips_nr;
    int cores_nr;
    int sync_mode;
    int enabled;
    float voltage;
    double range_scan_time;
    double hash_rate;
    double freq;
    uint32_t chip_bist[CHIP_NR_MAX];
    int chip_perf[CHIP_NR_MAX];
    int nonce_ood;
    struct timeval valid_tv;
#if SYNC_MODE == 1
    struct qomo_wq wp;
#else
    struct qomo_wq wp[CHIP_NR_MAX];
#endif
};

enum qomo_commands {
    QOMO_REG_WRITE = 0x0900,
    QOMO_REG_READ = 0x0a00,
    QOMO_BIST = 0x0100,
    QOMO_BIST_FIX = 0x0300,
    QOMO_AUTO_ADDRESSING = 0x0200,
    QOMO_WRITE_WORK = 0x0700,
    QOMO_GET_NONCE = 0x0800,
    QOMO_RESET = 0x0400,
};

static struct qomo_device qomo_devices[] = {
    {0, 0, 0, 0, "/dev/spidev0.0", "/dev/i2c-dev1", 0x20},
#if 0
    {0, 0, 0, 0, "/dev/spidev0.1", "/dev/i2c-dev1", 0x21},
    {0, 0, 0, 0, "/dev/spidev1.0", "/dev/i2c-dev1", 0x22},
    {0, 0, 0, 0, "/dev/spidev1.1", "/dev/i2c-dev1", 0x23},
#endif
};

static inline void qomo_wp_invalidate(struct qomo_wp *wp) {
    int i;
    for (i = 0; i < WORK_ID_NR_MAX; i++)
        wp->work[i] = NULL;
}

static inline int qomo_wp_alloc_id(struct qomo_wp *wp, struct work * work) {

    ++wp->cursor;
    if (wp->cursor == WORK_ID_NR_MAX) wp->cursor = 1;
    timer_set_now(&wp->stamp_tv[wp->cursor]);
    wp->work[wp->cursor] = work;
    
    return id;
}

static inline int __qomo_exec_cmd(struct qomo_device *dev,
        uint16_t cmd, int chip_addr, uint8_t *in, int in_len) {

    struct spi_port *spi = &dev->spi;
    int ret, dn, i;

    /* back up tx data to dev->prep */
    cmd |= chip_addr;
    cmd = bswap_16(cmd);

    memcpy(dev->prep, "\xa5\x3c", 2);
    memcpy(dev->prep + 2, &cmd, 2);
    if (likely(in_len)) {
        if (in)
            memcpy(dev->prep + 4, in, in_len);
        else
            memset(dev->prep + 4, 0, in_len);
    }
    memcpy(dev->prep + 4 + in_len, "\x00\x00", 2);
    dev->prep_sz = in_len + 6;

    dn = 4 * CHIP_NR_MAX;

    spi_clear_buf(spi);
    spi_emit_buf(spi, dev->prep, in_len + 6);

    /* add extra dummy data */
    spi_emit_nop(spi, dn);

    /* send tx data */
//    printbin("sent:", spi->spibuf, in_len + 6 + dn);
    spi_txrx(spi);
//    printbin("echo:", spi->spibuf_rx, in_len + 6 + dn);

    /* set valid response loc */
    dev->echo_loc = -1;
    for (i = 0; i < spi->spibufsz - 1; i++) {
        if (spi->spibuf_rx[i] == 0xa5 &&
            spi->spibuf_rx[i + 1] == 0x3c) {
            dev->echo_loc = i;
            break;
        }
    }

    return ret;
}

/* check if the last m bytes in rx buffer is identical with
 * dev->prep
 */
static inline int
__qomo_echo_assert(struct qomo_device *dev) {
    if (dev->echo_loc < 0) return -1;
    if (memcmp(dev->prep, dev->spi.spibuf_rx
                + dev->echo_loc, dev->prep_sz) != 0) {
        applog(LOG_ERR, "echo is not same as expected!");
        // bin2hex ...
        printbin("prep:", dev->prep, dev->prep_sz);
        printbin("echo:", dev->spi.spibuf_rx + dev->echo_loc, dev->prep_sz);
        return -1;
    }

    return 0;
}

/* get a word (16bit) from the m bytes position */
static inline uint16_t
__qomo_echo_get_word(struct qomo_device *dev, int m) {
    if (dev->echo_loc < 0) return 0xeeee;

    return bswap_16(*(uint16_t *)(dev->spi.spibuf_rx
            + dev->echo_loc + m));
}

static int qomo_exec_cmd(struct qomo_device *dev, enum qomo_commands cmd,
        int chip_addr, void *in, void *out) {
    struct spi_port *spi = &dev->spi;
    int *ret = out, val;

#define refuse_single(x...) do { \
    if (chip_addr) { \
        applog(LOG_ERR, "cmd %d does not support single mode", cmd); \
        return -1; \
    } } while (0)

#define refuse_bcast(x...) do { \
    if (!chip_addr) { \
        applog(LOG_ERR, "cmd %d does not support broadcast mode", cmd); \
        return -1; \
    } } while (0)

    switch(cmd) {
        case QOMO_REG_WRITE:
            refuse_single();
            __qomo_exec_cmd(dev, cmd, 0, in, 8);
            return __qomo_echo_assert(dev);

        case QOMO_REG_READ:
            refuse_bcast();
            __qomo_exec_cmd(dev, cmd, chip_addr, NULL, 14);
            if (__qomo_echo_get_word(dev, 2) != (cmd | 0x8000 | chip_addr)) {
                applog(LOG_ERR, "unexpected echo for cmd 0x%04x", cmd);
                return -1;
            }
            memcpy(out, spi->spibuf_rx + dev->echo_loc + 4, 16);
            return 0;

        case QOMO_BIST:
            __qomo_exec_cmd(dev, cmd, chip_addr, NULL, 2);
            return __qomo_echo_assert(dev);

        case QOMO_BIST_FIX:
            refuse_single();
            __qomo_exec_cmd(dev, cmd, 0, NULL, 0);
            return __qomo_echo_assert(dev);

        case QOMO_AUTO_ADDRESSING:
            refuse_single();
            __qomo_exec_cmd(dev, cmd, 0, in, 2);
            if (__qomo_echo_get_word(dev, 2) != cmd) {
                applog(LOG_ERR, "unexpected echo for cmd 0x%04x", cmd);
                return -1;
            }
            *(int *)out = __qomo_echo_get_word(dev, 4) - 1;
            return 0;

        case QOMO_WRITE_WORK:
            __qomo_exec_cmd(dev, cmd, chip_addr, in, 88);
            memset(dev->prep + 4, 0, 88);
            return __qomo_echo_assert(dev);

        case QOMO_GET_NONCE:
            __qomo_exec_cmd(dev, cmd, chip_addr, in, 4);
            val = __qomo_echo_get_word(dev, 2);
            if ((val & 0x0f00) != cmd) {
                applog(LOG_ERR, "unexpected echo for cmd 0x%04x", cmd);
                return -1;
            }

            ret[0] = val >> 12;
            ret[1] = __qomo_echo_get_word(dev, 4);
            ret[1] <<= 16;
            ret[1] |= __qomo_echo_get_word(dev, 6);
            ret[2] = val & 0xff;
            return 0;

        case QOMO_RESET:
            __qomo_exec_cmd(dev, cmd, chip_addr, in, 2);
            return __qomo_echo_assert(dev);
    }

    applog(LOG_NOTICE, "%s, %d", __func__, __LINE__);
    applog(LOG_ERR, "unknown qomo cmd 0x%04x", cmd);
    return -1;
}

struct device_drv qomo_drv;
static const struct bfg_set_device_definition qomo_set_device_funcs[];
static void qomo_drv_detect(void)
{
    int chains = 0, chips, ret, d, id = 0;
    char data[64];

    
    for (d = 0; d < ARRAY_SIZE(qomo_devices); d++) {
        struct qomo_device *dev = &qomo_devices[d];
        dev->spi.speed = DEFAULT_SPI_FREQ;
        dev->spi.delay = 0;
        dev->spi.mode = SPI_MODE_0;
        dev->spi.bits = 8;
        dev->spi.txrx = linux_spi_txrx;
        dev->freq = DEFAULT_FREQ;
        dev->id = id++;
        dev->sync_mode = SYNC_MODE;
        dev->enabled = 1;

#ifdef HAVE_LINUX_SPI_SPIDEV_H
        if(spi_open(&dev->spi, dev->spi_path) < 0) {
            applog(LOG_NOTICE, "%s not found", dev->spi_path);
            continue;
        }
#endif

        /* apply other default settings: freq, board, etc. */
        drv_set_defaults(&qomo_drv, qomo_set_device_funcs,
                dev, dev->spi_path, NULL, 1);

        /* add cgpu, use calloc which init memory to zero */
        struct cgpu_info *cgpu = calloc(1, sizeof(struct cgpu_info));
        cgpu->drv = &qomo_drv;
        cgpu->device_data = dev;
        cgpu->threads = 1;
        cgpu->procs = chips;
        cgpu->set_device_funcs = qomo_set_device_funcs;

        add_cgpu(cgpu);
        chains++;
    }

    if (chains) applog(LOG_NOTICE, "QomoMiner detected, %d chains totally!", chains);
}

static bool qomo_thread_init(struct thr_info *thr)
{
    struct cgpu_info *cgpu = thr->cgpu;
    struct qomo_device *dev = cgpu->device_data;
    uint8_t reg[32];
    int ret, i;

    applog(LOG_NOTICE, "starting dev %d", dev->id);
    /*
     * 1. configure DCDC to get correct voltage for each computing board
     * 2. perform HW reset for each computing board
     */

    /* 3. configure PLL and SPI clock for each compute board
    */
    ret = qomo_exec_cmd(dev, QOMO_REG_WRITE, 0,
            /* 32 bits: pll */
            "\x00\x00\x00\x00"
            /* 16 bits: time-delta interval between each core power up */
            "\x00\x00"
            /* 16 bits:
             * reserved 4
             * BIST mode, mask update, broadcast work, power up mode
             * RO 8 */
            "\x0a\x00", NULL);
    if (ret < 0) {
        return false;
    }

    /* TODO: calc difficulty and range_scan_time and hash_rate */
    dev->range_scan_time = 4096.0;
    dev->range_scan_time /= dev->freq * CHIP_CORES_NR / HASH_CYCLE;

    applog(LOG_NOTICE, "range scan time for a chip is %.2lfs@%.2lfMHz",
            dev->range_scan_time, dev->freq);

    /* 4. BIST
    */
    if(qomo_exec_cmd(dev, QOMO_BIST, 0, NULL, NULL) < 0) {
        return false;
    };

    /* 5. auto addressing
    */
    if(qomo_exec_cmd(dev, QOMO_AUTO_ADDRESSING, 0, "\x00\x01",
                &dev->chips_nr) < 0) {
        return false;
    }
    applog(LOG_NOTICE, "%d chips detected in chain", dev->chips_nr);

    /* 6. Mask out bad cores inside each chip
    */
    if(qomo_exec_cmd(dev, QOMO_BIST_FIX, 0, NULL, NULL) < 0) {
        return false;
    }

    /* 7. count good cores inside each chip (optional)
    */
    for (i = 1; i <= dev->chips_nr; i++) {
        if(qomo_exec_cmd(dev, QOMO_REG_READ, i, NULL, reg) < 0) {
            applog(LOG_ERR, "dev %d: read reg for chip %d failed", dev->id, i);
            continue;
        }
        dev->cores_nr += reg[7];
        dev->chip_bist[i] = *(uint64_t *)&reg[8];
    }
    applog(LOG_NOTICE, "dev %d: %d good cores", dev->id, dev->cores_nr);

    return true;
}

static void qomo_thread_enable(struct thr_info * thr)
{
    applog(LOG_NOTICE, "%s, %d", __func__, __LINE__);
}

static void qomo_thread_disable(struct thr_info * thr)
{
    applog(LOG_NOTICE, "%s, %d", __func__, __LINE__);
}

static void qomo_thread_shutdown(struct thr_info * thr)
{
    applog(LOG_NOTICE, "%s, %d", __func__, __LINE__);
}

static void wswap32cp(void *dst, void *src, int len)
{
    uint16_t *t = dst, *s = src;
    int i = 0;
    for (; i < len; i += 2, t += 2, s += 2) {
        *(t + i) = *(s + i + 1);
        *(t + i + 1) = *(s + i);
    }
}

static void qomo_prepare_payload(struct qomo_device *dev, struct work *work)
{
    uint32_t i;
    uint8_t *pl = dev->payload;

    bswap_32mult(pl, work->data, 32 / 4);
    bswap_32mult(pl + 32, work->data + 32, 32 / 4);
    bswap_32mult(pl + 64, work->data + 64, 16 / 4);

    /* start nonce */
    *(uint32_t *)&pl[64] = bswap_32(0x0);

    /* end nonce */
    *(uint32_t *)&pl[84] = bswap_32(0xffffffff);

    /* difficulty */
    for (i = 0; !work->target[31 - i]; i++);
    pl[80] = i;
    pl[81] = work->target[31 - i];
    pl[82] = work->target[31 - i - 1];
    pl[83] = work->target[31 - i - 2];
}

/* the code here is for matching block/byte order from network
 * to asic. just call test_byte_order(work) in function scanhash
 */
#if 0
void test_byte_feed(uint8_t *buf, uint8_t *src, int n, int loop, int s4, int s2)
{
    uint8_t buf_s2[32], buf_s4[32], buf_loop[32];
    int i;

    if (loop)
        bswap_32mult(buf_loop, src, n / 4);
    else
        memcpy(buf_loop, src, n);

    if (s4)
        swap32yes(buf_s4, buf_loop, n / 4);
    else
        memcpy(buf_s4, buf_loop, n);

    if (!s2) {
        memcpy(buf_s2, buf_s4, n);
    }
    else {
        for (i = 0; i < n / 2; i++)
            *(((uint16_t *)buf_s2) + i) = bswap_16(*(((uint16_t *)buf_s4) + i));
    }

    memcpy(buf, buf_s2, n);
}

int test_byte_hash(uint8_t *g, uint8_t *hash)
{
    int i, j;
    for (i = 0; i < 32; i++) {
        for (j = 0; j < 32; j++)
            if (g[j] == hash[i])
                break;
        if (j == 32) {
            printf("DO NOT MATCH\n");
            return -1;
        }
    }

    printf("Matched !!!\n");
    return 0;
}

void test_byte_order(struct work *work) {
    uint8_t data1[32] = {
        0x18, 0xe7, 0xb1, 0xe8, 0xea, 0xf0, 0xb6, 0x2a,
        0x90, 0xd1, 0x94, 0x2e, 0xa6, 0x4d, 0x25, 0x03,
        0x57, 0xe9, 0xa0, 0x9c, 0x06, 0x3a, 0x47, 0x82,
        0x7c, 0x57, 0xb4, 0x4e, 0x01, 0x00, 0x00, 0x00
    }, data2[32] = {
        0xc7, 0x91, 0xd4, 0x64, 0x62, 0x40, 0xfc, 0x2a,
        0x2d, 0x1b, 0x80, 0x90, 0x00, 0x20, 0xa2, 0x4d,
        0xc5, 0x01, 0xef, 0x15, 0x99, 0xfc, 0x48, 0xed,
        0x6c, 0xba, 0xc9, 0x20, 0xaf, 0x75, 0x57, 0x56
    }, data3[16] = {
        0x00, 0x00, 0x31, 0x8f, 0x7e, 0x71, 0x44, 0x1b,
        0x14, 0x1f, 0xe9, 0x51, 0xb2, 0xb0, 0xc7, 0xdf
    }, hash[32] = {
        0xb3, 0x03, 0x00, 0x00, 0x66, 0xbd, 0x9f, 0x06,
        0x8a, 0xa1, 0x15, 0x33, 0x9c, 0xfd, 0x01, 0xcd,
        0x94, 0x12, 0x1e, 0xf6, 0x5b, 0x2f, 0xf4, 0xd7,
        0xd3, 0x79, 0x6b, 0x73, 0x8a, 0x17, 0x4f, 0x7b
    };

    int loop, s4, s2; 

    for (s4 = 0; s4 < 2; s4++)
        for (s2 = 0; s2 < 2; s2++)
            for (loop = 0; loop < 2; loop++) {
#define _test(a, b, c) do { \
    printf("TRY: %s | %s | %s, loop_swap %d, 4_byte_swap %d, 2_byte_swap %d ... ", \
#a, #b, #c, loop, s4, s2); \
    fflush(stdout); \
    test_byte_feed(work->data, a, ARRAY_SIZE(a), loop, s4, s2); \
    test_byte_feed(work->data + ARRAY_SIZE(a), b, ARRAY_SIZE(b), loop, s4, s2); \
    test_byte_feed(work->data + ARRAY_SIZE(a) + ARRAY_SIZE(b), c, ARRAY_SIZE(c), loop, s4, s2); \
    work_hash(work); \
    if (test_byte_hash(hash, work->hash) == 0) { \
        printbin("data:", work->data, 80); \
        printbin("hash:", work->hash, 32); \
    } \
} while (0)

            _test(data1, data2, data3);
            _test(data1, data3, data2);
            _test(data2, data1, data3);
            _test(data2, data3, data1);
            _test(data3, data1, data2);
            _test(data3, data2, data1);
            }
printf("\n\nTHE END\n");
for (;;);
}
#endif

#if SYNC_MODE == 1
static int64_t qomo_scanhash(struct thr_info *thr, struct work *work,
        int64_t __maybe_unused max_nonce)
{
    int nonce, ret[4], i, work_id;
    struct cgpu_info *cgpu = thr->cgpu;
    struct qomo_device *dev = cgpu->device_data;
    struct timeval start_tv, end_tv;

    work_id = qomo_wp_alloc_id(&dev->wp, work);

    qomo_prepare_payload(dev, work);
//    printbin("payload:", dev->payload, 88);
    qomo_exec_cmd(dev, QOMO_WRITE_WORK, work_id << 12, dev->payload, NULL);
    timer_set_delay_from_now(&dev->wp.expire,
            (int)(dev->range_scan_time * 1000000 / CHIP_NR_MAX));

    timer_set_now(&start_tv);

    for(i = 0;;i++) {
        /* we control the hash loop for ourselves, and set
         * work->blk.nonce to 0xffffffff at the end, so that
         * the outer loop (minerloop_scanhash@deviceapi.c)
         * will abandon this work and trigger the next one.
         */

        /* new block arrived, abandon current works in all chips */
        if(thr->work_restart) {
            applog(LOG_NOTICE, "work_restart required");
            break;
        }

        /* range nearly scanned */
        if(timer_passed(&dev->wp.expire, NULL)) {
            applog(LOG_NOTICE, "range scanned (time passed)");
            break;
        }

        /* scan for nonce */
        qomo_exec_cmd(dev, QOMO_GET_NONCE, 0, "\x00\x00\x00\x00", ret);
        work_id = ret[0];
        nonce = ret[1];
        chip_id = ret[2];

        if (work_id) {
            /* valid response */
            struct qomo_wp *wp = &dev->wp;
            int ood = 0;

            if (timercmp(&wp->stamp_tv[work_id],
                        &dev->valid_tv, >)) { /* valid nonce */
                if (wp->work[work_id] == work)
                    submit_nonce(thr, work, nonce);
                else
                    /* because we have cleared output queue
                     * for previous works, so this is almost
                     * imporssible. unless it is IC bug.
                     */
                    applog(LOG_ERR, "impossible thing happened");
            } else {
                ood = 1;
                dev->nonce_ood++ ;
            }

            applog(LOG_NOTICE, "! nonce 0x%08x from %d.%d %s", nonce,
                    dev->id, chip_id, ood ? "(out of date)": "");
            dev->chip_perf[chip_id]++;

            /* lucky time: try one more nonce */
            continue ;
        }

        /* if no nonce is found, sleep for a while
         * thus to let go of the SPI bus */
        cgsleep_ms(20);
    }

    /* clear potential results for the current work
     * reserved 5, output queue, BIST, cores
     */
    qomo_exec_cmd(dev, QOMO_RESET, 0, "\x00\x04", NULL);

    /* in scanhash mode, work is freed immediately out this function,
     * so there is no need to maintain this work any more
     */
    timer_set_now(&dev->valid_tv);
    qomo_wp_invalidate(&dev->wp);

    work->blk.nonce = 0xffffffff;
    applog(LOG_NOTICE, "%d times poll for nonce", i);

    timer_set_now(&end_tv);
    return (int)(dev->hash_rate * ms_tdiff(&end_tv, &start_tv) / 1000);
}
#else /* manage works for each chip separately */

bool qomo_queue_append(struct thr_info *thr, struct work * const work)
{
    struct qomo_device *dev = cthr->cgpu->device_data;

    for (i = 1; i <= dev->chips_nr; i++) {
        struct qomo_wp *wp = &dev->wp[i];
        if (timer_passed(&wp->expire, NULL)) { // null is now
            int work_id = qomo_wp_alloc_id(wp, work);
            timer_set_delay_from_now(wp->expire,
                    (int)(dev->range_scan_time * 1000000));

            qomo_prepare_payload(dev, work);
            qomo_exec_cmd(dev, QOMO_WRITE_WORK, work_id << 12, dev->payload, NULL);
            return true;
        }
    }

    return false;
}

static
void qomo_queue_flush(struct thr_info * const thr)
{
    struct qomo_device *dev = cthr->cgpu->device_data;
    struct qomo_wp *wp;
    int i, j;

    timer_set_now(&dev->valid_tv);
    for (i = 1; i <= dev->chips_nr; i++) {
        wp = &dev->wp[i];
        qomo_exec_cmd(dev, QOMO_RESET, 0, "\x00\x04", NULL);
        qomo_wp_invalidate(wp);
    }
}

static
void qomo_poll(struct thr_info * const thr)
{
    struct qomo_device *dev = cthr->cgpu->device_data;
    uint32_t ret[4], work_id, nonce, chip_id, ood;
    struct qomo_wp *wp;

one_more_nonce:
    if(qomo_exec_cmd(dev, QOMO_GET_NONCE, 0, "\x00\x00"
                "\x00\x00", ret) < 0) goto out ;

    work_id = ret[0];
    nonce = ret[1];
    chip_id = ret[2];
    ood = 0;

    if (work_id) {
        if (chip_id < 1 || chip_id >= CHIP_NR_MAX) {
            applog(LOG_ERR, "illeagle response: work id: %d, nonce:"
                    " 0x%08x, chip: %d", work_id, nonce, chip_id);
            goto out ;
        }

        /* valid response */
        wp = dev->wp[chip_id];
        if (timercmp(&wp->stamp_tv[work_id], &dev->valid_tv, >)) { /* valid nonce */
            submit_nonce(thr, wp->work[work_id], nonce);
        } else {
            ood = 1;
            dev->nonce_ood++ ;
        }

        applog(LOG_NOTICE, "! nonce 0x%08x from %d.%d %s", nonce,
                dev->id, chip_id, ood ? "(out of date)": "");
        dev->chip_perf[chip_id]++;
        goto one_more_nonce;
    }

    /* no nonce yet */
out:
    cgsleep_ms(20);
}

#endif /* end SYNC_MDOE */

const char *qomo_set_device(struct cgpu_info * const cgpu,
        const char *opt, const char * newval, char * reply,
        enum bfg_set_device_replytype *out_success)
{
    applog(LOG_NOTICE, "qomo set_device: opt: %s, newval: %s", opt, newval);

    if (strcmp(opt, "clock") == 0) {
        dev->freq = atof(newval);
        applog(LOG_NOTICE, "dev %d: freq set to %lf", dev->id, dev->freq);
    }

    return NULL;
}

/* APIs support by qomo */
static const struct bfg_set_device_definition
qomo_set_device_funcs_live[] = {
    {"clock", qomo_set_device, "set chips frequency, number, unit MHz"},
    {"voltage", qomo_set_device, "set chips voltage, number, unit mV"},
    {"fan-speed", qomo_set_device, "set fan-speed, 5 levels, [1, 2, 3, 4, 5]"},
    {NULL}
};
static const struct bfg_set_device_definition
*qomo_set_device_funcs = qomo_set_device_funcs_live;

struct api_data* qomo_get_api_extra_device_detail(struct cgpu_info *cgpu) {
    struct qomo_device *dev = cgpu->device_data;
    struct api_data *res = NULL;
    char name[16];
    int i;

    res = api_add_int(res, "chips_nr", &dev->chips_nr, 1);
    res = api_add_string(res, "work mode", dev->sync_mode? "sync": "async", 1);

    for (i = 0; i < dev->chips_nr; i++) {
        sprintf(name, "c_bist_%d", i);
        res = api_add_uint32(res, name, &dev->chip_bist[i], 1);
    }

    return res;
}

struct api_data* qomo_get_api_extra_device_status(struct cgpu_info *cgpu) {
    struct qomo_device *dev = cgpu->device_data;
    struct api_data *res = NULL;
    char name[16];
    int i;

    res = api_add_volts(res, "voltage", &dev->voltage, 1);
    res = api_add_freq(res, "frequency", &dev->freq, 1);
    res = api_add_mhs(res, "estimate_hash_rate", &dev->hash_rate, 1);
    res = api_add_double(res, "range_scan_time", &dev->range_scan_time, 1);

    for (i = 0; i < dev->chips_nr; i++) {
        sprintf(name, "c_pf_%d", i);
        res = api_add_double(res, name, &dev->chip_perf[i], 1);
    }

    return res;
}

struct device_drv qomo_drv = {
    .dname = "intchains_qomo",
    .name = "INCS",

    /* bfgminer does not support spi lowl_probe
     * not implemented in lowl_scan()
     * so use drv_detect here
     */
    .drv_detect = qomo_drv_detect,

    .thread_init = qomo_thread_init,
    .thread_enable = qomo_thread_enable,
    .thread_disable = qomo_thread_disable,
    .thread_shutdown = qomo_thread_shutdown,

#ifdef SYNC_MODE == 1
    .scanhash = qomo_scanhash,
#else
    .minierloop = minerloop_queue,
    .queue_append = qomo_queue_append,
    .queue_flush = qomo_queue_flush,
    .poll = qomo_poll,
#endif

    /* qomo is for litecoin, so modify min_nonce_diff to pass
     * the algorithm check
     */
    .drv_min_nonce_diff = common_scrypt_min_nonce_diff,

    /* API */
    .get_api_extra_device_detail = qomo_get_api_extra_device_detail,
    .get_api_extra_device_status = qomo_get_api_extra_device_status,
};

