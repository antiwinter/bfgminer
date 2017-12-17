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
#include <stdbool.h>

#ifdef HAVE_LINUX_SPI_SPIDEV_H
#include <linux/spi/spidev.h>
#else
#define SPI_MODE_0 0
#endif

#include "deviceapi.h"
#include "lowl-spi.h"
#include "util.h"

/* SYNC_MODE: write job in braodcast mode, all chips in the chain will do the
 *            same job (nonce is divided into 128 range for each chip)
 * if not defined: request a work for each chip separately from bfgminer
 */
#define SYNC_MODE

#define CHIP_NR_MAX 128

BFG_REGISTER_DRIVER(qomo_drv)

struct qomo_device {
    int sig0, sig1, sig2, sig3;
    char *spi_path, *iic_path;
    char prep[512];
    int prep_sz;
    int slave_addr;
    struct spi_port spi;
    int chip_count;
    int range_scan_us;
    int hash_rate;
    int chip_perf[CHIP_NR_MAX];
    int chip_good_core[CHIP_NR_MAX];
};

enum qomo_commands {
    QOMO_REG_WRITE = 0x0900,
    QOMO_REG_READ = 0x0a00,
    QOMO_BIST = 0x0100,
    QOMO_BIST_FIX = 0x0300,
    QOMO_AUTO_ADDRESSING = 0x0200,
    QOMO_WRITE_JOB = 0x0700,
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

static inline int __qomo_exec_cmd(struct qomo_device *dev,
        uint16_t cmd, int chip_addr, uint8_t *in, int in_len) {

    struct spi_port *spi = &dev->spi;
    int ret;

    /* back up tx data to dev->prep */
    cmd |= chip_addr;
    cmd = bswap_16(cmd);

    memcpy(dev->prep, "\x5a\x3c", 2);
    memcpy(dev->prep, &cmd, 2);
    if (likely(in_len)) {
        if (in)
            memcpy(dev->prep + 4, in, in_len);
        else
            memset(dev->prep + 4, 0, in_len);
    }
    memcpy(dev->prep + 4 + in_len, "\x00\x00", 2);
    dev->prep_sz = in_len + 6;

    /* send tx data */
    spi_clear_buf(spi);
    spi_emit_buf(spi, dev->prep, in_len + 6);
    printbin("cmd send:", spi->spibuf, in_len + 6);
    spi_txrx(spi);

    /* send demanding dummy to get the response */
    spi_clear_buf(spi);
    spi_emit_nop(spi, chip_addr? 4 * chip_addr - 2: 4 * dev->chip_count);
    ret = spi_txrx(spi);
    printbin("cmd echo:", spi->spibuf_rx, chip_addr? 4 * chip_addr
            - 2: 4 * dev->chip_count);
    return ret;
}

/* check if the last m bytes in rx buffer is identical with
 * dev->prep
 */
static inline int
__qomo_echo_assert(struct qomo_device *dev) {
    if (memcmp(dev->prep, dev->spi.spibuf_rx
                + dev->spi.spibufsz - dev->prep_sz, dev->prep_sz) != 0) {
        applog(LOG_ERR, "echo is not same as expected!");
        // bin2hex ...
        return -1;
    }

    return 0;
}

/* get a word (16bit) from the last m bytes position */
static inline uint16_t
__qomo_echo_get_word(struct qomo_device *dev, int m) {
    return *(uint16_t *)(dev->spi.spibuf_rx + dev->spi.spibufsz - m);
}

static int qomo_exec_cmd(struct qomo_device *dev, enum qomo_commands cmd,
        int chip_addr, void *in, void *out) {
    struct spi_port *spi = &dev->spi;
    int *ret = out, val;

    return 0;
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
            if (__qomo_echo_get_word(dev, 20) != (cmd | 0x1000 | chip_addr)) {
                applog(LOG_ERR, "unexpected echo for cmd 0x%04x", cmd);
                return -1;
            }
            memcpy(out, spi->spibuf_rx + spi->spibufsz - 18, 16);
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
            if (__qomo_echo_get_word(dev, 6) != cmd) {
                applog(LOG_ERR, "unexpected echo for cmd 0x%04x", cmd);
                return -1;
            }
            *(int *)out = __qomo_echo_get_word(dev, 4);
            return 0;

        case QOMO_WRITE_JOB:
            __qomo_exec_cmd(dev, cmd, chip_addr, in, 88);
            memset(dev->prep + 2, 0, 88);
            return __qomo_echo_assert(dev);

        case QOMO_GET_NONCE:
            __qomo_exec_cmd(dev, cmd, chip_addr, in, 4);
            val = __qomo_echo_get_word(dev, 8);
            if ((val & 0x0f00) != cmd) {
                applog(LOG_ERR, "unexpected echo for cmd 0x%04x", cmd);
                return -1;
            }

            ret[0] = val >> 12;
            ret[1] = __qomo_echo_get_word(dev, 6);
            ret[1] <<= 16;
            ret[1] |= __qomo_echo_get_word(dev, 4);
            ret[2] = val & 0xff;
            return 0;

        case QOMO_RESET:
            __qomo_exec_cmd(dev, cmd, chip_addr, in, 2);
            return __qomo_echo_assert(dev);
    }

    applog(LOG_ERR, "unknown qomo cmd 0x%04x", cmd);
    return -1;
}

struct device_drv qomo_drv;
static void qomo_drv_detect(void)
{
    int chains = 0, chips, ret, d;
    char data[64];
    
    applog(LOG_NOTICE, "%s, %d", __func__, __LINE__);
    for (d = 0; d < ARRAY_SIZE(qomo_devices); d++) {
    applog(LOG_NOTICE, "%s, %d", __func__, __LINE__);
        struct qomo_device *dev = &qomo_devices[d];
        dev->spi.speed = 3000000;
        dev->spi.delay = 0;
        dev->spi.mode = SPI_MODE_0;
        dev->spi.bits = 8;
    applog(LOG_NOTICE, "%s, %d", __func__, __LINE__);

#ifdef HAVE_LINUX_SPI_SPIDEV_H
        if(spi_open(&dev->spi, dev->spi_path) < 0) {
            applog(LOG_NOTICE, "%s not found", dev->spi_path);
            continue;
        }
#endif
    applog(LOG_NOTICE, "%s, %d", __func__, __LINE__);

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
                 * BIST mode, mask update, job mode, power up mode
                 * RO 8 */
                "\x00\x0a", NULL);
        if (ret < 0) {
            continue;
        }
    applog(LOG_NOTICE, "%s, %d", __func__, __LINE__);
        /* TODO: calc difficulty and range_scan_us and hash_rate */
        dev->range_scan_us = 500000000;

        /* 4. BIST
        */
        if(qomo_exec_cmd(dev, QOMO_BIST, 0, NULL, NULL) < 0) {
            continue;
        };

    applog(LOG_NOTICE, "%s, %d", __func__, __LINE__);
        /* 5. auto addressing
        */
        if(qomo_exec_cmd(dev, QOMO_AUTO_ADDRESSING, 0, "\x00\x01",
                    &dev->chip_count) < 0) {
            continue;
        }
        applog(LOG_NOTICE, "%d chips detected in chain", dev->chip_count);

        /* 6. Mask out bad cores inside each chip
        */
        if(qomo_exec_cmd(dev, QOMO_BIST_FIX, 0, NULL, NULL) < 0) {
            continue;
        }
    applog(LOG_NOTICE, "%s, %d", __func__, __LINE__);

        /* 7. count good cores inside each chip (optional)
         */

        /* add cgpu, use calloc which init memory to zero */
    applog(LOG_NOTICE, "%s, %d", __func__, __LINE__);
        struct cgpu_info *cgpu = calloc(1, sizeof(struct cgpu_info));
    applog(LOG_NOTICE, "%s, %d, %p, %lu", __func__, __LINE__, cgpu, sizeof(struct cgpu_info));
        cgpu->drv = &qomo_drv;
        cgpu->device_data = dev;
        cgpu->threads = 1;
        cgpu->procs = chips;
    applog(LOG_NOTICE, "%s, %d", __func__, __LINE__);
        add_cgpu(cgpu);
    applog(LOG_NOTICE, "%s, %d", __func__, __LINE__);
        chains++;
    applog(LOG_NOTICE, "%s, %d", __func__, __LINE__);
    }

    if (chains) applog(LOG_NOTICE, "QomoMiner detected, %d chains totally!", chains);
}


static bool qomo_thread_init(struct thr_info *thr)
{
    applog(LOG_NOTICE, "%s, %d", __func__, __LINE__);
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

static void qomo_prepare_payload(uint8_t *pl, struct work *work)
{
    uint32_t i;

    applog(LOG_NOTICE, "%s, %d", __func__, __LINE__);
    wswap32cp(pl, &work->data[16], 64);
    wswap32cp(pl + 64, work->data, 16);

    /* start nonce */
    *(uint32_t *)&pl[64] = 0;

    /* end nonce */
    *(uint32_t *)&pl[84] = 0xffffffff;

    /* difficulty */
    for (i = 31; i >= 2 && !work->target[i]; --i);
    wswap32cp(pl + 80, work->target + i - 2, 4);
    pl[81] = i + 1;
}

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
for (;;);
}

#ifdef SYNC_MODE    
static int64_t qomo_scanhash(struct thr_info *thr, struct work *work,
        int64_t __maybe_unused max_nonce)
{
    int nonce, ret[4];
    struct cgpu_info *cgpu = thr->cgpu;
    struct qomo_device *dev = cgpu->device_data;
    struct timeval start_tv, end_tv, nonce_scanned_tv;
    uint8_t payload[128];

    test_byte_order(work);
    qomo_prepare_payload(payload, work);
    qomo_exec_cmd(dev, QOMO_WRITE_JOB, 0, payload, NULL);
    timer_set_now(&start_tv);
    timer_set_delay_from_now(&nonce_scanned_tv,
            dev->range_scan_us / CHIP_NR_MAX);

    for(;;) {
        /* we control the hash loop for ourselves, and set
         * work->blk.nonce to 0xffffffff at the end, so that
         * the outer loop (minerloop_scanhash@deviceapi.c)
         * will abandon this work and trigger the next one.
         */

        /* new block arrived, abandon current jobs in all chips */
        if(thr->work_restart) {
            applog(LOG_NOTICE, "work_restart required");
            break;
        }

        /* range nearly scanned */
        if(timer_passed(&nonce_scanned_tv, NULL)) {
            applog(LOG_NOTICE, "range scanned (time passed)");
            break;
        }

        /* scan for nonce */
        qomo_exec_cmd(dev, QOMO_GET_NONCE, 0, "\x00\x00\x00\x00", ret);
//        if (ret[0]) {
        if (1) {
            uint32_t _nonce[6] = {
                0xbe8c9a19,
                0x8e54a019,
                0xd8249a19,
                0x1a5d9a19,
                0x029c9a19,
                0x70c19a19,
            };
            static int nnn = 0;
            ret[1] = _nonce[nnn++];
            nnn = nnn > 3? 0: nnn;
 //           applog(LOG_NOTICE, "submitting nonce: 0x%08x", ret[1]);
            submit_nonce(thr, work, ret[1]);
            dev->chip_perf[ret[2]]++;
        }

        /* sleep for a while thus let go of the bus */
        cgsleep_ms(1000);
    }

    /* clear potential results for the current work
     * reserved 5, output queue, BIST, cores
     */
    qomo_exec_cmd(dev, QOMO_RESET, 0, "\x00\x04", NULL);

    work->blk.nonce = 0xffffffff;
    timer_set_now(&end_tv);
    return (int)(dev->hash_rate * ms_tdiff(&end_tv, &start_tv) / 1000);
}
#else /* manage jobs for each chip separately */
#endif

#if 0
static const struct bfg_set_device_definition
qomo_set_device_funcsp[] = {
    {"clock", qomo_}
};
#endif

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

#ifdef SYNC_MODE    
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
};

