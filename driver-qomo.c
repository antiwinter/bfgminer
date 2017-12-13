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

#define UNION_MODE

struct qomo_device {
    int sig0, sig1, sig2, sig3;
    char *spi_path, *iic_path;
    int slave_addr;
    struct spi_port spi;
    int chip_count;
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

static const struct qomo_device qomo_devices[] = {
    {0, 0, 0, 0, "/dev/spidev0.0", "/dev/i2c-dev1", 0x20},
    {0, 0, 0, 0, "/dev/spidev0.1", "/dev/i2c-dev1", 0x21},
    {0, 0, 0, 0, "/dev/spidev1.0", "/dev/i2c-dev1", 0x22},
    {0, 0, 0, 0, "/dev/spidev1.1", "/dev/i2c-dev1", 0x23},
};

static inline void __qomo_issue(struct qomo_device *dev, uint16_t cmd,
        uint8_t *in, int in_len) {

    memcpy(dev->echo, &cmd, 2);
    memcpy(dev->echo, in, in_len);
    memcpy(dev->echo, "\x00\x00", 2);

    spi_clear_buf(dev->spi);
    spi_emit_buf(dev->spi, dev->issue, in_len + 4);
    spi_txrx(dev->spi);
}

static inline int __qomo_echo(struct qomo_device *dev, int n) {
    spi_clear_buf(dev->spi);
    spi_emit_nop(dev->spi, n);
    return spi_txrx(dev->spi);
}

static inline int __qomo_exec_cmd(struct qomo_device *dev,
        uint16_t cmd, int chip_addr, uint8_t *in, int in_len) {

    /* back up tx data to dev->echo */
    cmd |= chip_addr;
    memcpy(dev->echo, &cmd, 2);
    if (likely(in_len)) {
        if (in)
            memcpy(dev->echo + 2, in, in_len);
        else
            memset(dev->echo + 2, 0, in_len);
    }
    memcpy(dev->echo + 2 + in_len, "\x00\x00", 2);

    /* send tx data */
    spi_clear_buf(dev->spi);
    spi_emit_buf(dev->spi, dev->echo, in_len + 4);
    spi_txrx(dev->spi);

    /* send demanding dummy to get the response */
    n = chip_addr? 4 * chip_addr - 2: 4 * dev->chip_count;
    spi_clear_buf(dev->spi);
    spi_emit_nop(dev->spi, n);
    return spi_txrx(dev->spi);
}

/* check if the last m bytes in rx buffer is identical with
 * dev->echo
 */
static inline int
__qomo_echo_assert(struct qomo_device *dev, int m) {
    if (memcmp(dev->echo, dev->spi->spibuf_rx
                + dev->spi->spibufsz - m, m) != 0) {
        applog(LOG_ERROR, "echo is not same as expected!");
        // bin2hex ...
        return -1;
    }

    return 0;
}

static inline uint16_t
__qomo_echo_get_word(struct qomo_device *dev, int m) {
    return (uint16_t)(dev->spi->spibuf_rx + dev->spi->spibufsz - m);
}

static int qomo_exec_cmd(struct qomo_device *dev, enum qomo_commands cmd,
        int chip_addr, uint8_t *in, int *out) {
    struct spi_port *spi = &dev->spi_port;

#define refuse_single(x...) do { \
    if (chip_addr) { \
        applog(LOG_ERROR, "cmd %d does not support single mode", cmd); \
        return -1; \
    } } while (0)

#define refuse_bcast(x...) do { \
    if (!chip_addr) { \
        applog(LOG_ERROR, "cmd %d does not support broadcast mode", cmd); \
        return -1; \
    } } while (0)

    switch(cmd) {
        case QOMO_REG_WRITE:
            refuse_single();
            __qomo_exec_cmd(dev, cmd, 0, in, 8);
            return __qomo_echo_assert(dev, 12);

        case QOMO_REG_READ:
            refuse_bcast();
            __qomo_exec_cmd(dev, cmd, chip_addr, NULL, 14);
            if (__qomo_echo_get_word(dev, 20) != cmd | 0x1000 | chip_addr) {
                applog(LOG_ERROR, "unexpected echo for cmd 0x%04x", cmd);
                return -1;
            }
            memcpy(out, spi->spibuf_rx + spi->spibufsz - 18, 16);
            return 0;

        case QOMO_BIST:
            __qomo_exec_cmd(dev, cmd, chip_addr, NULL, 2);
            return __qomo_echo_assert(dev, 6);

        case QOMO_BIST_FIX:
            refuse_single();
            __qomo_exec_cmd(dev, cmd, 0, NULL, 0);
            return __qomo_echo_assert(dev, 4);

        case QOMO_AUTO_ADDRESSING:
            refuse_single();
            __qomo_exec_cmd(dev, cmd, 0, in, 2);
            if (__qomo_echo_get_word(dev, 6) != cmd) {
                applog(LOG_ERROR, "unexpected echo for cmd 0x%04x", cmd);
                return -1;
            }
            *(int *)out = __qomo_echo_get_word(dev, 4);
            return 0;

        case QOMO_WRITE_JOB:
            __qomo_exec_cmd(dev, cmd, chip_addr, in, 88);
            memset(dev->echo + 2, 0, 88);
            return __qomo_echo_assert(dev, 92);

        case QOMO_GET_NONCE:
            int *ret = out, val;
            __qomo_exec_cmd(dev, cmd, chip_addr, in, 4);
            val = __qomo_echo_get_word(dev, 8);
            if (val & 0x0f00 != cmd) {
                applog(LOG_ERROR, "unexpected echo for cmd 0x%04x", cmd);
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

    applog(LOG_ERROR, "unknown qomo cmd 0x%04x", cmd);
    return -1;
}

static bool qomo_lowl_probe(const struct lowlevel_device_info * info)
{
    int bus, cs, chains = 0, chips;
    char data[64];

    for (d = 0; d < ARRAY_SIZE(qomo_devices); d++) {
        struct qomo_device *dev = qomo_devices[d];
        dev->spi.speed = 3000000;
        dev->spi.delay = 0;
        dev->spi.mode = SPI_MODE_1;
        dev->spi.bits = 8;

        if(spi_open(&dev->spi, dev->spi_path) < 0) {
            applog(LOG_NOTICE, "%s not found", dev->spi_path);
            continue;
        }

        /*
         * 1. configure DCDC to get correct voltage for each computing board
         * 2. perform HW reset for each computing board
         */

        /* 3. configure PLL and SPI clock for each compute board
        */
        if(qomo_exec_cmd(dev, QOMO_WRITE_REG, data) < 0) {
            applog(LOG_ERROR, "board %d does not response to cmd 0x%x", d, QOMO_WRITE_REG);
        }

        /* 4. BIST
        */
        if(qomo_exec_cmd(dev, QOMO_BIST, data) < 0) {
        }

        /* 5. auto addressing
        */
        if(qomo_exec_cmd(dev, QOMO_AUTO_ADDRESSING, data) < 0) {
            chips = 0;
        }

        /* 6. Mask out bad cores inside each chip
        */
        if(qomo_exec_cmd(dev, QOMO_BIST_FIX, data) < 0) {
        }

        /* 7. count good cores inside each chip (optional)
         */
        if(qomo_exec_cmd(dev, QOMO_BIST_FIX, data) < 0) {
        }

        /* add cgpu */
        struct cgpu_info *cgpu = malloc(sizeof(struct cgpu_info));
        cgpu->drv = &qomo_drv;
        cgpu->device_data = dev;
        cgpu->threads = 1;
        cgpu->procs = chips;
        add_cgpu(cgpu);
        chains++;
    }

    return !!chains;
}


static qomo_thread_init(struct thr_info *thr)
{
    applog(LOG_DEBUG, "%s, %d", __func__, __LINE__);
}

struct qomo_thread_enable(const struct thr_info * thr)
{
    applog(LOG_DEBUG, "%s, %d", __func__, __LINE__);
}

struct qomo_thread_disable(const struct thr_info * thr)
{
    applog(LOG_DEBUG, "%s, %d", __func__, __LINE__);
}

struct qomo_thread_shutdown(const struct thr_info * thr)
{
    applog(LOG_DEBUG, "%s, %d", __func__, __LINE__);
}

#ifdef UNION_MODE    
struct qomo_scanhash(struct thr_info *thr, struct work *work, int64_t __maybe_unused max_nonce)
{
    int nonce;
    struct cgpu_info *cgpu = thr->cgpu;
    struct qomo_device *dev = cgpu->device_data;
    struct timeval start_tv, end_tv, nonce_scanned_tv;

    qomo_exec_cmd(QOMO_WRITE_JOB, work);
    timer_set_now(&start_tv);
    timer_set_delay_from_now(&nonce_scanned_tv,
            dev->range_scan_us / dev->chip_nr_max);

    for(;;) {
        /* we control the hash loop for ourselves, and set
         * work->blk.nonce to 0xffffffff at the end, so that
         * the outer loop (minerloop_scanhash@deviceapi.c)
         * will abandon this work and trigger the next one.
         */

        /* new block arrived, abandon current jobs in all chips */
        if(thr->restart) {
            break;
        }

        /* range nearly scanned */
        if(timer_passed(&nonce_scanned_tv, NULL)) {
            goto __out;
        }

        /* scan for nonce */
        qomo_exec_cmd(QOMO_SCAN_NONCE, &nonce);
        if (nonce_found) {
            submit_nonce(thr, work, nonce);
        }

        /* sleep for a while thus let go of the bus */
        cgsleep_ms(100);
    }

    /* clear potential results for the current work */
    qomo_exec_cmd(QOMO_RESET);

    work->blk.nonce = 0xffffffff;
    timer_set_now(&end_tv);
    return (int)(dev->hash_rate * ms_tdiff(&end_tv, &start_tv) / 1000);
}
#else /* manage jobs for each chip separately */
#endif

struct device_drv qomo_drv = {
    .dname = "intchains_qomo",
    .name = "INCS",

    .lowl_probe = qomo_lowl_probe,

    .thread_init = qomo_thread_init,
    .thread_enable = qomo_thread_enable,
    .thread_disable = qomo_thread_disable,
    .thread_shutdown = qomo_thread_shutdown,

#ifdef UNION_MODE    
    .scanhash = qomo_scanhash,
#else
    .minierloop = minerloop_queue,
    .queue_append = qomo_queue_append,
    .queue_flush = qomo_queue_flush,
    .poll = qomo_poll,
#endif
};

