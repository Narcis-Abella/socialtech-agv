// Fake AIO device for test_nvme_heartbeat.sh; nvme-heartbeat.c is built with -Dsyscall=fake_syscall. Reads submitted before
// HEAL_MS (ms after the first call) are stuck and complete all at once then (a lost MSI cured by the next IRQ of the queue);
// later reads complete at the next poll. Single worker thread only (the test pins one CPU).
#define _GNU_SOURCE
#include <linux/aio_abi.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdlib.h>
#include <sys/syscall.h>
#include <time.h>

static int64_t ms(void) { struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t); return t.tv_sec * 1000LL + t.tv_nsec / 1000000; }
static int64_t t0;
static uint64_t pend[4096];
static int np;

long fake_syscall(long no, ...) {
    va_list ap; va_start(ap, no);
    long a1 = va_arg(ap, long), a2 = va_arg(ap, long), a3 = va_arg(ap, long), a4 = va_arg(ap, long);
    va_end(ap); (void)a1;
    if (!t0) t0 = ms();
    if (no == SYS_io_setup) { *(aio_context_t *)a2 = 1; return 0; }
    if (no == SYS_io_submit) { pend[np++] = ((struct iocb **)a3)[0]->aio_data; return 1; }
    if (no == SYS_io_getevents) {
        if (ms() - t0 < atol(getenv("HEAL_MS"))) return 0;
        struct io_event *ev = (struct io_event *)a4; int n = np;
        for (int i = 0; i < n; i++) { ev[i].data = pend[i]; ev[i].res = 4096; }
        np = 0; return n;
    }
    return -1;
}
