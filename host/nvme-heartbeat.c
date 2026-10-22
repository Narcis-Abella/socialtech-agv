// nvme-heartbeat: one 4 KiB O_DIRECT read per online CPU every PERIOD_MS, issued from a thread pinned to that CPU.
//
// Why: on Jetson Orin (L4T r39, kernel 6.8 tegra) NVMe completion MSIs are sometimes lost; the CQE sits in the queue
// until the 30 s blk-mq timeout ("nvme nvme0: I/O tag N QID Q timeout, completion polled") and its issuer blocks.
// blk-mq sends I/O from CPU n to hw queue n, and the next completion IRQ of a queue drains every pending CQE, so a
// read on every CPU each period bounds such a stall to about one period. Reads are async (Linux AIO): a heartbeat
// read whose own MSI is lost must not stop the next one from being issued.
//
// usage: nvme-heartbeat FILE PERIOD_MS SECONDS [CPULIST]
//   FILE      the NVMe namespace (/dev/nvme0n1, read-only, bytes 0..128 KiB) or a regular file on it of >= 128 KiB
//             whose blocks are really written (not sparse/fallocated: O_DIRECT reads of holes never reach the device).
//   PERIOD_MS 1..1000. SECONDS: 0 = run until SIGTERM/SIGINT. CPULIST: e.g. 1,3-5 (default: all online CPUs).
// exit: 0 stopped by signal or SECONDS; 1 setup error (open, test read, io_setup, thread/pin); 2 usage;
//       3 online CPU set changed (restart to cover the new set).
// log (stderr, one key=value line each; event lines are limited to one per CPU and event kind per second, summary counts all):
//   event=start file=F period_ms=P cpus=L
//   event=slow_read cpu=C ms=X        read not reaped by the next tick (> 1.5 periods): lost MSI healed, or slow disk. X is the
//                                     slowest read reaped at that tick. Latency is sampled at period ticks (one-period resolution).
//   event=read_error cpu=C res=R | event=submit_error cpu=C errno=E | event=queue_full cpu=C (all 32 reads in flight)
//   event=summary cpu=C reads= slow= errors= full= max_ms= max_gap_ms= inflight=   (hourly and at exit;
//                                     max_gap_ms = longest interval between two reads issued on C = the heal bound)
//   event=stop reason=signal|cpu_online_changed rc=N | event=warn ... | event=fatal ...
// build: gcc -O2 -Wall -Wextra -Werror -pthread -o nvme-heartbeat nvme-heartbeat.c
#define _GNU_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <linux/aio_abi.h>
#include <pthread.h>
#include <sched.h>
#include <signal.h>
#include <stdatomic.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/syscall.h>
#include <time.h>
#include <unistd.h>

#define DEPTH 32      // reads in flight per CPU before event=queue_full: 32 periods without any completion
#define BLK 4096
#define SEC 1000000000LL
#define ONLINE "/sys/devices/system/cpu/online"

struct beat { int cpu; aio_context_t ctx; char *buf; pthread_t th; };
struct stats { long reads, slow, errors, full; int64_t max_lat, max_gap; };
static int fd, period_ms;
static int64_t period;    // ns
static atomic_int stop;

static int64_t now_ns(void) { struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t); return t.tv_sec * SEC + t.tv_nsec; }
// One limiter per event kind: a flood of one (queue_full, every tick while 32 reads are stuck) must not hide another (slow_read).
enum { EV_GET, EV_READ, EV_SLOW, EV_FULL, EV_SUBMIT, EV_KINDS };
static int may_log(int64_t now, int64_t *last) { if (now - *last < SEC) return 0; *last = now; return 1; }

static void summary(int cpu, const struct stats *s, int inflight) {
    fprintf(stderr, "event=summary cpu=%d reads=%ld slow=%ld errors=%ld full=%ld max_ms=%.1f max_gap_ms=%.1f inflight=%d\n",
            cpu, s->reads, s->slow, s->errors, s->full, s->max_lat / 1e6, s->max_gap / 1e6, inflight);
}

static void *run(void *arg) {
    struct beat *b = arg;
    struct iocb cb[DEPTH], *cbp; struct io_event ev[DEPTH]; struct timespec zero = {0, 0};
    int64_t t_sub[DEPTH], next = now_ns(), last_sub = 0, last_log[EV_KINDS]; int busy[DEPTH] = {0}, inflight = 0, grace = 0;
    struct stats st = {0};
    for (int k = 0; k < EV_KINDS; k++) last_log[k] = -SEC;
    for (long tick = 1;; tick++) {
        int64_t now = now_ns();
        int n = syscall(SYS_io_getevents, b->ctx, 0, DEPTH, ev, &zero);
        if (n < 0 && may_log(now, &last_log[EV_GET])) fprintf(stderr, "event=getevents_error cpu=%d errno=%d\n", b->cpu, errno);
        int64_t worst = 0;   // slowest read reaped now
        for (int i = 0; i < n; i++) {
            int s = (int)ev[i].data; int64_t d = now - t_sub[s];
            busy[s] = 0; inflight--; st.reads++;
            if (d > st.max_lat) st.max_lat = d;
            if (ev[i].res != BLK) {
                st.errors++;
                if (may_log(now, &last_log[EV_READ])) fprintf(stderr, "event=read_error cpu=%d res=%lld\n", b->cpu, (long long)ev[i].res);
            } else if (d > period * 3 / 2) {
                st.slow++;
                if (d > worst) worst = d;
            }
        }
        if (worst && may_log(now, &last_log[EV_SLOW])) fprintf(stderr, "event=slow_read cpu=%d ms=%.1f\n", b->cpu, worst / 1e6);
        // On stop keep beating until our own reads are reaped: exit_aio() waits for them, and one stuck on a lost
        // MSI would otherwise hold process exit for the 30 s timeout. Give up after 1 s.
        if (atomic_load_explicit(&stop, memory_order_relaxed) && (inflight == 0 || ++grace * period > SEC)) break;
        int s = 0;
        while (s < DEPTH && busy[s]) s++;
        if (s == DEPTH) {
            st.full++;
            if (may_log(now, &last_log[EV_FULL])) fprintf(stderr, "event=queue_full cpu=%d inflight=%d\n", b->cpu, inflight);
        } else {
            memset(&cb[s], 0, sizeof cb[s]);
            cb[s].aio_lio_opcode = IOCB_CMD_PREAD; cb[s].aio_fildes = fd; cb[s].aio_data = s;
            cb[s].aio_buf = (uintptr_t)(b->buf + (size_t)s * BLK); cb[s].aio_nbytes = BLK; cb[s].aio_offset = (int64_t)s * BLK;
            cbp = &cb[s]; t_sub[s] = now_ns();
            if (syscall(SYS_io_submit, b->ctx, 1, &cbp) == 1) {
                busy[s] = 1; inflight++;
                if (last_sub && t_sub[s] - last_sub > st.max_gap) st.max_gap = t_sub[s] - last_sub;
                last_sub = t_sub[s];
            } else {
                st.errors++;
                if (may_log(now, &last_log[EV_SUBMIT])) fprintf(stderr, "event=submit_error cpu=%d errno=%d\n", b->cpu, errno);
            }
        }
        if (tick % (3600000 / period_ms) == 0) summary(b->cpu, &st, inflight);
        next += period; now = now_ns();
        if (next <= now) next += ((now - next) / period + 1) * period;   // fell behind: skip missed ticks, no burst
        struct timespec ts = {next / SEC, next % SEC};
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL);   // signals are blocked here: no EINTR
    }
    summary(b->cpu, &st, inflight);
    return NULL;
}

static int parse_cpus(const char *s, cpu_set_t *set) {   // "0-3,5" -> set; -1 on syntax/range error
    CPU_ZERO(set);
    for (;;) {
        char *e; long a = strtol(s, &e, 10), b = a;
        if (e == s || a < 0) return -1;
        if (*e == '-') { s = e + 1; b = strtol(s, &e, 10); if (e == s || b < a) return -1; }
        if (b >= CPU_SETSIZE) return -1;
        for (long c = a; c <= b; c++) CPU_SET(c, set);
        if (!*e) return 0;
        if (*e != ',') return -1;
        s = e + 1;
    }
}

static int read_online(char *buf, size_t len) {
    FILE *f = fopen(ONLINE, "r"); int ok = f && fgets(buf, (int)len, f);
    if (f) fclose(f);
    if (ok) buf[strcspn(buf, "\n")] = 0;
    return ok ? 0 : -1;
}

static long num(const char *s, long lo, long hi) { char *e; long v = strtol(s, &e, 10); return (e == s || *e || v < lo || v > hi) ? -1 : v; }

int main(int argc, char **argv) {
    char online[256], now_online[256]; cpu_set_t set;
    long secs = argc >= 4 ? num(argv[3], 0, 100000000) : -1;
    period_ms = argc >= 4 ? (int)num(argv[2], 1, 1000) : -1;
    if (argc < 4 || argc > 5 || period_ms < 0 || secs < 0) {
        fprintf(stderr, "usage: %s FILE PERIOD_MS(1..1000) SECONDS(0=until signal) [CPULIST e.g. 0-3,5]\n"
                        "One 4 KiB O_DIRECT read per CPU every PERIOD_MS (NVMe lost-MSI workaround). See source header.\n", argv[0]);
        return 2;
    }
    period = period_ms * 1000000LL;
    if (read_online(online, sizeof online)) { fprintf(stderr, "event=fatal read " ONLINE " errno=%d\n", errno); return 1; }
    if (parse_cpus(argc == 5 ? argv[4] : online, &set)) { fprintf(stderr, "event=fatal bad cpu list '%s'\n", argc == 5 ? argv[4] : online); return 2; }
    int ncpu = CPU_COUNT(&set);
    struct beat *b = calloc(ncpu, sizeof *b); char *buf;
    if ((fd = open(argv[1], O_RDONLY | O_DIRECT)) < 0) { fprintf(stderr, "event=fatal open %s errno=%d (%s)\n", argv[1], errno, strerror(errno)); return 1; }
    if (!b || posix_memalign((void **)&buf, BLK, (size_t)ncpu * DEPTH * BLK)) { fprintf(stderr, "event=fatal out of memory\n"); return 1; }
    // One O_DIRECT read of the last block of the window checks alignment (logical block size <= 4 KiB), size and access.
    ssize_t r = pread(fd, buf, BLK, (DEPTH - 1) * BLK);
    if (r != BLK) {
        fprintf(stderr, "event=fatal test read of %s returned %zd errno=%d (%s): need >= %d KiB readable with O_DIRECT\n",
                argv[1], r, r < 0 ? errno : 0, r < 0 ? strerror(errno) : "short", DEPTH * BLK / 1024);
        return 1;
    }
    sigset_t sigs; sigemptyset(&sigs); sigaddset(&sigs, SIGINT); sigaddset(&sigs, SIGTERM); sigaddset(&sigs, SIGALRM);
    pthread_sigmask(SIG_BLOCK, &sigs, NULL);   // inherited by the workers; only main takes signals, via sigtimedwait
    for (int c = 0, k = 0; k < ncpu; c++) {
        if (!CPU_ISSET(c, &set)) continue;
        b[k].cpu = c; b[k].buf = buf + (size_t)k * DEPTH * BLK;
        if (syscall(SYS_io_setup, DEPTH, &b[k].ctx)) { fprintf(stderr, "event=fatal io_setup cpu=%d errno=%d (%s)\n", c, errno, strerror(errno)); return 1; }
        cpu_set_t one; CPU_ZERO(&one); CPU_SET(c, &one);
        pthread_attr_t at; pthread_attr_init(&at);
        int e = pthread_attr_setaffinity_np(&at, sizeof one, &one);   // pthread_create fails if the CPU cannot be pinned
        if (!e) e = pthread_create(&b[k].th, &at, run, &b[k]);
        pthread_attr_destroy(&at);
        if (e) { fprintf(stderr, "event=fatal start cpu=%d err=%d (%s)\n", c, e, strerror(e)); return 1; }
        k++;
    }
    // Try to keep pages resident: a page fault in the heartbeat would itself need an NVMe read that may lose its MSI.
    // After thread start and without MCL_FUTURE: the loop maps nothing new, and a too small RLIMIT_MEMLOCK
    // (no CAP_IPC_LOCK) then only costs this warning instead of failing pthread_create for the stacks.
    // MCL_ONFAULT because MCL_CURRENT alone would populate every 8 MiB thread stack. Measured on x86 (smaps): it locks little of
    // the shared libc text (~6 of 1184 KiB), so this narrows the risk, it does not remove it. Open: small stacks + MCL_CURRENT.
    if (mlockall(MCL_CURRENT | MCL_ONFAULT)) fprintf(stderr, "event=warn mlockall errno=%d (%s)\n", errno, strerror(errno));
    fprintf(stderr, "event=start file=%s period_ms=%d cpus=%s\n", argv[1], period_ms, argc == 5 ? argv[4] : online);
    if (secs) alarm((unsigned)secs);
    int rc = 0; const char *why = "signal";
    for (;;) {
        struct timespec one_s = {1, 0};
        if (sigtimedwait(&sigs, NULL, &one_s) > 0) break;
        if (read_online(now_online, sizeof now_online) || strcmp(online, now_online)) { rc = 3; why = "cpu_online_changed"; break; }
    }
    atomic_store(&stop, 1);
    for (int k = 0; k < ncpu; k++) pthread_join(b[k].th, NULL);
    fprintf(stderr, "event=stop reason=%s rc=%d\n", why, rc);
    return rc;
}
