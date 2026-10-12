/**
 * CAN 直连正弦测试 — 绕过软件栈, 直接发 PTM 帧到电机
 *
 * 编译: gcc -O2 -o can_sine_test can_sine_test.c -lm
 * 用法:
 *   sudo ./can_sine_test --bus bcan2 --motors 1,2,3,4
 *   sudo ./can_sine_test --bus bcan2 --motors 1,2,3,4,9,10 --sine 1,2,3
 *   sudo ./can_sine_test --bus bcan3 --motors 5,6,7,8 --sine 5,6,7
 *   sudo ./can_sine_test --bus bcan2 --motors 1,2,3,4 --amp 0.1 --freq 200 --duration 20
 *
 * 运行前必须停止 hardware_node!
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include <signal.h>
#include <time.h>
#include <errno.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>

/* ── PTM 协议编码范围 ── */
#define THETA_MIN  (-12.5f)
#define THETA_MAX  (12.5f)
#define VEL_MIN    (-10.0f)
#define VEL_MAX    (10.0f)
#define KP_MIN     (0.0f)
#define KP_MAX     (250.0f)
#define KD_MIN     (0.0f)
#define KD_MAX     (50.0f)
#define TAU_MIN    (-50.0f)
#define TAU_MAX    (50.0f)

#define MAX_MOTORS     16
#define MAX_LOG_ROWS   500000

static volatile int g_running = 1;
static void sig_handler(int s) { (void)s; g_running = 0; }

/* ── 配置 ── */
typedef struct {
    char     bus[32];
    int      motor_ids[MAX_MOTORS];
    int      n_motors;
    int      sine_ids[MAX_MOTORS];    /* 做正弦的电机 ID, 0 = 未指定 */
    int      n_sine;
    int      ctrl_freq;
    double   sine_hz;
    double   amp;
    double   kp;
    double   kd;
    double   head_kp;
    double   head_kd;
    int      duration;
} Config;

static void config_defaults(Config *c) {
    strcpy(c->bus, "bcan2");
    c->n_motors = 0;
    c->n_sine = 0;
    c->ctrl_freq = 250;
    c->sine_hz = 0.833;
    c->amp = 0.125;
    c->kp = 14.25;
    c->kd = 0.907;
    c->head_kp = 10.0;
    c->head_kd = 1.0;
    c->duration = 15;
}

static int parse_id_list(const char *str, int *out, int max) {
    int n = 0;
    char buf[256];
    strncpy(buf, str, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';
    char *tok = strtok(buf, ",");
    while (tok && n < max) {
        out[n++] = atoi(tok);
        tok = strtok(NULL, ",");
    }
    return n;
}

static int is_sine_motor(const Config *c, int mid) {
    if (c->n_sine == 0) return 1;  /* 未指定 --sine 则全部做正弦 */
    for (int i = 0; i < c->n_sine; i++)
        if (c->sine_ids[i] == mid) return 1;
    return 0;
}

static int is_head_motor(int mid) { return mid == 9 || mid == 10; }

static void parse_args(int argc, char *argv[], Config *c) {
    config_defaults(c);
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--bus") && i + 1 < argc)
            strncpy(c->bus, argv[++i], sizeof(c->bus) - 1);
        else if (!strcmp(argv[i], "--motors") && i + 1 < argc)
            c->n_motors = parse_id_list(argv[++i], c->motor_ids, MAX_MOTORS);
        else if (!strcmp(argv[i], "--sine") && i + 1 < argc)
            c->n_sine = parse_id_list(argv[++i], c->sine_ids, MAX_MOTORS);
        else if (!strcmp(argv[i], "--freq") && i + 1 < argc)
            c->ctrl_freq = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--sine-hz") && i + 1 < argc)
            c->sine_hz = atof(argv[++i]);
        else if (!strcmp(argv[i], "--amp") && i + 1 < argc)
            c->amp = atof(argv[++i]);
        else if (!strcmp(argv[i], "--kp") && i + 1 < argc)
            c->kp = atof(argv[++i]);
        else if (!strcmp(argv[i], "--kd") && i + 1 < argc)
            c->kd = atof(argv[++i]);
        else if (!strcmp(argv[i], "--duration") && i + 1 < argc)
            c->duration = atoi(argv[++i]);
        else if (!strcmp(argv[i], "-h") || !strcmp(argv[i], "--help")) {
            fprintf(stderr,
                "用法: sudo %s --bus <iface> --motors <id,...> [选项]\n\n"
                "选项:\n"
                "  --bus <iface>      CAN 接口 (默认 bcan2)\n"
                "  --motors <ids>     电机 ID 列表 (必需)\n"
                "  --sine <ids>       只对这些电机做正弦, 其余保持 (默认全部)\n"
                "  --freq <Hz>        控制频率 (默认 250)\n"
                "  --sine-hz <Hz>     正弦频率 (默认 0.833)\n"
                "  --amp <rad>        正弦振幅 (默认 0.125)\n"
                "  --kp <val>         位置 Kp (默认 14.25)\n"
                "  --kd <val>         位置 Kd (默认 0.907)\n"
                "  --duration <sec>   测试时长 (默认 15)\n\n"
                "示例:\n"
                "  sudo %s --bus bcan2 --motors 1,2,3,4\n"
                "  sudo %s --bus bcan2 --motors 1,2,3,4,9,10 --sine 1,2,3\n"
                "  sudo %s --bus bcan3 --motors 5,6,7,8 --amp 0.1\n",
                argv[0], argv[0], argv[0], argv[0]);
            exit(0);
        }
    }
    if (c->n_motors == 0) {
        fprintf(stderr, "错误: 必须指定 --motors\n运行 %s --help 查看用法\n", argv[0]);
        exit(1);
    }
}

/* ── 编解码 ── */

static inline unsigned int float_to_uint(float x, float min, float max, int bits) {
    if (x < min) x = min;
    if (x > max) x = max;
    return (unsigned int)((x - min) / (max - min) * ((1 << bits) - 1));
}

static inline float uint_to_float(unsigned int x, float min, float max, int bits) {
    return (float)x / (float)((1 << bits) - 1) * (max - min) + min;
}

static void encode_ptm(float pos, float vel, float kp, float kd, float tau, __u8 out[8]) {
    unsigned int th   = float_to_uint(pos, THETA_MIN, THETA_MAX, 16);
    unsigned int v    = float_to_uint(vel, VEL_MIN, VEL_MAX, 12);
    unsigned int kp_i = float_to_uint(kp, KP_MIN, KP_MAX, 12);
    unsigned int kd_i = float_to_uint(kd, KD_MIN, KD_MAX, 12);
    unsigned int t    = float_to_uint(tau, TAU_MIN, TAU_MAX, 12);
    out[0] = (th >> 8) & 0xFF;
    out[1] = th & 0xFF;
    out[2] = (v >> 4) & 0xFF;
    out[3] = ((v & 0x0F) << 4) | ((kp_i >> 8) & 0x0F);
    out[4] = kp_i & 0xFF;
    out[5] = (kd_i >> 4) & 0xFF;
    out[6] = ((kd_i & 0x0F) << 4) | ((t >> 8) & 0x0F);
    out[7] = t & 0xFF;
}

static void decode_feedback(const __u8 d[8], float *pos, float *vel, float *tau) {
    unsigned int th = ((unsigned int)d[0] << 8) | d[1];
    unsigned int v  = ((unsigned int)d[2] << 4) | (d[3] >> 4);
    unsigned int t  = ((unsigned int)(d[6] & 0x0F) << 8) | d[7];
    *pos = uint_to_float(th, THETA_MIN, THETA_MAX, 16);
    *vel = uint_to_float(v, VEL_MIN, VEL_MAX, 12);
    *tau = uint_to_float(t, TAU_MIN, TAU_MAX, 12);
}

/* ── 日志 ── */

typedef struct {
    double time_ms;
    int    motor_id;
    char   dir;       /* 'T' = TX cmd, 'R' = RX feedback */
    float  pos;
    float  vel;
    float  tau;
} LogEntry;

static LogEntry g_log[MAX_LOG_ROWS];
static int g_log_count = 0;

static inline void log_add(double t_ms, int mid, char dir, float p, float v, float t) {
    if (g_log_count < MAX_LOG_ROWS) {
        LogEntry *e = &g_log[g_log_count++];
        e->time_ms = t_ms;
        e->motor_id = mid;
        e->dir = dir;
        e->pos = p;
        e->vel = v;
        e->tau = t;
    }
}

/* ── 时间 ── */

static inline double ts_ms(struct timespec *ts, struct timespec *t0) {
    return (ts->tv_sec - t0->tv_sec) * 1000.0 + (ts->tv_nsec - t0->tv_nsec) / 1e6;
}

/* ── 读取所有 RX 帧 ── */
static void drain_rx(int sock, struct timespec *t0, const int *ids, int n_ids) {
    struct can_frame frame;
    struct timespec now;
    while (1) {
        int n = recv(sock, &frame, sizeof(frame), MSG_DONTWAIT);
        if (n <= 0) break;
        clock_gettime(CLOCK_MONOTONIC, &now);
        canid_t cid = frame.can_id & CAN_SFF_MASK;
        /* 只记录我们关心的电机 */
        for (int i = 0; i < n_ids; i++) {
            if ((int)cid == ids[i]) {
                float fp, fv, ft;
                decode_feedback(frame.data, &fp, &fv, &ft);
                log_add(ts_ms(&now, t0), cid, 'R', fp, fv, ft);
                break;
            }
        }
    }
}

/* ── 读初始位置: 从使能后的反馈帧获取, 不发任何运动指令 ── */
static int read_init_positions(int sock, const Config *cfg, float *init_pos, int *valid) {
    struct can_frame rx;

    /* 标记每个电机是否已读到有效位置 */
    for (int i = 0; i < cfg->n_motors; i++) valid[i] = 0;

    /* 使能帧本身会触发反馈, 额外等待并读取 */
    for (int attempt = 0; attempt < 50; attempt++) {
        usleep(10000);
        while (recv(sock, &rx, sizeof(rx), MSG_DONTWAIT) > 0) {
            canid_t cid = rx.can_id & CAN_SFF_MASK;
            for (int i = 0; i < cfg->n_motors; i++) {
                if (cfg->motor_ids[i] == (int)cid && rx.can_dlc >= 8) {
                    float p, v, t;
                    decode_feedback(rx.data, &p, &v, &t);
                    /* 合法性: 位置不应在编码范围的极值附近 (垃圾数据标志) */
                    if (p > THETA_MIN + 0.5f && p < THETA_MAX - 0.5f) {
                        init_pos[i] = p;
                        valid[i] = 1;
                    }
                }
            }
        }

        /* 所有电机都读到了? */
        int all_valid = 1;
        for (int i = 0; i < cfg->n_motors; i++)
            if (!valid[i]) { all_valid = 0; break; }
        if (all_valid) return 1;
    }
    return 0;  /* 超时, 部分电机未读到有效位置 */
}

/* ── main ── */

int main(int argc, char *argv[]) {
    Config cfg;
    parse_args(argc, argv, &cfg);

    signal(SIGINT, sig_handler);
    signal(SIGTERM, sig_handler);

    printf("══════════════════════════════════════════\n");
    printf("  CAN 直连正弦测试\n");
    printf("  总线: %s  电机:", cfg.bus);
    for (int i = 0; i < cfg.n_motors; i++) printf(" 0x%02X", cfg.motor_ids[i]);
    printf("\n  正弦电机:");
    if (cfg.n_sine == 0) printf(" (全部)");
    else for (int i = 0; i < cfg.n_sine; i++) printf(" 0x%02X", cfg.sine_ids[i]);
    printf("\n  控制频率: %dHz  正弦: %.3fHz  振幅: %.3frad\n", cfg.ctrl_freq, cfg.sine_hz, cfg.amp);
    printf("  Kp: %.2f  Kd: %.3f  时长: %ds\n", cfg.kp, cfg.kd, cfg.duration);
    printf("══════════════════════════════════════════\n\n");

    /* ── 打开 SocketCAN ── */
    int sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock < 0) { perror("socket"); return 1; }

    struct ifreq ifr;
    strncpy(ifr.ifr_name, cfg.bus, IFNAMSIZ - 1);
    if (ioctl(sock, SIOCGIFINDEX, &ifr) < 0) { perror("ioctl SIOCGIFINDEX"); return 1; }

    struct sockaddr_can addr;
    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) { perror("bind"); return 1; }

    /* 非阻塞 + 接收自发帧 */
    int recv_own = 1;
    setsockopt(sock, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &recv_own, sizeof(recv_own));
    printf("[OK] SocketCAN %s 已打开\n", cfg.bus);

    /* ── 使能电机 ── */
    struct can_frame frame;
    memset(&frame, 0, sizeof(frame));
    frame.can_dlc = 8;

    printf("使能电机...\n");
    __u8 enable[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC};
    for (int i = 0; i < cfg.n_motors; i++) {
        frame.can_id = cfg.motor_ids[i];
        memcpy(frame.data, enable, 8);
        write(sock, &frame, sizeof(frame));
        usleep(5000);
    }
    usleep(500000);
    while (recv(sock, &frame, sizeof(frame), MSG_DONTWAIT) > 0) {} /* 清空 */
    printf("[OK] 电机已使能\n");

    /* ── 读初始位置 (从使能响应帧, 不发运动指令) ── */
    float init_pos[MAX_MOTORS];
    int pos_valid[MAX_MOTORS];
    memset(init_pos, 0, sizeof(init_pos));
    memset(pos_valid, 0, sizeof(pos_valid));

    printf("读取初始位置...\n");
    int all_ok = read_init_positions(sock, &cfg, init_pos, pos_valid);

    printf("\n初始位置:\n");
    int any_invalid = 0;
    for (int i = 0; i < cfg.n_motors; i++) {
        int mid = cfg.motor_ids[i];
        const char *role = is_sine_motor(&cfg, mid) ? "正弦" : "保持";
        if (pos_valid[i]) {
            printf("  0x%02X: %+.4f rad (%+.2f°) [%s]\n",
                   mid, init_pos[i], init_pos[i] * 180.0 / M_PI, role);
        } else {
            printf("  0x%02X: *** 未读到有效位置 *** [%s]\n", mid, role);
            any_invalid = 1;
        }
    }
    printf("\n");

    if (any_invalid) {
        printf("\033[31m错误: 部分电机未返回有效位置反馈, 中止测试 (安全保护)\033[0m\n");
        printf("可能原因: 电机未连接 / 电机未使能成功 / CAN 通信故障\n");
        /* 尝试停用已使能的电机 */
        __u8 dis[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFD};
        for (int i = 0; i < cfg.n_motors; i++) {
            frame.can_id = cfg.motor_ids[i];
            memcpy(frame.data, dis, 8);
            write(sock, &frame, sizeof(frame));
            usleep(2000);
        }
        close(sock);
        return 1;
    }

    /* ── 先用真实 kp/kd 保持位置 1 秒, 让电机稳定 ── */
    printf("稳定中 (1s)...\n");
    {
        struct timespec stab_next;
        clock_gettime(CLOCK_MONOTONIC, &stab_next);
        long period_ns = 1000000000L / cfg.ctrl_freq;
        for (int c = 0; c < cfg.ctrl_freq && g_running; c++) {
            stab_next.tv_nsec += period_ns;
            if (stab_next.tv_nsec >= 1000000000L) { stab_next.tv_sec++; stab_next.tv_nsec -= 1000000000L; }
            for (int i = 0; i < cfg.n_motors; i++) {
                int mid = cfg.motor_ids[i];
                float kp = is_head_motor(mid) ? cfg.head_kp : cfg.kp;
                float kd = is_head_motor(mid) ? cfg.head_kd : cfg.kd;
                frame.can_id = mid;
                frame.can_dlc = 8;
                encode_ptm(init_pos[i], 0, kp, kd, 0, frame.data);
                write(sock, &frame, sizeof(frame));
            }
            while (recv(sock, &frame, sizeof(frame), MSG_DONTWAIT) > 0) {}
            clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &stab_next, NULL);
        }
    }
    if (!g_running) goto cleanup;
    printf("[OK] 电机已稳定\n\n");

    /* ── 正弦控制循环 ── */
    {
        struct timespec t0, next_time, now;
        clock_gettime(CLOCK_MONOTONIC, &t0);
        next_time = t0;
        long period_ns = 1000000000L / cfg.ctrl_freq;
        int cycle = 0, tx_count = 0, tx_drop = 0;
        int total_cycles = cfg.duration * cfg.ctrl_freq;

        printf("开始正弦测试 (%ds, %d cycles)... Ctrl+C 停止\n\n", cfg.duration, total_cycles);

        while (g_running && cycle < total_cycles) {
            next_time.tv_nsec += period_ns;
            if (next_time.tv_nsec >= 1000000000L) { next_time.tv_sec++; next_time.tv_nsec -= 1000000000L; }

            clock_gettime(CLOCK_MONOTONIC, &now);
            double t_sec = (now.tv_sec - t0.tv_sec) + (now.tv_nsec - t0.tv_nsec) / 1e9;

            for (int i = 0; i < cfg.n_motors; i++) {
                int mid = cfg.motor_ids[i];
                float center = init_pos[i];
                float cmd_pos;

                if (is_sine_motor(&cfg, mid))
                    cmd_pos = center + (float)(cfg.amp * sin(2.0 * M_PI * cfg.sine_hz * t_sec));
                else
                    cmd_pos = center;

                float kp = is_head_motor(mid) ? cfg.head_kp : cfg.kp;
                float kd = is_head_motor(mid) ? cfg.head_kd : cfg.kd;

                frame.can_id = mid;
                frame.can_dlc = 8;
                encode_ptm(cmd_pos, 0, kp, kd, 0, frame.data);

                clock_gettime(CLOCK_MONOTONIC, &now);
                int ret = write(sock, &frame, sizeof(frame));
                if (ret > 0) {
                    log_add(ts_ms(&now, &t0), mid, 'T', cmd_pos, 0, 0);
                    tx_count++;
                } else {
                    tx_drop++;
                }
            }

            /* 读反馈 */
            drain_rx(sock, &t0, cfg.motor_ids, cfg.n_motors);

            if (cycle % (cfg.ctrl_freq * 2) == 0 && cycle > 0) {
                int rx_cnt = 0;
                for (int j = 0; j < g_log_count; j++)
                    if (g_log[j].dir == 'R') rx_cnt++;
                printf("\r  %.1fs  tx=%d  rx=%d  drop=%d",
                       t_sec, tx_count, rx_cnt, tx_drop);
                fflush(stdout);
            }

            cycle++;
            clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, NULL);
        }

        usleep(20000);
        drain_rx(sock, &t0, cfg.motor_ids, cfg.n_motors);

        int rx_total = 0;
        for (int j = 0; j < g_log_count; j++)
            if (g_log[j].dir == 'R') rx_total++;

        printf("\n\n测试结束: %d cycles, TX=%d, RX=%d, drop=%d\n",
               cycle, tx_count, rx_total, tx_drop);
    }

cleanup:
    /* ── 渐停: 1 秒内正弦振幅渐减到 0, 回到 init_pos (正弦中心) ── */
    printf("渐停中 (回到正弦中心)...\n");
    {
        struct timespec fade_next;
        clock_gettime(CLOCK_MONOTONIC, &fade_next);
        long pns = 1000000000L / cfg.ctrl_freq;
        int fade_cycles = cfg.ctrl_freq;  /* 1 秒 */
        for (int c = 0; c < fade_cycles && g_running; c++) {
            fade_next.tv_nsec += pns;
            if (fade_next.tv_nsec >= 1000000000L) { fade_next.tv_sec++; fade_next.tv_nsec -= 1000000000L; }
            double fade = 1.0 - (double)c / fade_cycles;  /* 1.0 → 0.0 */
            double t_sec = (double)c / cfg.ctrl_freq;
            for (int i = 0; i < cfg.n_motors; i++) {
                int mid = cfg.motor_ids[i];
                float kp = is_head_motor(mid) ? cfg.head_kp : cfg.kp;
                float kd = is_head_motor(mid) ? cfg.head_kd : cfg.kd;
                float cmd = init_pos[i];
                if (is_sine_motor(&cfg, mid))
                    cmd += (float)(cfg.amp * fade * sin(2.0 * M_PI * cfg.sine_hz * t_sec));
                frame.can_id = mid;
                frame.can_dlc = 8;
                encode_ptm(cmd, 0, kp, kd, 0, frame.data);
                write(sock, &frame, sizeof(frame));
            }
            while (recv(sock, &frame, sizeof(frame), MSG_DONTWAIT) > 0) {}
            clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &fade_next, NULL);
        }
    }

    /* 保持在 init_pos 半秒 */
    {
        struct timespec hold_next;
        clock_gettime(CLOCK_MONOTONIC, &hold_next);
        long pns = 1000000000L / cfg.ctrl_freq;
        for (int c = 0; c < cfg.ctrl_freq / 2; c++) {
            hold_next.tv_nsec += pns;
            if (hold_next.tv_nsec >= 1000000000L) { hold_next.tv_sec++; hold_next.tv_nsec -= 1000000000L; }
            for (int i = 0; i < cfg.n_motors; i++) {
                int mid = cfg.motor_ids[i];
                float kp = is_head_motor(mid) ? cfg.head_kp : cfg.kp;
                float kd = is_head_motor(mid) ? cfg.head_kd : cfg.kd;
                frame.can_id = mid;
                frame.can_dlc = 8;
                encode_ptm(init_pos[i], 0, kp, kd, 0, frame.data);
                write(sock, &frame, sizeof(frame));
            }
            while (recv(sock, &frame, sizeof(frame), MSG_DONTWAIT) > 0) {}
            clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &hold_next, NULL);
        }
    }

    /* 停用电机 */
    __u8 disable[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFD};
    for (int i = 0; i < cfg.n_motors; i++) {
        frame.can_id = cfg.motor_ids[i];
        memcpy(frame.data, disable, 8);
        frame.can_dlc = 8;
        write(sock, &frame, sizeof(frame));
        usleep(2000);
    }
    printf("[OK] 电机已停用 (在正弦中心位置)\n");

    /* ── 保存 CSV ── */
    char csv_path[256];
    snprintf(csv_path, sizeof(csv_path), "/tmp/can_sine_%s_%dm.csv", cfg.bus, cfg.n_motors);
    FILE *fp = fopen(csv_path, "w");
    if (!fp) { perror("fopen"); return 1; }

    fprintf(fp, "time_ms,dir,motor_id,pos_rad,vel_rad_s,torque_Nm\n");
    for (int i = 0; i < g_log_count; i++) {
        LogEntry *e = &g_log[i];
        fprintf(fp, "%.3f,%c,0x%02X,%.6f,%.6f,%.6f\n",
                e->time_ms, e->dir, e->motor_id, e->pos, e->vel, e->tau);
    }
    fclose(fp);
    printf("\n[OK] CSV: %s (%d rows)\n", csv_path, g_log_count);

    /* ── 快速分析 ── */
    printf("\n══════════ 快速分析 ══════════\n");
    for (int i = 0; i < cfg.n_motors; i++) {
        int mid = cfg.motor_ids[i];
        int tx_n = 0, rx_n = 0, gaps = 0;
        double rx_dt_sum = 0, rx_dt_max = 0, last_rx = -1;

        for (int j = 0; j < g_log_count; j++) {
            if (g_log[j].motor_id != mid) continue;
            if (g_log[j].dir == 'T') { tx_n++; continue; }
            rx_n++;
            if (last_rx >= 0) {
                double dt = g_log[j].time_ms - last_rx;
                rx_dt_sum += dt;
                if (dt > rx_dt_max) rx_dt_max = dt;
                if (dt > 1000.0 / cfg.ctrl_freq * 2.5) gaps++;
            }
            last_rx = g_log[j].time_ms;
        }

        double rx_avg = rx_n > 1 ? rx_dt_sum / (rx_n - 1) : 0;
        double rx_hz = rx_avg > 0 ? 1000.0 / rx_avg : 0;
        const char *role = is_sine_motor(&cfg, mid) ? "正弦" : "保持";
        printf("  0x%02X [%s]: TX=%5d  RX=%5d (%5.1fHz)  avg=%.2fms  max=%.1fms  gaps=%d%s\n",
               mid, role, tx_n, rx_n, rx_hz, rx_avg, rx_dt_max, gaps,
               gaps > 0 ? " !!!" : "");
    }

    printf("\n数据文件: %s\n", csv_path);
    close(sock);
    return 0;
}
