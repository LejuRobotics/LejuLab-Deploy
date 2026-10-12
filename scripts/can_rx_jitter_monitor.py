#!/usr/bin/env python3
"""Read-only CAN RX jitter monitor: sample /proc+/sys into CSV.

Never writes kernel state, services, USB, or other processes' schedulers.
Stdlib only. One-line start (run as root so it can read other threads' schedstat):

  sudo python3 scripts/can_rx_jitter_monitor.py &

With no args it writes ./canmon_<date>_<hhmm>.csv, auto-finds the leju-hardware
stdout log for events, and samples at 1 s. Override via env (or the old flags):

  CANMON_OUT       output CSV path        (default ./canmon_<date>_<hhmm>.csv)
  CANMON_HW_LOG    hardware stdout log    (default: newest .ros/lejulab/stdout/*/stdout.log)
  CANMON_INTERVAL  sample seconds         (default 1)
  CANMON_DURATION  run seconds, 0=until signal (default 0)

SIGINT/SIGTERM flush and exit 0. Join *.events.csv on wall_epoch.
Rising w_*rx*_rqwait_pct -> H-RX; rising w_fifo_tx_* or drop in
bcanN_tx_packets_per_s -> H-TX; bcanN_bus_errors/rx_missed/error_warning
growth -> H-BUS.
"""
from __future__ import annotations

import argparse, csv, glob, os, re, shutil, signal, subprocess, sys, tempfile, threading, time
from datetime import datetime

HW_LOG_GLOBS = (
    "/root/.ros/lejulab/stdout/*/stdout.log",
    os.path.expanduser("~/.ros/lejulab/stdout/*/stdout.log"),
)


def _env_float(name, default):
    try:
        return float(os.environ[name])
    except (KeyError, ValueError):
        return default


def default_out_path():
    return os.environ.get("CANMON_OUT") or os.path.join(
        ".", f"canmon_{datetime.now():%Y-%m-%d_%H%M}.csv")


def autodetect_hw_log():
    """Newest readable hardware stdout log, or None (events then disabled)."""
    env = os.environ.get("CANMON_HW_LOG")
    if env:
        return env
    hits = []
    for pat in HW_LOG_GLOBS:
        hits.extend(p for p in glob.glob(pat) if os.path.isfile(p) and os.access(p, os.R_OK))
    return max(hits, key=os.path.getmtime) if hits else None

REQUIRED_COLS = (
    "bcan0_tx_packets_per_s", "bcan1_tx_packets_per_s", "bcan0_rx_packets_per_s",
    "bcan1_rx_missed_errors", "bcan1_rx_over_errors", "softirq_net_rx_per_s",
    "cpu0_busy_pct", "sample_cost_ms",
)
BCAN_RATE = ("rx_packets", "tx_packets", "rx_bytes")
BCAN_CUM = ("rx_errors", "rx_dropped", "rx_over_errors", "rx_missed_errors",
            "tx_errors", "tx_dropped")
IP_FIELDS = ("state", "berr_tx", "berr_rx", "bus_errors", "error_warning",
             "error_passive", "bus_off", "restarts")
SOFT_KEYS = (("NET_RX", "net_rx"), ("SCHED", "sched"), ("HI", "hi"), ("TASKLET", "tasklet"))
MOTOR_RE = re.compile(
    r"\[MotorUnavailable\]\s+bus=(\S+)\s+id=(\S+)\s+joint=(\S+)\s+"
    r"reason=(\S+)\s+feedback_age=(\d+)ms")
IRQ_COMM_RE = re.compile(r"^irq/\d+-bcan(\d)$")
RX_EXCLUDE_RE = re.compile(
    r"^(gc|tev|KeepAlive|recv|recvMC|recvUC|dq\.builtins|dq\.user|"
    r"rcu_|ksoftirqd|migration|idle_inject|kworker)", re.I)
USB_RE = re.compile(r"ehci|xhci|dwc3|usb", re.I)
EVENT_COLS = ("wall_iso", "wall_epoch", "mono_s", "kind", "bus", "id", "joint",
              "reason", "feedback_age_ms", "raw")
STOP = threading.Event()


def _read(path):
    try:
        with open(path, "r", errors="replace") as fh:
            return fh.read()
    except OSError:
        return ""


def _cmd(argv):
    try:
        r = subprocess.run(argv, capture_output=True, text=True, timeout=2, check=False)
        return r.stdout if r.returncode == 0 else ""
    except (OSError, subprocess.SubprocessError):
        return ""


def _pgrep(args):
    return [int(x) for x in _cmd(["pgrep"] + args).split() if x.isdigit()]


def _fmt(v):
    if v is None or v == "":
        return ""
    if isinstance(v, float):
        return f"{v:.6f}".rstrip("0").rstrip(".")
    return str(v)


def _rate(n, o, dt):
    return None if n is None or o is None or dt <= 0 else (n - o) / dt


def _pct(part, total):
    return None if not total else 100.0 * part / total


def _ns_pct(dns, dt):
    return None if dns is None or dt <= 0 else dns / dt / 1e9 * 100.0


def parse_loadavg(text):
    p = text.split()
    if len(p) < 4:
        return None
    try:
        run, tot = p[3].split("/", 1)
        return float(p[0]), float(p[1]), float(p[2]), int(run), int(tot)
    except ValueError:
        return None


def parse_stat(text):
    cpus, ctxt, intr, blocked = {}, None, None, None
    for line in text.splitlines():
        p = line.split()
        if not p:
            continue
        if p[0].startswith("cpu") and len(p) > 5:
            v = [int(x) for x in p[1:]]
            cpus[p[0]] = (sum(v), v[3], v[4], v[2], v[5] if len(v) > 5 else 0,
                          v[6] if len(v) > 6 else 0)
        elif p[0] == "ctxt":
            ctxt = int(p[1])
        elif p[0] == "intr":
            intr = int(p[1])
        elif p[0] == "procs_blocked":
            blocked = int(p[1])
    return cpus, ctxt, intr, blocked


def cpu_pcts(prev, cur):
    out = {}
    for name, (t, idle, iow, sy, irq, sirq) in cur.items():
        if name not in prev:
            continue
        pt, pidle, piow, psy, pirq, psirq = prev[name]
        dtot = t - pt
        out[f"{name}_busy_pct"] = _pct(dtot - (idle - pidle) - (iow - piow), dtot)
        if name == "cpu0":
            out["cpu0_sys_pct"] = _pct(sy - psy, dtot)
            out["cpu0_irq_pct"] = _pct(irq - pirq, dtot)
            out["cpu0_softirq_pct"] = _pct(sirq - psirq, dtot)
    return out


def parse_softirqs(text):
    out = {}
    for line in text.splitlines():
        p = line.split()
        if len(p) > 1 and p[0].endswith(":"):
            try:
                out[p[0][:-1]] = sum(int(x) for x in p[1:])
            except ValueError:
                pass
    return out


def parse_interrupts(text):
    spi, bcan, usb, dma = {}, {i: 0 for i in range(4)}, 0, 0
    for line in text.splitlines()[1:]:
        parts = line.split()
        nums, labels = [], []
        for tok in parts[1:]:
            (nums if not labels and tok.isdigit() else labels).append(
                int(tok) if not labels and tok.isdigit() else tok)
        total, last, joined = sum(x for x in nums if isinstance(x, int)), (
            labels[-1] if labels else ""), " ".join(str(x) for x in labels)
        if "spi" in str(last):
            b = str(last).split(".")[0]
            spi[b] = spi.get(b, 0) + total
        elif re.fullmatch(r"bcan[0-3]", str(last)):
            bcan[int(str(last)[-1])] += total
        elif USB_RE.search(joined):
            usb += total
        elif "dma-controller" in joined:
            dma += total
    return spi, bcan, usb, dma


def parse_ip_link(text):
    d = {k: None for k in IP_FIELDS}
    if not text:
        return d
    m = re.search(r"\bcan\b[^\n]*\bstate\s+(\S+)", text)
    if m:
        d["state"] = m.group(1).strip(",")
    m = re.search(r"berr-counter\s+tx\s+(\d+)\s+rx\s+(\d+)", text)
    if m:
        d["berr_tx"], d["berr_rx"] = int(m.group(1)), int(m.group(2))
    for key, pat in (("bus_errors", r"bus-errors?\s+(\d+)"),
                     ("error_warning", r"error-warn(?:ing)?\s+(\d+)"),
                     ("error_passive", r"error-pass(?:ive)?\s+(\d+)"),
                     ("bus_off", r"bus-off\s+(\d+)"),
                     ("restarts", r"re-?starts?\s+(\d+)")):
        m = re.search(pat, text)
        if m:
            d[key] = int(m.group(1))
    if d["state"] is None:
        m = re.search(r"can\s+<([^>]+)>", text)
        if m:
            d["state"] = m.group(1).split(",")[0]
    hdr = re.search(
        r"re-started\s+bus-errors\s+arbit-lost\s+error-warn\s+error-pass[^\n]*\n"
        r"\s*(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)", text)
    if hdr:
        d["restarts"], d["bus_errors"] = int(hdr.group(1)), int(hdr.group(2))
        d["error_warning"], d["error_passive"] = int(hdr.group(4)), int(hdr.group(5))
    return d


def parse_schedstat(text):
    p = text.split()
    try:
        return (int(p[0]), int(p[1]), int(p[2])) if len(p) >= 3 else None
    except ValueError:
        return None


def parse_tid_stat(text):
    text = text.strip()
    lp, rp = text.find("("), text.rfind(")")
    if lp < 0 or rp < lp:
        return None
    rest = text[rp + 2:].split()
    if len(rest) < 39:
        return None
    try:
        return text[lp + 1:rp], int(rest[11]), int(rest[12]), int(rest[37]), int(rest[38])
    except (ValueError, IndexError):
        return None


def parse_event_line(line):
    m = MOTOR_RE.search(line)
    if m:
        return {"kind": "motor_unavailable", "bus": m.group(1), "id": m.group(2),
                "joint": m.group(3), "reason": m.group(4), "feedback_age_ms": m.group(5),
                "raw": line.rstrip("\n")}
    if "进入保护模式" in line or "/rt/hardware/stop" in line:
        return {"kind": "protective_stop", "bus": "", "id": "", "joint": "",
                "reason": "", "feedback_age_ms": "", "raw": line.rstrip("\n")}
    return None


def cpu_order(names):
    def key(n):
        if n == "cpu":
            return (-1, 0)
        if n.startswith("cpu") and n[3:].isdigit():
            return (0, int(n[3:]))
        return (1, n)
    ordered = sorted(names, key=key)
    if "cpu" not in ordered:
        ordered.insert(0, "cpu")
    if "cpu0" not in ordered:
        ordered.insert(1, "cpu0")
    return ordered


def default_watchers(top_other):
    w = [(f"irqbcan{i}", 0, "") for i in range(4)]
    w += [("fifo_tx", 0, ""), ("fifo_sensor", 0, ""), ("fifo_ctl", 0, "")]
    w += [("rx", i, "") for i in range(top_other)]
    return w


def make_headers(cpu_names, spi_bases, watchers):
    h = ["wall_iso", "wall_epoch", "mono_s", "interval_s", "load1", "load5", "load15",
         "procs_running", "procs_total", "ctxt_per_s", "intr_total_per_s", "procs_blocked"]
    for n in cpu_names:
        h.append(f"{n}_busy_pct")
        if n == "cpu0":
            h.extend(["cpu0_sys_pct", "cpu0_irq_pct", "cpu0_softirq_pct"])
    h += [f"softirq_{s}_per_s" for _k, s in SOFT_KEYS]
    h += [f"irq_bcan{i}_per_s" for i in range(4)] + ["irq_usb_per_s", "irq_dma_per_s"]
    h += [f"irq_spi_{b}_per_s" for b in spi_bases]
    for i in range(4):
        h += [f"bcan{i}_{r}_per_s" for r in BCAN_RATE] + [f"bcan{i}_{c}" for c in BCAN_CUM]
    for i in (0, 1):
        h += [f"bcan{i}_{f}" for f in IP_FIELDS]
    for label, tid, _c in watchers:
        h += [f"w_{label}_{tid}_{m}" for m in ("exec_pct", "rqwait_pct", "nrsw_per_s")]
    h += ["joymon_cpu_pct", "joystick_cpu_pct", "recorder_cpu_pct",
          "disk_free_mb_out", "disk_free_mb_root", "sample_cost_ms"]
    return h


def _comm(tid):
    return _read(f"/proc/{tid}/comm").strip()


def _tid_sched(tid):
    return parse_schedstat(_read(f"/proc/{tid}/schedstat"))


def discover_threads(pid, top_other, warmup=2.0):
    irq = {}
    try:
        for name in os.listdir("/proc"):
            if name.isdigit():
                c = _comm(int(name))
                m = IRQ_COMM_RE.match(c)
                if m:
                    irq[int(m.group(1))] = (int(name), c)
    except OSError:
        pass
    w = [(f"irqbcan{i}",) + irq.get(i, (0, "")) for i in range(4)]
    fifo_tx = fifo_ctl = fifo_sensor = (0, "")
    rx = []
    if pid:
        try:
            tasks = [int(x) for x in os.listdir(f"/proc/{pid}/task")]
        except OSError:
            tasks = []
        infos = []
        for tid in tasks:
            parsed = parse_tid_stat(_read(f"/proc/{tid}/stat"))
            if parsed:
                comm, _u, _s, rtprio, pol = parsed
                infos.append((tid, comm, rtprio, pol, _tid_sched(tid)))
        fifo = [(t, c, rp) for t, c, rp, pol, _ss in infos if pol == 1]
        if fifo:
            t, c, _rp = min(fifo, key=lambda x: abs(x[2] - 40))
            fifo_tx = (t, c)
            # sensor/health loop (feedback_age "now" side): SCHED_FIFO ~30, 1ms
            t, c, _rp = min(fifo, key=lambda x: abs(x[2] - 30))
            fifo_sensor = (t, c)
            t, c, _rp = max(fifo, key=lambda x: x[2])
            fifo_ctl = (t, c)
        # canbus_sdk RX threads are unnamed (inherit the process comm); the only
        # reliable signal is CPU work. Rank SCHED_OTHER threads by exec-delta over
        # the warmup window, excluding known non-CAN helpers (DDS recv*, gc,
        # kworker, ...) whose comm would otherwise pollute the picks.
        others = [(t, c, ss) for t, c, rp, pol, ss in infos
                  if pol == 0 and not RX_EXCLUDE_RE.match(c)]
        deltas = []
        if others and warmup > 0:
            s1 = {t: ss for t, c, ss in others}
            STOP.wait(warmup)
            for t, c, _ss in others:
                ss2 = _tid_sched(t)
                e1 = s1.get(t)
                deltas.append(((ss2[0] - e1[0]) if ss2 and e1 else 0, t, c))
            deltas.sort(reverse=True)
        else:
            deltas = [(0, t, c) for t, c, _ss in others]
        seen, chosen = set(), []
        for _d, t, c in deltas:
            if t not in seen:
                chosen.append((t, c)); seen.add(t)
            if len(chosen) >= top_other:
                break
        rx = chosen[:top_other]
    w += [("fifo_tx", fifo_tx[0], fifo_tx[1]),
          ("fifo_sensor", fifo_sensor[0], fifo_sensor[1]),
          ("fifo_ctl", fifo_ctl[0], fifo_ctl[1])]
    used = set()
    for i in range(top_other):
        if i < len(rx):
            w.append(("rx", rx[i][0], rx[i][1])); used.add(rx[i][0])
        else:
            ph = 0
            while ph in used:
                ph += 1
            w.append(("rx", ph, "")); used.add(ph)
    return w


def resolve_hw_pid(explicit):
    if explicit:
        return explicit
    pids = _pgrep(["-x", "leju-hardware"])
    if len(pids) == 1:
        return pids[0]
    if len(pids) > 1:
        print("multiple leju-hardware pids; pass --pid", file=sys.stderr)
    return None


def resolve_preempt_pids():
    joy = None
    m = re.search(r"MainPID=(\d+)", _cmd(
        ["systemctl", "show", "-p", "MainPID", "lejulab_joy_monitor.service"]))
    if m and int(m.group(1)):
        joy = int(m.group(1))
    if not joy:
        p = _pgrep(["-f", "monitor_lejulab_joy"])
        joy = p[0] if p else None
    js, rec = _pgrep(["-x", "leju-joystick"]), _pgrep(["-x", "lejusdk_recorder"])
    return {"joymon": joy, "joystick": js[0] if js else None,
            "recorder": rec[0] if rec else None}


def _ticks(pid):
    parsed = parse_tid_stat(_read(f"/proc/{pid}/stat")) if pid else None
    return None if not parsed else parsed[1] + parsed[2]


def _bcan_stats(n):
    d = {}
    for name in BCAN_RATE + BCAN_CUM:
        raw = _read(f"/sys/class/net/bcan{n}/statistics/{name}").strip()
        try:
            d[name] = int(raw)
        except ValueError:
            d[name] = None
    return d


def _free_mb(path):
    try:
        st = os.statvfs(path)
        return st.f_bavail * st.f_frsize / (1024 * 1024)
    except OSError:
        return None


def take_snapshot(cfg):
    s = {"mono": time.monotonic(), "err": 0}
    try:
        s["load"] = parse_loadavg(_read("/proc/loadavg"))
    except Exception:
        s["load"] = None; s["err"] += 1
    try:
        s["cpus"], s["ctxt"], s["intr"], s["blocked"] = parse_stat(_read("/proc/stat"))
    except Exception:
        s["cpus"], s["ctxt"], s["intr"], s["blocked"] = {}, None, None, None; s["err"] += 1
    try:
        s["soft"] = parse_softirqs(_read("/proc/softirqs"))
    except Exception:
        s["soft"] = {}; s["err"] += 1
    try:
        spi, bcan, usb, dma = parse_interrupts(_read("/proc/interrupts"))
        irq = {f"spi_{k}": v for k, v in spi.items()}
        irq.update({f"bcan{i}": v for i, v in bcan.items()})
        irq["usb"], irq["dma"] = usb, dma
        s["irq"] = irq
    except Exception:
        s["irq"] = {}; s["err"] += 1
    s["bcan"] = {}
    for i in range(4):
        try:
            s["bcan"][i] = _bcan_stats(i)
        except Exception:
            s["bcan"][i] = {n: None for n in BCAN_RATE + BCAN_CUM}; s["err"] += 1
    if cfg["do_ip"]:
        s["ip"] = {}
        for i in (0, 1):
            try:
                s["ip"][i] = parse_ip_link(_cmd(
                    ["ip", "-details", "-statistics", "link", "show", f"bcan{i}"]))
            except Exception:
                s["ip"][i] = {k: None for k in IP_FIELDS}; s["err"] += 1
    else:
        s["ip"] = {i: {k: None for k in IP_FIELDS} for i in (0, 1)}
    s["sched"] = {}
    for _lab, tid, _c in cfg["watchers"]:
        if tid:
            try:
                s["sched"][tid] = _tid_sched(tid)
            except Exception:
                s["sched"][tid] = None; s["err"] += 1
    s["pticks"] = {}
    for name, pid in cfg["ppids"].items():
        try:
            s["pticks"][name] = _ticks(pid)
        except Exception:
            s["pticks"][name] = None; s["err"] += 1
    return s


def build_row(prev, cur, cfg, cost_ms):
    dt = cur["mono"] - prev["mono"] if prev else 0.0
    row = {"wall_iso": datetime.now().isoformat(timespec="milliseconds"),
           "wall_epoch": time.time(), "mono_s": cur["mono"],
           "interval_s": dt if prev else None, "sample_cost_ms": cost_ms}
    if cur.get("load"):
        row["load1"], row["load5"], row["load15"], row["procs_running"], row["procs_total"] = cur["load"]
    row["procs_blocked"] = cur.get("blocked")
    if prev:
        row["ctxt_per_s"] = _rate(cur.get("ctxt"), prev.get("ctxt"), dt)
        row["intr_total_per_s"] = _rate(cur.get("intr"), prev.get("intr"), dt)
        if cur.get("cpus") and prev.get("cpus"):
            row.update(cpu_pcts(prev["cpus"], cur["cpus"]))
        for key, slug in SOFT_KEYS:
            row[f"softirq_{slug}_per_s"] = _rate(
                cur.get("soft", {}).get(key), prev.get("soft", {}).get(key), dt)
        irq, pirq = cur.get("irq", {}), prev.get("irq", {})
        for i in range(4):
            row[f"irq_bcan{i}_per_s"] = _rate(irq.get(f"bcan{i}"), pirq.get(f"bcan{i}"), dt)
        row["irq_usb_per_s"] = _rate(irq.get("usb"), pirq.get("usb"), dt)
        row["irq_dma_per_s"] = _rate(irq.get("dma"), pirq.get("dma"), dt)
        for b in cfg["spi"]:
            row[f"irq_spi_{b}_per_s"] = _rate(irq.get(f"spi_{b}"), pirq.get(f"spi_{b}"), dt)
    for i in range(4):
        nowb = cur.get("bcan", {}).get(i, {})
        oldb = (prev or {}).get("bcan", {}).get(i, {})
        for r in BCAN_RATE:
            row[f"bcan{i}_{r}_per_s"] = _rate(nowb.get(r), oldb.get(r), dt) if prev else None
        for c in BCAN_CUM:
            row[f"bcan{i}_{c}"] = nowb.get(c)
    for i in (0, 1):
        ipd = cur.get("ip", {}).get(i) or {}
        for f in IP_FIELDS:
            row[f"bcan{i}_{f}"] = ipd.get(f)
    clk = cfg["clk"]
    for label, tid, _c in cfg["watchers"]:
        ss = cur.get("sched", {}).get(tid) if tid else None
        pss = (prev or {}).get("sched", {}).get(tid) if prev and tid else None
        de = (ss[0] - pss[0]) if ss and pss else None
        dw = (ss[1] - pss[1]) if ss and pss else None
        row[f"w_{label}_{tid}_exec_pct"] = _ns_pct(de, dt) if prev else None
        row[f"w_{label}_{tid}_rqwait_pct"] = _ns_pct(dw, dt) if prev else None
        row[f"w_{label}_{tid}_nrsw_per_s"] = _rate(ss[2], pss[2], dt) if prev and ss and pss else None
    for name in ("joymon", "joystick", "recorder"):
        nt, ot = cur.get("pticks", {}).get(name), (prev or {}).get("pticks", {}).get(name)
        row[f"{name}_cpu_pct"] = (
            100.0 * (nt - ot) / clk / dt if prev and nt is not None and ot is not None and dt > 0 and clk else None)
    row["disk_free_mb_out"] = _free_mb(cfg["out_dir"])
    row["disk_free_mb_root"] = _free_mb("/")
    return row


def apply_idle():
    try:
        os.nice(19)
    except OSError:
        pass
    try:
        os.sched_setscheduler(0, os.SCHED_IDLE, os.sched_param(0))
    except (OSError, AttributeError, PermissionError):
        pass


def check_disk(out_path, min_mb):
    d = os.path.dirname(os.path.abspath(out_path)) or "."
    if not os.path.isdir(d):
        print(f"output directory missing: {d}", file=sys.stderr)
        return False
    free = _free_mb(d)
    if free is not None and free < min_mb:
        print(f"refusing to start: {free:.1f} MB free < --min-free-mb {min_mb}", file=sys.stderr)
        return False
    return True


def start_tail(path, poll, ev_path, append):
    exists = os.path.exists(ev_path)
    ev_f = open(ev_path, "a" if append and exists else "x", newline="")
    w = csv.writer(ev_f)
    if not (append and exists and os.path.getsize(ev_path) > 0):
        w.writerow(EVENT_COLS); ev_f.flush()
    lock = threading.Lock()

    def run():
        fh = ino = None
        while not STOP.is_set():
            try:
                st = os.stat(path)
                if fh is None or st.st_ino != ino:
                    if fh:
                        fh.close()
                    fh = open(path, "r", errors="replace")
                    fh.seek(0, 2)
                    ino = st.st_ino
                elif fh.tell() > st.st_size:
                    fh.seek(0)
                pos, line = fh.tell(), fh.readline()
                if not line:
                    STOP.wait(poll); continue
                if not line.endswith("\n"):
                    fh.seek(pos); STOP.wait(poll); continue
                ev = parse_event_line(line)
                if not ev:
                    continue
                rec = [datetime.now().isoformat(timespec="milliseconds"), time.time(),
                       time.monotonic(), ev["kind"], ev["bus"], ev["id"], ev["joint"],
                       ev["reason"], ev["feedback_age_ms"], ev["raw"]]
                with lock:
                    w.writerow(rec); ev_f.flush()
            except OSError:
                STOP.wait(poll)
            except Exception:
                STOP.wait(poll)
        if fh:
            try:
                fh.close()
            except OSError:
                pass

    th = threading.Thread(target=run, name="hw-log-tail", daemon=True)
    th.start()
    return ev_f, th


def build_parser():
    p = argparse.ArgumentParser(
        description="Read-only CAN RX jitter CSV sampler (does not affect robot runtime).",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="SIGINT/SIGTERM flush+exit 0. Join events.csv on wall_epoch for H-RX/H-TX/H-BUS.")
    p.add_argument("--out", help="CSV output path (default $CANMON_OUT or ./canmon_<date>_<hhmm>.csv)")
    p.add_argument("--interval", type=float, default=_env_float("CANMON_INTERVAL", 1.0))
    p.add_argument("--duration", type=float, default=_env_float("CANMON_DURATION", 0.0))
    p.add_argument("--max-rows", type=int, default=500000)
    p.add_argument("--max-bytes", type=int, default=268435456)
    p.add_argument("--min-free-mb", type=int, default=512)
    p.add_argument("--append", action="store_true")
    p.add_argument("--pid", type=int, default=None)
    p.add_argument("--top-other", type=int, default=6)
    p.add_argument("--hw-log", default=None)
    p.add_argument("--tail-poll", type=float, default=0.5)
    p.add_argument("--selftest", action="store_true")
    return p


def _on_sig(_s, _f):
    STOP.set()


def run_monitor(args):
    STOP.clear()
    signal.signal(signal.SIGINT, _on_sig)
    signal.signal(signal.SIGTERM, _on_sig)
    apply_idle()
    out_path = args.out
    if not check_disk(out_path, args.min_free_mb):
        return 2
    if os.path.exists(out_path) and not args.append:
        print(f"refusing to overwrite existing file: {out_path}", file=sys.stderr)
        return 2
    out_dir = os.path.dirname(os.path.abspath(out_path)) or "."
    spi, _b, _u, _d = parse_interrupts(_read("/proc/interrupts"))
    cpus, *_r = parse_stat(_read("/proc/stat"))
    pid = None if STOP.is_set() else resolve_hw_pid(args.pid)
    watchers = discover_threads(pid, args.top_other, warmup=2.0 if pid else 0.0)
    headers = make_headers(cpu_order(cpus.keys() if cpus else ["cpu", "cpu0"]),
                           sorted(spi.keys()), watchers)
    tpath = out_path + ".threads.txt"
    tmode = "a" if args.append and os.path.exists(tpath) else "x"
    try:
        tf = open(tpath, tmode)
    except FileExistsError:
        tf = open(tpath, "a")
    with tf:
        tf.write("tid label comm\n")
        for label, tid, comm in watchers:
            tf.write(f"{tid} {label} {comm}\n")
    mode = "a" if args.append and os.path.exists(out_path) else "x"
    out_f = ev_f = th = None
    try:
        out_f = open(out_path, mode, newline="")
    except FileExistsError:
        print(f"refusing to overwrite existing file: {out_path}", file=sys.stderr)
        return 2
    writer = csv.writer(out_f)
    if not (args.append and os.path.getsize(out_path) > 0):
        writer.writerow(headers); out_f.flush()
    if args.hw_log:
        if os.path.isfile(args.hw_log) and os.access(args.hw_log, os.R_OK):
            ev_f, th = start_tail(args.hw_log, args.tail_poll, out_path + ".events.csv", args.append)
        else:
            print(f"hw-log not readable, events disabled: {args.hw_log}", file=sys.stderr)
    try:
        clk = os.sysconf("SC_CLK_TCK")
    except (ValueError, OSError):
        clk = 100
    empty_ppids = {"joymon": None, "joystick": None, "recorder": None}
    cfg = {"watchers": watchers, "spi": sorted(spi.keys()),
           "do_ip": args.interval >= 1.0 and bool(shutil.which("ip")),
           "ppids": empty_ppids if STOP.is_set() else resolve_preempt_pids(),
           "clk": clk, "out_dir": out_dir}
    t0, prev, rows = time.monotonic(), take_snapshot(cfg), 0
    try:
        while not STOP.is_set():
            now = time.monotonic()
            if args.duration > 0 and now - t0 >= args.duration:
                break
            wait = args.interval
            if args.duration > 0:
                wait = min(wait, max(0.0, args.duration - (now - t0)))
            if wait <= 0 or STOP.wait(wait):
                break
            if args.duration > 0 and time.monotonic() - t0 >= args.duration:
                break
            t_s = time.monotonic()
            cur = take_snapshot(cfg)
            rec = build_row(prev, cur, cfg, (time.monotonic() - t_s) * 1000.0)
            writer.writerow([_fmt(rec.get(c)) for c in headers])
            out_f.flush()
            rows += 1
            prev = cur
            if rows >= args.max_rows:
                break
            try:
                if os.path.getsize(out_path) >= args.max_bytes:
                    break
            except OSError:
                break
    finally:
        STOP.set()
        out_f.flush(); out_f.close()
        if ev_f:
            ev_f.flush(); ev_f.close()
        if th:
            th.join(1.0)
    return 0


def run_selftest():
    d = tempfile.mkdtemp(prefix="canmon_selftest_")
    out = os.path.join(d, "m.csv")
    try:
        rc = run_monitor(argparse.Namespace(
            out=out, interval=0.1, duration=0.45, max_rows=3, max_bytes=10**9,
            min_free_mb=0, append=False, pid=None, top_other=6, hw_log=None,
            tail_poll=0.5, selftest=True))
        if rc != 0:
            print(f"selftest run_monitor rc={rc}", file=sys.stderr); return 1
        with open(out, "r", newline="") as fh:
            lines = fh.read().splitlines()
        if len(lines) < 3:
            print("selftest: need header + >=2 data rows", file=sys.stderr); return 1
        hdr = lines[0].split(",")
        for col in REQUIRED_COLS:
            if col not in hdr:
                print(f"selftest missing column {col}", file=sys.stderr); return 1
        if not any("_rqwait_pct" in c for c in hdr):
            print("selftest missing _rqwait_pct", file=sys.stderr); return 1
        idx = hdr.index("sample_cost_ms")
        for line in lines[1:]:
            parts = line.split(",")
            if len(parts) != len(hdr):
                print("selftest column count mismatch", file=sys.stderr); return 1
            try:
                cost = float(parts[idx])
            except ValueError:
                print("selftest sample_cost_ms not numeric", file=sys.stderr); return 1
            if cost >= 50:
                print(f"selftest sample_cost_ms {cost} >= 50", file=sys.stderr); return 1
        print("SELFTEST OK")
        return 0
    finally:
        shutil.rmtree(d, ignore_errors=True)


def main(argv=None):
    args = build_parser().parse_args(argv)
    if args.selftest:
        return run_selftest()
    if not args.out:
        args.out = default_out_path()
    if args.hw_log is None:
        args.hw_log = autodetect_hw_log()
    print(f"canmon: out={args.out} hw-log={args.hw_log or 'none (events off)'} "
          f"interval={args.interval}s", file=sys.stderr)
    return run_monitor(args)


if __name__ == "__main__":
    sys.exit(main())
