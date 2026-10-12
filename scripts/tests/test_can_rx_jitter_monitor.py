import csv
import importlib.util
import os
import re
import signal
import subprocess
import sys
import time
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT = REPO_ROOT / "scripts" / "can_rx_jitter_monitor.py"
spec = importlib.util.spec_from_file_location("can_rx_jitter_monitor", SCRIPT)
mon = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mon)

STAT_A = """cpu  100 0 20 80 10 0 5 0 0 0
cpu0 50 0 10 40 5 0 2 0 0 0
ctxt 1000
intr 2000 1 2
procs_blocked 1
"""
STAT_B = """cpu  140 0 30 90 12 0 8 0 0 0
cpu0 59 0 15 44 6 0 3 0 0 0
ctxt 1500
intr 2600 1 2
procs_blocked 2
"""
SOFT_A = """                CPU0       CPU1
          HI:          1          3
       NET_RX:        10         20
     TASKLET:          2          2
       SCHED:        100        200
"""
SOFT_B = """                CPU0       CPU1
          HI:          2          4
       NET_RX:        30         40
     TASKLET:          4          4
       SCHED:        130        250
"""
IRQ_TXT = """           CPU0       CPU1
 45:        10         20  GICv2  123  feb00000.spi
 46:         1          2  GICv2  124  bcan1
 47:         3          4  IR-PCI-MSI  xhci_hcd
 48:         5          6  GICv2  dma-controller
"""
IP_TXT = """4: bcan1: <NOARP,UP,LOWER_UP> mtu 16 qdisc pfifo_fast state UNKNOWN
    link/can
    can <ERROR-ACTIVE,ECHO> state ERROR-ACTIVE (berr-counter tx 2 rx 3) restart-ms 100
          re-started bus-errors arbit-lost error-warn error-pass rx-errors tx-errors
                  4          5          0          6          7         0         0
"""
MOTOR_LINE = (
    "RUIWO [WARN] [MotorUnavailable] bus=bcan1 id=0x03 joint=Rleg_joint_03 "
    "reason=feedback_timeout feedback_age=51ms status=0x0001 fault=0x00 stop_enabled=1"
)
TID_STAT = (
    "1234 (leju-hardware) S 1 1 1 0 -1 0 0 0 0 0 10 20 0 0 0 0 1 0 0 0 0 0 0 0 0 0 "
    "0 0 0 0 0 0 0 0 0 0 40 1"
)


def _run(args, **kw):
    return subprocess.run(
        [sys.executable, str(SCRIPT)] + args,
        capture_output=True, text=True, **kw)


def test_parse_loadavg():
    assert mon.parse_loadavg("1.04 0.80 0.81 2/2555 99") == (1.04, 0.80, 0.81, 2, 2555)
    assert mon.parse_loadavg("bad") is None


def test_parse_stat_and_cpu_pcts():
    c1, ctxt1, intr1, b1 = mon.parse_stat(STAT_A)
    c2, ctxt2, intr2, b2 = mon.parse_stat(STAT_B)
    assert ctxt1 == 1000 and ctxt2 == 1500 and b2 == 2
    assert mon._rate(ctxt2, ctxt1, 2.0) == 250.0
    assert mon._rate(intr2, intr1, 2.0) == 300.0
    pct = mon.cpu_pcts(c1, c2)
    # dtot cpu0 = 20; busy = 20 - 4 idle - 1 iowait = 15 -> 75%
    assert abs(pct["cpu0_busy_pct"] - 75.0) < 1e-6
    assert abs(pct["cpu0_sys_pct"] - 25.0) < 1e-6


def test_parse_softirqs_rate():
    a, b = mon.parse_softirqs(SOFT_A), mon.parse_softirqs(SOFT_B)
    assert a["NET_RX"] == 30 and b["NET_RX"] == 70
    assert mon._rate(b["NET_RX"], a["NET_RX"], 2.0) == 20.0
    assert mon._rate(b["SCHED"], a["SCHED"], 2.0) == 40.0


def test_parse_interrupts_labels():
    spi, bcan, usb, dma = mon.parse_interrupts(IRQ_TXT)
    assert spi["feb00000"] == 30
    assert bcan[1] == 3 and bcan[0] == 0
    assert usb == 7 and dma == 11


def test_parse_ip_link():
    d = mon.parse_ip_link(IP_TXT)
    assert d["state"] == "ERROR-ACTIVE"
    assert d["berr_tx"] == 2 and d["berr_rx"] == 3
    assert d["bus_errors"] == 5 and d["restarts"] == 4
    assert d["error_warning"] == 6 and d["error_passive"] == 7
    empty = mon.parse_ip_link("")
    assert empty["state"] is None


def test_schedstat_rqwait_pct():
    a, b = mon.parse_schedstat("100 0 1"), mon.parse_schedstat("100 51000000 3")
    assert a[1] == 0 and b[1] == 51000000
    assert abs(mon._ns_pct(b[1] - a[1], 1.0) - 5.1) < 1e-9


def test_parse_tid_stat_policy_rtprio():
    parsed = mon.parse_tid_stat(TID_STAT)
    assert parsed is not None
    comm, utime, stime, rtprio, pol = parsed
    assert comm == "leju-hardware" and utime == 10 and stime == 20
    assert rtprio == 40 and pol == 1


def test_parse_event_line_motor_and_stop():
    ev = mon.parse_event_line(MOTOR_LINE)
    assert ev["kind"] == "motor_unavailable"
    assert ev["bus"] == "bcan1" and ev["id"] == "0x03" and ev["feedback_age_ms"] == "51"
    stop = mon.parse_event_line("node 进入保护模式 now")
    assert stop["kind"] == "protective_stop" and stop["bus"] == ""
    stop2 = mon.parse_event_line("publish /rt/hardware/stop")
    assert stop2["kind"] == "protective_stop"
    assert mon.parse_event_line("noise") is None


def test_header_schema_and_rqwait_placeholders():
    watchers = mon.default_watchers(6)
    hdr = mon.make_headers(["cpu", "cpu0"], [], watchers)
    for col in mon.REQUIRED_COLS:
        assert col in hdr, col
    assert any("_rqwait_pct" in c for c in hdr)
    assert "w_irqbcan0_0_rqwait_pct" in hdr
    assert "w_fifo_tx_0_rqwait_pct" in hdr
    assert "w_fifo_sensor_0_rqwait_pct" in hdr
    assert "w_rx_0_rqwait_pct" in hdr
    assert "bcan1_rx_over_errors" in hdr


def test_min_free_mb_refuses(tmp_path):
    out = tmp_path / "x.csv"
    r = _run(["--out", str(out), "--min-free-mb", "99999999"])
    assert r.returncode == 2
    assert not out.exists()
    assert "min-free-mb" in r.stderr or "free" in r.stderr.lower()


def test_max_rows_stops(tmp_path):
    out = tmp_path / "m.csv"
    t0 = time.monotonic()
    r = _run(["--out", str(out), "--interval", "0.1", "--max-rows", "5", "--min-free-mb", "0"])
    elapsed = time.monotonic() - t0
    assert r.returncode == 0, r.stderr
    assert elapsed < 2.0
    rows = out.read_text().splitlines()
    assert len(rows) == 6  # header + 5
    hdr = rows[0]
    for col in mon.REQUIRED_COLS:
        assert col in hdr
    assert "_rqwait_pct" in hdr


def test_sigterm_flushes_and_exits_zero(tmp_path):
    out = tmp_path / "sig.csv"
    proc = subprocess.Popen(
        [sys.executable, str(SCRIPT), "--out", str(out), "--interval", "0.2",
         "--duration", "30", "--min-free-mb", "0"],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    time.sleep(1.5)
    proc.send_signal(signal.SIGTERM)
    rc = proc.wait(timeout=8)
    assert rc == 0, proc.stderr.read() if proc.stderr else ""
    body = out.read_text()
    assert body and body.endswith("\n")
    hdr = body.splitlines()[0]
    assert "sample_cost_ms" in hdr and "_rqwait_pct" in hdr


def test_events_csv_from_hw_log(tmp_path):
    hw = tmp_path / "hw.log"
    hw.write_text("seed\n")
    out = tmp_path / "m.csv"
    proc = subprocess.Popen(
        [sys.executable, str(SCRIPT), "--out", str(out), "--interval", "0.2",
         "--duration", "2.5", "--hw-log", str(hw), "--tail-poll", "0.2", "--min-free-mb", "0"],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    time.sleep(0.6)
    with hw.open("a") as fh:
        fh.write(MOTOR_LINE + "\n")
        fh.flush()
    rc = proc.wait(timeout=8)
    assert rc == 0, proc.stderr.read() if proc.stderr else ""
    ev = Path(str(out) + ".events.csv")
    assert ev.is_file()
    body = ev.read_text()
    assert "motor_unavailable,bcan1,0x03" in body


def test_a7_readonly_source():
    src = SCRIPT.read_text()
    assert "os.system" not in src
    assert "ip link set" not in src
    assert not re.search(r"systemctl (start|stop|restart)", src)
    assert not re.search(r"sched_setscheduler\([^0]", src)
    assert "/proc/sys" not in src
    assert "/sys/kernel" not in src
    assert "/sys/fs/cgroup" not in src
    assert "subprocess" in src
    for m in re.finditer(r"open\(.+[\"']([wax]|r\+)", src):
        line = src[:m.start()].count("\n") + 1
        snippet = src.splitlines()[line - 1]
        assert any(tok in snippet for tok in (
            "out_path", "ev_path", "tpath", "selftest", "tmp", "canmon", "events", "threads",
            '"x"', '"a"', "mode")), snippet


def test_rx_exclude_regex():
    for noise in ("gc", "recv", "recvMC", "recvUC", "tev", "KeepAlive",
                  "dq.builtins", "kworker/0:1", "ksoftirqd/3"):
        assert mon.RX_EXCLUDE_RE.match(noise), noise
    for keep in ("leju-hardware", "", "canbus_rx", "SchedWorker"):
        assert not mon.RX_EXCLUDE_RE.match(keep), keep


def test_parser_selftest_optional_out():
    p = mon.build_parser()
    args = p.parse_args(["--selftest"])
    assert args.selftest is True and args.out is None
    args = p.parse_args(["--out", "/tmp/m.csv", "--interval", "1"])
    assert args.interval == 1.0 and args.min_free_mb == 512


def test_default_out_path(monkeypatch):
    monkeypatch.delenv("CANMON_OUT", raising=False)
    p = mon.default_out_path()
    assert os.path.basename(p).startswith("canmon_") and p.endswith(".csv")
    monkeypatch.setenv("CANMON_OUT", "/tmp/explicit.csv")
    assert mon.default_out_path() == "/tmp/explicit.csv"


def test_env_interval_and_duration(monkeypatch):
    monkeypatch.setenv("CANMON_INTERVAL", "0.25")
    monkeypatch.setenv("CANMON_DURATION", "7")
    a = mon.build_parser().parse_args([])
    assert a.interval == 0.25 and a.duration == 7.0
    monkeypatch.setenv("CANMON_INTERVAL", "garbage")
    assert mon.build_parser().parse_args([]).interval == 1.0


def test_autodetect_hw_log(monkeypatch, tmp_path):
    monkeypatch.setenv("CANMON_HW_LOG", "/some/explicit/stdout.log")
    assert mon.autodetect_hw_log() == "/some/explicit/stdout.log"
    monkeypatch.delenv("CANMON_HW_LOG", raising=False)
    monkeypatch.setattr(mon, "HW_LOG_GLOBS", (str(tmp_path / "none" / "*" / "stdout.log"),))
    assert mon.autodetect_hw_log() is None
    d = tmp_path / "lejulab" / "run1"
    d.mkdir(parents=True)
    (d / "stdout.log").write_text("x")
    monkeypatch.setattr(mon, "HW_LOG_GLOBS", (str(tmp_path / "lejulab" / "*" / "stdout.log"),))
    assert mon.autodetect_hw_log() == str(d / "stdout.log")


def test_one_line_run_uses_env(tmp_path):
    out = tmp_path / "canmon.csv"
    env = dict(os.environ, CANMON_OUT=str(out), CANMON_DURATION="0.6",
               CANMON_INTERVAL="0.15")
    r = subprocess.run([sys.executable, str(SCRIPT)], capture_output=True,
                       text=True, cwd=str(tmp_path), env=env, timeout=15)
    assert r.returncode == 0, r.stderr
    assert out.is_file() and out.read_text().count("\n") >= 2
    assert "canmon: out=" in r.stderr
