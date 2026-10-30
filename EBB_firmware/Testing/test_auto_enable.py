#!/usr/bin/env python
# -*- encoding: utf-8 -#-

'''
test_auto_enable.py

Tests for automatic motor enable (CU,50) with each driver configuration (SC,2),
and its interaction with the stepper idle timeout (CU,61).
GitHub issues #245 (auto enable ignores external driver configuration) and
#246 (idle timeout can disable motors for a new move).

Run against an EBB with no motors attached:

    python3 test_auto_enable.py [port] [race_trials]

port defaults to the first EiBotBoard found. race_trials (default 300) sets the
number of trials in the timing race test; use 0 to skip it.
Settings changed by the tests are restored at the end.

Uses QE, which reports the enable state of the drivers selected by SC,2, to
see which enable pins a command has changed.
'''

import random
import sys
import time

import serial
from serial.tools.list_ports import comports

# One small move for each motion command type. Together these cover the three
# compiled parse-time auto-enable sites in ebb.c: process_simple_motor_move_fp
# (SM, XM), process_simple_rate_move_fp (HM), and process_low_level_move
# (LM, LT, L3, T3, TD). CM is compiled out in current builds ("CM command
# disabled in this build"), so it is not tested.
MOVES = {
    "SM": "SM,20,10,10",
    "XM": "XM,20,10,10",
    "HM": "HM,1000,10,10",
    "LM": "LM,85899346,10,0,85899346,10,0",
    "LT": "LT,250,85899346,0,85899346,0",
    "L3": "L3,85899346,10,0,0,85899346,10,0,0",
    "T3": "T3,250,85899346,0,0,85899346,0,0",
    "TD": "TD,250,85899346,85899346,0,0,85899346,85899346,0,0",
}

results = []


def find_port():
    for port in comports():
        if port.vid == 0x04D8 and port.pid == 0xFD92:
            return port.device
    sys.exit("No EiBotBoard found")


class EBB:
    '''Minimal EBB connection, using legacy (default) response mode.'''

    def __init__(self, port):
        self.ser = serial.Serial(port, timeout=1)
        time.sleep(0.2)
        self.ser.reset_input_buffer()

    def lines_until_ok(self, cmd, timeout=3.0):
        '''Send cmd and return response lines through the final OK or error.'''
        self.ser.write((cmd + "\r").encode("ascii"))
        out = []
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            line = self.ser.readline().decode("ascii", "replace").strip()
            if not line:
                continue
            out.append(line)
            if line.startswith("OK") or "Err" in line:
                return out
        raise RuntimeError(f"no OK for {cmd!r}: {out}")

    def ok(self, cmd):
        out = self.lines_until_ok(cmd)
        if not out[-1].startswith("OK") or any("Err" in line for line in out):
            raise RuntimeError(f"{cmd!r} -> {out}")
        return out

    def version(self):
        self.ser.write(b"V\r")
        return self.ser.readline().decode("ascii", "replace").strip()

    def qe(self):
        '''Return (motor1, motor2) from QE; 0 means disabled.'''
        m1, m2 = (int(x) for x in self.ok("QE")[0].split(","))
        return m1, m2

    def wait_idle(self, timeout=5.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.ser.write(b"QM\r")
            if self.ser.readline().decode("ascii", "replace").strip() == "QM,0,0,0,0":
                return
        raise RuntimeError("motion did not finish")

    def close(self):
        self.ser.close()


def enabled(state):
    return state[0] != 0 and state[1] != 0


def check(name, cond, detail):
    results.append((name, cond))
    print(f"  {'PASS' if cond else 'FAIL'}  {name}: {detail}")


def test_enable_each_command(ebb, config, auto, expect_enabled):
    '''Disable both drivers, run one move of each type, check QE afterwards.'''
    ebb.ok(f"SC,2,{config}")
    ebb.ok(f"CU,50,{auto}")
    for name, move in MOVES.items():
        ebb.ok("EM,0,0")
        ebb.wait_idle()
        before = ebb.qe()
        ebb.ok(move)
        ebb.wait_idle()
        after = ebb.qe()
        check(f"SC,2,{config} CU,50,{auto} {name}",
              before == (0, 0) and enabled(after) == expect_enabled,
              f"QE before {before}, after {after}, expected "
              f"{'enabled' if expect_enabled else 'disabled'}")


def test_external_after_timeout(ebb):
    '''#245, in use: after the idle timeout disables external drivers,
    the next move must re-enable them.'''
    ebb.ok("SC,2,1")
    ebb.ok("CU,50,1")
    ebb.ok("CU,61,1")
    ebb.ok("EM,1,1")
    ebb.wait_idle()
    ebb.ok(MOVES["SM"])
    ebb.wait_idle()
    time.sleep(1.5)
    idle = ebb.qe()
    ebb.ok(MOVES["SM"])
    ebb.wait_idle()
    after = ebb.qe()
    check("SC,2,1 move after idle timeout re-enables",
          idle == (0, 0) and enabled(after),
          f"QE after timeout {idle}, after next move {after}")
    ebb.ok("CU,61,0")
    ebb.ok("SC,2,0")


def test_timeout_restarts_on_parse(ebb):
    '''#246: parsing a motion command must restart the idle countdown.
    CU,256,1 parses motion commands without queuing them, so the move never
    loads; the countdown must then restart from the parse.'''
    ebb.ok("SC,2,0")
    ebb.ok("CU,50,1")
    ebb.ok("CU,61,1")                        # 1 s idle timeout
    ebb.ok(MOVES["SM"])
    ebb.wait_idle()                          # FIFO empty: countdown starts
    t_idle = time.monotonic()
    time.sleep(0.5)
    ebb.ok("CU,256,1")
    ebb.ok(MOVES["T3"])                      # parse-time auto enable, not queued
    ebb.ok("CU,256,0")
    time.sleep(max(0.0, t_idle + 1.2 - time.monotonic()))
    mid = ebb.qe()                           # original countdown would have fired at 1.0 s
    time.sleep(max(0.0, t_idle + 1.9 - time.monotonic()))
    late = ebb.qe()                          # restarted countdown fires at about 1.5 s
    check("parsed motion command restarts idle countdown",
          enabled(mid), f"QE 1.2 s after idle (0.7 s after parse): {mid}")
    check("restarted idle countdown still fires",
          late == (0, 0), f"QE 1.9 s after idle (1.4 s after parse): {late}")
    ebb.ok("CU,61,0")


def test_timeout_regression(ebb):
    '''The idle timeout still disables after inactivity, and motion re-enables.'''
    ebb.ok("SC,2,0")
    ebb.ok("CU,50,1")
    ebb.ok("CU,61,1")
    ebb.ok(MOVES["SM"])
    ebb.wait_idle()
    time.sleep(1.5)
    idle = ebb.qe()
    ebb.ok(MOVES["SM"])
    ebb.wait_idle()
    moved = ebb.qe()
    time.sleep(1.5)
    idle2 = ebb.qe()
    check("idle timeout disables after 1 s", idle == (0, 0), f"QE {idle}")
    check("next move re-enables", enabled(moved), f"QE {moved}")
    check("idle timeout disables again", idle2 == (0, 0), f"QE {idle2}")
    ebb.ok("CU,61,0")


def test_timeout_race(ebb, trials):
    '''#246, in use: send a move timed to arrive as the 1 s idle timeout
    expires (swept over +/- 2 ms), then check that the move did not run with
    the drivers disabled. The first move's OK arrives as it is queued; with an
    empty FIFO it starts within one 40 us tick and lasts 20 ms.'''
    ebb.ok("SC,2,0")
    ebb.ok("CU,50,1")
    ebb.ok("CU,61,1")
    hits = 0
    for _ in range(trials):
        ebb.ser.reset_input_buffer()
        ebb.ser.write((MOVES["SM"] + "\r").encode("ascii"))
        ebb.ser.readline()
        target = time.perf_counter() + 0.020 + 1.000 + random.uniform(-0.002, 0.002)
        while time.perf_counter() < target:
            pass
        ebb.ser.write((MOVES["SM"] + "\r").encode("ascii"))
        ebb.ser.readline()
        ebb.wait_idle()
        if ebb.qe() == (0, 0):
            hits += 1
            ebb.ok("EM,1,1")
            ebb.wait_idle()
        time.sleep(0.05)
    check("no move runs with drivers disabled near idle timeout expiry",
          hits == 0, f"{hits} of {trials} moves ran with drivers disabled")
    ebb.ok("CU,61,0")


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else find_port()
    race_trials = int(sys.argv[2]) if len(sys.argv) > 2 else 300
    ebb = EBB(port)
    print("Firmware:", ebb.version())
    try:
        print("\n#245: auto enable, PIC controls external drivers (SC,2,1)")
        test_enable_each_command(ebb, config=1, auto=1, expect_enabled=True)
        test_external_after_timeout(ebb)
        print("\nRegression: built-in drivers (SC,2,0)")
        test_enable_each_command(ebb, config=0, auto=1, expect_enabled=True)
        print("\nRegression: auto enable off (CU,50,0) with SC,2,1")
        test_enable_each_command(ebb, config=1, auto=0, expect_enabled=False)
        print("\n#246: idle timeout and parse-time enable")
        test_timeout_restarts_on_parse(ebb)
        print("\nRegression: idle timeout behavior")
        test_timeout_regression(ebb)
        if race_trials:
            print(f"\n#246: timing race, {race_trials} trials")
            test_timeout_race(ebb, race_trials)
    finally:
        for cmd in ("CU,256,0", "ES", "CU,61,0", "SC,2,0", "CU,50,1", "EM,0,0"):
            try:
                ebb.lines_until_ok(cmd)
            except RuntimeError as err:
                print("restore", cmd, "failed:", err)
        ebb.close()
    failed = [name for name, passed in results if not passed]
    print(f"\n{len(results) - len(failed)} passed, {len(failed)} failed")
    for name in failed:
        print("  FAILED:", name)
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
