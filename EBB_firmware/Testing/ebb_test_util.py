#!/usr/bin/env python
# -*- encoding: utf-8 -#-

'''
ebb_test_util.py

Shared helper for the EBB command regression tests (test_cmd_*.py).
Run the tests against an EBB with nothing attached except USB.

Provides:
- EBB: a serial connection that sends commands and returns their raw
  response bytes, and reboots the EBB safely (the port is closed before
  the EBB leaves USB, then reopened once it is back).
- Results: records checks (each tied to a documented command, e.g.
  "CU,10" or "QM"), omitted items (documented behavior not tested because
  of a known firmware issue), and a transcript of every command and its raw
  response, which can be saved as JSON and compared across firmware
  versions with compare_transcripts.py.

Common options for every test script:
    [port]                 serial port (default: the EBB with the nickname)
    --nickname NAME        use only the EBB whose nickname (USB serial number)
                           is NAME; default from the EBB_NICKNAME environment
                           variable. If neither is given and more than one EBB
                           is connected, the test refuses to run.
    --transcript FILE      save the transcript (JSON) to FILE
'''

import argparse
import json
import os
import sys
import time

import serial
from serial.tools.list_ports import comports

EBB_VID, EBB_PID = 0x04D8, 0xFD92
RESYNC = bytes(242) + b"\r"       # see "Resynchronizing" in the Y docs


NICKNAME = os.environ.get("EBB_NICKNAME")


def find_port(nickname=None):
    '''Serial port of the EBB with this nickname (its USB serial number).
    With no nickname, the only EBB connected; None if there is no single
    match.'''
    nickname = nickname or NICKNAME
    ebbs = [p for p in comports() if p.vid == EBB_VID and p.pid == EBB_PID]
    if nickname:
        ebbs = [p for p in ebbs if p.serial_number == nickname]
    return ebbs[0].device if len(ebbs) == 1 else None


class Results:
    def __init__(self):
        self.checks = []          # (item, name, passed, detail)
        self.omitted = []         # (item, reason)
        self.transcript = []      # {"item", "cmd", "response"}
        self.item = ""

    def check(self, name, cond, detail=""):
        self.checks.append((self.item, name, bool(cond), detail))
        mark = "PASS" if cond else "FAIL"
        print(f"  {mark}  [{self.item}] {name}" + (f": {detail}" if detail and not cond else ""))

    def omit(self, item, reason):
        self.omitted.append((item, reason))
        print(f"  OMIT  [{item}] {reason}")

    def summary(self):
        passed = sum(1 for c in self.checks if c[2])
        print(f"\n{passed}/{len(self.checks)} checks passed, {len(self.omitted)} items omitted")
        for item, name, ok, detail in self.checks:
            if not ok:
                print(f"  FAIL  [{item}] {name}: {detail}")
        return passed == len(self.checks)

    def save(self, path, version):
        with open(path, "w") as f:
            json.dump({"version": version, "transcript": self.transcript,
                       "checks": [{"item": i, "name": n, "passed": p} for i, n, p, _ in self.checks],
                       "omitted": [{"item": i, "reason": r} for i, r in self.omitted]},
                      f, indent=1)


class EBB:
    '''Serial connection to an EBB. Every exchange is recorded in results.'''

    def __init__(self, port=None, results=None, nickname=None):
        self.nickname = nickname or NICKNAME
        self.port = port or find_port(self.nickname)
        if not self.port:
            sys.exit(f"No single EiBotBoard found (nickname {self.nickname!r}); "
                     "use --nickname or EBB_NICKNAME when several are connected")
        self.results = results or Results()
        self.ser = None
        self.open()

    def open(self):
        self.ser = serial.Serial(self.port, timeout=0.05)
        time.sleep(0.2)
        self.resync()

    def close(self):
        if self.ser and self.ser.is_open:
            time.sleep(0.1)
            self.ser.read(self.ser.in_waiting or 1)
            self.ser.close()

    def resync(self):
        '''Return to a known state between commands; discard any responses.'''
        self.ser.write(RESYNC)
        self.ser.flush()
        time.sleep(0.2)
        self.ser.reset_input_buffer()

    def raw(self, data, quiet=0.05, timeout=2.0, expect=None, record=True):
        '''Send data (bytes or str; a str gets "\\r" appended) and return
        all response bytes. Reading stops when expect (bytes) has been
        received, or when no byte has arrived for `quiet` seconds after the
        first one, or after `timeout`. record=False leaves the exchange out
        of the transcript (for build-dependent values such as RAM
        addresses).'''
        if isinstance(data, str):
            data = (data + "\r").encode("ascii")
        self.ser.write(data)
        self.ser.flush()
        out = b""
        end = time.monotonic() + timeout
        last = None
        while time.monotonic() < end:
            chunk = self.ser.read(self.ser.in_waiting or 1)
            if chunk:
                out += chunk
                last = time.monotonic()
                if expect is not None and out.endswith(expect):
                    break
            elif last is not None and time.monotonic() - last > quiet:
                break
        if record:
            self.results.transcript.append({"item": self.results.item,
                                            "cmd": data.decode("latin-1"),
                                            "response": out.decode("latin-1")})
        return out

    def mr(self, address):
        '''MR: one RAM byte (not recorded in the transcript).'''
        r = self.raw(f"MR,{address}", expect=b"\r\n", record=False)
        return int(r.split(b",")[1])

    def ok(self, cmd):
        '''Send a command that must answer OK (legacy mode).'''
        r = self.raw(cmd, expect=b"OK\r\n")
        if not r.endswith(b"OK\r\n") or b"Err" in r:
            raise RuntimeError(f"{cmd!r} -> {r!r}")
        return r

    def wait_idle(self, timeout=30.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if self.raw("QM").lstrip(b"\r").startswith(b"QM,0,0,0,0"):
                return
            time.sleep(0.02)
        raise RuntimeError("motion did not finish")

    def version(self):
        return self.raw("V").strip().decode("ascii", "replace")

    def reboot(self, cmd="RB", wait=2.0, timeout=15.0):
        '''Send a command that makes the EBB leave USB (RB), closing the port
        before it disconnects, then wait for it to return and reopen. Never
        holds the port across the disconnect.'''
        self.ser.write((cmd + "\r").encode("ascii"))
        self.ser.flush()
        time.sleep(0.05)
        self.ser.close()
        self.results.transcript.append({"item": self.results.item, "cmd": cmd + "\r",
                                        "response": "<port closed for reboot>"})
        time.sleep(wait)
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            port = find_port(self.nickname)
            if port:
                self.port = port
                time.sleep(1.0)
                self.open()
                return True
            time.sleep(0.25)
        return False

    def reset(self):
        '''R, then back to legacy mode defaults.'''
        self.raw("R", expect=b"\r\n")
        self.resync()


def parse_args(description, options=()):
    ap = argparse.ArgumentParser(description=description)
    ap.add_argument("port", nargs="?")
    ap.add_argument("--nickname", help="use only the EBB with this nickname")
    ap.add_argument("--transcript", help="save the transcript (JSON) to this file")
    for flag, help_text in options:
        ap.add_argument(flag, action="store_true", help=help_text)
    return ap.parse_args()


def run(description, tests, options=()):
    '''Run a list of test functions, each test(ebb, results), resetting the
    EBB (R) before each. options: (flag, help) pairs for opt-in tests; the
    parsed arguments are available to tests as results.args. Prints a
    summary; exits 0 if all checks passed.'''
    args = parse_args(description, options)
    results = Results()
    results.args = args
    ebb = EBB(args.port, results, args.nickname)
    version = ebb.version()
    print(version)
    try:
        for test in tests:
            print(test.__name__)
            ebb.reset()
            try:
                test(ebb, results)
            except Exception as e:      # keep going; record the failure
                results.check(f"{test.__name__} completed", False, repr(e))
                ebb.resync()
        ebb.reset()
    finally:
        ebb.close()
    ok = results.summary()
    if args.transcript:
        results.save(args.transcript, version)
        print(f"transcript saved to {args.transcript}")
    sys.exit(0 if ok else 1)
