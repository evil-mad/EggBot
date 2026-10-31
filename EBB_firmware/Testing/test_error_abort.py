#!/usr/bin/env python
# -*- encoding: utf-8 -#-

'''
test_error_abort.py

EBB regression tests for the "Returned Errors" rule (docs/ebb.html): once an
error is detected the command is aborted, and a command that reports an
error does not also reply OK (or its normal reply). Run against an EBB with
nothing attached except USB:

    python3 test_error_abort.py [port] [--transcript FILE]

Every command that is safe to send (not R, RB, BL, ST, C or O) is sent with
one extra parameter, in legacy and future mode. The only reply must be the
!7 error, and the command must have no effect, judged by a query or register
read taken before and after. Each such case then sends the valid command
(without the extra parameter) and checks that the probe changes, so a probe
that cannot see the effect does not pass by default. Commands whose effect
no query can read (CK) are checked for the reply only. PC, PG and T were
removed in v3.0 and are not tested. A few errors found inside
a handler (XM parse and rate errors, LM/L3 negative values) are checked the
same way.
'''

import time

import ebb_test_util as u
from test_cmd_pen_servo import RATE_DOWN, RATE_UP, channel, mr16

# PIC18F46J50 special function registers (fixed by the chip, not the build)
ANCON0, ANCON1 = 0xF48, 0xF49
LATB, TRISB = 0xF8A, 0xF93
CCP1CON, CCPR1L, T2CON = 0xFBA, 0xFBB, 0xFCA
RPOR6 = 0xECC

EXTRA = b"!7 Err: Extra parameter"


def modes(ebb):
    '''Yield "legacy" then "future", switching CU,10 and restoring legacy
    mode afterward.'''
    yield "legacy"
    ebb.raw("CU,10,1")
    try:
        yield "future"
    finally:
        ebb.raw("CU,10,0")
        ebb.resync()


def error_reply(mode, cmd, text):
    '''The complete reply for an error, in the documented format.'''
    if mode == "legacy":
        return text + b"\r\n"
    return cmd.split(",")[0].encode() + b"," + text + b"\n"


def query(cmd):
    return lambda ebb: ebb.raw(cmd)


def regs(*addresses, mask=0xFF):
    return lambda ebb: [ebb.mr(a) & mask for a in addresses]


def settle(ebb):
    time.sleep(0.1)
    ebb.wait_idle()


def check_case(ebb, r, mode, cmd, setup=(), probe=None, cleanup=(), want=EXTRA):
    '''Send setup commands, read the probe, send cmd, and check that the only
    reply is the error `want` and that the probe reads the same afterward.
    For an extra parameter, then send cmd without it and check that the
    probe changes.'''
    for c in setup:
        ebb.raw(c)
    settle(ebb)
    before = probe(ebb) if probe else None
    resp = ebb.raw(cmd)
    expected = error_reply(mode, cmd, want)
    r.check(f"{mode}: {cmd} replies only the error", resp == expected,
            f"{resp!r} (expected {expected!r})")
    if probe:
        settle(ebb)
        after = probe(ebb)
        r.check(f"{mode}: {cmd} has no effect", after == before, f"{before!r} -> {after!r}")
        if want == EXTRA:
            valid = cmd.rsplit(",", 1)[0]
            ebb.raw(valid)
            settle(ebb)
            changed = probe(ebb)
            r.check(f"{mode}: control: {valid} changes the probe", changed != before,
                    f"{before!r} -> {changed!r}")
    for c in cleanup:
        ebb.raw(c)
    settle(ebb)


def run_cases(ebb, r, item, cases):
    for mode in modes(ebb):
        for case in cases:
            r.item = item
            check_case(ebb, r, mode, **case)


def test_extra_motion(ebb, r):
    qs = query("QS")
    zero = ("EM,1,1", "CS")
    run_cases(ebb, r, "Returned Errors !7 (motion)", [
        dict(cmd="SM,100,10,0,0,9", setup=zero, probe=qs),
        dict(cmd="XM,100,10,10,0,9", setup=zero, probe=qs),
        dict(cmd="HM,1000,0,0,9", setup=zero + ("SM,100,10",), probe=qs),
        dict(cmd="LM,85899346,10,0,0,0,0,0,9", setup=zero, probe=qs),
        dict(cmd="LT,1000,85899346,0,0,0,0,9", setup=zero, probe=qs),
        dict(cmd="L3,85899346,10,0,0,0,0,0,0,0,9", setup=zero, probe=qs),
        dict(cmd="T3,1000,85899346,0,0,0,0,0,0,9", setup=zero, probe=qs),
        dict(cmd="TD,1000,85899346,85899346,0,0,0,0,0,0,0,9", setup=zero, probe=qs),
        dict(cmd="TR,1000,10,0,0,9", setup=zero, probe=qs),
        dict(cmd="CS,9", setup=zero + ("SM,100,10",), probe=qs),
        dict(cmd="EM,1,1,9", setup=("EM,0,0",), probe=query("QE")),
    ])
    # ES with an extra parameter must not stop a move in progress
    for mode in modes(ebb):
        r.item = "Returned Errors !7 (motion)"
        for c in zero:
            ebb.raw(c)
        settle(ebb)
        ebb.raw("SM,1500,150")
        resp = ebb.raw("ES,0,9")
        expected = error_reply(mode, "ES", EXTRA)
        r.check(f"{mode}: ES,0,9 replies only the error", resp == expected,
                f"{resp!r} (expected {expected!r})")
        settle(ebb)
        pos = ebb.raw("QS")
        r.check(f"{mode}: ES,0,9 does not stop the move (QS 150,0)", b"150,0" in pos, pos)


def test_extra_pen_servo(ebb, r):
    # One stepping move first, so the FIFO slot holds no stale step bits (#253)
    ebb.raw("EM,1,1")
    ebb.raw("SM,50,10")
    ebb.raw("SM,50,-10")
    settle(ebb)
    qp = query("QP")
    run_cases(ebb, r, "Returned Errors !7 (pen and servo)", [
        dict(cmd="SP,0,0,4,9", setup=("SP,1",), probe=qp),
        dict(cmd="TP,0,9", setup=("SP,1",), probe=qp),
        dict(cmd="S2,12000,5,0,0,9", setup=("S2,0,5",), probe=lambda e: channel(e, 5),
             cleanup=("S2,0,5",)),
        dict(cmd="SE,1,1023,0,9", setup=("SE,0",), probe=regs(CCPR1L, T2CON, RPOR6),
             cleanup=("SE,0",)),
        dict(cmd="SR,60000,1,9", setup=("SR,60000,0",), probe=query("QR")),
        dict(cmd="SC,10,500,9", setup=("SC,10,0",),
             probe=lambda e: (mr16(e, RATE_UP), mr16(e, RATE_DOWN))),
    ])


def test_extra_config(ebb, r):
    qn = query("QN")
    run_cases(ebb, r, "Returned Errors !7 (configuration)", [
        dict(cmd="SL,5,0,9", setup=("SL,0,0",), probe=query("QL,0")),
        dict(cmd="SN,5,9", setup=("SN,0",), probe=qn),
        dict(cmd="NI,9", setup=("SN,5",), probe=qn),
        dict(cmd="ND,9", setup=("SN,5",), probe=qn),
        dict(cmd="CU,60,404,9", setup=("CU,60,0",), probe=query("QU,60")),
        # CCPR1L (engraver duty) is harmless to write with the engraver off
        dict(cmd=f"MW,{CCPR1L},5,9", setup=("SE,0",), probe=regs(CCPR1L), cleanup=("SE,0",)),
        dict(cmd="CK,1,2,3,4,5,6,7,8,9"),
    ])


def test_extra_io(ebb, r):
    run_cases(ebb, r, "Returned Errors !7 (I/O)", [
        dict(cmd="PD,B,0,0,9", probe=regs(TRISB, mask=0x01), cleanup=("PD,B,0,1",)),
        dict(cmd="PO,B,0,0,9", probe=regs(LATB, TRISB, mask=0x01),
             cleanup=("PO,B,0,1", "PD,B,0,1", "QB")),
        dict(cmd="AC,1,1,9", setup=("AC,1,0",), probe=regs(ANCON0, ANCON1),
             cleanup=("AC,1,0",)),
    ])


def test_extra_queries(ebb, r):
    run_cases(ebb, r, "Returned Errors !7 (queries)", [dict(cmd=c) for c in (
        "V,9", "QS,9", "QG,9", "QP,9", "QE,9", "QM,9", "QB,9", "QC,9", "QN,9",
        "QR,9", "QT,9", "QL,0,9", "QU,3,9", "MR,0,9", "PI,B,0,9", "I,9", "A,9")])


def test_errors_without_ok(ebb, r):
    qs = query("QS")
    zero = ("EM,1,1", "CS")
    run_cases(ebb, r, "Returned Errors (no OK after an error)", [
        dict(cmd="XM,abc,10,10", setup=zero, probe=qs,
             want=b"!5 Err: Need comma next, found: 'a'"),
        dict(cmd="XM,100,10", setup=zero, probe=qs, want=b"!4 Err: Missing parameter(s)"),
        dict(cmd="LM,-85899346,-10,0,0,0,0", setup=zero, probe=qs,
             want=b"!6 Err: Invalid parameter value"),
        dict(cmd="L3,-85899346,-10,0,0,0,0,0,0", setup=zero, probe=qs,
             want=b"!6 Err: Invalid parameter value"),
        # SN parse errors: no OK, node count unchanged (#262)
        dict(cmd="SN", setup=("SN,7",), probe=query("QN"), want=b"!4 Err: Missing parameter(s)"),
        dict(cmd="SN,-5", setup=("SN,7",), probe=query("QN"),
             want=b"!6 Err: Invalid parameter value"),
    ])
    # XM rate error: no OK, and no motion. The line ending after !0 is
    # covered by the !0 tests (#265), not here.
    for mode in modes(ebb):
        r.item = "Returned Errors (no OK after an error)"
        for c in zero:
            ebb.raw(c)
        settle(ebb)
        before = ebb.raw("QS")
        resp = ebb.raw("XM,1,15,15")
        r.check(f"{mode}: XM,1,15,15 reports !0", b"!0 Err: <axis1> step rate too high" in resp,
                resp)
        r.check(f"{mode}: XM,1,15,15 does not also reply OK",
                b"OK" not in resp and not resp.rstrip(b"\r\n").endswith(b"XM"), resp)
        settle(ebb)
        pos = ebb.raw("QS")
        r.check(f"{mode}: XM,1,15,15 does not move", pos == before, f"{before!r} -> {pos!r}")


def test_error_format(ebb, r):
    # Commands whose replies start with their name report errors in the
    # same form as every other command (#254)
    missing, bad = b"!4 Err: Missing parameter(s)", b"!6 Err: Invalid parameter value"
    run_cases(ebb, r, "Returned Errors (format)", [
        dict(cmd="PI,B3", want=b"!5 Err: Need comma next, found: '3'"),
        dict(cmd="PI,B,9", want=bad),
        dict(cmd="PI", want=missing),
        dict(cmd="MR", want=missing),
        dict(cmd="QU", want=missing),
        dict(cmd="QU,0", want=bad),
        dict(cmd="QL,-1", want=bad),
        dict(cmd="ES,-1", want=bad),
    ])


if __name__ == "__main__":
    u.run("EBB Returned Errors: an erroring command is aborted and sends no OK",
          [test_extra_motion, test_extra_pen_servo, test_extra_config, test_extra_io,
           test_extra_queries, test_errors_without_ok, test_error_format])
