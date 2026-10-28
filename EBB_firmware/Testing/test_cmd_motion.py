#!/usr/bin/env python
# -*- encoding: utf-8 -#-

'''
test_cmd_motion.py

EBB regression tests: motion commands and queries. SM, XM, HM, LM, LT, L3,
T3, TD, TR, ES, EM, QE, QM, QS, CS, and the SC,2 driver modes. Run against
an EBB with nothing attached except USB:

    python3 test_cmd_motion.py [port] [--transcript FILE]

Positions are checked with QS. For time-limited moves (LT, T3, TD), both
positions and both step accumulators (QU,200) are checked against a model
of the motion ISR.

Documented behavior that current firmware does not follow is listed as
omitted, with the reason, rather than tested.
'''

import re
import time

import ebb_test_util as u

TRISC, TRISD, TRISE = 3988, 3989, 3990
M32 = 0xFFFFFFFF


def s32(x):
    x &= M32
    return x - (1 << 32) if x & 0x80000000 else x


def tdiv(a, d):
    q = abs(a) // d
    return q if a >= 0 else -q


def model_t3(intervals, axes, acc=(0, 0)):
    '''Positions and accumulators after a T3 move (Rate, Accel, Jerk per
    axis), from the given accumulators: Rate and Accel are pre-adjusted,
    then each 40 us tick Accel += Jerk, Rate += Accel, accumulator += Rate,
    and a step is taken (in the direction of Rate) when bit 31 is set.'''
    pos, out = [], []
    for (r, a, j), a0 in zip(axes, acc):
        rate, accel, steps = s32(r - tdiv(a, 2) + tdiv(j, 6)), s32(a - j), 0
        for _ in range(intervals):
            accel = s32(accel + j)
            rate = s32(rate + accel)
            a0 = (a0 + rate) & M32
            if a0 & 0x80000000:
                a0 &= 0x7FFFFFFF
                steps += -1 if rate < 0 else 1
        pos.append(steps)
        out.append(a0)
    return tuple(pos), tuple(out)


def qs(ebb):
    m = re.fullmatch(rb"(-?\d+),(-?\d+)\n\rOK\r\n", ebb.raw("QS"))
    return (int(m.group(1)), int(m.group(2))) if m else None


def accumulators(ebb):
    m = re.fullmatch(rb"QU,(\d+),(\d+)\r\nOK\r\n", ebb.raw("QU,200"))
    return (int(m.group(1)), int(m.group(2))) if m else None


def run_move(ebb, cmd):
    '''CS, the move, wait until idle; returns (QS, accumulators, seconds).'''
    ebb.ok("CS")
    t0 = time.monotonic()
    ebb.ok(cmd)
    ebb.wait_idle(60)
    return qs(ebb), accumulators(ebb), time.monotonic() - t0


def test_sm(ebb, r):
    r.item = "SM"
    pos, _, dt = run_move(ebb, "SM,1000,250,-766")
    r.check("doc example SM,1000,250,-766 ends at 250,-766", pos == (250, -766), pos)
    r.check("  ...and takes about 1000 ms", 0.95 < dt < 1.3, f"{dt:.2f} s")
    pos, _, dt = run_move(ebb, "SM,500,0,0")
    r.check("zero steps: a 500 ms delay", pos == (0, 0) and 0.45 < dt < 0.8, (pos, f"{dt:.2f} s"))
    pos, _, _ = run_move(ebb, "SM,100,40")
    r.check("AxisSteps2 optional", pos == (40, 0), pos)
    resp = ebb.raw("SM,0,10,10")
    r.check("Duration 0 is rejected", resp == b"!6 Err: Invalid parameter value\r\n", resp)
    ebb.raw("CU,10,1")
    resp = ebb.raw("SM,10,1,1")
    ebb.raw("CU,10,0")
    r.check("future format: SM<NL>", resp == b"SM\n", resp)
    ebb.wait_idle()
    r.omit("SM (delay capped at 100000 ms)", "takes 100 s to observe")
    r.omit("SM (step rates above 25,000 steps/s)",
           "rates above about 50,000 steps/s are accepted (D51); 25,000-50,000 are tested in test_cmd_general.py")


def test_xm(ebb, r):
    r.item = "XM"
    pos, _, _ = run_move(ebb, "XM,1000,550,-1234")
    r.check("doc example XM,1000,550,-1234: axis 1 = A+B, axis 2 = A-B", pos == (-684, 1784), pos)
    pos, _, dt = run_move(ebb, "XM,300,0,0")
    r.check("zero steps: a delay", pos == (0, 0) and 0.25 < dt < 0.6, (pos, f"{dt:.2f} s"))
    ebb.ok("CS")
    resp = ebb.raw("XM,1,15,15")
    r.check("rate is checked on axes 1 and 2 (A+B = 30 steps in 1 ms)",
            resp.startswith(b"!0 Err: <axis1> step rate too high"), resp)
    ebb.wait_idle()
    r.check("  ...and the move did not run", qs(ebb) == (0, 0))
    r.omit("XM (parse errors)", "XM runs after a parse error, then reports it (D16)")


def test_hm(ebb, r):
    r.item = "HM"
    ebb.ok("CS")
    ebb.ok("SM,300,100,-50")
    ebb.ok("HM,1000")
    ebb.wait_idle()
    r.check("HM waits for the earlier move, then returns home (0,0)", qs(ebb) == (0, 0))
    ebb.ok("HM,2000,300,-200")
    ebb.wait_idle()
    r.check("HM,2000,300,-200 moves to an absolute position", qs(ebb) == (300, -200))
    ebb.ok("HM,2000,300,-200")
    ebb.wait_idle()
    r.check("repeating it does not move", qs(ebb) == (300, -200))
    ebb.ok("HM,25000")
    ebb.wait_idle()
    r.check("HM at the maximum rate returns home", qs(ebb) == (0, 0))
    r.omit("HM (Position1 without Position2, overflow, parse errors)",
           "Position1 alone is accepted, overflow is not reported, and parse errors still move (D15)")


def test_lm_l3(ebb, r):
    r.item = "LM"
    pos, _, _ = run_move(ebb, "LM,3865471,60,1732,0,0,0")
    r.check("doc example 1 (60 steps)", pos == (60, 0), pos)
    pos, _, _ = run_move(ebb, "LM,3865471,100,0,0,0,0")
    r.check("constant rate, 100 steps", pos == (100, 0), pos)
    pos, _, _ = run_move(ebb, "LM,3865471,-60,1732,0,0,0")
    r.check("negative Steps with positive Rate reverses direction", pos == (-60, 0), pos)
    pos, _, _ = run_move(ebb, "LM,20000000,300,0,-10000000,200,0")
    r.check("both axes, Rate sign sets direction", pos == (300, -200), pos)
    resp = ebb.raw("LM,0,0,0,0,0,0")
    r.check("no possible motion: !6", resp == b"!6 Err: Invalid parameter value\r\n", resp)
    ebb.raw("CU,10,1")
    resp = ebb.raw("LM,20000000,10,0,0,0,0")
    ebb.raw("CU,10,0")
    r.check("future format: LM<NL>", resp == b"LM\n", resp)
    ebb.wait_idle()
    r.omit("LM (negative Steps with negative Rate)", "replies OK, then !6 (D53)")
    r.item = "L3"
    pos, _, _ = run_move(ebb, "L3,0,500,0,100,0,0,0,0,3")
    r.check("doc example 1 (500 steps)", pos == (500, 0), pos)
    pos, _, _ = run_move(ebb, "L3,80000000,1000,0,50,40000000,600,0,-30")
    r.check("doc example 2: axis 1 moves 1000 steps", pos is not None and pos[0] == 1000, pos)
    resp = ebb.raw("L3,0,0,0,0,0,0,0,0")
    r.check("no possible motion: !6", resp == b"!6 Err: Invalid parameter value\r\n", resp)


def check_t3_move(ebb, r, name, cmd, intervals, axes):
    pos, acc, _ = run_move(ebb, cmd)
    want = model_t3(intervals, axes)
    r.check(f"{name}: positions and accumulators match the model",
            (pos, acc) == want, f"EBB {pos} {acc}, model {want}")


def test_lt_t3_td(ebb, r):
    r.item = "LT"
    check_t3_move(ebb, r, "doc example 1", "LT,10169,3865471,1732,0,0,3", 10169,
                  ((3865471, 1732, 0), (0, 0, 0)))
    check_t3_move(ebb, r, "doc example 2", "LT,25000,3865471,0,0,0", 25000,
                  ((3865471, 0, 0), (0, 0, 0)))
    check_t3_move(ebb, r, "both axes, negative rate", "LT,2000,-20000000,500,30000000,-700", 2000,
                  ((-20000000, 500, 0), (30000000, -700, 0)))
    r.item = "T3"
    check_t3_move(ebb, r, "doc example 1", "T3,5000,0,0,100,0,0,0,3", 5000,
                  ((0, 0, 100), (0, 0, 0)))
    check_t3_move(ebb, r, "doc example 2", "T3,10000,50000000,10000,5,30000000,8000,-3", 10000,
                  ((50000000, 10000, 5), (30000000, 8000, -3)))
    resp = ebb.raw("T3,0,1000,0,0,1000,0,0")
    r.check("Intervals 0: !6", resp == b"!6 Err: Invalid parameter value\r\n", resp)
    r.item = "TD"
    # TD = T3,Intervals,Rate1A,0,Jerk1,... then T3,Intervals,Rate1B,Accel1,-Jerk1,...
    pos, acc, _ = run_move(ebb, "TD,1000,0,49950016,100000,100,0,0,0,0,3")
    first = model_t3(1000, ((0, 0, 100), (0, 0, 0)))
    second = model_t3(1000, ((49950016, 100000, -100), (0, 0, 0)), acc=first[1])
    want = ((first[0][0] + second[0][0], first[0][1] + second[0][1]), second[1])
    r.check("doc example: the two T3 halves, by the model", (pos, acc) == want,
            f"EBB {pos} {acc}, model {want}")


def test_tr(ebb, r):
    r.item = "TR"
    pos, _, dt = run_move(ebb, "TR,1000,100,50")
    r.check("TR,1000,100,50 moves 100,50", pos == (100, 50), pos)
    r.check("  ...at about 1000 steps/s (0.1 s)", dt < 0.5, f"{dt:.2f} s")
    r.omit("TR (AxisSteps2 omitted, StepRate 0 or above 25000)",
           "AxisSteps2 is required, and rate errors come after OK (D17)")


def test_es(ebb, r):
    r.item = "ES"
    r.check("idle: 0", ebb.raw("ES") == b"0\n\rOK\r\n")
    ebb.ok("SM,2000,0,0")
    time.sleep(0.1)
    r.check("stopping only a delay: 0", ebb.raw("ES") == b"0\n\rOK\r\n")
    ebb.ok("CS")
    ebb.ok("SM,2000,200,200")
    time.sleep(0.3)
    r.check("stopping a move: 1", ebb.raw("ES") == b"1\n\rOK\r\n")
    pos = qs(ebb)
    time.sleep(0.3)
    r.check("motion stopped (QS unchanged)", qs(ebb) == pos and pos != (200, 200), pos)
    ebb.ok("SM,2000,0,0")
    ebb.ok("SM,2000,0,0")
    r.check("a command waiting in the FIFO: 1", ebb.raw("ES") == b"1\n\rOK\r\n")
    r.check("FIFO flushed (idle)", ebb.raw("QM") == b"QM,0,0,0,0\n\r")
    ebb.ok("EM,1,1")
    r.check("ES,1 disables the motors", ebb.raw("ES,1") == b"0\n\rOK\r\n"
            and ebb.raw("QE") == b"0,0\n\rOK\r\n")
    ebb.ok("EM,1,1")
    ebb.raw("ES,0")
    r.check("ES,0 leaves the motors enabled", ebb.raw("QE") == b"16,16\n\rOK\r\n")
    ebb.raw("CU,10,1")
    resp = ebb.raw("ES")
    ebb.raw("CU,10,0")
    r.check("future format: ES,0<NL>", resp == b"ES,0\n", resp)


def test_em_qe(ebb, r):
    r.item = "EM"
    for value, divisor in ((1, 16), (2, 8), (3, 4), (4, 2), (5, 1)):
        ebb.ok(f"EM,{value},{value}")
        resp = ebb.raw("QE")
        r.check(f"EM,{value},{value}: QE {divisor},{divisor}",
                resp == f"{divisor},{divisor}\n\rOK\r\n".encode(), resp)
    ebb.ok("EM,0,1")
    r.check("EM,0,1: motor 1 off, motor 2 on at the previous mode", ebb.raw("QE") == b"0,1\n\rOK\r\n")
    ebb.ok("EM,0,0")
    r.check("EM,0,0: both off", ebb.raw("QE") == b"0,0\n\rOK\r\n")
    ebb.ok("SM,1500,0,0")
    ebb.ok("EM,3,3")
    r.check("EM is queued: no change while a delay executes", ebb.raw("QE") == b"0,0\n\rOK\r\n")
    ebb.wait_idle()
    r.check("  ...and takes effect after it", ebb.raw("QE") == b"4,4\n\rOK\r\n")
    ebb.ok("CS")
    ebb.ok("SM,100,50,50")
    ebb.wait_idle()
    ebb.ok("EM,1,1")
    time.sleep(0.05)
    r.check("EM resets the global step counters (QS 0,0)", qs(ebb) == (0, 0))
    r.item = "QE"
    ebb.raw("CU,10,1")
    resp = ebb.raw("QE")
    ebb.raw("CU,10,0")
    r.check("future format: QE,16,16<NL>", resp == b"QE,16,16\n", resp)
    r.omit("EM (Enable2 omitted, e.g. EM,2)", "the command is ignored and nothing is sent (D9)")


def test_qm_qs_cs(ebb, r):
    r.item = "QM"
    r.check("idle: QM,0,0,0,0<NL><CR> (no OK)", ebb.raw("QM") == b"QM,0,0,0,0\n\r")
    ebb.ok("SM,1000,300,0")
    ebb.ok("SM,1000,0,0")
    time.sleep(0.1)
    r.check("axis 1 moving with a command queued: QM,1,1,0,1", ebb.raw("QM") == b"QM,1,1,0,1\n\r")
    ebb.wait_idle()
    r.omit("QM (motor status during a delay)", "a delay (SM with no steps) reports both motors as moving (D57)")
    ebb.raw("CU,10,1")
    resp = ebb.raw("QM")
    ebb.raw("CU,10,0")
    r.check("future format: QM,0,0,0,0<NL>", resp == b"QM,0,0,0,0\n", resp)
    r.item = "QS"
    ebb.ok("CS")
    ebb.ok("SM,200,123,-45")
    ebb.wait_idle()
    r.check("legacy format: 123,-45<NL><CR>OK<CR><NL>", ebb.raw("QS") == b"123,-45\n\rOK\r\n")
    ebb.raw("CU,10,1")
    resp = ebb.raw("QS")
    ebb.raw("CU,10,0")
    r.check("future format: QS,123,-45<NL>", resp == b"QS,123,-45\n", resp)
    r.item = "CS"
    r.check("CS replies OK", ebb.raw("CS") == b"OK\r\n")
    r.check("CS zeroes the positions", qs(ebb) == (0, 0))
    r.check("CS zeroes both accumulators", accumulators(ebb) == (0, 0))


def test_sc_drivers(ebb, r):
    r.item = "SC,2"
    ebb.ok("SC,2,1")
    ebb.ok("EM,1,1")
    r.check("SC,2,1: EM and QE use the external driver enable pins", ebb.raw("QE") == b"16,16\n\rOK\r\n")
    r.check("  ...enable pins RD1 and RA1 driven low (enabled)",
            not ebb.mr(3980) & 0x02 and not ebb.mr(3977) & 0x02)
    ebb.ok("EM,0,0")
    r.check("  ...EM,0,0 disables them", ebb.raw("QE") == b"0,0\n\rOK\r\n")
    ebb.ok("SC,2,2")
    r.check("SC,2,2: on-board step/dir (RD4-RD7) and enable (RE0, RC1) pins are inputs",
            (ebb.mr(TRISD) & 0xF0) == 0xF0 and ebb.mr(TRISE) & 0x01 and ebb.mr(TRISC) & 0x02)
    ebb.ok("SC,2,0")
    r.check("SC,2,0: they are outputs again",
            (ebb.mr(TRISD) & 0xF0) == 0 and not ebb.mr(TRISE) & 0x01 and not ebb.mr(TRISC) & 0x02)
    r.omit("SC,2,1 (automatic motor enable)", "auto enable drives the on-board pins in v3.1.7 (#245, fixed in v3.1.10)")


if __name__ == "__main__":
    u.run("EBB motion commands and queries",
          [test_sm, test_xm, test_hm, test_lm_l3, test_lt_t3_td, test_tr, test_es,
           test_em_qe, test_qm_qs_cs, test_sc_drivers])
