#!/usr/bin/env python
# -*- encoding: utf-8 -#-

'''
test_cmd_pen_servo.py

EBB regression tests: pen, servo and engraver commands. SP, TP, QP, S2, SE,
SR, QR, and the SC pen/servo parameters (SC,1, 4, 5, 8, 9, 10, 11, 12, 13).
Run against an EBB with nothing attached except USB:

    python3 test_cmd_pen_servo.py [port] [--transcript FILE]

Servo positions, targets and rates are read from the RC servo channel table
in RAM (MR), which has been at the same address since v3.0. Pulse outputs are
checked by sampling the pin (MR of PORTB) and measuring how often it is high.
Engraver power is read from the PWM duty registers.

Documented behavior that current firmware does not follow is listed as
omitted, with the reason, rather than tested.
'''

import time

import ebb_test_util as u

PORTB, LATA, LATB, TRISB = 3969, 3977, 3978, 3987
CCP1CON, CCPR1L = 4026, 4027

# RC servo channel table (RCServo2.c, "RC2Servo_globals" at 0x060)
CHANNELS = 8
VALUE, RPN, TARGET, RATE = 0x61, 0x71, 0x79, 0x89
SLOTS, SLOT_MS = 0x9A, 0x9B
SERVO_MAX, SERVO_MIN, RATE_UP, RATE_DOWN, PEN_RPN = 0x9E, 0xA0, 0xA2, 0xA4, 0xA6

UP, DOWN = 22565, 15302         # documented Servo_Min, Servo_Max at boot
PEN = 4                         # RP4 = RB1, the pen servo output


def mr16(ebb, address):
    return ebb.mr(address) | (ebb.mr(address + 1) << 8)


def channel(ebb, rpn):
    '''(value, target, rate) of the servo channel on RPn, or None.'''
    for i in range(CHANNELS):
        if ebb.mr(RPN + i) == rpn:
            return (mr16(ebb, VALUE + 2 * i), mr16(ebb, TARGET + 2 * i), mr16(ebb, RATE + 2 * i))
    return None


def target(ebb, rpn=PEN):
    ch = channel(ebb, rpn)
    return ch and ch[1]


def query(ebb, cmd):
    '''Value of a legacy-format query reply "N<NL><CR>OK<CR><NL>".'''
    resp = ebb.raw(cmd)
    return int(resp[:-len(b"\n\rOK\r\n")]) if resp.endswith(b"\n\rOK\r\n") else resp


def bit(ebb, address, n):
    return (ebb.mr(address) >> n) & 1


def high_fraction(ebb, pin, samples=400):
    '''Fraction of samples in which PORTB pin reads high.'''
    resp = ebb.raw(f"MR,{PORTB}\r".encode() * samples, timeout=10, record=False, quiet=0.2)
    values = [int(line.split(b",")[1]) for line in resp.split(b"\r\n") if line.startswith(b"MR,")]
    if len(values) != samples:
        raise RuntimeError(f"{len(values)} of {samples} PORTB samples")
    return sum((v >> pin) & 1 for v in values) / samples


def busy_for(ebb, ms):
    '''Queue a delay so that queued commands wait behind it.'''
    ebb.ok(f"SM,{ms},0,0")


def timed_idle(ebb, cmd):
    '''Send cmd and return the seconds until the queue is idle.'''
    t0 = time.monotonic()
    ebb.ok(cmd)
    ebb.wait_idle()
    return time.monotonic() - t0


def future(ebb, cmd):
    ebb.raw("CU,10,1")
    resp = ebb.raw(cmd)
    ebb.raw("CU,10,0")
    return resp


def test_defaults(ebb, r):
    ebb.wait_idle()
    r.item = "SP"
    r.check("Servo_Min at boot is 22565", mr16(ebb, SERVO_MIN) == UP, mr16(ebb, SERVO_MIN))
    r.check("Servo_Max at boot is 15302", mr16(ebb, SERVO_MAX) == DOWN, mr16(ebb, SERVO_MAX))
    r.check("pen servo output on RB1 (RP4) at reset", ebb.mr(PEN_RPN) == PEN, ebb.mr(PEN_RPN))
    r.check("pen is up (Servo_Min) at reset", target(ebb) == UP, channel(ebb, PEN))
    r.item = "QP"
    r.check("QP 1 (up) at reset", query(ebb, "QP") == 1)
    r.item = "SC,8"
    r.check("8 RC servo channels at reset", ebb.mr(SLOTS) == 8, ebb.mr(SLOTS))
    r.item = "SC,9"
    r.check("3 ms per channel at reset", ebb.mr(SLOT_MS) == 3, ebb.mr(SLOT_MS))
    r.item = "SR"
    r.check("RC servo power off at reset", query(ebb, "QR") == 0)


def test_sp(ebb, r):
    r.item = "SP"
    r.check("SP,0 replies OK", ebb.raw("SP,0") == b"OK\r\n")
    ebb.wait_idle()
    r.check("SP,0 moves to Servo_Max (pen down)", target(ebb) == DOWN, channel(ebb, PEN))
    r.check("  ...turns RB4 on", bit(ebb, LATB, 4) == 1)
    r.check("  ...and turns servo power on", query(ebb, "QR") == 1)
    ebb.ok("SP,1")
    ebb.wait_idle()
    r.check("SP,1 moves to Servo_Min (pen up)", target(ebb) == UP, channel(ebb, PEN))
    r.check("  ...turns RB4 off", bit(ebb, LATB, 4) == 0)

    busy_for(ebb, 600)
    ebb.ok("SP,0")
    r.check("SP,0 is queued: no change while an earlier move runs", target(ebb) == UP)
    r.item = "QP"
    r.check("QP reports the old state until the queued move starts", query(ebb, "QP") == 1)
    ebb.wait_idle()
    r.check("  ...and the new state after", query(ebb, "QP") == 0)
    r.item = "SP"
    r.check("the queued SP,0 then runs", target(ebb) == DOWN)

    dt = timed_idle(ebb, "SP,1,500")
    r.check("Duration 500: the next command waits 500 ms", 0.45 < dt < 0.8, f"{dt:.2f} s")
    dt = timed_idle(ebb, "SP,0")
    r.check("no Duration: no added delay", dt < 0.25, f"{dt:.2f} s")

    busy_for(ebb, 800)
    ebb.ok("SP,2,2000")
    r.check("SP,2 moves to Servo_Min at once, bypassing the queue", target(ebb) == UP)
    r.item = "QP"
    r.check("QP 1 right after SP,2", query(ebb, "QP") == 1)
    r.item = "SP"
    ebb.wait_idle()
    dt = timed_idle(ebb, "SP,2,2000")
    r.check("SP,2 ignores Duration", dt < 0.25, f"{dt:.2f} s")

    ebb.ok("SP,0")
    ebb.wait_idle()
    busy_for(ebb, 600)
    ebb.ok("SP,0")
    ebb.ok("SP,1")
    ebb.ok("SP,0")
    ebb.ok("SP,3")
    r.check("SP,3 moves to Servo_Min at once", target(ebb) == UP)
    r.check("SP,3 sets Servo_Max equal to Servo_Min", mr16(ebb, SERVO_MAX) == UP, mr16(ebb, SERVO_MAX))
    ebb.wait_idle()
    r.check("  ...queued pen-down moves do not lower the pen", target(ebb) == UP and query(ebb, "QP") == 1)
    ebb.ok("SP,0")
    ebb.wait_idle()
    r.check("  ...nor does a later SP,0", target(ebb) == UP)

    ebb.reset()
    ebb.wait_idle()
    ebb.ok("SP,0,0,2")
    ebb.wait_idle()
    r.check("PortB_Pin 2: the pen servo moves to RB2 (RP5)",
            ebb.mr(PEN_RPN) == 5 and target(ebb, 5) == DOWN, channel(ebb, 5))
    r.check("  ...RB2 is an output", bit(ebb, TRISB, 2) == 0)
    r.check("  ...and RB1 stops", channel(ebb, PEN) is None)
    ebb.ok("SP,1")
    ebb.wait_idle()
    r.check("PortB_Pin omitted: pin 1 (RB1) is used",
            ebb.mr(PEN_RPN) == PEN and target(ebb) == UP and channel(ebb, 5) is None)

    r.check("example SP,1 replies OK", ebb.raw("SP,1") == b"OK\r\n")
    r.check("future format: SP<NL>", future(ebb, "SP,1") == b"SP\n")
    ebb.wait_idle()
    r.omit("SP (Value 4 and above, PortB_Pin above 7)",
           "SP,4 acts as SP,3 and PortB_Pin above 7 becomes 1 instead of an error (D21)")


def test_tp_qp(ebb, r):
    ebb.wait_idle()
    r.item = "TP"
    r.check("TP replies OK", ebb.raw("TP") == b"OK\r\n")
    ebb.wait_idle()
    r.check("TP from up: pen down (Servo_Max)", target(ebb) == DOWN and query(ebb, "QP") == 0)
    ebb.ok("TP")
    ebb.wait_idle()
    r.check("TP from down: pen up (Servo_Min)", target(ebb) == UP and query(ebb, "QP") == 1)
    busy_for(ebb, 500)
    ebb.ok("TP")
    r.check("TP is queued", target(ebb) == UP)
    ebb.wait_idle()
    r.check("  ...and runs after the earlier move", target(ebb) == DOWN)
    dt = timed_idle(ebb, "TP,400")
    r.check("Duration 400: the next command waits 400 ms", 0.35 < dt < 0.7, f"{dt:.2f} s")
    r.check("future format: TP<NL>", future(ebb, "TP") == b"TP\n")
    ebb.wait_idle()
    r.omit("TP (two TPs queued behind a move)",
           "the direction is chosen when TP is received, so both can go the same way (D19)")

    r.item = "QP"
    ebb.ok("SP,1")
    ebb.wait_idle()
    r.check("legacy format, up: 1<NL><CR>OK<CR><NL>", ebb.raw("QP") == b"1\n\rOK\r\n")
    r.check("future format, up: QP,1<NL>", future(ebb, "QP") == b"QP,1\n")
    ebb.ok("SP,0")
    ebb.wait_idle()
    r.check("legacy format, down: 0<NL><CR>OK<CR><NL>", ebb.raw("QP") == b"0\n\rOK\r\n")
    r.check("future format, down: QP,0<NL>", future(ebb, "QP") == b"QP,0\n")


def test_s2(ebb, r):
    r.item = "S2"
    ebb.wait_idle()
    r.check("example S2,24000,6 replies OK", ebb.raw("S2,24000,6") == b"OK\r\n")
    ebb.wait_idle()
    r.check("  ...RP6 (RB3) gets a servo channel at 24000", channel(ebb, 6) == (24000, 24000, 0), channel(ebb, 6))
    r.check("  ...RB3 is made an output", bit(ebb, TRISB, 3) == 0)
    ebb.ok("S2,0,6")
    r.check("S2,0,6 turns it off", channel(ebb, 6) is None)

    ebb.ok("S2,12000,5")
    ebb.wait_idle()
    f = high_fraction(ebb, 2)
    r.check("1 ms pulses on RB2 every 24 ms (8 channels x 3 ms)", 0.025 < f < 0.06, f"{f:.3f}")
    busy_for(ebb, 800)
    ebb.ok("S2,0,5")
    r.check("S2,0,5 runs at once, not queued", channel(ebb, 5) is None)
    f = high_fraction(ebb, 2)
    r.check("  ...and the pulses stop", f == 0, f"{f:.3f}")
    ebb.wait_idle()

    busy_for(ebb, 600)
    ebb.ok("S2,20000,5")
    r.check("S2 with a position is queued", channel(ebb, 5) is None)
    ebb.wait_idle()
    r.check("  ...and runs after the earlier move", channel(ebb, 5) == (20000, 20000, 0), channel(ebb, 5))

    ebb.ok("S2,10000,5")
    ebb.wait_idle()
    ebb.ok("S2,27500,5,1000")
    time.sleep(0.12)
    ch = channel(ebb, 5)
    r.check("Rate 1000: the position slews towards the target",
            ch and 10000 < ch[0] < 27500 and ch[1:] == (27500, 1000), ch)
    time.sleep(0.6)
    r.check("  ...and reaches it (1000 per 24 ms)", channel(ebb, 5) == (27500, 27500, 1000), channel(ebb, 5))
    dt = timed_idle(ebb, "S2,10000,5,0,500")
    r.check("Delay 500: the next command waits 500 ms", 0.45 < dt < 0.8, f"{dt:.2f} s")
    dt = timed_idle(ebb, "S2,12000,5,50")
    r.check("Rate does not delay the next command", dt < 0.25, f"{dt:.2f} s")
    dt = timed_idle(ebb, "S2,10000,5,0,100")
    r.check("doc example S2,10000,5,0,100 waits 100 ms", 0.08 < dt < 0.35, f"{dt:.2f} s")
    ebb.ok("S2,27500,5,50,10")
    ebb.wait_idle()
    ch = channel(ebb, 5)
    r.check("doc example S2,27500,5,50,10 starts moving at rate 50", ch and ch[1:] == (27500, 50), ch)
    ebb.ok("S2,0,5")

    r.check("future format: S2<NL>", future(ebb, "S2,0,5") == b"S2\n")
    ebb.ok("S2,0,4")
    r.check("S2 can turn off the pen servo channel", channel(ebb, PEN) is None)
    r.omit("S2 (no parameters)", "bare S2 acts as S2,0,0 and replies OK (D22a)")
    r.omit("S2 (future mode errors)", "the error is printed before the S2 echo (D22b)")
    r.omit("S2 (all channels in use)", "a further S2 is dropped and still replies OK (D22c)")
    # S2 on RP6 (RB3) leaves SE without output on RB3 until a reboot (D59)
    r.check("reboot after S2 on RB3", ebb.reboot())


def test_sc_slots(ebb, r):
    ebb.wait_idle()
    ebb.ok("S2,12000,5")
    ebb.wait_idle()
    r.item = "SC,8"
    ebb.ok("SC,8,2")
    f = high_fraction(ebb, 2)
    r.check("SC,8,2: two channels, 1 ms pulses every 6 ms", 0.12 < f < 0.21, f"{f:.3f}")
    r.check("SC,8 replies OK", ebb.raw("SC,8,8") == b"OK\r\n")
    r.item = "SC,9"
    ebb.ok("SC,9,1")
    f = high_fraction(ebb, 2)
    r.check("SC,9,1: 1 ms per channel, pulses every 8 ms", 0.09 < f < 0.16, f"{f:.3f}")
    ebb.ok("SC,9,6")
    f = high_fraction(ebb, 2)
    r.check("SC,9,6: 6 ms per channel, pulses every 48 ms", 0.01 < f < 0.035, f"{f:.3f}")
    ebb.ok("S2,0,5")


def test_sc_pen(ebb, r):
    ebb.wait_idle()
    r.item = "SC,4"
    r.check("example SC,4,8000 replies OK", ebb.raw("SC,4,8000") == b"OK\r\n")
    ebb.ok("SP,1")
    ebb.wait_idle()
    r.check("  ...pen up now goes to 8000", target(ebb) == 8000, channel(ebb, PEN))
    for v in (1, 65535):
        ebb.ok(f"SC,4,{v}")
        r.check(f"Servo_Min {v} is accepted", mr16(ebb, SERVO_MIN) == v)
    r.item = "SC,5"
    ebb.ok("SC,5,20000")
    ebb.ok("SP,0")
    ebb.wait_idle()
    r.check("SC,5,20000: pen down goes to 20000", target(ebb) == 20000, channel(ebb, PEN))
    for v in (1, 65535):
        ebb.ok(f"SC,5,{v}")
        r.check(f"Servo_Max {v} is accepted", mr16(ebb, SERVO_MAX) == v)

    ebb.reset()
    ebb.wait_idle()
    r.item = "SC,10"
    ebb.ok("SC,10,1000")
    ebb.ok("SP,0")
    time.sleep(0.1)
    ch = channel(ebb, PEN)
    r.check("SC,10,1000: pen down slews at 1000 per 24 ms",
            ch and DOWN < ch[0] < UP and ch[2] == 1000, ch)
    ebb.ok("SP,1")
    ebb.wait_idle()
    r.check("  ...and so does pen up", channel(ebb, PEN)[2] == 1000, channel(ebb, PEN))
    r.item = "SC,11"
    ebb.ok("SC,11,200")
    ebb.ok("SP,0")
    ebb.ok("SP,1")
    ebb.wait_idle()
    r.check("SC,11,200 sets the up rate", channel(ebb, PEN)[2] == 200, channel(ebb, PEN))
    r.item = "SC,12"
    ebb.ok("SC,12,300")
    ebb.ok("SP,0")
    ebb.wait_idle()
    r.check("SC,12,300 sets the down rate", channel(ebb, PEN)[2] == 300, channel(ebb, PEN))
    ebb.ok("SP,1")
    ebb.wait_idle()
    r.check("  ...and leaves the up rate", channel(ebb, PEN)[2] == 200, channel(ebb, PEN))

    ebb.reset()
    ebb.wait_idle()
    r.item = "SC,1"
    r.check("example SC,1,1 replies OK", ebb.raw("SC,1,1") == b"OK\r\n")
    ebb.ok("SP,0")
    ebb.wait_idle()
    r.check("SC,1,1: SP,0 moves the servo", target(ebb) == DOWN)
    r.check("  ...and leaves RB4 off", bit(ebb, LATB, 4) == 0)
    ebb.ok("SP,1")
    ebb.ok("SC,1,0")
    ebb.ok("SP,0")
    ebb.wait_idle()
    r.check("SC,1,0: SP,0 turns RB4 on", bit(ebb, LATB, 4) == 1)
    r.check("  ...with no servo output on RB1", channel(ebb, PEN) is None and high_fraction(ebb, 1) == 0)
    ebb.ok("SP,1")
    ebb.wait_idle()
    r.check("  ...SP,1 turns RB4 off", bit(ebb, LATB, 4) == 0)
    ebb.ok("SC,1,2")
    ebb.ok("SP,0")
    ebb.wait_idle()
    r.check("SC,1,2: SP,0 turns RB4 on and moves the servo",
            bit(ebb, LATB, 4) == 1 and target(ebb) == DOWN)
    r.omit("SC,1,0 (S2 outputs)", "SC,1,0 stops all S2 outputs from updating, not only the pen (D22d)")

    ebb.reset()
    r.item = "SC,13"
    press_b0 = lambda: [ebb.ok(c) for c in ("PD,B,0,0", "PO,B,0,0", "PO,B,0,1", "PD,B,0,1")]
    ebb.raw("QB")
    ebb.ok("SC,13,0")
    press_b0()
    r.check("SC,13,0: B0 taken low is not a button press", ebb.raw("QB") == b"0\r\nOK\r\n")
    ebb.ok("SC,13,1")
    press_b0()
    r.check("SC,13,1: B0 taken low is a button press", ebb.raw("QB") == b"1\r\nOK\r\n")
    r.check("future format: SC<NL>", future(ebb, "SC,13,1") == b"SC\n")


def engraver_power(ebb):
    return (ebb.mr(CCPR1L) << 2) | ((ebb.mr(CCP1CON) >> 4) & 3)


def engraver_on(ebb):
    '''RB3 on: power 1023 is a 1023/1024 duty cycle, so a sample can be low.'''
    return engraver_power(ebb) == 1023 and high_fraction(ebb, 3) > 0.95


def test_se(ebb, r):
    r.item = "SE"
    r.check("example SE,1,1023 replies OK", ebb.raw("SE,1,1023") == b"OK\r\n")
    r.check("  ...RB3 on at power 1023", engraver_on(ebb))
    r.check("example SE,0 turns it off", ebb.raw("SE,0") == b"OK\r\n" and high_fraction(ebb, 3) == 0)
    ebb.ok("SE,1")
    r.check("Power omitted: 512 (50%)", engraver_power(ebb) == 512, engraver_power(ebb))
    ebb.ok("SE,1,301")
    r.check("Power 301", engraver_power(ebb) == 301, engraver_power(ebb))
    ebb.ok("SE,1,0")
    r.check("Power 0 is always off", high_fraction(ebb, 3) == 0)
    ebb.ok("SE,0,500")
    r.check("State 0 with a Power: off", high_fraction(ebb, 3) == 0)

    busy_for(ebb, 800)
    ebb.ok("SE,1,1023,0")
    r.check("Use_Motion_Queue 0: on at once", engraver_on(ebb))
    ebb.wait_idle()
    ebb.ok("SE,0")
    busy_for(ebb, 800)
    ebb.ok("SE,1,601,1")
    r.check("Use_Motion_Queue 1: no change while an earlier move runs", engraver_power(ebb) == 0)
    ebb.wait_idle()
    r.check("  ...power 601 once it runs", engraver_power(ebb) == 601, engraver_power(ebb))
    busy_for(ebb, 600)
    ebb.ok("SE,0,0,1")
    r.check("example SE,0,0,1 is queued", engraver_power(ebb) == 601)
    ebb.wait_idle()
    r.check("  ...and turns the engraver off", engraver_power(ebb) == 0 and high_fraction(ebb, 3) == 0)
    r.check("future format: SE<NL>", future(ebb, "SE,0") == b"SE\n")
    r.omit("SE (after S2 on RB3)", "SE has no output on RB3 after S2 used RP6, until a reboot (D59)")
    r.omit("SE (queued, different Power)",
           "a queued SE takes the two low Power bits from the most recent SE (D20)")


def test_sr_qr(ebb, r):
    r.item = "QR"
    r.check("legacy format, off: 0<NL><CR>OK<CR><NL>", ebb.raw("QR") == b"0\n\rOK\r\n")
    r.check("future format, off: QR,0<NL>", future(ebb, "QR") == b"QR,0\n")
    r.item = "SR"
    r.check("example SR,60000,1 replies OK", ebb.raw("SR,60000,1") == b"OK\r\n")
    r.check("  ...turns servo power on (RA3)", bit(ebb, LATA, 3) == 1)
    r.item = "QR"
    r.check("legacy format, on: 1<NL><CR>OK<CR><NL>", ebb.raw("QR") == b"1\n\rOK\r\n")
    r.check("future format, on: QR,1<NL>", future(ebb, "QR") == b"QR,1\n")
    r.item = "SR"
    ebb.ok("SR,60000,0")
    r.check("State 0 turns it off", query(ebb, "QR") == 0 and bit(ebb, LATA, 3) == 0)
    ebb.ok("SR,60000")
    r.check("State omitted: power unchanged", query(ebb, "QR") == 0)

    ebb.ok("SR,500")
    ebb.wait_idle()
    ebb.ok("SP,0")
    r.check("a pen move turns servo power on", query(ebb, "QR") == 1)
    time.sleep(0.3)
    r.check("  ...still on before the timeout", query(ebb, "QR") == 1)
    time.sleep(0.6)
    r.check("  ...off 500 ms after the last servo move", query(ebb, "QR") == 0)
    ebb.ok("TP")
    r.check("TP turns servo power on", query(ebb, "QR") == 1)
    ebb.ok("SR,500,0")
    ebb.ok("S2,20000,4")
    r.check("S2 on RB1 turns servo power on", query(ebb, "QR") == 1)
    ebb.ok("SR,0")
    ebb.ok("SP,1")
    time.sleep(1.0)
    r.check("Value 0: power is never turned off", query(ebb, "QR") == 1)
    r.check("future format: SR<NL>", future(ebb, "SR,60000") == b"SR\n")
    r.omit("SR (60 s default timeout)", "takes 60 s to observe")


if __name__ == "__main__":
    u.run("EBB pen, servo and engraver commands",
          [test_defaults, test_sp, test_tp_qp, test_s2, test_sc_slots, test_sc_pen, test_se, test_sr_qr])
