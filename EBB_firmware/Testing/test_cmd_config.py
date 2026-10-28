#!/usr/bin/env python
# -*- encoding: utf-8 -#-

'''
test_cmd_config.py

EBB regression tests: configuration and query commands. Every CU and QU
parameter, QG, QB, SL/QL, SN/QN/NI/ND, MR/MW, ST/QT and CK. Run against an
EBB with nothing attached except USB:

    python3 test_cmd_config.py [port] [--transcript FILE] [--write-flash]

--write-flash also tests ST, which writes the nickname to flash (and then
restores it). The ST docs advise against repeated automated nickname
changes, so this is off by default.

Documented behavior that current firmware does not follow is listed as
omitted, with the reason, rather than tested.
'''

import re
import time

import ebb_test_util as u

TRISA, TRISC, TRISD = 3986, 3988, 3989     # SFR addresses for MR / MW
LATD, RCSTA1 = 3980, 4012


def qu(ebb, n):
    '''QU,n value (legacy mode), or None if the reply is malformed.'''
    m = re.fullmatch(rb"QU,([0-9A-Fa-f,]+)\r\nOK\r\n", ebb.raw(f"QU,{n}"))
    return m.group(1).decode() if m else None


def qg(ebb):
    m = re.fullmatch(rb"([0-9A-F]{2})\r\n", ebb.raw("QG"))
    return int(m.group(1), 16) if m else None


def qg_bits(ebb, mask):
    '''QG bits under mask, or None if the QG reply is malformed.'''
    v = qg(ebb)
    return None if v is None else v & mask


def b0_level(ebb):
    return int(ebb.raw("PI,B,0").split(b",")[-1])


def test_qu(ebb, r):
    r.item = "QU,2"
    v = qu(ebb, 2)
    r.check("maximum FIFO length, decimal", v is not None and v.isdigit() and int(v) >= 1, v)
    r.item = "QU,3"
    r.check("FIFO length is 1 at reset", qu(ebb, 3) == "1")
    for n in (4, 5):
        r.item = f"QU,{n}"
        v = qu(ebb, n)
        r.check("stack high water, three hex digits (stack starts at 0xE00)",
                v is not None and re.fullmatch(r"E[0-9A-F]{2}", v), v)
    r.item = "QU,6"
    r.check("no commands waiting when idle", qu(ebb, 6) == "0")
    for n in (60, 61):
        r.item = f"QU,{n}"
        r.check("zero at reset", qu(ebb, n) == "0")
    r.item = "QU,200"
    ebb.ok("CS")
    r.check("accumulators 0,0 after CS", qu(ebb, 200) == "0,0")
    r.omit("QU (future mode)", "future-mode replies end with two newlines (D7)")


def test_cu_fifo(ebb, r):
    r.item = "CU,4"
    maximum = qu(ebb, 2)
    ebb.ok("CU,4,8")
    r.check("CU,4,8 then QU,3 is 8", qu(ebb, 3) == "8")
    ebb.ok("CU,4,1000")
    r.check("a size above the maximum sets the maximum (QU,2)", qu(ebb, 3) == maximum, maximum)
    ebb.ok("CU,4,4")
    t0 = time.monotonic()
    for _ in range(4):
        ebb.ok("SM,500,0,0")
    r.check("depth 4: four 0.5 s delays are all accepted at once", time.monotonic() - t0 < 0.3,
            f"{time.monotonic() - t0:.2f} s")
    r.item = "QU,6"
    v = qu(ebb, 6)
    r.check("commands waiting in the FIFO (delays queued behind the first)", v in ("2", "3"), v)
    ebb.wait_idle()
    r.check("no commands waiting after they finish", qu(ebb, 6) == "0")
    r.omit("CU,4 (values below 1)", "CU,4,0 is accepted and stops all motion (fixed in v3.2.0)")


def test_cu_limits(ebb, r):
    r.item = "CU,2"
    ebb.ok("CS")
    resp = ebb.raw("SM,1,30")
    r.check("limit checks on: SM above 25,000 steps/s is rejected", resp.startswith(b"!0 Err"), resp)
    ebb.ok("CU,2,0")
    r.check("limit checks off: the same move is accepted", ebb.raw("SM,1,30") == b"OK\r\n")
    ebb.wait_idle()
    ebb.ok("CU,2,1")
    r.check("the accepted move ran (QS 30,0)", ebb.raw("QS").startswith(b"30,0"))


def test_cu_led(ebb, r):
    r.item = "CU,3"
    ebb.ok("CU,3,1")
    time.sleep(0.05)
    r.check("red LED (RD2) on while the FIFO is empty", ebb.mr(LATD) & 0x04)
    ebb.ok("SM,1000,0,0")
    ebb.ok("SM,1000,0,0")
    r.check("LED off while a command waits in the FIFO", not ebb.mr(LATD) & 0x04)
    ebb.wait_idle()
    time.sleep(0.05)
    r.check("LED on again when empty", ebb.mr(LATD) & 0x04)
    ebb.ok("CU,3,0")
    r.check("CU,3,0: LED off", not ebb.mr(LATD) & 0x04)


def test_cu_motors(ebb, r):
    r.item = "CU,50"
    ebb.ok("EM,0,0")
    ebb.ok("CU,50,0")
    ebb.ok("SM,20,5,5")
    ebb.wait_idle()
    r.check("CU,50,0: a move leaves the motors disabled", ebb.raw("QE") == b"0,0\n\rOK\r\n")
    ebb.ok("CU,50,1")
    ebb.ok("SM,20,5,5")
    ebb.wait_idle()
    resp = ebb.raw("QE")
    r.check("CU,50,1: a move enables the motors", resp.startswith(b"16,16"), resp)
    r.item = "CU,61"
    ebb.ok("CU,61,1")
    r.check("QU,61 reads back 1", qu(ebb, 61) == "1")
    ebb.ok("SM,20,5,5")
    ebb.wait_idle()
    time.sleep(2.0)
    r.check("motors disabled 1 s after the last move", ebb.raw("QE") == b"0,0\n\rOK\r\n")
    ebb.ok("CU,61,0")
    r.check("CU,61,0: QU,61 is 0", qu(ebb, 61) == "0")
    r.omit("CU,61 (values above 32767)", "documented range is 0 to 65534; values above 32767 return !6 (D12)")


def test_cu_limit_switch(ebb, r):
    r.item = "CU,51"
    level = b0_level(ebb)
    ebb.ok("CS")
    ebb.ok(f"CU,52,{level}")
    ebb.ok("CU,53,1")
    resp = ebb.raw("CU,51,1", quiet=0.2)
    r.item = "CU,53"
    m = re.search(rb"Limit switch triggered\. PortB=([0-9A-F]{2})\r\n", resp)
    # The reply is sent after a command; it may come after CU,51,1 or the next command
    if not m:
        resp = ebb.raw("V", quiet=0.2)
        m = re.search(rb"Limit switch triggered\. PortB=([0-9A-F]{2})\r\n", resp)
    r.check("Limit Switch Reply packet", m, resp)
    r.item = "QU,1"
    v = qu(ebb, 1)
    r.check("captured PortB matches the reply", m and v == m.group(1).decode(), (v, m and m.group(1)))
    r.check("captured PortB has B0 at the target level", v and (int(v, 16) & 1) == level, v)
    r.item = "QG"
    r.check("QG bit 7 set", qg_bits(ebb, 0x80) == 0x80)
    r.check("QG bit 7 still set after reading QG", qg_bits(ebb, 0x80) == 0x80)
    r.item = "CU,51"
    ebb.ok("SM,100,50,50")
    time.sleep(0.2)
    r.check("Limit Switch Stop: motion command acknowledged but ignored",
            ebb.raw("QS").startswith(b"0,0"))
    ebb.ok("CU,51,0")
    r.check("CU,51,0 clears it (QG bit 7)", qg_bits(ebb, 0x80) == 0)
    ebb.ok("SM,100,50,50")
    ebb.wait_idle()
    r.check("moves run again after CU,51,0", ebb.raw("QS").startswith(b"50,50"))
    ebb.ok("CU,53,0")
    r.item = "CU,52"
    ebb.ok(f"CU,52,{1 - level}")
    ebb.ok("CU,51,1")
    time.sleep(0.05)
    r.check("no trip when B0 does not match the target", qg_bits(ebb, 0x80) == 0)
    ebb.ok("CU,51,0")
    r.omit("CU,51 (EM clears a Limit Switch Stop)", "EM does not clear it (D55)")


def test_cu_power(ebb, r):
    r.item = "CU,60"
    ebb.ok("CU,60,1000")
    r.check("QU,60 reads back 1000", qu(ebb, 60) == "1000")
    time.sleep(0.05)
    r.item = "QG"
    r.check("QG bit 6 (power lost) set with V+ below the threshold", qg_bits(ebb, 0x40) == 0x40)
    ebb.ok("CU,60,0")
    time.sleep(0.05)
    first, second = qg_bits(ebb, 0x40), qg_bits(ebb, 0x40)
    r.check("after the threshold is cleared, bit 6 reads once, then is cleared by QG",
            first == 0x40 and second == 0, (first, second))


def test_qg_qb(ebb, r):
    r.item = "QG"
    resp = ebb.raw("QG")
    r.check("legacy format: two hex digits, <CR><NL>", re.fullmatch(rb"[0-9A-F]{2}\r\n", resp), resp)
    ebb.raw("CU,10,1")
    resp = ebb.raw("QG")
    ebb.raw("CU,10,0")
    r.check("future format: QG,XX<NL>", re.fullmatch(rb"QG,[0-9A-F]{2}\n", resp), resp)
    ebb.ok("SP,0")
    time.sleep(0.6)
    r.check("bit 4 (pen up) is 0 after SP,0", qg_bits(ebb, 0x10) == 0)
    ebb.ok("SP,1")
    time.sleep(0.6)
    r.check("bit 4 is 1 after SP,1", qg_bits(ebb, 0x10) == 0x10)
    ebb.ok("SM,1000,200,200")
    ebb.ok("SM,1000,0,0")
    v = qg_bits(ebb, 0x0F)
    r.check("during a move with another queued: CMD, MTR1, MTR2, FIFO set (bits 3-0)",
            v == 0x0F, v)
    qm = ebb.raw("QM")
    r.check("bits 3-0 equal the QM fields", qm.startswith(b"QM,1,1,1,1"), qm)
    ebb.wait_idle()
    r.check("bits 3-0 clear when idle", qg_bits(ebb, 0x0F) == 0)
    r.item = "QB"
    ebb.raw("QB")
    ebb.ok("PD,B,0,0")
    ebb.ok("PO,B,0,0")      # B0 low registers a PRG button press
    time.sleep(0.01)
    ebb.ok("PO,B,0,1")
    ebb.ok("PD,B,0,1")
    r.check("B0 taken low registers a press: QB 1", ebb.raw("QB") == b"1\r\nOK\r\n")
    r.check("QB 0 on the next query", ebb.raw("QB") == b"0\r\nOK\r\n")
    ebb.ok("PD,B,0,0")
    ebb.ok("PO,B,0,0")
    time.sleep(0.01)
    ebb.ok("PO,B,0,1")
    ebb.ok("PD,B,0,1")
    r.item = "QG"
    r.check("QG bit 5 (PRG pressed) set by a B0 press", qg_bits(ebb, 0x20) == 0x20)
    r.check("bit 5 cleared by reading QG", qg_bits(ebb, 0x20) == 0)


def test_debug_modes(ebb, r):
    r.item = "CU,255"
    ebb.ok("CU,255,1")
    resp = ebb.raw("T3,100,1000000,0,0,0,0,0")
    ebb.ok("CU,255,0")
    r.check("prints the move values", resp.startswith(b"R1=1000000 S1=100 A1=0 J1=0 R2=0 S2=0"), resp)
    ebb.wait_idle()
    r.item = "CU,256"
    ebb.ok("CS")
    ebb.ok("CU,256,1")
    ebb.ok("SM,100,50,50")
    time.sleep(0.2)
    r.check("moves are parsed but not queued", ebb.raw("QS").startswith(b"0,0"))
    ebb.ok("CU,256,0")
    r.item = "CU,257"
    ebb.ok("CU,257,1")
    r.check("RC0 is an output", not ebb.mr(TRISC) & 0x01)
    ebb.ok("CU,257,0")
    r.check("CU,257,0: RC0 is an input", ebb.mr(TRISC) & 0x01)
    r.item = "CU,250"
    trisa, trisd = ebb.mr(TRISA), ebb.mr(TRISD)
    ebb.ok("CU,250,1")
    r.check("debug pins RD0, RD1, RA1, RA3 are outputs",
            not ebb.mr(TRISD) & 0x03 and not ebb.mr(TRISA) & 0x0A)
    ebb.ok("CU,250,0")
    ebb.ok(f"MW,{TRISA},{trisa}")
    ebb.ok(f"MW,{TRISD},{trisd}")
    for n in (251, 252, 253):
        r.item = f"CU,{n}"
        ebb.ok(f"CU,{n},1")
        r.check("debug UART enabled (RCSTA1 SPEN)", ebb.mr(RCSTA1) & 0x80)
        ebb.ok(f"CU,{n},0")
    r.omit("CU,254", "lock up mode needs a power cycle to exit")


def test_sl_ql(ebb, r):
    r.item = "SL"
    r.check("SL,4 (doc example)", ebb.raw("SL,4") == b"OK\r\n")
    r.check("SL,125,19 (doc example)", ebb.raw("SL,125,19") == b"OK\r\n")
    r.item = "QL"
    r.check("QL reads variable 0", ebb.raw("QL") == b"4\r\nOK\r\n")
    r.check("QL,19 reads variable 19", ebb.raw("QL,19") == b"125\r\nOK\r\n")
    ebb.ok("SL,255,31")
    r.check("QL,31 (index 31, value 255)", ebb.raw("QL,31") == b"255\r\nOK\r\n")
    ebb.raw("CU,10,1")
    resp = ebb.raw("QL,19")
    ebb.raw("CU,10,0")
    r.check("future format: QL,125<NL>", resp == b"QL,125\n", resp)
    ebb.reset()
    r.check("values are cleared to 0 by R", ebb.raw("QL,19") == b"0\r\nOK\r\n")


def test_node_count(ebb, r):
    r.item = "SN"
    r.check("SN,123456789 (doc example)", ebb.raw("SN,123456789") == b"OK\r\n")
    r.item = "QN"
    r.check("QN reads 123456789", ebb.raw("QN") == b"123456789\r\nOK\r\n")
    r.item = "NI"
    ebb.ok("NI")
    r.check("NI increments", ebb.raw("QN") == b"123456790\r\nOK\r\n")
    r.item = "ND"
    ebb.ok("ND")
    ebb.ok("ND")
    r.check("ND decrements", ebb.raw("QN") == b"123456788\r\nOK\r\n")
    r.item = "SN"
    ebb.ok("SN,4294967295")
    r.check("SN accepts the full 32 bit range", ebb.raw("QN") == b"4294967295\r\nOK\r\n")
    ebb.raw("CU,10,1")
    resp = ebb.raw("QN")
    ebb.raw("CU,10,0")
    r.item = "QN"
    r.check("future format: QN,4294967295<NL>", resp == b"QN,4294967295\n", resp)
    ebb.ok("SN,0")
    r.omit("SN (missing value)", "SN replies OK before reporting the error (D25)")


def test_mr_mw(ebb, r):
    # Find the SL variable storage in RAM: write a distinct pattern with SL,
    # then search for it with MR (not recorded: the address depends on the
    # build). Then MW one of those bytes and read it back with QL.
    pattern = [0xA0 + i for i in range(32)]
    for i, v in enumerate(pattern):
        ebb.ok(f"SL,{v},{i}")
    r.item = "MR"
    window = []
    found = None
    for address in range(0x060, 0x600):
        window.append(ebb.mr(address))
        if window[-32:] == pattern:
            found = address - 31
            break
    r.check("MR finds the SL variables written with SL", found is not None)
    if found is None:
        return
    resp = ebb.raw(f"MR,{found + 5}", record=False)
    r.check("legacy format: MR,Data<CR><NL> (decimal)", resp == b"MR,165\r\n", resp)
    r.item = "MW"
    r.check("MW accepted", ebb.raw(f"MW,{found + 5},42", record=False) == b"OK\r\n")
    r.check("MW wrote the byte (QL,5 is 42)", ebb.raw("QL,5") == b"42\r\nOK\r\n")
    r.check("MR reads the written byte", ebb.mr(found + 5) == 42)


def test_nickname(ebb, r):
    r.item = "QT"
    resp = ebb.raw("QT")
    r.check("legacy format: Name<CR><NL>OK<CR><NL>", resp.endswith(b"\r\nOK\r\n"), resp)
    original = resp[:-len(b"\r\nOK\r\n")].decode("ascii", "replace")
    if not getattr(r.args, "write_flash", False):
        r.omit("ST", "writes flash; run with --write-flash to test")
        return
    r.item = "ST"
    r.check("ST,Regression Test accepted", ebb.raw("ST,Regression Test") == b"OK\r\n")
    r.item = "QT"
    r.check("QT reads the new name", ebb.raw("QT") == b"Regression Test\r\nOK\r\n")
    ebb.raw("CU,10,1")
    resp = ebb.raw("QT")
    ebb.raw("CU,10,0")
    r.check("future format: QT,Name<NL>", resp == b"QT,Regression Test\n", resp)
    r.item = "ST"
    r.check(f"ST restores the original name {original!r}", ebb.raw(f"ST,{original}") == b"OK\r\n")
    r.check("QT reads the original name", ebb.raw("QT") == (original + "\r\nOK\r\n").encode())
    r.omit("ST (no parameter, or over 16 characters)", "writes flash before reporting the error (D24)")


def test_ck(ebb, r):
    r.omit("CK", "parameter types are swapped within each pair, and extra lines are printed (D8)")


if __name__ == "__main__":
    u.run("EBB configuration and query commands",
          [test_qu, test_cu_fifo, test_cu_limits, test_cu_led, test_cu_motors,
           test_cu_limit_switch, test_cu_power, test_qg_qb, test_debug_modes,
           test_sl_ql, test_node_count, test_mr_mw, test_nickname, test_ck],
          options=[("--write-flash", "also test ST, which writes the nickname to flash")])
