#!/usr/bin/env python
# -*- encoding: utf-8 -#-

'''
test_cmd_general.py

EBB regression tests: general syntax rules, returned errors, response modes
(CU,1 and CU,10), command checksums (CU,54), and V. Run against an EBB with
nothing attached except USB:

    python3 test_cmd_general.py [port] [--transcript FILE]

Each check names the documented command or rule it covers (docs/ebb.html).
Documented behavior that current firmware does not follow is listed as
omitted, with the reason, rather than tested.
'''

import re

import ebb_test_util as u

VERSION = re.compile(rb"EBBv13_and_above EB Firmware Version (\d+)\.(\d+)\.(\d+)")


def legacy_and_future(ebb):
    '''Yield ("legacy", "") then ("future", "XX,"-style prefix flag),
    switching CU,10 and restoring legacy mode afterward.'''
    yield "legacy"
    ebb.raw("CU,10,1")
    try:
        yield "future"
    finally:
        ebb.raw("CU,10,0")
        ebb.resync()


def test_version(ebb, r):
    r.item = "V"
    resp = ebb.raw("V")
    r.check("legacy: version string, <CR><NL>", VERSION.fullmatch(resp.rstrip(b"\r\n"))
            and resp.endswith(b"\r\n") and not resp.endswith(b"\n\r"), resp)
    ebb.raw("CU,10,1")
    resp = ebb.raw("V")
    ebb.raw("CU,10,0")
    r.check("future: V, prefix and <NL>", resp.startswith(b"V,") and resp.endswith(b"\n")
            and VERSION.fullmatch(resp[2:-1]), resp)


def test_syntax(ebb, r):
    r.item = "Syntax conventions"
    version = ebb.raw("V")
    r.check("commands are case insensitive (v)", ebb.raw("v") == version, version)
    r.check("commands are case insensitive (cs)", ebb.raw("cs") == b"OK\r\n")
    for ending in (b"\n", b"\r\n", b"\n\r"):
        r.check(f"V terminated by {ending!r} gives one response", ebb.raw(b"V" + ending) == version)
    r.check("several line endings before a command are ignored", ebb.raw(b"\r\r\nV\r") == version)
    for cmd in (b"V \r", b" V\r", b"S L,3\r"):
        resp = ebb.raw(cmd)
        r.check(f"space in {cmd!r} is an error", resp.startswith(b"!8 Err: Unknown command"), resp)
    ebb.ok("SL,0,0")
    r.check("tab within a command is ignored", ebb.raw(b"S\tL,3\r") == b"OK\r\n")
    r.check("  ...and the command took effect", ebb.raw("QL,0").startswith(b"3\r\n"))
    resp = ebb.raw(b"XV\x08\x08V\r")
    r.check("backspace deletes a character and echoes ' \\b'",
            resp == b" \x08 \x08" + version, resp)
    resp = ebb.raw("SL,12345678901")
    r.check("numbers have at most 10 digits (11 digits: !6)",
            resp == b"!6 Err: Invalid parameter value\r\n", resp)
    r.check("10 digit number accepted", ebb.raw("SL,0000000005,0") == b"OK\r\n")
    r.omit("Syntax conventions (256 byte limit)",
           "longer commands are not reported (the overflow check can never fire, D45)")


def test_errors(ebb, r):
    cases = [
        ("!4", "SM,1", "SM", b"!4 Err: Missing parameter(s)"),
        ("!5", "SC,1X", "SC", b"!5 Err: Need comma next, found: 'X'"),
        ("!6", "CU,1000,1", "CU", b"!6 Err: Invalid parameter value"),
        ("!8", "Z", "Z", b"!8 Err: Unknown command 'Z:5A'"),
        ("!8", "ZZ", "ZZ", b"!8 Err: Unknown command 'ZZ:5A5A'"),
    ]
    for mode in legacy_and_future(ebb):
        for code, cmd, name, text in cases:
            r.item = f"Returned Errors {code}"
            resp = ebb.raw(cmd)
            want = text + b"\r\n" if mode == "legacy" else name.encode() + b"," + text + b"\n"
            r.check(f"{mode}: {cmd}", resp == want, f"{resp!r} (expected {want!r})")
    # !0: SM step rate above 25,000 steps/s. The text, and that the move is
    # not executed, are checked; the line ending is not (an extra blank line
    # follows the error, D2).
    r.item = "Returned Errors !0"
    ebb.ok("CS")
    for cmd, text in (("SM,1,26", b"!0 Err: <axis1> step rate too high"),
                      ("SM,1,0,26", b"!0 Err: <axis2> step rate too high"),
                      ("SM,4294967295,1", b"!0 Err: <axis1> step rate too slow"),
                      ("SM,4294967295,0,1", b"!0 Err: <axis2> step rate too slow")):
        resp = ebb.raw(cmd)
        r.check(f"{cmd}: '{text.decode()}'", resp.startswith(text), resp)
    ebb.wait_idle()
    r.check("rejected moves did not run (QS 0,0)", ebb.raw("QS").startswith(b"0,0"))
    r.check("SM,1,25 (25,000 steps/s) is accepted", ebb.raw("SM,1,25") == b"OK\r\n")
    ebb.wait_idle()
    r.omit("Returned Errors !0 (line ending)", "an extra line ending follows !0 errors (D2)")
    r.omit("Returned Errors !1", "only TR reports it, and only after OK (D17)")
    r.omit("Returned Errors !2", "internal TX buffer overrun; not reachable by command")
    r.omit("Returned Errors !3", "the RX overflow check can never fire (D45)")
    r.omit("Returned Errors !7", "commands with an extra parameter run, then report !7 (D3)")


def test_checksums(ebb, r):
    r.item = "CU,54"
    r.check("CU,54,1 accepted", ebb.raw("CU,54,1") == b"OK\r\n")
    version = None
    resp = ebb.raw("V")
    r.check("no checksum: !10", resp == b"!10 Err: No checksum found but required\r\n", resp)
    resp = ebb.raw("V,0")
    r.check("wrong checksum: !9 with the expected value",
            resp == b"!9 Err: Checksum incorrect, expected 170\r\n", resp)
    resp = ebb.raw("V,170")
    r.check("correct checksum (V,170) accepted", VERSION.match(resp), resp)
    body = "SL,7,2"
    cs = (0x100 - sum(body.encode())) & 0xFF
    r.check(f"command with parameters and checksum ({body},{cs})", ebb.raw(f"{body},{cs}") == b"OK\r\n")
    r.check("doc example CU,54,0,119 turns checksums off", ebb.raw("CU,54,0,119") == b"OK\r\n")
    r.check("checksums off: V accepted", VERSION.match(ebb.raw("V")))
    r.check("SL with checksum took effect (QL,2 = 7)", ebb.raw("QL,2").startswith(b"7\r\n"))
    ebb.raw("CU,10,1")
    ebb.raw("CU,54,1")
    resp = ebb.raw("V")
    r.check("future: !10", resp == b"V,!10 Err: No checksum found but required\n", resp)
    resp = ebb.raw("V,0")
    r.check("future: !9", resp == b"V,!9 Err: Checksum incorrect, expected 170\n", resp)
    ebb.raw("CU,54,0,119")
    ebb.raw("CU,10,0")


def test_response_modes(ebb, r):
    r.item = "CU,10"
    r.check("CU,10,1 from legacy mode replies <NL>", ebb.raw("CU,10,1") == b"\n")
    r.check("future: CS replies CS<NL>", ebb.raw("CS") == b"CS\n")
    r.check("future: QS replies QS,0,0<NL>", ebb.raw("QS") == b"QS,0,0\n")
    r.item = "CU,1"
    r.check("future: CU,1,0 has no effect", ebb.raw("CU,1,0") == b"CU\n" and ebb.raw("CS") == b"CS\n")
    ebb.raw("CU,1,1")
    r.item = "CU,10"
    r.check("CU,10,0 from future mode replies CUOK<CR><NL>", ebb.raw("CU,10,0") == b"CUOK\r\n")
    r.check("legacy: CS replies OK<CR><NL>", ebb.raw("CS") == b"OK\r\n")
    r.item = "CU,1"
    r.check("CU,1,0: no OK", b"OK" not in ebb.raw("CU,1,0"))
    resp = ebb.raw("CS")
    r.check("CU,1,0: CS response has no OK", b"OK" not in resp, resp)
    r.check("CU,1,1 restores OK", ebb.raw("CU,1,1") == b"OK\r\n" and ebb.raw("CS") == b"OK\r\n")


if __name__ == "__main__":
    u.run("EBB general syntax, errors, modes and checksums",
          [test_version, test_syntax, test_errors, test_checksums, test_response_modes])
