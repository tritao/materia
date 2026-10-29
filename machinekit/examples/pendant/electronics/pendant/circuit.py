"""Pendant board v1: the jog pendant's controller board (machinekit/PENDANT_PLAN.md, "Board v1 and the enclosure", EN4).

24 V from the M12 pigtail (fuse, TVS, reverse diode) feeds a 65 V buck to 5 V and an LDO to 3.3 V. An STM32H503RBT6 reads
the hand wheel (through a 5 V-tolerant Schmitt buffer), the axis and step selectors (two 74HC165 on SPI1), seven keys and
the E-stop loop (an isolated read-back), drives seven key LEDs (ULN2003 from the LED supply) and the OLED (I2C1), and
talks CAN FD through an isolated transceiver with its own isolated supply. The MCU's pins follow the board's pin table,
`pendant.pins.json`, which the firmware build reads too. Every part is a proposal: the parts review (DEC-PENDANT-PARTS)
and every later review are pending. Placement waits for the enclosure envelope (EN3).
"""

import json
from pathlib import Path
import sys

from skidl import POWER
from pcbkit import assets, blocks, enclosure, netlist, part_interfaces, pinassign
from pcbkit.blocks.parts import (
    C0603,
    C0805,
    C_100N,
    C_10U,
    D_SMA,
    D_SMB,
    FUSE1812,
    R0603,
    R_10K,
    R_1K,
    SOIC16,
    TP,
    nc,
)
from pcbkit.board import Board
from pcbkit.circuit_run import domains_from_ports, run_erc
from pcbkit.intent import Intent
from pcbkit import quote
from pcbkit.sourced import sourced

HERE = Path(__file__).resolve().parent
BOARD = "machinekit/examples/pendant/electronics/pendant/"

C1206 = "Capacitor_SMD:C_1206_3216Metric"
R1206 = "Resistor_SMD:R_1206_3216Metric"
SOT23 = "Package_TO_SOT_SMD:SOT-23"
SOT23_3 = "Package_TO_SOT_SMD:SOT-23-3"
SOT23_6 = "Package_TO_SOT_SMD:SOT-23-6"
LQFP64 = "Package_QFP:LQFP-64_10x10mm_P0.5mm"
XTAL3225 = "Crystal:Crystal_SMD_3225-4Pin_3.2x2.5mm"
L_6045 = "Inductor_SMD:L_Sunlord_SWPA6045S"
RPACK4 = "Resistor_SMD:R_Array_Convex_4x0603"
LGA16 = "pcbkit_parts:Chipanalog_LGA-16_4.65x5.2mm_P0.65mm"
TC2030 = "Connector:Tag-Connect_TC2030-IDC-NL_2x03_P1.27mm_Vertical"


def jst_ph(n):
    return f"Connector_JST:JST_PH_S{n}B-PH-SM4-TB_1x{n:02d}-1MP_P2.00mm_Horizontal"


# PH 2 mm side-entry SMD headers on the KiCad JST footprint: (LCSC, value). Genuine JST where its record exists; the
# 13-way is Megastar's PH-compatible ZX-PH2.0-WT13P (JST's own has no EasyEDA record, so no pin or assembly check), the same
# land pattern within 0.05 mm. Other sizes are footprints only, not fitted in v1.
HEADERS = {
    4: ("C265102", "JST S4B-PH-SM4-TB"),
    8: ("C265121", "JST S8B-PH-SM4-TB"),
    13: ("C22366240", "Megastar ZX-PH2.0-WT13P"),
}

# Passives (LCSC catalog, basic tier unless noted; pcbkit parts/ records).
R_0, R_4K7, R_49K9, R_100K, R_1M, R_60R4 = "C21189", "C23162", "C23184", "C25803", "C22935", "C57572"
RPACK_10K = "C29718"
C_1U, C_2U2, C_4U7, C_12P, C_4N7 = "C15849", "C23630", "C19666", "C38523", "C53987"
C_10U_50V, C_22U, C_4N7_2KV = "C13585", "C45783", "C41017"


def proposed(value, reason):
    return sourced(value, placeholder="pendant draft: " + reason + "; review pending")


def build(out_dir=HERE, refs_path=None):
    out_dir = Path(out_dir)
    assets.install_owned(out_dir)
    pins = pinassign.load(HERE, "pendant")
    b = Board("pendant", refs_path or out_dir / "refs.lock", blocks_path=HERE / "blocks.lock.json")
    i = Intent(b.id)
    i.fabrication(
        "jlcpcb-fr4-2l-v1",
        "FR4-2L-1.6-1OZ-NOMINAL",
        surface_finish="lead_free_hasl",
        mask_colour="green",
        silkscreen_colour="white",
        via_covering="tented",
        assembly_service="standard",
        assembly_side="top",
        reason="pendant v1 two-layer draft: 1.6 mm, the thickness the case model assumes; every review pending",
        process_options=quote.physical_defaults(impedance_requested=False),
    )
    i.workflow(
        requirements=BOARD + "requirements.json",
        decisions=BOARD + "decisions.json",
        netlist=BOARD + "pendant.net",
        parts_lock=BOARD + "parts.lock.json",
    )

    rails = ("GND", "+24V_IN", "+24V_F", "+24V", "+5V", "+3V3", "LED_SUPPLY", "CAN_GND", "CAN_V5ISO")
    root = {name: b.net(name) for name in rails}
    for name in rails:
        root[name].drive = POWER
    for name in ("CAN_H", "CAN_L", "SHIELD", "ESTOP_SENSE_IN", "ESTOP_SENSE_REF", "NRST", "BOOT0"):
        root[name] = b.net(name)
    for name in pinassign.assigned(pins).values():
        root[name] = b.net(name)
    axis = ["AXIS_" + str(n) for n in range(12)]
    step = ["STEP_1", "STEP_10", "STEP_100"]
    keys = ["START", "HOLD", "STOP", "WHEEL_EN", "F1", "F2", "F3"]
    for name in axis + step + ["SR_SPARE", "WHEEL_A", "WHEEL_B"] + ["LED_K_" + k for k in keys]:
        root[name] = b.net(name)
    gnd, v3, v5 = root["GND"], root["+3V3"], root["+5V"]

    def r(local, value, lcsc, a, c, *, footprint=R0603, fit=None):
        part = b.part("Device", "R", local, value=value, footprint=footprint, lcsc=None if fit else lcsc, fit=fit)
        a += part[1]
        c += part[2]
        return part

    def cap(local, value, lcsc, a, c=None, *, footprint=C0603, fit=None):
        part = b.part("Device", "C", local, value=value, footprint=footprint, lcsc=None if fit else lcsc, fit=fit)
        other = gnd if c is None else c
        a += part[1]
        other += part[2]
        return part

    def header(local, n, nets, *, fit=None):
        """A PH 2 mm side-entry header: `nets` by contact, None for an unused contact; the fitting nails are mechanical."""
        part = b.part(
            "Connector_Generic_MountingPin",
            f"Conn_01x{n:02d}_MountingPin",
            local,
            value=f"PH 2 mm {n}-way, reserved" if fit else HEADERS[n][1],
            footprint=jst_ph(n),
            lcsc=None if fit else HEADERS[n][0],
            fit=fit,
        )
        for contact, net in enumerate(nets, 1):
            if net is not None:
                net += part[contact]
        nc(part, "MP", *[str(c) for c, net in enumerate(nets, 1) if net is None])
        return part

    def esd(local, lines, rail, *, fit=None):
        """SRV05-4 (C85364): four lines and the rail they are clamped to."""
        part = b.part("Power_Protection", "SRV05-4", local, value="SRV05-4", footprint=SOT23_6, lcsc=None if fit else "C85364", fit=fit)
        for pin, line in zip(("1", "3", "4", "6"), lines):
            if line is not None:
                line += part[pin]
        rail += part["5"]
        gnd.connect(part["2"])
        nc(part, *[p for p, line in zip(("1", "3", "4", "6"), list(lines) + [None] * 4) if line is None])
        return part

    def pullups(local, lines, rail, *, fit=None):
        """Four 10 kohm in one 0603x4 array: `lines` to `rail`, None leaves a resistor unused."""
        part = b.part("Device", "R_Pack04", local, value="4x10k", footprint=RPACK4, lcsc=None if fit else RPACK_10K, fit=fit)
        for (top, bottom), line in zip(((1, 8), (2, 7), (3, 6), (4, 5)), lines):
            if line is None:
                nc(part, str(top), str(bottom))
                continue
            line += part[top]
            rail += part[bottom]
        return part

    # ---------------------------------------------------------------- 24 V entry, 5 V and 3.3 V
    with b.sheet("power"):
        f1 = b.part("Device", "Polyfuse", "f1", value="1812L050/60GR", footprint=FUSE1812, lcsc="C19078719")
        root["+24V_IN"] += f1[1]
        root["+24V_F"] += f1[2]
        tvs = b.part("Device", "D_TVS", "d_tvs", value="SMBJ30CA", footprint=D_SMB, lcsc="C19077585")
        root["+24V_F"] += tvs["A1"]
        gnd += tvs["A2"]
        # Schottky: about half an M7's drop, 100 V reverse (above the TVS clamp), 2 A.
        d_rev = b.part("Device", "D", "d_rev", value="SS210", footprint=D_SMA, lcsc="C14996")
        root["+24V_F"] += d_rev["A"]
        root["+24V"] += d_rev["K"]
        cap("c_bulk", "10u 50V", C_10U_50V, root["+24V"], footprint=C1206)
        buck = b.part("pcbkit_parts", "LMR51606", "u_buck", value="LMR51606XDBVR", footprint=SOT23_6, lcsc="C21260865")
        en, sw, cb, fb = b.net("en"), b.net("sw"), b.net("cb"), b.net("fb")
        root["+24V"] += buck["VIN"]
        gnd += buck["GND"]
        en += buck["EN"]
        sw += buck["SW"]
        cb += buck["CB"]
        fb += buck["FB"]
        cap("c_in", "10u 50V", C_10U_50V, root["+24V"], footprint=C1206)
        cap("c_in_hf", "100n", C_100N, root["+24V"])
        # EN divider: starts at 1.227 V x 11 = 13.5 V rising, stops at 11 V falling (datasheet 6.5), so a sagging cable
        # supply stops the pendant cleanly instead of letting it brown out.
        r("r_en_top", "100k", R_100K, root["+24V"], en)
        r("r_en_bot", "10k", R_10K, en, gnd)
        cap("c_boot", "100n", C_100N, cb, sw)
        l1 = b.part("Device", "L", "l1", value="33u SWPA6045S330MT", footprint=L_6045, lcsc="C83451")
        sw += l1[1]
        v5 += l1[2]
        # 0.8 V x (1 + 49.9k / 10k) = 4.79 V (datasheet equation 7): over the reference and 1 % tolerances 4.64 to 4.95 V,
        # inside the CAN transceiver's 4.5 V minimum and the ESD arrays' 5.0 V stand-off (the power check derives both).
        r("r_fb_top", "49.9k", R_49K9, v5, fb)
        r("r_fb_bot", "10k", R_10K, fb, gnd)
        cap("c_out", "22u", C_22U, v5, footprint=C0805)
        cap("c_out_hf", "100n", C_100N, v5)
        ldo = b.part("Regulator_Linear", "XC6206PxxxMR", "u_ldo", value="XC6206P332MR-G", footprint=SOT23_3, lcsc="C5446")
        v5 += ldo["VI"]
        v3 += ldo["VO"]
        gnd += ldo["GND"]
        cap("c_ldo_in", "1u", C_1U, v5)
        cap("c_ldo_out", "1u", C_1U, v3)
        # The key LEDs' supply (DEC-PENDANT-LED-SUPPLY, open): 24 V fitted; the 5 V link is the alternative.
        r("r_led_24v", "0", R_0, root["+24V"], root["LED_SUPPLY"])
        r("r_led_5v", "0", R_0, v5, root["LED_SUPPLY"], fit="not-fitted")

    # ---------------------------------------------------------------- MCU
    with b.sheet("mcu"):
        mcu = b.part("MCU_ST_STM32H5", "STM32H503RBTx", "u_mcu", value="STM32H503RBT6", footprint=LQFP64, lcsc="C6807804")
        for port, number in pinassign.numbers(pins).items():
            net = pins["pins"][port]["net"]
            if net is None:
                nc(mcu, number)
            else:
                root[net] += mcu.p[number]
        for number in ("1", "13", "19", "32", "48", "64"):  # VBAT, VDDA, VDD x4
            v3 += mcu.p[number]
        for number in ("12", "18", "31", "47", "63"):  # VSSA, VSS x4
            gnd += mcu.p[number]
        for n, number in enumerate(("19", "32", "48", "64", "1", "13"), 1):
            cap("c_vdd" + str(n), "100n", C_100N, v3)
            i.role("mcu/c_vdd" + str(n), "decoupling", net="+3V3", for_pin=("mcu/u_mcu", number))
        cap("c_bulk", "4.7u", C_4U7, v3)
        cap("c_vdda", "1u", C_1U, v3)
        for n, number in enumerate(("30", "62"), 1):  # 2.2 uF on each VCAP (DS14053 5.3.2, two pins on LQFP64)
            vcap = b.net("vcap" + str(n))
            vcap += mcu.p[number]
            cap("c_vcap" + str(n), "2.2u", C_2U2, vcap)
        root["NRST"] += mcu.p["7"]
        cap("c_nrst", "100n", C_100N, root["NRST"])
        root["BOOT0"] += mcu.p["60"]
        r("r_boot0", "10k", R_10K, root["BOOT0"], gnd)
        xtal = b.part("Device", "Crystal_GND24", "y1", value="16MHz", footprint=XTAL3225, lcsc="C13738")
        root["OSC_IN"] += xtal[1]
        root["OSC_OUT"] += xtal[3]
        gnd += xtal[2], xtal[4]
        # 2 x (CL - Cstray) = 2 x (9 - 3) = 12 pF for the crystal's 9 pF load (part record, a placeholder until read).
        cap("c_osc_in", "12p", C_12P, root["OSC_IN"])
        cap("c_osc_out", "12p", C_12P, root["OSC_OUT"])

    # ---------------------------------------------------------------- isolated CAN FD
    with b.sheet("can"):
        can = b.part("pcbkit_parts", "CA-IS2062A", "u_can", value="CA-IS2062A", footprint=LGA16, lcsc="C22438481")
        root["CAN_RX"] += can["RXD"]
        root["CAN_TX"] += can["TXD"]
        v5 += can["VDDP"]
        v3 += can["VDDL"]
        gnd += can["GNDP1"], can["GND1"]
        root["CAN_GND"] += can["GNDP2"], can["GND2"]
        root["CAN_V5ISO"] += can["VISOOUT"], can["VISOIN"]
        root["CAN_H"] += can["CANH"]
        root["CAN_L"] += can["CANL"]
        # Decoupling as the datasheet's section 10 layout note: 10 uF + 100 nF at VDDP and VISOOUT, 1 uF at VDDL and VISOIN.
        cap("c_vddp", "10u", C_10U, v5, footprint=C0805)
        cap("c_vddp_hf", "100n", C_100N, v5)
        cap("c_vddl", "1u", C_1U, v3)
        cap("c_viso", "10u", C_10U, root["CAN_V5ISO"], root["CAN_GND"], footprint=C0805)
        cap("c_viso_hf", "100n", C_100N, root["CAN_V5ISO"], root["CAN_GND"])
        cap("c_visoin", "1u", C_1U, root["CAN_V5ISO"], root["CAN_GND"])
        # The pendant is an end of the bus: split termination, 2 x 60.4 ohm with 4.7 nF to the bus ground.
        mid = b.net("term_mid")
        # 1206: a half can see 2.8 V (the full 5.5 V bus envelope split in two), 130 mW, over an 0603's rating.
        r("r_term_h", "60.4", R_60R4, root["CAN_H"], mid, footprint=R1206)
        r("r_term_l", "60.4", R_60R4, mid, root["CAN_L"], footprint=R1206)
        cap("c_term", "4.7n", C_4N7, mid, root["CAN_GND"])
        tvs = b.part("Device", "D_TVS_Dual_AAC", "d_can", value="PESD2CAN", footprint=SOT23, lcsc="C284078")
        root["CAN_H"] += tvs[1]
        root["CAN_L"] += tvs[2]
        root["CAN_GND"] += tvs[3]
        # The cable shield ends here through 1 Mohm and 4.7 nF (2 kV) to the pendant's 0 V: no DC ground loop, an HF path.
        r("r_shield", "1M", R_1M, root["SHIELD"], gnd)
        cap("c_shield", "4.7n 2kV", C_4N7_2KV, root["SHIELD"], footprint=C1206)

    # ---------------------------------------------------------------- the M12 pigtail and the E-stop read-back
    with b.sheet("cable"):
        header(
            "j_cable",
            8,
            [root[n] for n in ("+24V_IN", "GND", "CAN_H", "CAN_L", "CAN_GND", "SHIELD", "ESTOP_SENSE_IN", "ESTOP_SENSE_REF")],
        )
    # Tap 1 is the loop after the E-stop's NC contact, tap 2 the loop's own 0 V: the input only loads the loop toward its
    # 0 V (it can pull a weak input low, never hold it high) and is never across the contact, where it would sit in series.
    b.block(
        blocks.DinType3,
        "estop",
        ports={
            "IN": root["ESTOP_SENSE_IN"],
            "FIELD_GND": root["ESTOP_SENSE_REF"],
            "GND": gnd,
            "VLOGIC": v3,
            "OUT": root["ESTOP_SENSE"],
        },
    )

    # ---------------------------------------------------------------- the hand wheel
    with b.sheet("wheel"):
        header("j_wheel", 4, [v5, gnd, root["WHEEL_A"], root["WHEEL_B"]])
        esd("u_esd", [root["WHEEL_A"], root["WHEEL_B"]], v5)
        # The wheel's output type is not on the vendor page: pull-ups to its 5 V supply are footprints until measured.
        r("r_pu_a", "4.7k", R_4K7, root["WHEEL_A"], v5, fit="not-fitted")
        r("r_pu_b", "4.7k", R_4K7, root["WHEEL_B"], v5, fit="not-fitted")
        in_a, in_b = b.net("in_a"), b.net("in_b")
        r("r_a", "1k", R_1K, root["WHEEL_A"], in_a)
        r("r_b", "1k", R_1K, root["WHEEL_B"], in_b)
        buf = b.part("74xGxx", "74LVC2G17", "u_buf", value="SN74LVC2G17DBVR", footprint=SOT23_6, lcsc="C7394037")
        in_a += buf.p["1"]
        root["ENC_A"] += buf.p["6"]
        in_b += buf.p["3"]
        root["ENC_B"] += buf.p["4"]
        v3 += buf.p["5"]
        gnd += buf.p["2"]
        cap("c_buf", "100n", C_100N, v3)

    # ---------------------------------------------------------------- axis and step selectors
    with b.sheet("selectors"):
        header("j_axis", 13, [gnd] + [root[n] for n in axis])
        header("j_step", 4, [gnd] + [root[n] for n in step])
        lines = [root[n] for n in axis + step + ["SR_SPARE"]]
        for n in range(4):
            pullups("rn" + str(n + 1), lines[4 * n : 4 * n + 4], v3)
            esd("u_esd" + str(n + 1), [line if line is not root["SR_SPARE"] else None for line in lines[4 * n : 4 * n + 4]], v3)
        chain = b.net("chain")
        for n, (local, data, serial_in, serial_out) in enumerate(
            (("u_sr1", lines[:8], chain, root["SR_DATA"]), ("u_sr2", lines[8:], v3, chain))
        ):
            sr = b.part("74xx", "74HC165", local, value="74HC165D", footprint=SOIC16, lcsc="C5613")
            for bit, line in enumerate(data):
                line += sr["D" + str(bit)]
            root["SR_LOAD"] += sr.p["1"]
            root["SR_SCK"] += sr.p["2"]
            gnd += sr.p["15"], sr.p["8"]
            serial_in += sr["DS"]
            serial_out += sr.p["9"]
            v3 += sr.p["16"]
            nc(sr, "7")
            cap("c_" + local, "100n", C_100N, v3)

    # ---------------------------------------------------------------- keys and their LEDs
    with b.sheet("keys"):
        for key in keys:
            header("j_" + key.lower(), 4, [root["KEY_" + key], gnd, root["LED_SUPPLY"], root["LED_K_" + key]])
        key_lines = [root["KEY_" + k] for k in keys] + [None]
        pullups("rn1", key_lines[:4], v3)
        pullups("rn2", key_lines[4:], v3)
        esd("u_esd1", key_lines[:4], v3)
        esd("u_esd2", key_lines[4:], v3)
        drv = b.part("Transistor_Array", "ULN2003A", "u_led", value="ULN2003ADR", footprint=SOIC16, lcsc="C7512")
        for n, key in enumerate(keys, 1):
            root["LED_" + key] += drv["I" + str(n)]
            root["LED_K_" + key] += drv["O" + str(n)]
        gnd += drv["GND"]
        root["LED_SUPPLY"] += drv["COM"]

    # ---------------------------------------------------------------- reserved: enabling switch and membrane keypad
    with b.sheet("reserved"):
        enable = [root["ENABLE_" + str(n)] for n in range(1, 5)]
        header("j_enable", 5, enable + [gnd], fit="not-fitted")
        pullups("rn_enable", enable, v3)
        esd("u_esd_enable", enable, v3, fit="not-fitted")
        matrix = [root[n] for n in ("KEY_F1", "KEY_F2", "KEY_F3", "LED_F1", "LED_F2", "LED_F3", "MATRIX_5", "MATRIX_6")]
        header(
            "j_keypad",
            13,
            matrix + [root["KEYPAD_ID"], root["I2C1_SDA"], root["I2C1_SCL"], v3, gnd],
            fit="not-fitted",
        )
        r("r_keypad_id", "10k", R_10K, root["KEYPAD_ID"], v3)
        esd("u_esd_kp1", matrix[3:7], v3, fit="not-fitted")
        esd("u_esd_kp2", [matrix[7]], v3, fit="not-fitted")

    # ---------------------------------------------------------------- OLED
    with b.sheet("oled"):
        # Waveshare 2.42 in module, I2C mode: VCC GND DIN(SDA) CLK(SCL) CS DC RES; CS and DC low (address 0x3C).
        header(
            "j_oled", 8, [v3, gnd, root["I2C1_SDA"], root["I2C1_SCL"], gnd, gnd, root["OLED_RES"], None]
        )
        r("r_sda", "4.7k", R_4K7, root["I2C1_SDA"], v3)
        r("r_scl", "4.7k", R_4K7, root["I2C1_SCL"], v3)
        r("r_res", "10k", R_10K, root["OLED_RES"], v3)

    # ---------------------------------------------------------------- programming (SWD) and the bench-only USB pads
    with b.sheet("prog"):
        swd = b.part(
            "Connector", "Conn_ARM_SWD_TagConnect_TC2030-NL", "j_swd", value="TC2030-IDC-NL", footprint=TC2030, fit="not-fitted"
        )
        v3 += swd[1]
        root["SWDIO"] += swd[2]
        root["NRST"] += swd[3]
        root["SWCLK"] += swd[4]
        gnd += swd[5]
        nc(swd, "6")
        tp = b.part("Connector", "TestPoint", "tp_boot0", value="BOOT0", footprint=TP, fit="hand-fitted")
        root["BOOT0"] += tp[1]
        # USB is for the bench only: pads for D+, D- and 0 V, no connector (every USB connector at JLCPCB is an extended
        # part, a fee per order for a part no field unit uses). The bench pendant runs from its 24 V supply.
        for net in ("USB_DM", "USB_DP", "GND"):
            local = "tp_usb_gnd" if net == "GND" else "tp_" + net.lower()
            tp = b.part("Connector", "TestPoint", local, value="USB 0V" if net == "GND" else net, footprint=TP, fit="hand-fitted")
            root[net] += tp[1]

    # ---------------------------------------------------------------- the four mounting holes the case's standoffs take
    with b.sheet("mech"):
        for n in range(1, 5):
            b.part(
                "Mechanical",
                "MountingHole",
                "h" + str(n),
                value="M2.5",
                footprint="MountingHole:MountingHole_2.7mm_M2.5",
                fit="hand-fitted",
            )

    path = b.write(out_dir)
    run_erc(b, out_dir / "pendant.erc.json")
    b.fragments(i, path, bindings={"estop": {"IN": {"tag": "cable/j_cable", "pin": "7"}}})
    nl = netlist.load(path)
    i.barrier(
        "can/u_can",
        ["logic", "can_bus"],
        creepage_mm=proposed(3.0, "functional isolation (24 V system, ground offsets only); the LGA16 lands keep 3.15 mm"),
        clearance_mm=proposed(3.0, "functional isolation; the package's own CLR/CPG is 3.45 mm (datasheet section 7)"),
    )
    # The pigtail header carries the 24 V supply (logic), the isolated bus and the E-stop loop on neighbouring contacts:
    # its interface record (interfaces/cable.json) names each pin's domain, so it is a connector, not a barrier.
    pinouts = part_interfaces.apply(i, out_dir, {"cable/j_cable": "interfaces/cable.json"})
    extra = {net: "can_bus" for net in ("CAN_V5ISO", "can/term_mid")}
    domains = domains_from_ports(nl, b.block_uses, i.barriers, extra, interfaces=pinouts)
    for domain in sorted(set(domains.values())):
        i.domain(domain, *sorted(net for net, owner in domains.items() if owner == domain))
    for name, kind, voltage, current in (
        ("logic_gnd", "power", 0, 0.5),
        ("supply_24v", "field_power", 30, 0.5),
        ("logic_5v", "power", 5.25, 0.4),
        ("logic_3v3", "power", 3.465, 0.15),
        ("led_supply", "field_power", 30, 0.15),
        ("led_drive", "signal", 30, 0.03),
        ("logic_signal", "signal", 3.465, 0.01),
        ("wheel_signal", "signal", 5.25, 0.01),
        ("switching", "power", 30, 0.8),
        ("can_power", "power", 5.5, 0.1),
        ("can_signal", "signal", 5.5, 0.07),
        ("estop_input", "signal", 30, 0.015),
        ("estop_ref", "field_power", 0, 0.015),
        ("estop_led", "signal", 1.4, 0.015),
        ("cable_shield", "signal", 30, 0.01),
    ):
        i.net_class(
            name,
            kind,
            voltage_v=proposed(voltage, "maximum working class voltage"),
            current_a=proposed(current, "routing current reserve, not a qualified load rating"),
        )
    assigned = {
        "GND": "logic_gnd",
        "+24V_IN": "supply_24v",
        "+24V_F": "supply_24v",
        "+24V": "supply_24v",
        "+5V": "logic_5v",
        "+3V3": "logic_3v3",
        "LED_SUPPLY": "led_supply",
        "power/sw": "switching",
        "power/cb": "switching",
        "CAN_GND": "can_power",
        "CAN_V5ISO": "can_power",
        "CAN_H": "can_signal",
        "CAN_L": "can_signal",
        "can/term_mid": "can_signal",
        "SHIELD": "cable_shield",
        "WHEEL_A": "wheel_signal",
        "WHEEL_B": "wheel_signal",
        "ESTOP_SENSE_REF": "estop_ref",
        "estop/led_a": "estop_led",
    }
    for name in nl.nets:
        cls = assigned.get(name)
        if cls is None:
            cls = (
                "led_drive"
                if name.startswith("LED_K_")
                else "estop_input"
                if domains.get(name) == "field24"
                else "logic_signal"
            )
        i.classify(cls, name)
    # Which capacitor serves which supply pin (the power check's no-decoupling and no-bulk rules).
    for cap_tag, ic, pin, net in (
        ("selectors/c_u_sr1", "selectors/u_sr1", 16, "+3V3"),
        ("selectors/c_u_sr2", "selectors/u_sr2", 16, "+3V3"),
        ("wheel/c_buf", "wheel/u_buf", 5, "+3V3"),
        ("can/c_vddp_hf", "can/u_can", 6, "+5V"),
        ("can/c_vddl", "can/u_can", 7, "+3V3"),
        ("can/c_visoin", "can/u_can", 10, "CAN_V5ISO"),
        ("can/c_viso_hf", "can/u_can", 11, "CAN_V5ISO"),
        ("power/c_in_hf", "power/u_buck", 5, "+24V"),
        ("power/c_ldo_in", "power/u_ldo", 3, "+5V"),
        ("power/c_ldo_out", "power/u_ldo", 2, "+3V3"),
    ):
        i.role(cap_tag, "decoupling", net=net, for_pin=(ic, pin))
    for cap_tag, role, net in (
        ("mcu/c_vcap1", "bulk", "mcu/vcap1"),
        ("mcu/c_vcap2", "bulk", "mcu/vcap2"),
        ("mcu/c_nrst", "filter", "NRST"),
        ("mcu/c_osc_in", "crystal_load", "OSC_IN"),
        ("mcu/c_osc_out", "crystal_load", "OSC_OUT"),
        ("can/c_term", "termination", "can/term_mid"),
        ("can/c_shield", "filter", "SHIELD"),
        ("power/c_boot", "bootstrap", "power/cb"),
    ):
        i.role(cap_tag, role, net=net)
    for cap_tag, net in (
        ("power/c_bulk", "+24V"),
        ("power/c_in", "+24V"),
        ("power/c_out", "+5V"),
        ("power/c_out_hf", "+5V"),
        ("mcu/c_bulk", "+3V3"),
        ("mcu/c_vdda", "+3V3"),
        ("can/c_vddp", "+5V"),
        ("can/c_viso", "CAN_V5ISO"),
    ):
        i.role(cap_tag, "bulk", net=net)
    # What the board feeds through its connectors, which no part on it shows.
    for name, net, amps, what in (
        ("wheel", "+5V", 0.1, "the MPG hand wheel: the vendor listing gives at most 100 mA at 5 V; measure on arrival"),
        ("oled", "+3V3", 0.05, "the Waveshare 2.42 in OLED module: a budget for a full white screen until it is measured"),
        ("key_leds", "LED_SUPPLY", 0.14, "seven button LEDs: 7 x 20 mA, a budget until the buttons' LEDs are measured (DEC-PENDANT-LED-SUPPLY)"),
    ):
        i.external_load(name, net, current_max_a=proposed(amps, what), what=what.split(":")[0])
    i.power_source(
        "cabinet_24v",
        "+24V_IN",
        voltage_v=proposed(24, "the cabinet's pendant supply, through the M12 cable"),
        current_max_a=proposed(None, "the cabinet's supply capacity is a physical review"),
        range_v=proposed([18, 30], "cabinet 24 V supply envelope at the pendant end of the cable"),
    )
    # The case's envelope (pendant.envelope.json, written by machinekit's BoardEnvelope from the pendant assembly) gives
    # the outline, the mounting holes, the height zones on both sides and where each control's wires arrive.
    enclosure.apply(
        i,
        out_dir,
        holes={"h" + str(n): "mech/h" + str(n) for n in range(1, 5)},
        connections={
            "cable": "cable/j_cable",
            "enabling": "reserved/j_enable",
            "display-pins": "oled/j_oled",
            "jog-wheel-terminals": "wheel/j_wheel",
            "selectors-axis-body": "selectors/j_axis",
            "selectors-step-body": "selectors/j_step",
            "jog-wheel-enable-body": "keys/j_wheel_en",
            **{f"cycle-keys-{k}-body": f"keys/j_{k}" for k in ("start", "hold", "stop")},
            **{f"function-keys-{k}-body": f"keys/j_{k}" for k in ("f1", "f2", "f3")},
        },
    )
    i.page("A4", revision="pendant v1 draft", comment="every review pending; outline and zones from the case's envelope")
    for sheet in sorted({p.sheet for p in nl.parts}):
        i.group(sheet)
    b.gates(i)
    i.write(out_dir / "pendant.intent.json", path)
    return path


if __name__ == "__main__":
    build(
        Path(sys.argv[1]) if len(sys.argv) > 1 else HERE,
        Path(sys.argv[2]) if len(sys.argv) > 2 else None,
    )
