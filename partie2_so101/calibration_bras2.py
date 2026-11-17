# -*- coding: utf-8 -*-
"""
Calibration du bras SO-101 pour le code lerobot-kinematics.
Génère examples/main_follower.json (format attendu par feetech_arm).

Étapes
------
1. Couple coupé : le bras devient libre.
2. MuJoCo affiche la POSE DE RÉFÉRENCE (toutes articulations à 0) : place le vrai bras pareil.
3. Le script écrit un offset dans chaque servo pour que cette pose lise 2048 (milieu de course).
   -> évite le passage 4095 -> 0 en cours de mouvement.
4. Vérification miroir : MuJoCo suit le vrai bras en direct. Bouge chaque articulation à la main ;
   si MuJoCo tourne dans le sens inverse, tape son numéro (1-6) + Entrée pour l'inverser.
   Tape s + Entrée pour enregistrer.

Lancer depuis la racine du dépôt :  python calibration_bras2.py
"""
import functools
import json
import threading
import time

print = functools.partial(print, flush=True)

import numpy as np
import mujoco
import mujoco.viewer
import scservo_sdk as scs

PORT = "COM12"
CALIB_FILE = "examples/main_follower.json"
XML_PATH = "./examples/scene_so101.xml"
JOINT_NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
IDS = [1, 2, 3, 4, 5, 6]

BAUDRATE = 1_000_000
ADDR_HOMING_OFFSET = 31   # EEPROM, 2 octets, signe sur le bit 11
ADDR_TORQUE_ENABLE = 40
ADDR_LOCK = 55
ADDR_PRESENT_POSITION = 56


# -----------------------------------------------------------------------------
# Accès bas niveau aux servos
# -----------------------------------------------------------------------------
def encode_sign_magnitude(value, sign_bit=11):
    value = int(np.clip(value, -2047, 2047))
    return (1 << sign_bit) | abs(value) if value < 0 else value


N_RETRY = 20


def safe(fn):
    """Appel SDK protégé : une réponse tronquée fait planter le SDK (IndexError)."""
    try:
        return fn()
    except (IndexError, TypeError):
        return None


def retry(fn, what):
    """Exécute fn() jusqu'à N_RETRY fois (ratés ponctuels du bus Feetech)."""
    for k in range(N_RETRY):
        out = safe(fn)
        if out is not None:
            res = out[-2] if len(out) == 3 else out[0]
            if res == scs.COMM_SUCCESS:
                return out
        print(f"    {what} : pas de réponse, nouvel essai {k + 1}/{N_RETRY}")
        time.sleep(0.1)
    raise RuntimeError(f"{what} : pas de réponse valide après {N_RETRY} essais")


def read_pos(port, pkt, i):
    pos, res, err = retry(lambda: pkt.read2ByteTxRx(port, i, ADDR_PRESENT_POSITION),
                          f"lecture position ID {i}")
    return pos


def write1(port, pkt, i, addr, value, what):
    retry(lambda: pkt.write1ByteTxRx(port, i, addr, value), what)


def write_offset(port, pkt, i, offset):
    """Écrit l'offset en EEPROM puis relit le registre pour vérifier (la réponse
    du servo peut manquer pendant l'écriture EEPROM alors que l'écriture a réussi)."""
    target = encode_sign_magnitude(offset)
    write1(port, pkt, i, ADDR_LOCK, 0, f"déverrouillage ID {i}")
    for _ in range(N_RETRY):
        port.clearPort()                      # vide les octets parasites éventuels
        safe(lambda: pkt.write2ByteTxRx(port, i, ADDR_HOMING_OFFSET, target))
        time.sleep(0.2)                       # laisse le servo finir l'écriture EEPROM
        out = safe(lambda: pkt.read2ByteTxRx(port, i, ADDR_HOMING_OFFSET))
        if out is not None and out[1] == scs.COMM_SUCCESS and out[0] == target:
            break
        print(f"    ID {i} : offset pas encore confirmé, nouvel essai")
    else:
        raise RuntimeError(f"ID {i} : impossible d'écrire l'offset {offset}")
    write1(port, pkt, i, ADDR_LOCK, 1, f"verrouillage ID {i}")
    time.sleep(0.2)


def read_offset(port, pkt, i):
    out = safe(lambda: pkt.read2ByteTxRx(port, i, ADDR_HOMING_OFFSET))
    if out is None or out[1] != scs.COMM_SUCCESS:
        return None
    v = out[0]
    return -(v & 0x7FF) if v & 0x800 else v


def center_servo(port, pkt, i):
    """Méthode Feetech « Middle » : écrire 128 dans Torque_Enable fait que le servo
    prend sa position actuelle comme 2048 et enregistre l'offset lui-même."""
    for k in range(N_RETRY):
        port.clearPort()
        safe(lambda: pkt.write1ByteTxRx(port, i, ADDR_TORQUE_ENABLE, 128))
        time.sleep(0.3)
        out = safe(lambda: pkt.read2ByteTxRx(port, i, ADDR_PRESENT_POSITION))
        if out is not None and out[1] == scs.COMM_SUCCESS and abs(out[0] - 2048) <= 30:
            return out[0]
        lu = out[0] if out is not None and out[1] == scs.COMM_SUCCESS else "rien"
        print(f"    ID {i} : position lue {lu} (offset en mémoire : {read_offset(port, pkt, i)}), nouvel essai {k + 1}/{N_RETRY}")
    raise RuntimeError(f"ID {i} : la calibration « Middle » ne prend pas. Vérifie l'alimentation et le câble de ce servo.")


def set_servo_zero_at_2048():
    """Centre chaque servo (2048) sur la pose actuelle du bras."""
    port, pkt = scs.PortHandler(PORT), scs.PacketHandler(0)
    if not port.openPort() or not port.setBaudRate(BAUDRATE):
        raise SystemExit(f"Impossible d'ouvrir {PORT}")
    try:
        print("Couple coupé : le bras est libre.")
        for i in IDS:
            write1(port, pkt, i, ADDR_TORQUE_ENABLE, 0, f"couple off ID {i}")

        input("\nPlace le vrai bras dans la pose affichée par MuJoCo, puis appuie sur Entrée "
              "(5 s pour remettre les mains sur le bras)...")
        for k in range(5, 0, -1):
            print(f"  capture dans {k} s")
            time.sleep(1)
        print("  capture...")

        raw = {i: read_pos(port, pkt, i) for i in IDS}
        print("Positions avant centrage :", raw)

        for i in IDS:
            p = center_servo(port, pkt, i)
            write1(port, pkt, i, ADDR_TORQUE_ENABLE, 0, f"couple off ID {i}")
            print(f"  ID {i} : centré -> position lue {p} (offset en mémoire : {read_offset(port, pkt, i)})")
        print("Tous les servos sont centrés, tu peux lâcher le bras.")
    finally:
        port.closePort()


# -----------------------------------------------------------------------------
# Fichier de calibration au format lerobot-kinematics (feetech_arm)
# -----------------------------------------------------------------------------
def make_calib(drive_mode):
    # angle(deg) = (s*raw + homing_offset) * 180/2048, s = -1 si drive_mode
    # on veut angle = 0 pour raw = 2048
    return {
        "homing_offset": [2048 if d else -2048 for d in drive_mode],
        "drive_mode": list(drive_mode),
        "start_pos": [0] * 6,
        "end_pos": [0] * 6,
        "calib_mode": ["DEGREE"] * 6,
        "motor_names": JOINT_NAMES,
    }


def save_calib(drive_mode):
    with open(CALIB_FILE, "w") as f:
        json.dump(make_calib(drive_mode), f, indent=2)


# -----------------------------------------------------------------------------
# Programme
# -----------------------------------------------------------------------------
def main():
    mjmodel = mujoco.MjModel.from_xml_path(XML_PATH)
    mjdata = mujoco.MjData(mjmodel)
    idx = np.array([mjmodel.jnt_qposadr[mjmodel.joint(n).id] for n in JOINT_NAMES])

    with mujoco.viewer.launch_passive(mjmodel, mjdata) as viewer:
        # --- Étapes 1-3 : pose de référence ---
        mjdata.qpos[idx] = 0.0
        mujoco.mj_forward(mjmodel, mjdata)
        viewer.sync()
        print("Pose de référence affichée dans MuJoCo (toutes les articulations à 0).")
        set_servo_zero_at_2048()

        drive_mode = [0] * 6
        save_calib(drive_mode)

        # --- Étape 4 : vérification miroir ---
        from lerobot_kinematics import feetech_arm
        arm = feetech_arm(driver_port=PORT, calibration_file=CALIB_FILE)
        arm.arm_hardware.write("Torque_Enable", 0)

        cmd = {"text": None}

        def read_input():
            while True:
                cmd["text"] = input().strip().lower()
                if cmd["text"] == "s":
                    return

        print("\nVÉRIFICATION : bouge chaque articulation à la main et regarde MuJoCo.")
        print("  Sens inverse sur une articulation -> tape son numéro (1-6) + Entrée.")
        print("  Tout est bon -> tape s + Entrée pour enregistrer.\n")
        threading.Thread(target=read_input, daemon=True).start()

        try:
            while viewer.is_running():
                c, cmd["text"] = cmd["text"], None
                if c == "s":
                    break
                if c in [str(k) for k in range(1, 7)]:
                    j = int(c) - 1
                    drive_mode[j] = 1 - drive_mode[j]
                    arm.arm_hardware.set_calibration(make_calib(drive_mode))
                    print(f"  {JOINT_NAMES[j]} inversé (drive_mode = {drive_mode[j]})")

                q = np.array(arm.feedback(), dtype=float)
                mjdata.qpos[idx] = q
                mujoco.mj_forward(mjmodel, mjdata)
                viewer.sync()
                print("  q (rad) = " + " ".join(f"{v:+.2f}" for v in q), end="\r")
                time.sleep(0.03)
        finally:
            arm.disconnect()

    save_calib(drive_mode)
    print(f"\nCalibration enregistrée dans {CALIB_FILE} : drive_mode = {drive_mode}")


if __name__ == "__main__":
    main()
