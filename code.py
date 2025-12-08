# code.py – Minimaler MeshCom-Test-Sender für RF.Guru LoRa433APRSTracker
#
# Sendet periodisch MeshCom-Textnachrichten von config.callsign (z.B. DN9APW-8)
# an DEST_CALLSIGN (z.B. DN9APW-99) auf 433.175 MHz.
#
# Hardware: RF.Guru LoRa APRS Tracker (RP2040 + RFM9x + PA)
# Benötigte libs im /lib:
#   - adafruit_rfm9x.mpy (RF_Guru Variante)
#   - watchdog.mpy (falls nicht im Core)
#
# Philipp: Du kannst unten DEST_CALLSIGN, TEXT_MESSAGE und INTERVAL_SEC anpassen.

import time
import binascii

import board
import busio
import digitalio
import microcontroller
import supervisor
from microcontroller import watchdog as w
from watchdog import WatchDogMode

import adafruit_rfm9x
import config

# ============================================================
# USER-PARAMETER
# ============================================================

# Von hier sendet der Tracker – muss in config.py gesetzt sein (z.B. "DN9APW-8")
SOURCE_CALLSIGN = config.callsign.upper()

# MeshCom-Ziel (dein Gateway)
DEST_CALLSIGN = "DN9APW-99"

# Text der Nachricht
TEXT_MESSAGE = "MeshCom TEST von " + SOURCE_CALLSIGN

# Sendeintervall (Sekunden)
INTERVAL_SEC = 15

# MeshCom HF-Settings (EU Profil)
RADIO_FREQ_MHZ = 433.175  # MeshCom EU
MESHCOM_BW = 250000       # 250 kHz
MESHCOM_SF = 11           # Spreading Factor
MESHCOM_CR = 6            # Coding Rate 4/6

# Hardware-ID / Mod-ID (erstmal Dummy-Werte, aber gültig)
HW_ID = 3   # z.B. "TTGO ESP32 Paxcounter TLORA V2 1.6" – als generische ID
MOD_ID = 3  # "normaler Bereich, langsam/robust"


# ============================================================
# SYSTEM-SETUP
# ============================================================

supervisor.runtime.autoreload = False

# Watchdog
w.timeout = 5
w.mode = WatchDogMode.RESET
w.feed()

VERSION = "MeshCom-Test 0.1"


def log(color_code, msg):
    stamp = time.localtime()
    ts = f"{stamp.tm_mon:02}/{stamp.tm_mday:02}/{stamp.tm_year} " + f"{stamp.tm_hour:02}:{stamp.tm_min:02}:{stamp.tm_sec:02}"
    print(color_code + "[" + ts + "] " + msg + "\x1b[0m")


def yellow(msg):
    log("\x1b[38;5;220m", msg)


def green(msg):
    log("\x1b[1;5;32m", msg)


def red(msg):
    log("\x1b[1;5;31m", msg)


# ============================================================
# MESHCOM FRAME BUILDER
# ============================================================

def meshcom_fcs(data):
    """16-bit FCS = einfache Summe aller Bytes (unsigned) & 0xFFFF."""
    return sum(data) & 0xFFFF


def build_meshcom_text_frame(source,
                             dest,
                             text,
                             msg_id,
                             max_hop=5,
                             hw_id=HW_ID,
                             mod_id=MOD_ID):
    """
    Baut ein MeshCom 2.0 Text-Frame:

    ':' +
    MsgID (4 Byte, little endian) +
    HH (1 Byte, Bits 0–2 = Hopcount) +
    b"<SRC>><DEST>:<PAYLOAD>" +
    0x00 +
    HW_ID (1 Byte) +
    MOD_ID (1 Byte) +
    FCS (2 Byte, little endian, Summe über alles ab ':') +
    '#'
    """
    frame = bytearray()

    # 1) Identifier (Textnachricht)
    frame.append(ord(':'))

    # 2) Message-ID
    frame += int(msg_id & 0xFFFFFFFF).to_bytes(4, "little")

    # 3) MAX-HOP + Flags (nur Hopcount, Flags=0)
    frame.append(max_hop & 0x07)

    # 4) APRS-ähnlicher Header + Payload
    header_payload = f"{source}>{dest}:{text}"
    frame += header_payload.encode("ascii")

    # 5) Payload-Abschluss
    frame.append(0x00)

    # 6) Hardware-ID
    frame.append(hw_id & 0xFF)

    # 7) Mod-ID
    frame.append(mod_id & 0xFF)

    # 8) FCS über alle bisher gesammelten Bytes
    fcs = meshcom_fcs(frame)
    frame += fcs.to_bytes(2, "little")

    # 9) Abschlusszeichen '#'
    frame.append(ord('#'))

    return bytes(frame)


# ============================================================
# HARDWARE INITIALISIEREN
# ============================================================

yellow("Booting " + VERSION)
yellow("Source: " + SOURCE_CALLSIGN + "  Dest: " + DEST_CALLSIGN)

# LEDs
pwrLED = digitalio.DigitalInOut(board.GP9)
pwrLED.direction = digitalio.Direction.OUTPUT
pwrLED.value = True

gpsLED = digitalio.DigitalInOut(board.GP10)
gpsLED.direction = digitalio.Direction.OUTPUT
gpsLED.value = False

loraLED = digitalio.DigitalInOut(board.GP11)
loraLED.direction = digitalio.Direction.OUTPUT
loraLED.value = False

# PA Enable
amp = digitalio.DigitalInOut(board.GP2)
amp.direction = digitalio.Direction.OUTPUT
amp.value = False

# SPI + RFM9x
yellow("Init LoRa (RFM9x)…")
CS = digitalio.DigitalInOut(board.GP21)
RESET = digitalio.DigitalInOut(board.GP20)
spi = busio.SPI(board.GP18, MOSI=board.GP19, MISO=board.GP16)

try:
    rfm9x = adafruit_rfm9x.RFM9x(
        spi,
        CS,
        RESET,
        RADIO_FREQ_MHZ,
        baudrate=1_000_000,
    )

    # MeshCom-typische Parameter setzen (wenn verfügbar)
    try:
        rfm9x.signal_bandwidth = MESHCOM_BW
        rfm9x.spreading_factor = MESHCOM_SF
        rfm9x.coding_rate = MESHCOM_CR
        # MeshCom hat eigenes FCS, daher radio-seitiges CRC optional:
        rfm9x.enable_crc = False
    except AttributeError:
        yellow("Warnung: RFM9x-Modulation nicht komplett setzbar (custom lib).")

    rfm9x.tx_power = config.power  # 5–23 dBm aus config.py

    green(
        f"LoRa OK @ {RADIO_FREQ_MHZ} MHz, PWR={config.power} dBm, "
        f"BW={MESHCOM_BW}, SF={MESHCOM_SF}, CR=4/{MESHCOM_CR}"
    )
except Exception as e:
    red("LoRa INIT ERROR: " + str(e))
    while True:
        w.feed()
        time.sleep(1)


# ============================================================
# SENDESCHLEIFE
# ============================================================

yellow("Starte MeshCom-Testloop…")

msg_counter = 1
last_tx = 0

while True:
    w.feed()
    now = time.monotonic()

    if now - last_tx >= INTERVAL_SEC:
        # Frame bauen
        frame = build_meshcom_text_frame(
            source=SOURCE_CALLSIGN,
            dest=DEST_CALLSIGN,
            text=TEXT_MESSAGE,
            msg_id=msg_counter,
            max_hop=5,
            hw_id=HW_ID,
            mod_id=MOD_ID,
        )

        hex_preview = binascii.hexlify(frame).decode("ascii")
        yellow(f"TX #{msg_counter}: len={len(frame)} bytes  hex={hex_preview[:80]}…")

        try:
            loraLED.value = True
            if config.hasPa:
                amp.value = True
                time.sleep(0.25)

            # Achtung: RF_Guru-RFM9x erwartet send(watchdog, data)
            rfm9x.send(w, frame)

            if config.hasPa:
                time.sleep(0.1)
                amp.value = False
            loraLED.value = False

            green("MeshCom frame sent.")
        except Exception as txerr:
            red("LoRa TX ERROR: " + str(txerr))

        msg_counter = (msg_counter + 1) & 0xFFFFFFFF
        last_tx = now

    # kleine Pause, damit die CPU nicht 100% läuft
    time.sleep(0.05)
