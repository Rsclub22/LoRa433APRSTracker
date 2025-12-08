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
w.timeout = 8
w.mode = WatchDogMode.RESET
w.feed()

VERSION = "MeshCom-Test 0.1"


def log(color_code, msg):
    stamp = time.localtime()
    ts = "{:02}/{:02}/{} {:02}:{:02}:{:02}".format(stamp.tm_mon, stamp.tm_mday, stamp.tm_year, stamp.tm_hour, stamp.tm_min, stamp.tm_sec)
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
    header_payload = "{}>{}:{}".format(source, dest, text)
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
# MESHCOM RX HANDLER
# ============================================================

def parse_meshcom_frame(packet):
    """Parse MeshCom 2.0 Text-Frame und gibt Informationen aus."""
    if len(packet) < 15:
        return False
    
    # Frame-Format: ':' + MsgID(4) + HH(1) + Header+Payload + 0x00 + HW_ID(1) + MOD_ID(1) + FCS(2) + '#'
    if packet[0] != ord(':') or packet[-1] != ord('#'):
        return False
    
    try:
        msg_id = (packet[1] | (packet[2] << 8) | (packet[3] << 16) | (packet[4] << 24))
        max_hop = packet[5] & 0x07
        
        # Finde das Payload-Ende (0x00)
        payload_end = packet.find(b'\x00', 6)
        if payload_end < 0:
            return False
        
        # Extrahiere den Header+Payload Teil (Source>Dest:Text)
        header_payload = packet[6:payload_end].decode("ascii", errors="ignore")
        
        hw_id = packet[payload_end + 1] if payload_end + 1 < len(packet) else 0
        mod_id = packet[payload_end + 2] if payload_end + 2 < len(packet) else 0
        
        green("MeshCom RX: ID={:08X} Hop={} HW={} MOD={}".format(msg_id, max_hop, hw_id, mod_id))
        yellow("Data: " + header_payload)
        return True
    except Exception as e:
        yellow("Parse-ERROR: " + str(e))
        return False


rx_silence_counter = 0
rx_mode_check_counter = 0

def check_and_receive():
    """Prüft auf eingangene MeshCom-Pakete."""
    global rx_silence_counter, rx_mode_check_counter
    
    # Aktiviere RX-Mode regelmäßig neu (alle 50 Versuche)
    rx_mode_check_counter += 1
    if rx_mode_check_counter >= 50:
        rx_mode_check_counter = 0
        try:
            # Versuche, sicherzustellen dass RX aktiv ist
            rfm9x.idle()
            time.sleep(0.01)
            rfm9x.listen()
            w.feed()
        except:
            pass
    
    try:
        # RF.Guru-RFM9x braucht watchdog als erstes Argument!
        packet = rfm9x.receive(w, timeout=0.05)
        if packet is not None:
            rx_silence_counter = 0  # Reset counter
            hex_data = binascii.hexlify(packet).decode("ascii")
            green("RX: len={} bytes  hex={}...".format(len(packet), hex_data[:80]))
            
            # Versuche als MeshCom-Frame zu parsen
            if not parse_meshcom_frame(packet):
                # Fallback: Nur Hex ausgeben
                pass
            
            w.feed()
        else:
            rx_silence_counter += 1
            # Debug-Ausgabe alle 100 RX-Versuche (ca. alle 5 Sekunden bei 50ms timeout)
            if rx_silence_counter >= 100:
                yellow("[RX] Stille - kein Signal empfangen (Mode-Recheck in {})".format(50 - rx_mode_check_counter))
                rx_silence_counter = 0
    except Exception as rx_err:
        red("[RX-Exception] " + str(rx_err))
        w.feed()


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
    
    # Verifikation der Einstellungen
    try:
        actual_freq = rfm9x.frequency
        actual_bw = rfm9x.signal_bandwidth
        actual_sf = rfm9x.spreading_factor
        actual_cr = rfm9x.coding_rate
    except:
        actual_freq = RADIO_FREQ_MHZ
        actual_bw = MESHCOM_BW
        actual_sf = MESHCOM_SF
        actual_cr = MESHCOM_CR

    green(
        "LoRa OK @ {} MHz, PWR={} dBm, BW={}, SF={}, CR=4/{}".format(
            RADIO_FREQ_MHZ, config.power, MESHCOM_BW, MESHCOM_SF, MESHCOM_CR
        )
    )
    yellow("Actual: {} MHz, BW={}, SF={}, CR=4/{}".format(
        actual_freq, actual_bw, actual_sf, actual_cr
    ))
except Exception as e:
    red("LoRa INIT ERROR: " + str(e))
    while True:
        w.feed()
        time.sleep(1)


# ============================================================
# SENDESCHLEIFE
# ============================================================

yellow("Starte MeshCom-Testloop…")

# WICHTIG: Stelle sicher, dass RX aktiv ist
try:
    # Versuche verschiedene RX-Aktivierungsmethoden
    try:
        rfm9x.listen()
    except AttributeError:
        try:
            rfm9x.idle()
            time.sleep(0.1)
        except:
            pass
except Exception as e:
    yellow("RX-Init Warning: " + str(e))

yellow("Warte 5 Sekunden vor erstem Send…")
time.sleep(5)

msg_counter = 1
last_tx = time.monotonic()
rx_mode_active = True

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
        yellow("TX #{}: len={} bytes  hex={}…".format(msg_counter, len(frame), hex_preview[:80]))

        try:
            loraLED.value = True
            if config.hasPa:
                amp.value = True
                w.feed()
                time.sleep(0.25)

            # Achtung: RF_Guru-RFM9x erwartet send(watchdog, data)
            rfm9x.send(w, frame)
            w.feed()

            if config.hasPa:
                w.feed()
                time.sleep(0.1)
                amp.value = False
            
            loraLED.value = False
            
            # Nach dem Senden zurück in RX-Modus
            try:
                rfm9x.idle()
                w.feed()
                time.sleep(0.15)
                
                # Versuche RX zu aktivieren
                try:
                    rfm9x.listen()
                except AttributeError:
                    # Manche Versionen starten RX direkt bei receive()
                    pass
                
                w.feed()
            except Exception as rx_mode_err:
                red("RX-Mode-ERROR: " + str(rx_mode_err))
                w.feed()

            green("MeshCom frame sent.")
        except Exception as txerr:
            red("LoRa TX ERROR: " + str(txerr))
            w.feed()

        msg_counter = (msg_counter + 1) & 0xFFFFFFFF
        last_tx = now
    else:
        # Nicht sendezeit – RX-Modus
        check_and_receive()

    # kleine Pause, damit die CPU nicht 100% läuft
    time.sleep(0.05)
