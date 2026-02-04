"""
MeshCom Protocol Test - Standalone Hardware Test
Similar to purity-test.py but for MeshCom protocol messages

This test sends MeshCom protocol messages on 433.175 MHz
Requires the device to be connected and flashed with CircuitPython

Usage: Copy to device as code.py to run automatically
       or run interactively via serial console
"""

import binascii
import struct
import time

import adafruit_rfm9x
import board
import busio
import digitalio
import microcontroller
import supervisor
from microcontroller import watchdog as w

import config

# Stop autoreloading
supervisor.runtime.autoreload = False

# Configure PA (Power Amplifier)
amp = digitalio.DigitalInOut(board.GP2)
amp.direction = digitalio.Direction.OUTPUT
amp.value = False

# MeshCom frequency (433.175 MHz)
RADIO_FREQ_MHZ = 433.175

# LoRa Module initialization
CS = digitalio.DigitalInOut(board.GP21)
RESET = digitalio.DigitalInOut(board.GP20)
spi = busio.SPI(board.GP18, MOSI=board.GP19, MISO=board.GP16)

print("Initializing LoRa module...")
rfm9x = adafruit_rfm9x.RFM9x(spi, CS, RESET, RADIO_FREQ_MHZ, baudrate=1000000)
rfm9x.tx_power = 23
# Configure MeshCom modulation: SF11/CR4⁄6/BW250
rfm9x.spreading_factor = 11
rfm9x.coding_rate = 6
rfm9x.signal_bandwidth = 250000
print(f"LoRa initialized at {RADIO_FREQ_MHZ} MHz")
print(f"Modulation: SF{rfm9x.spreading_factor}/CR4⁄{rfm9x.coding_rate}/BW{rfm9x.signal_bandwidth//1000}")

# MeshCom Protocol Constants
HW_ID_TLORA = 0x03  # TLora hardware ID
MOD_SF11_CR46_BW250 = 0x03
FIRMWARE_VERSION = 0x04  # Version 4.35i - Major version 4
FIRMWARE_SUB_VERSION = ord('i')  # Version 4.35i - Sub-version 'i' (0x69)

# Node ID file for persistence
NODE_ID_FILE = "/node_id.txt"

def load_or_generate_node_id():
    """
    Load node ID from file if it exists, otherwise generate and save it.
    This ensures the node ID remains consistent across reboots.
    """
    try:
        # Try to read existing node ID from file
        with open(NODE_ID_FILE, 'r') as f:
            node_id_str = f.read().strip()
            node_id = int(node_id_str, 16)
            print(f"Loaded Node ID from file: 0x{node_id:06X}")
            return node_id
    except (OSError, ValueError):
        # File doesn't exist or is invalid, generate new node ID
        uid = microcontroller.cpu.uid
        # Use last 4 bytes of UID and mask to 22 bits
        node_id = (uid[-4] << 24 | uid[-3] << 16 | uid[-2] << 8 | uid[-1]) & 0x3FFFFF
        
        # Save node ID to file for future use
        try:
            with open(NODE_ID_FILE, 'w') as f:
                f.write(f"{node_id:06X}")
            print(f"Generated and saved new Node ID: 0x{node_id:06X}")
        except OSError as e:
            print(f"Warning: Could not save node ID to file: {e}")
            print(f"Generated Node ID (not saved): 0x{node_id:06X}")
        
        return node_id

NODE_ID = load_or_generate_node_id()

# Message counter
msg_counter = 0


def encode_meshcom_hey_packet(source_call, msg_id, max_hop=5):
    """
    Encode a MeshCom protocol HEY/announcement packet
    
    This packet announces the node's presence to the MeshCom network.
    
    Format:
    Byte 0: 'H' (0x48) = HEY/announcement packet type
    Bytes 1-4: Message ID (32-bit LSB first)
    Byte 5: MAX_HOP + flags
    Bytes 6+: SOURCE callsign
    Byte N: 0x00 (terminator)
    Byte N+1: Hardware ID
    Byte N+2: Modulation ID
    Bytes N+3-4: FCS checksum (16-bit sum, big-endian)
    Byte N+5: Firmware version
    Byte N+6: Last hardware
    Byte N+7: Firmware sub-version
    Byte N+8: 0x7E (end marker)
    """
    buffer = bytearray()
    
    # Payload type - HEY packet
    buffer.append(ord('H'))
    
    # Message ID (32-bit LSB first)
    buffer.extend(struct.pack('<I', msg_id))
    
    # MAX_HOP
    hop_byte = max_hop & 0x07
    buffer.append(hop_byte)
    
    # SOURCE callsign
    buffer.extend(source_call.upper().encode('utf-8'))
    
    # Terminator
    buffer.append(0x00)
    
    # Hardware ID
    buffer.append(HW_ID_TLORA)
    
    # Modulation ID
    buffer.append(MOD_SF11_CR46_BW250)
    
    # FCS: Sum of all bytes
    fcs = sum(buffer) & 0xFFFF
    buffer.extend(struct.pack('>H', fcs))
    
    # Firmware version
    buffer.append(FIRMWARE_VERSION)
    
    # Last hardware
    buffer.append(0x00)
    
    # Firmware sub-version
    buffer.append(FIRMWARE_SUB_VERSION)
    
    # End marker
    buffer.append(0x7E)
    
    return bytes(buffer)


def encode_meshcom_message(source_call, destination, text, msg_id, max_hop=5):
    """
    Encode a MeshCom protocol text message
    
    Format:
    Byte 0: ':' (0x3A) = text message type
    Bytes 1-4: Message ID (32-bit LSB first)
    Byte 5: MAX_HOP + flags
    Bytes 6+: SOURCE>DESTINATION: text
    Byte N: 0x00 (terminator)
    Byte N+1: Hardware ID
    Byte N+2: Modulation ID
    Bytes N+3-4: FCS checksum (16-bit sum, big-endian)
    Byte N+5: Firmware version
    Byte N+6: Last hardware
    Byte N+7: Firmware sub-version
    Byte N+8: 0x7E (end marker)
    """
    buffer = bytearray()
    
    # Payload type
    buffer.append(ord(':'))
    
    # Message ID (32-bit LSB first)
    buffer.extend(struct.pack('<I', msg_id))
    
    # MAX_HOP
    hop_byte = max_hop & 0x07
    buffer.append(hop_byte)
    
    # APRS path: SOURCE>DESTINATION: text
    aprs_line = f"{source_call.upper()}>{destination.upper()}: {text}"
    buffer.extend(aprs_line.encode('utf-8'))
    
    # Terminator
    buffer.append(0x00)
    
    # Hardware ID
    buffer.append(HW_ID_TLORA)
    
    # Modulation ID
    buffer.append(MOD_SF11_CR46_BW250)
    
    # FCS: Sum of all bytes
    fcs = sum(buffer) & 0xFFFF
    buffer.extend(struct.pack('>H', fcs))
    
    # Firmware version
    buffer.append(FIRMWARE_VERSION)
    
    # Last hardware
    buffer.append(0x00)
    
    # Firmware sub-version
    buffer.append(FIRMWARE_SUB_VERSION)
    
    # End marker
    buffer.append(0x7E)
    
    return bytes(buffer)


def generate_msg_id():
    """
    Generate a unique message ID using MeshCom format
    Format: (NODE_ID << 10) | (counter & 0x3FF)
    - Bits 31-10: Node ID (22 bits) - unique per device
    - Bits 9-0: Message counter (10 bits) - 0-999
    """
    global msg_counter
    msg_counter = (msg_counter + 1) % 1000
    # MeshCom format: (MAC_address & 0x3FFFFF) << 10 | (counter & 0x3FF)
    return (NODE_ID << 10) | (msg_counter & 0x3FF)


# Test configuration
SOURCE_CALL = "DN9APW-8"
DESTINATION = "*"  # Broadcast
TEST_MESSAGE = "MeshCom protocol test from RF.Guru TLora"

print(f"\nMeshCom Protocol Test")
print(f"Source: {SOURCE_CALL}")
print(f"Destination: {DESTINATION}")
print(f"Frequency: {RADIO_FREQ_MHZ} MHz")
print(f"Message: {TEST_MESSAGE}")

# Send HEY packet at startup to announce presence
print("\n=== Sending initial HEY packet to announce node ===")
try:
    # Generate message ID for HEY packet
    hey_msg_id = generate_msg_id()
    
    # Encode HEY packet
    hey_packet = encode_meshcom_hey_packet(SOURCE_CALL, hey_msg_id)
    
    # Display HEY packet info
    print(f"HEY Packet:")
    print(f"  MSG_ID: 0x{hey_msg_id:08X}")
    print(f"  Size: {len(hey_packet)} bytes")
    print(f"  Hex: {hey_packet.hex()}")
    
    # Enable PA if configured
    if config.hasPa:
        amp.value = True
        time.sleep(0.1)
    
    # Send HEY packet via LoRa
    rfm9x.send(w, hey_packet)
    
    # Disable PA if configured
    if config.hasPa:
        amp.value = False
    
    print("  Status: HEY packet sent OK")
    print("=== Node announced to MeshCom network ===\n")
    
    # Wait a moment before starting regular transmissions
    time.sleep(2)
    
except Exception as e:
    print(f"Error sending HEY packet: {e}")
    if config.hasPa:
        amp.value = False

print("\nSending messages every 5 seconds...")
print("Press Ctrl+C to stop\n")

# Main test loop
while True:
    try:
        # Generate message ID
        msg_id = generate_msg_id()
        
        # Encode MeshCom message
        meshcom_packet = encode_meshcom_message(
            SOURCE_CALL,
            DESTINATION,
            TEST_MESSAGE,
            msg_id
        )
        
        # Display packet info
        print(f"[{time.monotonic():.1f}s] Sending MeshCom message #{msg_counter}")
        print(f"  MSG_ID: 0x{msg_id:08X}")
        print(f"  Size: {len(meshcom_packet)} bytes")
        print(f"  Hex: {meshcom_packet.hex()}")
        
        # Enable PA if configured
        if config.hasPa:
            amp.value = True
            time.sleep(0.1)
        
        # Send via LoRa
        rfm9x.send(w, meshcom_packet)
        
        # Disable PA if configured
        if config.hasPa:
            amp.value = False
        
        print("  Status: Sent OK\n")
        
        # Wait before next transmission
        time.sleep(5)
        
    except KeyboardInterrupt:
        print("\nTest stopped by user")
        if config.hasPa:
            amp.value = False
        break
    except Exception as e:
        print(f"Error: {e}")
        if config.hasPa:
            amp.value = False
        time.sleep(1)
