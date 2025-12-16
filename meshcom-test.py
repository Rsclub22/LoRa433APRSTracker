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
import supervisor
from microcontroller import watchdog as w

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
print(f"LoRa initialized at {RADIO_FREQ_MHZ} MHz")

# MeshCom Protocol Constants
HW_ID_RP2040 = 0x0A
MOD_SF11_CR46_BW250 = 0x03
FIRMWARE_VERSION = 0x01
FIRMWARE_SUB_VERSION = ord('#')

# Message counter
msg_counter = 0


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
    buffer.append(HW_ID_RP2040)
    
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
    """Generate a unique message ID"""
    global msg_counter
    msg_counter = (msg_counter + 1) % 1000
    timestamp_part = (int(time.monotonic()) & 0x3FFFFF) << 10
    return timestamp_part | (msg_counter & 0x3FF)


# Test configuration
SOURCE_CALL = "TEST-1"
DESTINATION = "*"  # Broadcast
TEST_MESSAGE = "MeshCom protocol test from RF.Guru RP2040"

print(f"\nMeshCom Protocol Test")
print(f"Source: {SOURCE_CALL}")
print(f"Destination: {DESTINATION}")
print(f"Frequency: {RADIO_FREQ_MHZ} MHz")
print(f"Message: {TEST_MESSAGE}")
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
        
        # Enable PA
        amp.value = True
        time.sleep(0.1)
        
        # Send via LoRa
        rfm9x.send(w, meshcom_packet)
        
        # Disable PA
        amp.value = False
        
        print("  Status: Sent OK\n")
        
        # Wait before next transmission
        time.sleep(5)
        
    except KeyboardInterrupt:
        print("\nTest stopped by user")
        amp.value = False
        break
    except Exception as e:
        print(f"Error: {e}")
        amp.value = False
        time.sleep(1)
