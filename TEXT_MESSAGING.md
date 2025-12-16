# Text Messaging Feature

The LoRa APRS Tracker now supports simple text messaging similar to MeshCom firmware. This allows you to send and receive short text messages through the LoRa radio.

## Features

- **Send text messages** via USB serial console
- **Receive text messages** from other stations
- **Automatic acknowledgments** for directed messages
- **Broadcast messages** to all stations
- **Message queue** for pending outgoing messages
- **Compatible with APRS message format**

## Configuration

Enable or disable text messaging in `config.py`:

```python
# Text messaging
textMessaging = True        # Enable/disable text messaging feature
messageQueueSize = 10       # Max number of pending outgoing messages
```

## Sending Messages

Connect to the device via USB serial (e.g., using `tio`, `screen`, or PuTTY) and use the following format:

### Broadcast Message (to all stations)
```
MSG:*:Hello everyone!
```

### Direct Message (to specific callsign)
```
MSG:CALL-1:Hello John, how are you?
```

### Examples
```
MSG:*:Testing LoRa messaging
MSG:OE3XYZ:Please QSY to 144.800
MSG:OE3XYZ-5:Message for mobile station
```

## Message Format

Messages follow the APRS message format:
- Maximum message length: 67 characters
- Callsign format: CALL or CALL-SSID
- Use `*` or `CQ` for broadcast messages

## Receiving Messages

When a message is received that is addressed to your station or broadcast:

```
==================================================
MESSAGE from OE3ABC-5
To: OE3XYZ
Text: Hello, how are you?
ID: 123
==================================================
```

For directed messages (not broadcasts), the tracker automatically sends an acknowledgment (ACK) to the sender.

## Message Protocol

The implementation uses standard APRS message format:
- Message: `:ADDRESSEE:message text{msg_id`
- Acknowledgment: `:ADDRESSEE:ack{msg_id`

Where:
- `ADDRESSEE` is padded to 9 characters
- `msg_id` is a unique message identifier
- Messages are sent with LoRa header: `<\xFF\x01`

## Technical Details

### Message Queue
- Messages are queued when sent via serial
- Queue size is configurable (default: 10 messages)
- Oldest messages are dropped if queue is full
- Messages are sent between GPS updates

### Receive Mode
- Radio checks for incoming packets between GPS updates
- Non-blocking receive with 0.1s timeout
- Packets are decoded and parsed for APRS messages
- Messages not addressed to this station are ignored (but could be relayed in future)

### Compatibility
- Compatible with other APRS messaging stations
- Works with LoRa APRS iGates
- Message format follows APRS specification
- Can interoperate with MeshCom-style text messaging

## Serial Connection

### Using TIO (recommended)
```bash
tio /dev/ttyACM0
```

### Using Screen
```bash
screen /dev/ttyACM0 115200
```

### Using PuTTY
- Connection type: Serial
- Serial line: COM3 (Windows) or /dev/ttyACM0 (Linux)
- Speed: 115200

## LED Indicators

- **LoRa LED** flashes when transmitting or receiving messages
- **GPS LED** indicates GPS fix status
- **Power LED** indicates device is powered

## Troubleshooting

### Messages not sending
- Check that `textMessaging = True` in config.py
- Verify callsign is set correctly (not "--CALL--")
- Ensure LoRa module is initialized (check console output)
- Check message format: `MSG:DESTINATION:text`

### Messages not receiving
- Other station must be on same frequency (433.775 MHz)
- Same LoRa parameters must be used
- Check for "RX:" messages in console (if fullDebug = True)

### Serial input not working
- Verify USB data port is enabled in boot.py
- Check serial connection settings
- Try different serial terminal program

## Future Enhancements

Possible future improvements:
- Message retransmission on missing ACK
- Message history/storage
- Configurable retry attempts
- Message relay/digipeater function
- Read confirmation tracking
- Group messaging support
- Message filtering by callsign

## Notes

- Text messaging increases battery usage due to active receiving
- Messages are transmitted between position beacons
- Long messages are automatically truncated to 67 characters
- The message queue prevents overwhelming the radio
- Each message transmission takes approximately 1-2 seconds
