"""MeshCom packet builder compatible with MeshCom-Firmware.

This module follows the APRS-compatible packet layout implemented in
`src/aprs_functions.cpp` and the position payload generator in
`src/loop_functions.cpp` (function `PositionToAPRS`) from the
MeshCom-Firmware project (https://github.com/Rsclub22/MeshCom-Firmware).

Only the fields required for broadcasting a position/status packet are
implemented to keep the CircuitPython footprint small while ensuring the
on-air frame matches the C++ implementation byte-for-byte.
"""

# MeshCom defaults from configuration.h (RadioLib settings)
MAX_HOP_POS_DEFAULT = 2
MESH_DEFAULT_MOD = 3  # SF11 / CR 4/6 / BW 250k (set in initAPRS)
HW_DEFAULT = 4  # Matches common BOARD_HARDWARE value in MeshCom firmware
FW_VERSION_DEFAULT = 0  # no public versioning here
FW_SUBVERSION_SENTINEL = 0x23  # '#' when subversion is 0 in C++ code

_mesh_sequence = 0


def _parse_node_id(node_id):
    """Return a 32-bit integer node identifier.

    MeshCom-Firmware composes msg_id from a 22-bit gateway ID and a 10-bit
    counter. For this TX-only port we accept decimal or hex values and fit
    them into a 32-bit field.
    """

    if isinstance(node_id, int):
        value = node_id
    else:
        text = str(node_id).strip()
        base = 16 if text.lower().startswith("0x") else 10
        try:
            value = int(text, base)
        except ValueError:
            value = 0

    return value & 0xFFFFFFFF


def _coord_to_aprs(dec_coord):
    """Convert decimal degrees to APRS ddmm.mm format used by MeshCom."""

    degrees = int(abs(dec_coord))
    minutes = (abs(dec_coord) - degrees) * 60.0
    return (degrees * 100.0) + minutes


def _build_payload(fix, symbol):
    """Render the APRS-style position payload used by MeshCom."""

    lat = fix.get("lat")
    lon = fix.get("lon")
    altitude = fix.get("altitude")
    if lat is None or lon is None:
        raise ValueError("Latitude and longitude are required for MeshCom payload")

    lat_c = "N" if lat >= 0 else "S"
    lon_c = "E" if lon >= 0 else "W"

    lat_val = _coord_to_aprs(lat)
    lon_val = _coord_to_aprs(lon)

    symbol_table = symbol[0] if symbol and len(symbol) > 0 else "/"
    symbol_code = symbol[1] if symbol and len(symbol) > 1 else ">"

    segments = [
        f"{lat_val:07.2f}{lat_c}{symbol_table}{lon_val:08.2f}{lon_c}{symbol_code}",
    ]

    if altitude is not None:
        alt_feet = int(altitude * 3.2808399)
        segments.append("/A={:06d}".format(max(0, alt_feet)))

    return "".join(segments)


def _next_sequence():
    """Return a 10-bit rolling sequence like node_msgid in MeshCom."""

    global _mesh_sequence
    value = _mesh_sequence & 0x3FF
    _mesh_sequence = (value + 1) & 0x3FF
    return value


def _encode_header(buffer, payload_type, msg_id, max_hop, source, destination):
    """Port of encodeStartAPRS from aprs_functions.cpp."""

    buffer.append(payload_type)
    buffer.extend(
        [
            msg_id & 0xFF,
            (msg_id >> 8) & 0xFF,
            (msg_id >> 16) & 0xFF,
            (msg_id >> 24) & 0xFF,
        ]
    )

    buffer.append(max_hop & 0xFF)

    path = "{}>{}{}".format(source, destination, chr(payload_type))
    buffer.extend(path.encode("ascii"))


def _append_payload(buffer, payload_text):
    """Append payload bytes (encodePayloadAPRS)."""

    buffer.extend(payload_text.encode("ascii"))


def _finalize(buffer, source_hw, source_mod, fw_version, last_hw, fw_subversion):
    """Add checksum and trailer exactly like encodeAPRS."""

    buffer.append(0x00)
    buffer.append(source_hw & 0xFF)
    buffer.append(source_mod & 0xFF)

    fcs_sum = sum(buffer)
    buffer.extend([(fcs_sum >> 8) & 0xFF, fcs_sum & 0xFF])

    buffer.append(fw_version & 0xFF)
    buffer.append(last_hw & 0xFF)

    if fw_subversion == 0:
        buffer.append(FW_SUBVERSION_SENTINEL)
    else:
        buffer.append(fw_subversion & 0xFF)

    buffer.append(0x7E)


def build_mesh_position_frame(fix, node_id, callsign=None, symbol="/"):
    """Build a MeshCom position packet matching the C++ firmware layout.

    Parameters mirror the upstream `sendPosition`/`encodeAPRS` flow:
    - `fix`: dict with at least `lat`, `lon`, optional `altitude` and
      `timestamp` (ignored for now).
    - `node_id`: numeric or string identifier used to seed the msg_id.
    - `callsign`: source path (defaults to "NOCALL" if not provided).
    - `symbol`: two-character APRS symbol string.
    """

    gateway_id = _parse_node_id(node_id) & 0x3FFFFF
    msg_id = (gateway_id << 10) | _next_sequence()
    source_call = (callsign or "NOCALL").upper()
    payload_text = _build_payload(fix, symbol)

    buffer = bytearray()
    _encode_header(
        buffer,
        payload_type=0x21,  # '!'
        msg_id=msg_id,
        max_hop=MAX_HOP_POS_DEFAULT | 0x10,  # mesh flag set in encodeStartAPRS
        source=source_call,
        destination="*",
    )

    _append_payload(buffer, payload_text)

    _finalize(
        buffer,
        source_hw=HW_DEFAULT,
        source_mod=MESH_DEFAULT_MOD,
        fw_version=FW_VERSION_DEFAULT,
        last_hw=HW_DEFAULT,
        fw_subversion=0,
    )

    return bytes(buffer)

