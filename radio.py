"""Lightweight radio profile wrapper for APRS and MeshCom.

The wrapper keeps LoRa configuration values together so we can quickly
switch between APRS and MeshCom transmit parameters without duplicating
magic numbers throughout the tracker firmware.
"""


class Radio:
    def __init__(self, rfm9x, watchdog, profiles):
        self.rfm9x = rfm9x
        self.watchdog = watchdog
        self.profiles = profiles
        self.active_profile = None

    def set_profile(self, name: str):
        profile = self.profiles.get(name)
        if profile is None:
            raise ValueError("Unknown profile: {}".format(name))

        freq = profile.get("freq")
        if freq:
            # store in MHz for the CircuitPython driver
            try:
                self.rfm9x.frequency_mhz = freq / 1_000_000
            except AttributeError:
                # fallback for older driver versions
                self.rfm9x.set_frequency(freq / 1_000_000)

        bandwidth = profile.get("bw")
        if bandwidth is not None and hasattr(self.rfm9x, "signal_bandwidth"):
            self.rfm9x.signal_bandwidth = bandwidth

        spreading_factor = profile.get("sf")
        if spreading_factor is not None and hasattr(self.rfm9x, "spreading_factor"):
            self.rfm9x.spreading_factor = spreading_factor

        coding_rate = profile.get("cr")
        if coding_rate is not None and hasattr(self.rfm9x, "coding_rate"):
            self.rfm9x.coding_rate = coding_rate

        preamble_length = profile.get("preamble")
        if preamble_length is not None and hasattr(self.rfm9x, "preamble_length"):
            self.rfm9x.preamble_length = preamble_length

        sync_word = profile.get("sync_word")
        if sync_word is not None and hasattr(self.rfm9x, "sync_word"):
            self.rfm9x.sync_word = sync_word

        self.active_profile = name

    def tx(self, payload: bytes):
        self.rfm9x.send(self.watchdog, payload)


RADIO_PROFILES = {
    "aprs": {
        # APRS on 433.775 MHz
        "freq": 433_775_000,
        # APRS defaults used by the original firmware
        "bw": 125_000,
        "sf": 12,
        "cr": 5,
        "preamble": 8,
        # APRS uses the Semtech default 0x12 sync word
        "sync_word": 0x12,
    },
    "mesh": {
        # MeshCom on 433.175 MHz (TX only)
        # Parameters mirror MeshCom-Firmware defaults in configuration.h:
        #   LORA_BANDWIDTH 250 kHz, LORA_SF 11, LORA_CR 6 (4/6),
        #   LORA_PREAMBLE_LENGTH 32, SYNC_WORD_SX127x 0x2b
        "freq": 433_175_000,
        "bw": 250_000,
        "sf": 11,
        "cr": 6,
        "preamble": 32,
        "sync_word": 0x2B,
    },
}
