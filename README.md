# ESP-Pixhawk-WW: ESP32 Companion Computer for Pixhawk

## Motivation

This project connects an ESP32 microcontroller to a Pixhawk flight controller over MAVLink serial, creating a WiFi-accessible web interface for drone control. The ESP32 acts as a wireless bridge — it creates a WiFi access point ("Pixhawk-ESP32") and serves a web page at `http://192.168.4.1` that displays:

- Real-time attitude data (roll, pitch, yaw)
- Flight mode and arm state
- Arm/Disarm/Mode change controls
- Status logs

The idea: control a Pixhawk-based drone from a phone or laptop without needing Mission Planner, a telemetry radio, or a full GCS setup. Just connect to the ESP32's WiFi and open a browser.

A custom lightweight MAVLink library (`mavespstm`) was written from scratch to avoid pulling in the full MAVLink C headers, keeping the build lean for ESP32.

### Hardware Setup

- **ESP32** (any dev board with UART2 available)
- **Pixhawk** (tested with ArduPilot firmware)
- **Wiring**: ESP32 GPIO16 (TX) → Pixhawk TELEM2 RX, ESP32 GPIO17 (RX) → Pixhawk TELEM2 TX
- **Baud**: 57600 (must match `SERIAL2_BAUD` on Pixhawk)
- **Pixhawk config**: `SERIAL2_PROTOCOL = 2` (MAVLink2)

---

## Issues Encountered & Solutions

### Issue 1: Receiving Telemetry — SOLVED

**Problem**: Initially couldn't receive any MAVLink data from the Pixhawk.

**Root cause**: UART pin configuration and baud rate mismatch.

**Solution**: Set UART2 with correct TX/RX pins and matched baud rate to Pixhawk's TELEM2 port (57600). Pixhawk streams telemetry on configured serial ports automatically, so once the UART was correct, heartbeat and attitude data started flowing.

---

### Issue 2: Garbage Characters in STATUSTEXT — SOLVED

**Problem**: STATUSTEXT messages showed garbage after the actual text:
```
STATUSTEXT: [INFO] Mission Planner 1.3.838·7ÃÃÃd
```

**Root cause**: MAVLink's STATUSTEXT field is a fixed 50-byte buffer. If the actual message is shorter (e.g., 25 chars), the remaining bytes contain random data. Using `strncpy` copies all 50 bytes including the garbage.

**Solution**: Stop copying at the first non-printable character (ASCII < 32 or > 126):
```c
for (int i = 0; i < 50 && text.text[i] != '\0'; i++) {
    if (text.text[i] < 32 || text.text[i] > 126) break;
    safe_text[len++] = text.text[i];
}
```

---

### Issue 3: System ID Conflict — SOLVED

**Problem**: `COMPANION_SYSTEM_ID` was set to `1`, which is the same as `PIXHAWK_SYSTEM_ID` (1).

**Root cause**: ArduPilot ignores/drops messages that appear to come from its own system ID. With both set to 1, the Pixhawk treated our commands as its own echoed messages and discarded them.

**Solution**: Changed `COMPANION_SYSTEM_ID` to `200`. Must be different from the Pixhawk's system ID.

---

### Issue 4: No COMMAND_ACK Responses — FIXED

**Problem**: When sending ARM, DISARM, or mode change commands from the web interface, the Pixhawk never responds with a COMMAND_ACK. Commands have zero effect. No error messages, no rejections — just silence.

**Root cause: CRC invalidation by sequence number override.**

This was **the critical bug** that made ALL outgoing commands fail silently:

```
                    ┌─────────────────────────────────────┐
                    │  EVERY pack function in mavespstm:  │
                    │                                     │
                    │  buf[4] = 0;  // seq = 0            │
                    │  ...                                 │
                    │  CRC = crc(buf[1] ... buf[N]);      │
                    │  // CRC computed with buf[4] = 0    │
                    └─────────────────────────────────────┘
                                     │
                                     ▼
                    ┌─────────────────────────────────────┐
                    │  send_mavlink_message():            │
                    │                                     │
                    │  buf[4] = tx_seq++;                  │
                    │  // buf[4] is NOW NON-ZERO!         │
                    │  // But CRC was computed with 0!    │
                    │  uart_write_bytes(buf, len);        │
                    └─────────────────────────────────────┘
                                     │
                                     ▼
                    ┌─────────────────────────────────────┐
                    │  Pixhawk receives message:          │
                    │                                     │
                    │  Computes CRC over received bytes   │
                    │  (with buf[4] = tx_seq, NOT 0)      │
                    │  CRC MISMATCH → SILENT DROP         │
                    └─────────────────────────────────────┘
```

MAVLink's CRC covers bytes 1 through end of payload (header + payload), which **includes byte 4 (the sequence number)**. Our pack functions compute the CRC with `seq=0`, but `send_mavlink_message()` then overwrites `buf[4]` with an incrementing sequence counter **after** the CRC is already baked into the packet. This means:

- **1st message** (tx_seq=0): CRC valid ✓ (buf[4] stays 0)
- **2nd message** (tx_seq=1): CRC INVALID ✗
- **Every subsequent message**: CRC INVALID ✗

The Pixhawk silently drops all messages with bad CRC. No ACK, no error, nothing.

**Why we still received telemetry**: The Pixhawk streams telemetry on configured serial ports regardless of whether it receives valid data from us. So attitude, heartbeats, and STATUSTEXT always worked — only our **outgoing** commands were getting dropped.

**Fix applied**: `send_mavlink_message()` now sets `buf[4] = tx_seq++` **first**, then recomputes the CRC from scratch using `mavlink_get_crc_extra()` before sending. This ensures the CRC always matches the actual packet bytes.

---

### Issue 5: Heartbeat Type — FIXED

**Problem**: We sent heartbeats with `MAV_TYPE_ONBOARD_CONTROLLER` (18).

**Root cause**: ArduPilot tracks "known GCS" stations by their heartbeat type. It grants full command authority (arm, disarm, mode changes, param sets) to devices identifying as `MAV_TYPE_GCS` (6). Devices identifying as type 18 (onboard controller) may have restricted permissions depending on firmware version.

**Solution applied**: Changed heartbeat type to `MAV_TYPE_GCS` (6).

**Is this safe?** Yes:
- `MAV_TYPE` is just an identity label in the heartbeat, not a security key
- MAVLink has no authentication — there's no password or encryption
- Multiple GCS devices coexist peacefully (Mission Planner uses sysid=255, we use sysid=200)
- Each GCS is tracked separately by its `(sysid, compid)` pair
- Our ESP32 (sysid=200) is a **secondary GCS** — `SYSID_MYGCS` defaults to 255 (Mission Planner), so if our ESP32 disconnects, no failsafe triggers (which is safer)

**How multiple GCS coexist**:
```
Mission Planner:  sysid=255, type=GCS(6)  ← Primary GCS (SYSID_MYGCS=255)
ESP32:            sysid=200, type=GCS(6)  ← Secondary GCS (our device)
Pixhawk:          sysid=1,   type=QUADROTOR(2)
```

---

### Issue 6: target_component — FIXED

**Problem**: We targeted `target_component=1` (specific component) for all commands.

**Root cause**: Per ArduPilot wiki documentation on COMMAND_LONG: *"Component ID of flight controller or just 0"*. Using `target_component=0` broadcasts to all components and is more reliable.

**Solution applied**: Changed all commands (ARM, PARAM_SET, RC_CHANNELS_OVERRIDE) to use `target_component=0`.

---

### Issue 7: SYSID_MYGCS Parameter — Monitored

**Context**: ArduPilot has a parameter `SYSID_MYGCS` (default 255) that specifies which system ID is the primary GCS. This mainly affects failsafe behavior — if the Pixhawk stops receiving heartbeats from this sysid, it triggers GCS failsafe.

**Current status**: Our ESP32 uses sysid=200 as a secondary GCS. This means:
- It can send commands with full authority (since we identify as MAV_TYPE_GCS)
- It does NOT trigger GCS failsafe if disconnected (safer for testing)
- If needed for maximum compatibility, we could change to sysid=255

---

### Issue 8: Arming Pre-Requisites — Implemented

**Problem**: Even with valid commands, arming may fail due to pre-arm safety checks.

**Context from RPi Python script that successfully arms**:
```python
set_param(master, 'ARMING_CHECK', 0)    # Disable all pre-arm checks
set_param(master, 'FS_THR_ENABLE', 0)   # Disable throttle failsafe
set_param(master, 'FS_GCS_ENABLE', 0)   # Disable GCS failsafe
master.mav.rc_channels_override_send(... chan3=1000 ...)  # Throttle low
master.arducopter_arm()                  # Normal arm
# If that fails:
command_long_send(... param2=21196 ...)  # Force arm (magic number)
```

**ArduPilot documentation states** (from wiki):
- ARM: `COMMAND_LONG` with `command=400`, `param1=1`, `param2=0`
- FORCE ARM: `COMMAND_LONG` with `command=400`, `param1=1`, `param2=21196`
- `target_component` can be `0` (broadcast) or the specific component ID

**Solution implemented in web-arming.c**:
- **SETUP button**: Sends PARAM_SET to disable `ARMING_CHECK=0`, `FS_THR_ENABLE=0`, `FS_GCS_ENABLE=0`
- **ARM button**: Sends RC_CHANNELS_OVERRIDE (throttle low) + normal arm command
- **FORCE ARM button**: Sends RC_CHANNELS_OVERRIDE (throttle low) + force arm with `param2=21196`
- **Mode dropdown**: Uses SET_MODE message (MSG_ID=11), same as pymavlink's `set_mode()`

---

## File Structure

```
├── main/
│   ├── web-arming.c          # Current main file (web interface + arming)
│   ├── webassistance.c       # Original version (has the bugs documented above)
│   ├── heartbeat.c           # Early test: heartbeat only
│   └── attitude--11.c        # Early test: attitude display
├── components/
│   └── mavespstm/            # Custom lightweight MAVLink library
│       └── include/
│           ├── mavespstm.h
│           └── mavespstm/
│               ├── mavlink_types.h      # Constants, message IDs, structs
│               ├── mavlink_checksum.h   # X.25 CRC implementation
│               ├── mavlink_messages.h   # Pack/decode functions
│               └── mavlink_parser.h     # Byte-stream parser
├── CMakeLists.txt
└── README.md
```

## Next Steps

1. **Build, flash, and test** — CRC bug is fixed, heartbeat type changed to GCS, target_component set to broadcast
2. **Verify COMMAND_ACK** — After flashing, press SETUP on web UI and check for PARAM_VALUE responses (proves CRC fix works)
3. **Test ARM sequence** — SETUP → select STABILIZE → ARM (or FORCE ARM if needed)
4. **If still no ACKs** — Add hex dump logging to compare raw packet bytes against pymavlink output
5. **Consider sysid=255** — If ArduPilot still restricts some commands, try matching the SYSID_MYGCS default
