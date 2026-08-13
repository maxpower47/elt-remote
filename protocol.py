# Common Packet Protocol Definitions for ELT Remote System

# Command Strings / Types
CMD_PING = "PING"
CMD_ARM = "ARM"
CMD_ABORT = "ABORT"
CMD_ACTIVATE = "ACTIVATE"
CMD_STATUS = "STATUS"

# Beacon States
STATE_DISARMED = "DISARMED"
STATE_ARMED_TIMER = "ARMED_TIMER"
STATE_ACTIVE = "ACTIVE"

def format_telemetry(state, timer_remaining_sec, battery_v, lat, lon, fix_valid):
    """
    Format telemetry payload string into JSON or standard key-value format.
    Example payload format: TELEMETRY:STATE=ARMED_TIMER,REMAIN=3600,BAT=4.12,LAT=34.0522,LON=-118.2437,FIX=1
    """
    import json
    return json.dumps({
        "type": "TELEMETRY",
        "state": state,
        "remain_sec": timer_remaining_sec,
        "bat_v": round(battery_v, 2),
        "lat": round(lat, 6) if lat is not None else None,
        "lon": round(lon, 6) if lon is not None else None,
        "gps_fix": fix_valid
    })

def parse_packet(payload_str):
    """
    Parses incoming command strings or JSON packets.
    Returns (cmd_type, args_dict)
    """
    import json
    payload_str = payload_str.strip()
    if payload_str.startswith("{"):
        try:
            data = json.loads(payload_str)
            cmd_type = data.get("cmd") or data.get("type")
            return cmd_type, data
        except Exception:
            return None, {}
    else:
        parts = payload_str.split()
        if not parts:
            return None, {}
        cmd_type = parts[0].upper()
        args = parts[1:]
        return cmd_type, {"args": args}
