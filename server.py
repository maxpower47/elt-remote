import http.server
import socketserver
import json
import threading
import time
import urllib.parse
import serial

PORT = 8080
SERIAL_PORT = "/dev/ttyUSB0"
BAUD_RATE = 115200

# Global state store
latest_telemetry = {
    "type": "TELEMETRY",
    "state": "UNKNOWN",
    "batt": 0.0,
    "gps": {"lat": 0.0, "lon": 0.0, "valid": False},
    "rssi": 0,
    "snr": 0,
    "last_seen_sec": 9999,
    "packet_count": 0,
    "updated_at": 0
}

clients_sse = []
serial_conn = None

def serial_reader_thread():
    global latest_telemetry, serial_conn
    while True:
        try:
            print(f"[SERIAL] Connecting to {SERIAL_PORT} @ {BAUD_RATE}...")
            serial_conn = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
            print("[SERIAL] Connected OK.")
            
            packet_count = 0
            while True:
                line = serial_conn.readline().decode('utf-8', errors='ignore').strip()
                if not line:
                    continue
                
                # Check for RX payload line from Heltec V3
                if "[Heltec V3 RX]" in line:
                    try:
                        json_str = line.split("[Heltec V3 RX]")[1].strip()
                        data = json.loads(json_str)
                        packet_count += 1
                        
                        latest_telemetry["type"] = data.get("type", "TELEMETRY")
                        latest_telemetry["state"] = data.get("state", "UNKNOWN")
                        latest_telemetry["batt"] = data.get("batt", 0.0)
                        latest_telemetry["remaining_sec"] = data.get("remaining_sec", 0)
                        
                        if "gps" in data:
                            latest_telemetry["gps"] = data["gps"]
                        
                        latest_telemetry["packet_count"] = packet_count
                        latest_telemetry["updated_at"] = time.time()
                        
                        print(f"[RX UPDATE] State: {latest_telemetry['state']} | Batt: {latest_telemetry['batt']}V | GPS: {latest_telemetry['gps']}")
                    except Exception as parse_err:
                        print(f"[PARSE ERR] {parse_err} | Line: {line}")
                        
        except Exception as e:
            print(f"[SERIAL ERR] {e}. Retrying in 3 seconds...")
            time.sleep(3)

def send_serial_cmd(cmd_dict):
    global serial_conn
    if serial_conn and serial_conn.is_open:
        msg = json.dumps(cmd_dict) + "\n"
        serial_conn.write(msg.encode('utf-8'))
        print(f"[TX COMMAND SENT] {msg.strip()}")
        return True
    return False

class DashboardHTTPHandler(http.server.SimpleHTTPRequestHandler):
    def do_GET(self):
        parsed = urllib.parse.urlparse(self.path)
        if parsed.path == "/api/telemetry":
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            
            # Compute real-time age
            copy_data = dict(latest_telemetry)
            if copy_data["updated_at"] > 0:
                copy_data["last_seen_sec"] = int(time.time() - copy_data["updated_at"])
            else:
                copy_data["last_seen_sec"] = 9999
                
            self.wfile.write(json.dumps(copy_data).encode('utf-8'))
        else:
            super().do_GET()

    def do_POST(self):
        parsed = urllib.parse.urlparse(self.path)
        if parsed.path == "/api/command":
            content_length = int(self.headers.get('Content-Length', 0))
            body = self.rfile.read(content_length)
            try:
                cmd_data = json.loads(body.decode('utf-8'))
                success = send_serial_cmd(cmd_data)
                
                self.send_response(200 if success else 500)
                self.send_header("Content-Type", "application/json")
                self.send_header("Access-Control-Allow-Origin", "*")
                self.end_headers()
                self.wfile.write(json.dumps({"status": "OK" if success else "SERIAL_ERROR"}).encode('utf-8'))
            except Exception as err:
                self.send_response(400)
                self.end_headers()
                self.wfile.write(json.dumps({"error": str(err)}).encode('utf-8'))
        else:
            self.send_error(404)

if __name__ == "__main__":
    t = threading.Thread(target=serial_reader_thread, daemon=True)
    t.start()
    
    with socketserver.TCPServer(("", PORT), DashboardHTTPHandler) as httpd:
        print(f"[DASHBOARD SERVER] Running on http://localhost:{PORT}")
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            print("\nShutting down server.")
