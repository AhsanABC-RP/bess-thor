#!/usr/bin/env python3
import os
import time
import urllib3
import requests
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import NavSatFix, NavSatStatus
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from std_msgs.msg import Float32, Int32, String

urllib3.disable_warnings(urllib3.exceptions.InsecureRequestWarning)


class RutxTelemetryNode(Node):
    def __init__(self):
        super().__init__("rutx_telemetry")
        self.declare_parameter("host", "192.168.1.1")
        self.declare_parameter("username", "admin")
        self.declare_parameter("password", "J^v45dL=")
        self.declare_parameter("poll_hz", 1.0)
        self.declare_parameter("frame_id", "rutx_gps")

        self.host = self.get_parameter("host").value
        self.user = self.get_parameter("username").value
        self.pw = self.get_parameter("password").value
        self.frame = self.get_parameter("frame_id").value
        period = 1.0 / float(self.get_parameter("poll_hz").value)

        self.base = f"https://{self.host}/api"
        self.session = requests.Session()
        self.session.verify = False
        self.token = None

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.pub_fix = self.create_publisher(NavSatFix, "/rutx/gps/fix", qos)
        self.pub_diag = self.create_publisher(
            DiagnosticArray, "/diagnostics", qos
        )
        self.pub_rsrp = self.create_publisher(Float32, "/rutx/mobile/rsrp", qos)
        self.pub_rsrq = self.create_publisher(Float32, "/rutx/mobile/rsrq", qos)
        self.pub_sinr = self.create_publisher(Float32, "/rutx/mobile/sinr", qos)
        self.pub_rssi = self.create_publisher(Float32, "/rutx/mobile/rssi", qos)
        self.pub_band = self.create_publisher(String, "/rutx/mobile/band", qos)
        self.pub_conn = self.create_publisher(String, "/rutx/mobile/conntype", qos)
        self.pub_cell = self.create_publisher(Int32, "/rutx/mobile/cellid", qos)
        self.pub_op = self.create_publisher(String, "/rutx/mobile/operator", qos)

        self.timer = self.create_timer(period, self.tick)
        self.get_logger().info(
            f"rutx_telemetry polling {self.host} at {1.0/period:.1f} Hz"
        )

    def login(self):
        try:
            r = self.session.post(
                f"{self.base}/login",
                json={"username": self.user, "password": self.pw},
                timeout=5,
            )
            j = r.json()
            if j.get("success"):
                self.token = j["data"]["token"]
                self.session.headers["Authorization"] = f"Bearer {self.token}"
                self.get_logger().info("RUTX50 login OK")
                return True
            self.get_logger().warn(f"RUTX50 login failed: {j}")
        except Exception as e:
            self.get_logger().warn(f"RUTX50 login exception: {e}")
        return False

    def api_get(self, path):
        if self.token is None and not self.login():
            return None
        try:
            r = self.session.get(f"{self.base}/{path}", timeout=5)
            if r.status_code == 401:
                self.token = None
                if self.login():
                    r = self.session.get(f"{self.base}/{path}", timeout=5)
                else:
                    return None
            j = r.json()
            if j.get("success"):
                return j.get("data")
        except Exception as e:
            self.get_logger().warn(f"GET {path} failed: {e}")
        return None

    @staticmethod
    def _num(v, default=float("nan")):
        try:
            return float(v)
        except (TypeError, ValueError):
            return default

    def tick(self):
        modem = self.api_get("modems/status")
        gps = self.api_get("gps/position/status")
        stamp = self.get_clock().now().to_msg()

        if isinstance(modem, list) and modem:
            m = modem[0]
            rsrp = self._num(m.get("rsrp"))
            rsrq = self._num(m.get("rsrq"))
            sinr = self._num(m.get("sinr"))
            rssi = self._num(m.get("rssi"))
            band = str(m.get("band", ""))
            conntype = str(m.get("conntype", ""))
            operator = str(m.get("operator", ""))
            cellid = int(self._num(m.get("cellid"), 0))
            netstate = str(m.get("netstate", ""))
            temperature = self._num(m.get("temperature"))

            self.pub_rsrp.publish(Float32(data=rsrp))
            self.pub_rsrq.publish(Float32(data=rsrq))
            self.pub_sinr.publish(Float32(data=sinr))
            self.pub_rssi.publish(Float32(data=rssi))
            self.pub_band.publish(String(data=band))
            self.pub_conn.publish(String(data=conntype))
            self.pub_cell.publish(Int32(data=cellid))
            self.pub_op.publish(String(data=operator))

            diag = DiagnosticArray()
            diag.header.stamp = stamp
            st = DiagnosticStatus()
            st.name = "rutx_mobile"
            st.hardware_id = f"rutx50:{operator}"
            if rsrp >= -95:
                st.level, st.message = DiagnosticStatus.OK, "Excellent"
            elif rsrp >= -105:
                st.level, st.message = DiagnosticStatus.OK, "Good"
            elif rsrp >= -115:
                st.level, st.message = DiagnosticStatus.WARN, "Fair"
            else:
                st.level, st.message = DiagnosticStatus.ERROR, "Poor"
            st.values = [
                KeyValue(key="operator", value=operator),
                KeyValue(key="conntype", value=conntype),
                KeyValue(key="band", value=band),
                KeyValue(key="netstate", value=netstate),
                KeyValue(key="rsrp_dbm", value=str(rsrp)),
                KeyValue(key="rsrq_db", value=str(rsrq)),
                KeyValue(key="sinr_db", value=str(sinr)),
                KeyValue(key="rssi_dbm", value=str(rssi)),
                KeyValue(key="cellid", value=str(cellid)),
                KeyValue(key="modem_temp_c", value=str(temperature)),
                KeyValue(key="rxbytes", value=str(m.get("rxbytes", 0))),
                KeyValue(key="txbytes", value=str(m.get("txbytes", 0))),
            ]
            diag.status.append(st)
            self.pub_diag.publish(diag)

        if isinstance(gps, dict):
            fix = NavSatFix()
            fix.header.stamp = stamp
            fix.header.frame_id = self.frame
            lat = self._num(gps.get("latitude"), 0.0)
            lon = self._num(gps.get("longitude"), 0.0)
            alt = self._num(gps.get("altitude"), 0.0)
            sats = int(self._num(gps.get("satellites"), 0))
            fix_status = str(gps.get("fix_status", ""))
            accuracy = self._num(gps.get("accuracy"), 0.0)

            fix.latitude = lat
            fix.longitude = lon
            fix.altitude = alt
            fix.status.service = NavSatStatus.SERVICE_GPS
            if fix_status in ("2", "3") and (lat != 0.0 or lon != 0.0):
                fix.status.status = NavSatStatus.STATUS_FIX
            else:
                fix.status.status = NavSatStatus.STATUS_NO_FIX
            sigma = max(accuracy, 1.0)
            fix.position_covariance = [
                sigma * sigma, 0.0, 0.0,
                0.0, sigma * sigma, 0.0,
                0.0, 0.0, (sigma * 2) ** 2,
            ]
            fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            self.pub_fix.publish(fix)

            diag = DiagnosticArray()
            diag.header.stamp = stamp
            st = DiagnosticStatus()
            st.name = "rutx_gps"
            st.hardware_id = "rutx50:gnss"
            if fix.status.status == NavSatStatus.STATUS_FIX and sats >= 4:
                st.level, st.message = DiagnosticStatus.OK, f"{sats} sats"
            elif sats > 0:
                st.level, st.message = (
                    DiagnosticStatus.WARN,
                    f"searching ({sats} sats)",
                )
            else:
                st.level, st.message = DiagnosticStatus.WARN, "no satellites"
            st.values = [
                KeyValue(key="fix_status", value=fix_status),
                KeyValue(key="satellites", value=str(sats)),
                KeyValue(key="latitude", value=str(lat)),
                KeyValue(key="longitude", value=str(lon)),
                KeyValue(key="altitude", value=str(alt)),
                KeyValue(key="accuracy_m", value=str(accuracy)),
            ]
            diag.status.append(st)
            self.pub_diag.publish(diag)


def main():
    rclpy.init()
    node = RutxTelemetryNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
