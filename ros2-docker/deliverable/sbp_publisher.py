#!/usr/bin/env python3
# baseline program that published gps, raw imu/mag, and a heading using x,z axis trig
# Update: h_a and v_a in topic /gnss/accuracy
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus, Imu, MagneticField
from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3Stamped

# libsbp imports
from sbp.client.drivers.pyserial_driver import PySerialDriver
from sbp.client.framer import Framer
from sbp.navigation import MsgPosLLH, MsgVelNED
from sbp.mag import MsgMagRaw
from sbp.navigation import MsgPosLLHCov


try:
    from sbp.imu import MsgImuRaw
    HAVE_IMU = True
except Exception:
    HAVE_IMU = False


# ============================ MAG CONFIG ============================
EAST_FROM = "mag_z"
NORTH_FROM = "mag_x"
E_SIGN = -1.0
N_SIGN = +1.0
DECLINATION_DEG = 0.0


# ============================ HELPERS ============================
def wrap_deg(d: float) -> float:
    d = d % 360.0
    return d if d >= 0 else d + 360.0


class SBP2ROS(Node):
    def __init__(self, port="/dev/ttyACM0", baud=115200, frame_id="gps_link"):
        super().__init__("sbp2ros")

        self.fix_pub = self.create_publisher(NavSatFix, "/fix", 10)
        self.heading_pub = self.create_publisher(Float64, "/mag/heading", 10)
        self.imu_pub = self.create_publisher(Imu, "/imu/data_raw", 10)
        self.mag_pub = self.create_publisher(MagneticField, "/mag/data_raw", 10)
        self.acc_pub = self.create_publisher(Vector3Stamped, "/gnss/accuracy", 10)

        self.frame_id = frame_id

        self.driver = PySerialDriver(port, baud)
        self.framer = Framer(self.driver.read, self.driver.write)

        self.get_logger().info(f"Reading SBP on {port} @ {baud}")
        self.create_timer(0.02, self.spin_once)

    def spin_once(self):
        for _ in range(200):
            try:
                msg, _meta = next(self.framer)
            except StopIteration:
                break

            if msg is None:
                break

            # ---------------- H and V accuracy from PosLLH Message ----------------
            if isinstance(msg, MsgPosLLH):
                hacc_mm = getattr(msg, "h_accuracy", 5000)
                vacc_mm = getattr(msg, "v_accuracy", 8000)


                # Accuracy publisher
                hacc_m = float(hacc_mm) / 1000.0
                vacc_m = float(vacc_mm) / 1000.0

                acc_msg = Vector3Stamped()
                acc_msg.header.stamp = self.get_clock().now().to_msg()
                acc_msg.header.frame_id = self.frame_id
                acc_msg.vector.x = hacc_m
                acc_msg.vector.y = 0.0
                acc_msg.vector.z = vacc_m

                self.acc_pub.publish(acc_msg)

            # ---LLH lat,lon, heigh, and covariance message---
            elif isinstance(msg, MsgPosLLHCov):
                # MsgPosLLH provides NED covariance terms in meters^2:
                # cov_n_n, cov_n_e, cov_n_d, cov_e_e, cov_e_d, cov_d_d
                # Gets NED covariance and converts to ENU for NavSatFix covariance
                fix = NavSatFix()
                fix.header.stamp = self.get_clock().now().to_msg()
                fix.header.frame_id = self.frame_id

                # If you want to be conservative, you can leave STATUS_NO_FIX until you
                # also verify a separate status message, but usually this is fine:
                fix.status.status = NavSatStatus.STATUS_FIX
                fix.status.service = (
                    NavSatStatus.SERVICE_GPS
                    | NavSatStatus.SERVICE_GLONASS
                    | NavSatStatus.SERVICE_GALILEO
                )

                # Position
                fix.latitude = float(msg.lat)
                fix.longitude = float(msg.lon)
                fix.altitude = float(msg.height)

                # Covariance: SBP gives NED covariance in meters^2
                cov_n_n = float(msg.cov_n_n)
                cov_n_e = float(msg.cov_n_e)
                cov_n_d = float(msg.cov_n_d)
                cov_e_e = float(msg.cov_e_e)
                cov_e_d = float(msg.cov_e_d)
                cov_d_d = float(msg.cov_d_d)

                # Convert NED -> ENU (NavSatFix expects ENU, row-major).
                # ENU: x=E, y=N, z=U where U = -D
                Cxx = cov_e_e          # E,E
                Cyy = cov_n_n          # N,N
                Czz = cov_d_d          # U,U (variance unchanged)
                Cxy = cov_n_e          # E,N
                Cxz = -cov_e_d         # E,U (sign flip)
                Cyz = -cov_n_d         # N,U (sign flip)

                fix.position_covariance = [
                    Cxx, Cxy, Cxz,
                    Cxy, Cyy, Cyz,
                    Cxz, Cyz, Czz
                ]
                fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_KNOWN

                self.fix_pub.publish(fix)

            # ---------------- RAW IMU ----------------
            elif HAVE_IMU and isinstance(msg, MsgImuRaw):
                imu_msg = Imu()
                imu_msg.header.stamp = self.get_clock().now().to_msg()
                imu_msg.header.frame_id = "imu_link"

                imu_msg.linear_acceleration.x = float(msg.acc_x)
                imu_msg.linear_acceleration.y = float(msg.acc_y)
                imu_msg.linear_acceleration.z = float(msg.acc_z)

                imu_msg.angular_velocity.x = float(msg.gyr_x)
                imu_msg.angular_velocity.y = float(msg.gyr_y)
                imu_msg.angular_velocity.z = float(msg.gyr_z)

                self.imu_pub.publish(imu_msg)

            # ---------------- MAG + HEADING ----------------
            elif isinstance(msg, MsgMagRaw):
                mag_msg = MagneticField()
                mag_msg.header.stamp = self.get_clock().now().to_msg()
                mag_msg.header.frame_id = self.frame_id

                mag_msg.magnetic_field.x = float(msg.mag_x)
                mag_msg.magnetic_field.y = float(msg.mag_y)
                mag_msg.magnetic_field.z = float(msg.mag_z)

                self.mag_pub.publish(mag_msg)

                mx = float(msg.mag_x)
                my = float(msg.mag_y)
                mz = float(msg.mag_z)

                east = mz
                north = mx

                hdg_deg = math.degrees(
                    math.atan2(E_SIGN * east, N_SIGN * north)
                )
                hdg_deg = wrap_deg(hdg_deg + DECLINATION_DEG)

                hmsg = Float64()
                hmsg.data = float(hdg_deg)
                self.heading_pub.publish(hmsg)


def main():
    rclpy.init()
    node = SBP2ROS()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

