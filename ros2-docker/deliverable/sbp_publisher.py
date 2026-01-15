#!/usr/bin/env python3
#baseline program that published gps, raw imu/mag, and a heading using x,z axis trig
import math, rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus, Imu, MagneticField
from std_msgs.msg import Float64

# libsbp imports
from sbp.client.drivers.pyserial_driver import PySerialDriver
from sbp.client.framer import Framer
from sbp.navigation import MsgPosLLH, MsgVelNED
from sbp.mag import MsgMagRaw
try:
    from sbp.imu import MsgImuRaw   # present on many Piksi firmwares
    HAVE_IMU = True
except Exception:
    HAVE_IMU = False

# ============================ MAG CONFIG ============================
EAST_FROM   = "mag_z"     # try "mag_y" or "mag_z"
NORTH_FROM  = "mag_x"     # try "mag_x" or "mag_z"
E_SIGN      = -1.0        # flip to -1.0 if rotation feels backward
N_SIGN      = +1.0        # flip to -1.0 if N/S reversed
DECLINATION_DEG = 0.0     # add local declination later (+E / -W)

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
        self.frame_id = frame_id

        # Start SBP reader
        self.driver = PySerialDriver(port, baud)
        self.framer = Framer(self.driver.read, self.driver.write)

        self.get_logger().info(f"Reading SBP on {port} @ {baud}")
        self.create_timer(0.02, self.spin_once)  # run every cycle

    def spin_once(self):
	    # Pull as many messages as are available right now (bounded)
	    for _ in range(200):
	        try:
	            msg, _meta = next(self.framer)   # ← Python 3 style
	        except StopIteration:
	            break
	        if msg is None:
	            break
	        from sbp.navigation import MsgPosLLH, MsgVelNED
	        try:
	            from sbp.imu import MsgImuRaw
	            HAVE_IMU = True
	        except Exception:
	            HAVE_IMU = False
	 
	        if isinstance(msg, MsgPosLLH):
	            fix = NavSatFix()
	            fix.header.stamp = self.get_clock().now().to_msg()
	            fix.header.frame_id = self.frame_id
	            fix.status.status = (NavSatStatus.STATUS_FIX
	                                 if getattr(msg, "n_sats", 0) >= 4
	                                 else NavSatStatus.STATUS_NO_FIX)
	            fix.status.service = (NavSatStatus.SERVICE_GPS |
	                                  NavSatStatus.SERVICE_GLONASS |
	                                  NavSatStatus.SERVICE_GALILEO)
	            fix.latitude = msg.lat
	            fix.longitude = msg.lon
	            fix.altitude = msg.height
	            hacc = getattr(msg, "h_accuracy", 5.0) or 5.0
	            vacc = getattr(msg, "v_accuracy", 8.0) or 8.0
	            fix.position_covariance = [0.0]*9
	            fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
	            self.fix_pub.publish(fix)
		#RAW IMU data
	        elif HAVE_IMU and isinstance(msg, MsgImuRaw):
                    imu_msg = Imu()
                    imu_msg.header.stamp = self.get_clock().now().to_msg()
                    imu_msg.header.frame_id = "imu_link"
                    
                    # pull values
                    imu_msg.linear_acceleration.x = float(msg.acc_x)
                    imu_msg.linear_acceleration.y = float(msg.acc_y)
                    imu_msg.linear_acceleration.z = float(msg.acc_z)
                    imu_msg.angular_velocity.x = float(msg.gyr_x)
                    imu_msg.angular_velocity.y = float(msg.gyr_y)
                    imu_msg.angular_velocity.z = float(msg.gyr_z)
                    
                    self.imu_pub.publish(imu_msg)
                #Mag and heading
	        elif isinstance(msg, MsgMagRaw):
                    #RAW mag vals
                    mag_msg = MagneticField()
                    mag_msg.header.stamp = self.get_clock().now().to_msg()
                    mag_msg.header.frame_id = self.frame_id

                    mag_msg.magnetic_field.x = float(msg.mag_x)
                    mag_msg.magnetic_field.y = float(msg.mag_y)
                    mag_msg.magnetic_field.z = float(msg.mag_z)
                    
                    self.mag_pub.publish(mag_msg)
                    
                    #Heading calculations
                    mx, my, mz = float(msg.mag_x), float(msg.mag_y), float(msg.mag_z)
                    east = mz #axis orientation
                    north = mx #axis orientation
                    
                    hdg_deg = math.degrees(math.atan2(E_SIGN * east, N_SIGN * north))
                    hdg_deg = wrap_deg(hdg_deg + DECLINATION_DEG)

                    #Publish heading to /gps/heading
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

