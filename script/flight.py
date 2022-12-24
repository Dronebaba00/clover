import rospy
from clover import srv
from std_srvs.srv import Trigger
from mavros_msgs.srv import CommandBool
import tf2_ros
from geometry_msgs.msg import PoseStamped
from aruco_pose.msg import MarkerArray
import math


class CloverDrone:

    def __init__(self):
        """
        Initialize Variables
        """
        self.found_marker = False
        self.get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
        self.navigate = rospy.ServiceProxy('navigate', srv.Navigate)
        self.navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
        self.set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
        self.set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
        self.set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
        self.set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
        self.land = rospy.ServiceProxy('land', Trigger)
        self.arming = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
        self.aruco_id = str()

    def navigate_wait(self, x=0.0, y=0.0, z=0.0, yaw=float('nan'), yaw_rate=0, speed=0.5, frame_id='', auto_arm=False,
                      tolerance=1.0):
        """
        Function to fly to a point and wait for copter's arrival
        :param x: x-coordinate (m)
        :param y: y-coordinate (m)
        :param z: z-coordinate (m)
        :param yaw: yaw angle (radians)
        :param yaw_rate: angular yaw velocity (will be used if yaw is set to NaN) (rad/s)
        :param speed: flight speed (setpoint speed) (m/s)
        :param frame_id: coordinate system for values x, y, z and yaw. Example:map, body, aruco_map. Default value = map
        :param auto_arm: switch the drone to OFFBOARD mode and arm automatically (the drone will take off)
        :param tolerance: tolerance between the coordinates given to the drone and the coordinates to which the drone
        has arrived
        :return:
        """
        self.navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

        while not rospy.is_shutdown():
            telem = self.get_telemetry(frame_id='navigate_target')
            if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
                break
            rospy.sleep(0.2)

    def land_wait(self):
        """
        Function used to Land and wait until the copter lands:
        :return:
        """

        self.land()
        while self.get_telemetry().armed:
            rospy.sleep(0.2)

    def wait_arrival(self, tolerance=0.2):
        """
        Function used to wait for copter's arrival at the target:
        :param tolerance: tolerance between the coordinates given to the drone and the coordinates to which the drone
        has arrived
        :return:
        """
        while not rospy.is_shutdown():
            telem = self.get_telemetry(frame_id='navigate_target')
            if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
                break
            rospy.sleep(0.2)

    def get_distance(self, x1, y1, z1, x2, y2, z2):
        """
        Function used to calculate the distance between two points.
         (Important: the points are to be in the same coordinate system)
        :param x1: First x-coordinate
        :param y1: First y-coordinate
        :param z1: First z-coordinate
        :param x2: Second x-coordinate
        :param y2: Second y-coordinate
        :param z2: Second z-coordinate
        :return:
        """
        return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2 + (z1 - z2) ** 2)

    def get_distance_global(self, lat1, lon1, lat2, lon2):
        """
        Approximation of distance (in meters) between two global coordinates (latitude/longitude):
        :param lat1: Latitude of the first global coordinate
        :param lon1: Longitude of the first global coordinate
        :param lat2: Latitude of the second global coordinate
        :param lon2: Longitude of the second global coordinate
        :return:
        """
        return math.hypot(lat1 - lat2, lon1 - lon2) * 1.113195e5

    def set_drone_arm_status(self, arm=True):
        """
        Function used to arm/disarm the drone
        :param arm: Value to set for the drone arming status
        :return:
        """
        self.arming(arm)

    def transform_position(self):
        """
        Function used to transform the position (PoseStamped) from one coordinate system to another using tf2
        :return:
        """

        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer)

        # ...

        # Create PoseStamped object (or get it from a topic):
        pose = PoseStamped()
        pose.header.frame_id = 'map'  # coordinate frame, in which the position is specified
        pose.header.stamp = rospy.get_rostime()  # the time for which the position is specified (current time)
        pose.pose.position.x = 1
        pose.pose.position.y = 2
        pose.pose.position.z = 3
        pose.pose.orientation.w = 1

        frame_id = 'base_link'  # target coordinate frame
        transform_timeout = rospy.Duration(0.2)  # timeout for transformation

        # Transform the position from the old frame to the new one:
        new_pose = tf_buffer.transform(pose, frame_id, transform_timeout)

    def check_copter_heading(self):
        """
        Determine whether the copter is turned upside-down
        :return:
        """
        PI_2 = math.pi / 2
        telem = self.get_telemetry()

        flipped = abs(telem.pitch) > PI_2 or abs(telem.roll) > PI_2

    def calculate_copter_horizontal_angle(self):
        """
        Function used to calculate the copter horizontal angle
        :return:
        """
        PI_2 = math.pi / 2
        telem = self.get_telemetry()

        flipped = not -PI_2 <= telem.pitch <= PI_2 or not -PI_2 <= telem.roll <= PI_2
        angle_to_horizon = math.atan(math.hypot(math.tan(telem.pitch), math.tan(telem.roll)))
        if flipped:
            angle_to_horizon = math.pi - angle_to_horizon

    def move_drone_on_circular_path(self):
        """
        Function used to fly drone along a circular path
        :return:
        """
        RADIUS = 0.6  # m
        SPEED = 0.3  # rad / s

        start = self.get_telemetry()
        start_stamp = rospy.get_rostime()

        r = rospy.Rate(10)

        while not rospy.is_shutdown():
            angle = (rospy.get_rostime() - start_stamp).to_sec() * SPEED
            x = start.x + math.sin(angle) * RADIUS
            y = start.y + math.cos(angle) * RADIUS
            self.set_position(x=x, y=y, z=start.z)

            r.sleep()

    def markers_callback(self, msg):
        for marker in msg.markers:
            self.aruco_id = 'aruco_' + str(msg.markers[0].id)
            print(self.aruco_id, "aruco tag")

    def main(self):
        """
        Main Function
        :return:
        """
        rospy.Subscriber('aruco_detect/markers', MarkerArray, self.markers_callback)
        self.navigate_wait(x=0.0, y=0.0, z=0.5, speed=1.0, frame_id='body', auto_arm=True)
        count = 1
        while count <= 3:
            self.navigate_wait(x=5.0, y=5.0, z=count, speed=0.5, frame_id=self.aruco_id)
            rospy.sleep(5.0)
            telemetry = self.get_telemetry(frame_id=self.aruco_id)
            print('Height: ', telemetry.z, ' m')
            print("Success")

            self.navigate(x=5, y=5, z=count, yaw=float('nan'), yaw_rate=0.5, frame_id=self.aruco_id)
            rospy.sleep(12.56)
            telemetry = self.get_telemetry(frame_id=self.aruco_id)
            print('Yaw Rate: ', telemetry.yaw_rate, 'rad/sec')
            print("Success")
            self.navigate(yaw=float('nan'), yaw_rate=0.0, frame_id='body')

            count = count + 1

        rospy.sleep(5.0)

        self.navigate_wait(frame_id=self.aruco_id, x=5, y=5, z=0.4, speed=1.0)
        print('Landing on ' + self.aruco_id)
        res = self.land()
        if res.success:
            print('drone has landed')


if __name__ == "__main__":

    clover_drone = CloverDrone()
    clover_drone.main()