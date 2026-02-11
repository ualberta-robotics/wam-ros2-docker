import numpy as np
import rclpy
from rclpy.qos_event import PublisherEventCallbacks
import scipy
from geometry_msgs.msg import Point, Pose, Quaternion
from rclpy.duration import Duration
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty, Trigger
from wam_msgs.msg import (
    RTCartPose,
    RTJointPositions,
    RTJointVelocities,
    RTLinearandAngularVelocity,
)
from wam_srvs.srv import CartPoseMove, JointMove

# WAM PUBLISHERS:
# "/joint_states", JointState
# "wam/trajectoryStatus", Bool
# "/wam/jointVelocity", JointState
# "/wam/ToolPose", PoseStamped
# "/wam/toolVelocity", TwistStamped

# WAM SUBSCRIBERS:
# "/wam/RTJointPositionCMD", RTJointPositions
# "/wam/RTLinearandAngularVelocityCMD", RTLinearandAngularVelocity
# "/wam/RTAngularVelocityCMD", RTAngularVelocity
# "/wam/RTLinearVelocityCMD", RTLinearVelocity
# "/wam/RTCartPositionCMD", RTCartPosition
# "/wam/RTCartPoseCMD", RTCartPose
# "/wam/RTCartOrientationCMD", RTCartOrientation
# "/wam/RTJointVelocityCMD", RTJointVelocities

# WAM SERVICES:
# "/wam/moveHome", Trigger
# "/wam/gravityCompensate", SetBool
# "/wam/idle", Trigger;
# "/wam/holdCartPosition", SetBool
# "/wam/holdJointPosition", SetBool
# "/wam/holdCartOrientation", SetBool
# "/wam/holdCartPose", SetBool
# "/wam/moveToJointPosition", JointMove
# "/wam/moveToCartPosition", CartPositionMove
# "/wam/moveToCartOrientation", CartOrientationMove
# "/wam/moveToCartPose", CartPoseMove
# "/wam/setVelocityLimit" VelocityLimit>


class WamTestNode(Node):
    def __init__(self):
        super().__init__("wam_test_node")

        # DOF parameter
        self.declare_parameter("wam_dof", 7)
        self.wam_dof = self.get_parameter("wam_dof").get_parameter_value().integer_value

        self.cart_pose_move_client = self.create_client(
            CartPoseMove, "/wam/moveToCartPose"
        )
        self.joint_move_client = self.create_client(
            JointMove, "/wam/moveToJointPosition"
        )
        self.home_client = self.create_client(Trigger, "/wam/moveHome")

        self.twist_pub = self.create_publisher(
            RTLinearandAngularVelocity, "/wam/RTLinearandAngularVelocityCMD", 10
        )
        self.jnt_vels_pub = self.create_publisher(
            RTJointVelocities, "/wam/RTJointVelocityCMD", 10
        )
        self.jnt_pos_pub = self.create_publisher(
                RTJointPositions, "/wam/RTJointPositionCMD", 10
        )
        self.cart_pos_rt_pub = self.create_publisher(
            RTCartPose, "/wam/RTCartPoseCMD", 10
        )
        self.get_logger().info("WamTestNode initialized and ready to send commands.")

    def send_cart_pose_move(self, pose: list | np.ndarray, block=True):
        if isinstance(pose, list):
            pose = np.array(pose)

        while not self.cart_pose_move_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for cart_pose_move service...")
        request = CartPoseMove.Request()
        pose_msg = Pose()
        pose_msg.position.x = pose[0]
        pose_msg.position.y = pose[1]
        pose_msg.position.z = pose[2]
        quat = Rotation.from_euler("xyz", pose[3:6]).as_quat()
        pose_msg.orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
        request.pose = pose_msg
        request.blocking = block

        future = self.cart_pose_move_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def send_cart_pose_rt(
        self,
        pose: list | np.ndarray,
        position_rate_limit=[0.05] * 3,
        orientation_rate_limit=[0.05] * 4,
    ):
        if isinstance(pose, list):
            pose = np.array(pose)

        msg = RTCartPose()
        msg.point = Point(x=pose[0], y=pose[1], z=pose[2])
        quat = Rotation.from_euler("xyz", pose[3:6]).as_quat()
        msg.orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])

        # NOTE: these rate limits do seem to work. These values are not extensively tested though.
        msg.position_rate_limits = position_rate_limit
        msg.orientation_rate_limits = orientation_rate_limit

        self.cart_pos_rt_pub.publish(msg)

    def send_joint_pos(self, angles: list | np.ndarray, rate_limit=None):
        if rate_limit is None:
            rate_limit = [0.1] * self.wam_dof

        if isinstance(angles, list):
            angles = np.array(angles)

        msg = RTJointPositions()
        msg.joint_states = angles.tolist()
        # NOTE: make sure you have the edited version of the ros2 wam package,
        # as this is bugged in the code on barrett's gitlab.
        msg.rate_limits = rate_limit

        self.jnt_pos_pub.publish(msg)

    def send_joint_vels(self, vels: list | np.ndarray):
        if isinstance(vels, list):
            vels = np.array(vels)

        msg = RTJointVelocities()
        msg.velocities = vels.tolist()

        self.jnt_vels_pub.publish(msg)

    def send_twist(self, vels: list | np.ndarray):
        if isinstance(vels, list):
            vels = np.array(vels)

        msg = RTLinearandAngularVelocity()
        linear = vels[:3]
        angular = vels[3:]
        linear_norm = np.linalg.norm(linear)
        linear_dir = linear / linear_norm if linear_norm > 0 else np.zeros(3)
        angular_norm = np.linalg.norm(angular)
        angular_dir = angular / angular_norm if angular_norm > 0 else np.zeros(3)

        msg.linear_velocity_direction = linear_dir.tolist()
        msg.linear_velocity_magnitude = float(linear_norm)
        msg.angular_velocity_direction = angular_dir.tolist()
        msg.angular_velocity_magnitude = float(angular_norm)

        self.twist_pub.publish(msg)

    def send_joint_move(self, angles: list | np.ndarray, block=True):
        if isinstance(angles, list):
            angles = np.array(angles)

        while not self.joint_move_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for joint_move service...")
        self.get_logger().info(str(angles))
        request = JointMove.Request()
        angles_msg = JointState()
        angles_msg.position = angles.tolist()
        request.joint_state = angles_msg
        request.blocking = block

        future = self.joint_move_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def go_home(self):
        while not self.home_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for home service...")
        request = Trigger.Request()
        future = self.home_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()


def main(args=None):
    rclpy.init(args=args)
    wam_test_node = WamTestNode()

    # NOTE: I have not tested any of the topics that take in e.g.
    # an orientation only or a position only, since I feel they are less useful.
    # However, the blocking versions almost certainly work, the non-blocking ones I would use caution with.

    # test things here.
    # NOTE from what I remember you have to send the realtime topics at least somewhat
    # frequently or it will stop; I think the timeout is 0.5s. This would be fine for any
    # practical control.

    test_jnt_pos = np.array(
        [
            0.008692557798018632,
            0.3442632989449545,
            -0.03439152906740131,
            2.350058567040802,
            0.08255030631714483,
            0.2555580364147625,
            -0.25552647149843205,
        ]
    )
    # start for il
    wam_test_node.send_joint_move(test_jnt_pos)
      # home
    # wam_test_node.go_home()


    test_jnt_pos2 = np.array(
        [
            0.2508692557798018632,
            0.3442632989449545,
            -0.03439152906740131,
            2.350058567040802,
            0.08255030631714483,
            0.2555580364147625,
            -0.25552647149843205,
        ]
    )
    # wam_test_node.send_joint_move(test_jnt_pos2)

    # wam_test_node.get_clock().sleep_for(Duration(seconds=0.5))

    # wam_test_node.send_joint_move_block(test_jnt_pos2)

    # for i in range(5):
    #     wam_test_node.get_clock().sleep_for(Duration(seconds=0.2))
    #     wam_test_node.send_joint_pos(test_jnt_pos)

    # for i in range(5):
    #     wam_test_node.get_clock().sleep_for(Duration(seconds=0.2))
    #
    #     wam_test_node.send_joint_pos(test_jnt_pos)

    # while True:
    #     wam_test_node.get_clock().sleep_for(Duration(seconds=0.01))
    #
    #     wam_test_node.send_joint_pos(test_jnt_pos)
    
    # for i in range(5):
    #     test_jnt_vels = np.array([0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    #     wam_test_node.send_joint_vels(test_jnt_vels)
    #     wam_test_node.get_clock().sleep_for(Duration(seconds=0.2))
    # wam_test_node.send_joint_vels(np.zeros(7))

    # for i in range(5):
    #     test_vels = np.array([0.1, 0.0, 0.0, 0.0, 0.0, 0.0])
    #     wam_test_node.send_twist(test_vels)
    #     wam_test_node.get_clock().sleep_for(Duration(seconds=0.2))
    # wam_test_node.send_twist(np.zeros(6))

    # wam_test_node.send_twist(test_vels)
    # wam_test_node.get_clock().sleep_for(Duration(seconds=2))
    # wam_test_node.send_twist(np.zeros(6))

    # a test pose:
    # pose:
    # position:
    #     x: 0.40976376669767367
    #     y: -0.006062028809306298
    #     z: 0.19157529607001925
    # orientation:
    #     x: -0.0729143374657968
    #     y: 0.992655613782666
    #     z: -0.00457232952325213
    #     w: -0.09642315914972584
    testrot = scipy.spatial.transform.Rotation.from_quat(
        [
            -0.0729143374657968,
            0.992655613782666,
            -0.00457232952325213,
            -0.09642315914972584,
        ]
    ).as_euler("xyz")
    # wam_test_node.get_logger().info(f"testrot euler: {testrot}")
    test_pose = [
        0.40,
        0.0,
        0.2,
        testrot[0],
        testrot[1],
        testrot[2],
    ]
    test_pose2 = [
        0.45,
        0.0,
        0.2,
        testrot[0],
        testrot[1],
        testrot[2],
    ]

    # wam_test_node.send_cart_pose_move(test_pose)

    # for i in range(5):
    #     wam_test_node.get_clock().sleep_for(Duration(seconds=0.2))
    #     wam_test_node.send_cart_pose_rt(test_pose)

    # # wam_test_node.get_clock().sleep_for(Duration(seconds=0.3))

    # # wam_test_node.send_cart_pose_move(test_pose2)
    # for i in range(5):
    #     wam_test_node.get_clock().sleep_for(Duration(seconds=0.2))
    #     wam_test_node.send_cart_pose_rt(test_pose2)

    wam_test_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
