import h5py
import time
import numpy as np
from movement_primitives.dmp import CartesianDMP
import pytransform3d.rotations as pr
import rclpy
from geometry_msgs.msg import Point, Quaternion
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from std_srvs.srv import Trigger
from wam_msgs.msg import (
    RTCartPose,
)

class WamDmpNode(Node):
    def __init__(self):
        super().__init__("wam_dmp_node")

        self.cart_pos_rt_pub = self.create_publisher(
            RTCartPose, "/wam/RTCartPoseCMD", 10
        )
        self.get_logger().info("WamTestNode initialized and ready to send commands.")

    def load_and_process_trajectory(self, file_path):
        print(f"Loading data from {file_path}...")
        
        with h5py.File(file_path, 'r') as f:
            raw_trajectory = f['observations/cartesian/position'][:]
            
        n_steps = raw_trajectory.shape[0]
        
        positions = raw_trajectory[:, :3]
        euler_angles = raw_trajectory[:, 3:]
        
        quat = Rotation.from_euler('xyz', euler_angles, degrees=False).as_quat()
        
        # Combine to form Y: Shape (n, 7)
        Y = np.hstack((positions, quat))
        
        return Y, n_steps

    def send_cart_pose_rt(
        self,
        pose: list | np.ndarray,
        position_rate_limit=[0.05] * 3,
        orientation_rate_limit=[0.15] * 4,
    ):
        if isinstance(pose, list):
            pose = np.array(pose)

        assert len(pose) == 7

        msg = RTCartPose()
        msg.point = Point(x=pose[0], y=pose[1], z=pose[2])
        msg.orientation = Quaternion(x=pose[3], y=pose[4], z=pose[5], w=pose[6])

        # NOTE: these rate limits do seem to work. These values are not extensively tested though.
        msg.position_rate_limits = position_rate_limit
        msg.orientation_rate_limits = orientation_rate_limit

        self.get_logger().info(f"Publishing EULER: {str(msg.orientation)}")
        self.get_logger().info(
            f"Publishing EULER: {str(msg.point.x)}, {str(msg.point.y)}, {str(msg.point.z)}"
        )

        self.cart_pos_rt_pub.publish(msg)

    def loop_through_traj(self, traj):
        for i in range(traj.shape[0]):
            self.send_cart_pose_rt(traj[i])
            time.sleep(0.1)

        # repeat last thing until we are there
        for i in range(100):
            self.send_cart_pose_rt(traj[-1])
            time.sleep(0.1)


def main(args=None):
    rclpy.init(args=args)
    wam_dmp_node = WamDmpNode()

    file_path = './data/episode_2.h5'
    Y_demonstration, n_steps = wam_dmp_node.load_and_process_trajectory(file_path)

    dt = 0.01 
    execution_time = (n_steps - 1) * dt
    T = np.linspace(0, execution_time, n_steps)

    print("Training DMP...")
    dmp = CartesianDMP(execution_time=execution_time, dt=dt, n_weights_per_dim=10)
    dmp.imitate(T, Y_demonstration)

    _, Y_generated = dmp.open_loop()

    print(f"Generated Trajectory Shape: {Y_generated.shape}")
    print(f"Generated Trajectory last: {Y_generated[-1]}")

    wam_dmp_node.loop_through_traj(Y_generated)


    wam_dmp_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

