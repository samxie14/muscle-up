'''typewriter.py

   The G1 hangs from a wide grip, does a pull-up, then shifts its body
   side to side along the bar ("typewriter"): toward the left hand
   (left arm bent, right arm extended), then toward the right hand.
   After 3 reps it lowers back down, rests, and repeats forever.

   Node:        /typewriter
   Parameter:   shift       How far (m) the pelvis moves to each side
   Publish:     /joint_states                 sensor_msgs/JointState
                /pose                         geometry_msgs/PoseStamped
                /twist                        geometry_msgs/TwistStamped
                /visualization_marker_array   visualization_msgs/MarkerArray
   Broadcast:   'world' -> 'pelvis'           geometry_msgs/TransformStamped
'''

import rclpy
import numpy as np

from math                       import pi
from rclpy.node                 import Node
from rclpy.task                 import Future
from tf2_ros                    import TransformBroadcaster
from geometry_msgs.msg          import PoseStamped, TwistStamped
from geometry_msgs.msg          import TransformStamped, Vector3
from sensor_msgs.msg            import JointState
from std_msgs.msg               import Header, ColorRGBA
from visualization_msgs.msg     import Marker, MarkerArray

from utils.TransformHelpers     import *
from utils.TrajectoryUtils      import goto
from muscleup_update1.KinematicChain import KinematicChain
from muscleup_update1.g1        import *


#
#   Pelvis Trajectory
#
#   Pelvis positions relative to the bar center (x forward, y left, z up).
#   The pelvis stays upright; only its position moves.
#
HANG = PBAR + pxyz(-0.05, 0.0, -0.64)       # Bottom of the pull-up
UP   = PBAR + pxyz(-0.15, 0.0, -0.42)       # Top of the pull-up
REPS = 3                                    # Typewriter reps per cycle

def segments(shift):
    '''One cycle as (start, end, duration) moves, each a cubic goto.'''
    LEFT  = UP + shift * ny()
    RIGHT = UP - shift * ny()
    return ([(HANG,  HANG,  2.0),                   # Rest in the hang
             (HANG,  UP,    2.0),                   # Pull up
             (UP,    LEFT,  1.5)] +                 # Shift to the left hand
            [(LEFT,  RIGHT, 3.0),                   # Typewriter rep:
             (RIGHT, LEFT,  3.0)] * REPS +          #   left -> right -> left
            [(LEFT,  UP,    1.5),                   # Back to the center
             (UP,    HANG,  2.0)])                  # Lower down

def trajectory(t, shift):
    # Wrap the time into the cycle, then find the current move.
    moves = segments(shift)
    t = t % sum(T for (_, _, T) in moves)
    for (p0, pf, T) in moves:
        if t < T:
            break
        t = t - T
    (pd, vd) = goto(min(t, T), T, p0, pf)
    return (pd, vd, Reye(), vzero())


#
#   Typewriter Node
#
class TypewriterNode(Node):
    # Initialization.
    def __init__(self, name, future):
        super().__init__(name)
        self.future = future
        self.shift  = self.declare_parameter('shift', 0.15).value

        # The two kinematic chains (pelvis to each palm), and the IK,
        # which solves for the hanging start.
        chains = {s: KinematicChain(self, 'pelvis', PALM[s], CHAIN[s])
                  for s in SIDES}
        (pd, _, Rd, _) = trajectory(0.0, self.shift)
        self.dt = 0.01
        self.g1 = G1(chains, pd, Rd, self.dt)

        # Publishers, the TF broadcaster, and the bar marker.
        self.pubjoint  = self.create_publisher(JointState, '/joint_states', 10)
        self.pubpose   = self.create_publisher(PoseStamped, '/pose', 10)
        self.pubtwist  = self.create_publisher(TwistStamped, '/twist', 10)
        self.pubmarker = self.create_publisher(MarkerArray, '/visualization_marker_array', 10)
        self.tfbroad   = TransformBroadcaster(self)
        self.bar = Marker(type=Marker.CYLINDER, ns='bar', action=Marker.ADD,
                          pose=Pose_from_Rp(Rotx(pi/2), PBAR),
                          scale=Vector3(x=2*BAR_RADIUS, y=2*BAR_RADIUS, z=1.4),
                          color=ColorRGBA(r=0.8, g=0.8, b=0.85, a=1.0))
        self.bar.header.frame_id = 'world'

        # Wait for RVIZ, then run at 100Hz (t=0 in the first cycle).
        self.get_logger().info("Waiting for a /joint_states subscriber...")
        while(not self.count_subscribers('/joint_states')):
            pass
        self.t     = -self.dt
        self.now   = self.get_clock().now()
        self.timer = self.create_timer(self.dt, self.update)
        self.get_logger().info("Running, shift %.2fm" % self.shift)

    def shutdown(self):
        self.timer.destroy()
        self.destroy_node()

    # Update - one IK step every cycle.
    def update(self):
        self.t   = self.t   + self.dt
        self.now = self.now + rclpy.time.Duration(seconds=self.dt)

        # Desired pelvis motion, then the whole-body IK.
        (pd, vd, Rd, wd) = trajectory(self.t, self.shift)
        (qdot, e) = self.g1.step(pd, vd, Rd, wd)

        # Publish the pelvis TF, joints (plus fixed fingers), the
        # desired pelvis pose/twist, and the bar.
        header = Header(stamp=self.now.to_msg(), frame_id='world')
        self.tfbroad.sendTransform(TransformStamped(
            header=header, child_frame_id='pelvis',
            transform=Transform_from_Rp(self.g1.R, self.g1.p)))
        self.pubjoint.publish(JointState(
            header=header, name=JOINTS + list(FINGERS),
            position=self.g1.q.tolist() + list(FINGERS.values()),
            velocity=qdot[6:].tolist() + [0.0] * len(FINGERS)))
        self.pubpose.publish(PoseStamped(header=header, pose=Pose_from_Rp(Rd, pd)))
        self.pubtwist.publish(TwistStamped(header=header, twist=Twist_from_vw(vd, wd)))
        self.bar.header.stamp = header.stamp
        self.pubmarker.publish(MarkerArray(markers=[self.bar]))


def main(args=None):
    rclpy.init(args=args)
    future = Future()
    node = TypewriterNode('typewriter', future)
    try:
        rclpy.spin_until_future_complete(node, future)
    except KeyboardInterrupt:
        pass
    node.shutdown()
    if rclpy.ok():
        rclpy.shutdown()

if __name__ == "__main__":
    main()
