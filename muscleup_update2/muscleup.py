'''muscleup.py   (update 2)

   The G1 does a full bar muscle-up: hang, pull, transition (dip),
   press to straight-arm support, hold, then lower back down.  Repeats
   every 12 s.  Two test motions push the pelvis target out of reach:

      motion:=muscleup              the muscle-up (default)
      motion:=low   target:=-1.0    hang, then ask the pelvis to drop to
                                    z = target below the bar (too low)
      motion:=side  target:=0.40    pull up, then ask the pelvis to slide
                                    y = target along the bar (+ left, - right)
      priority:=false               update 1's equal-weight IK, to compare

   Node:        /muscleup
   Parameters:  motion, target, priority
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
from utils.TrajectoryUtils      import spline
from muscleup_update1.KinematicChain import KinematicChain
from muscleup_update2.g1        import *


#
#   Trajectory
#
#   Waypoints: pelvis position relative to the bar (x forward, y left,
#   z up), pelvis pitch (lean forward +), and the leg posture.
#
#                      x     y      z    pitch   hip   knee
HANG    = np.array([-0.03, 0.0, -0.73,  0.00, -0.10, 0.10])  # straight-ish arms
PULL    = np.array([-0.13, 0.0, -0.44, -0.10, -0.35, 0.20])  # chin near the bar
DIP     = np.array([-0.25, 0.0, -0.15,  0.40, -0.70, 0.40])  # chest over the bar
SUPPORT = np.array([-0.12, 0.0,  0.12,  0.12, -0.15, 0.05])  # arms locked, on top

def knots(motion, target):
    '''The motion as a list of (waypoint, arrival time).  A repeated
    waypoint is a hold.  Each motion ends where it starts, so it loops.'''
    if motion == 'low':
        LOW  = HANG + [0, 0, target - HANG[2], 0, 0, 0]
        return [(HANG, 0.0), (HANG, 1.0), (LOW, 3.0), (LOW, 6.0), (HANG, 8.0)]
    if motion == 'side':
        SIDE = PULL + [0, target, 0, 0, 0, 0]
        return [(HANG, 0.0), (HANG, 1.0), (PULL, 3.0), (SIDE, 5.0),
                (SIDE, 8.0), (PULL, 10.0), (HANG, 12.0)]
    return [(HANG,    0.0), (HANG,    1.0),          # hold
            (PULL,    3.0), (DIP,     4.5),          # pull, transition
            (SUPPORT, 6.0), (SUPPORT, 7.5),          # press, hold
            (DIP,     9.0), (PULL,   10.5),          # lower
            (HANG,   12.0)]

def trajectory(t, motion='muscleup', target=0.0):
    '''Desired pelvis (pd, vd, Rd, wd) and posture qnom at time t:
    cubic splines through the knots.  An interior knot gets the average
    (Catmull-Rom) velocity of its neighbors, so the motion flows
    through it, unless it starts or ends a hold (then velocity 0).'''
    K = knots(motion, target)
    (P, T) = ([k[0] for k in K], [k[1] for k in K])
    t = t % T[-1]

    def vel(k):
        if k == 0 or k == len(P)-1 or np.array_equal(P[k], P[k-1]) \
                                   or np.array_equal(P[k], P[k+1]):
            return np.zeros(6)
        return (P[k+1] - P[k-1]) / (T[k+1] - T[k-1])

    k = next(k for k in range(len(P)-1) if t <= T[k+1])
    (s, sdot) = spline(t - T[k], T[k+1] - T[k], P[k], P[k+1], vel(k), vel(k+1))
    (x, y, z, pitch, hip, knee) = s

    pd = PBAR + pxyz(x, y, z)
    vd = vxyz(sdot[0], sdot[1], sdot[2])
    return (pd, vd, Roty(pitch), ny() * sdot[3], posture(hip, knee))


#
#   Muscle-up Node
#
class MuscleUpNode(Node):
    def __init__(self, name, future):
        super().__init__(name)
        self.future = future
        self.motion = self.declare_parameter('motion', 'muscleup').value
        self.target = self.declare_parameter('target', 0.0).value
        self.prio   = self.declare_parameter('priority', True).value

        # The two kinematic chains (pelvis to each palm), and the IK,
        # which solves for the hanging start.
        chains = {s: KinematicChain(self, 'pelvis', PALM[s], CHAIN[s])
                  for s in SIDES}
        (pd, _, Rd, _, qnom) = trajectory(0.0, self.motion, self.target)
        self.dt = 0.01
        self.g1 = G1(chains, pd, Rd, qnom, priority=self.prio, dt=self.dt)

        # Publishers, the TF broadcaster, and the bar marker.
        self.pubjoint  = self.create_publisher(JointState, '/joint_states', 10)
        self.pubpose   = self.create_publisher(PoseStamped, '/pose', 10)
        self.pubtwist  = self.create_publisher(TwistStamped, '/twist', 10)
        self.pubmarker = self.create_publisher(MarkerArray, '/visualization_marker_array', 10)
        self.tfbroad   = TransformBroadcaster(self)
        self.bar = Marker(type=Marker.CYLINDER, ns='bar', action=Marker.ADD,
                          pose=Pose_from_Rp(Rotx(pi/2), PBAR),
                          scale=Vector3(x=2*BAR_RADIUS, y=2*BAR_RADIUS, z=1.2),
                          color=ColorRGBA(r=0.8, g=0.8, b=0.85, a=1.0))
        self.bar.header.frame_id = 'world'

        # Wait for RVIZ, then run at 100Hz.
        self.get_logger().info("Waiting for a /joint_states subscriber...")
        while(not self.count_subscribers('/joint_states')):
            pass
        self.t     = -self.dt
        self.now   = self.get_clock().now()
        self.timer = self.create_timer(self.dt, self.update)
        self.get_logger().info("Running '%s', target %.2f, %s" % (self.motion,
            self.target, 'grip first' if self.prio else 'equal weights'))

    def shutdown(self):
        self.timer.destroy()
        self.destroy_node()

    # Update - one IK step every cycle.
    def update(self):
        self.t   = self.t   + self.dt
        self.now = self.now + rclpy.time.Duration(seconds=self.dt)

        (pd, vd, Rd, wd, qnom) = trajectory(self.t, self.motion, self.target)
        (qdot, e) = self.g1.step(pd, vd, Rd, wd, qnom)

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
    node = MuscleUpNode('muscleup', future)
    try:
        rclpy.spin_until_future_complete(node, future)
    except KeyboardInterrupt:
        pass
    node.shutdown()
    if rclpy.ok():
        rclpy.shutdown()

if __name__ == "__main__":
    main()
