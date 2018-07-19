"""Return control from trajectory optimizer."""
import rospy
from std_msgs.msg import Bool, Float64
from autorally_msgs.msg import chassisCommand, chassisState, pathIntegralStatus
from imitation_learning.expert import Expert


class AutoRallyExpertMPPI(Expert):
    """Translate control commands from the MPPI trajectory optimizer."""

    def __init__(self, control_topic='/expert_controller/expert_command',
                 status_topic='/expert_controller/expertStatus',
                 cost_topic='/expert_controller/expertCost',
                 autonomous = False):
        """Initialize the node."""
        self._autonomous = autonomous
        self._action = None
        self._status = None
        self._cost = None
        self.status_topic = status_topic
        self.cost_topic = cost_topic
        self.policy_sub = rospy.Subscriber(control_topic, chassisCommand, self.save_control)
        self.cost_sub = rospy.Subscriber(cost_topic, pathIntegralStatus, self.save_cost)
        self.status_sub = rospy.Subscriber(status_topic, pathIntegralStatus, self.save_status)
        rospy.loginfo("Expert listener started\n")

    def save_control(self, msg):
        """Save the latest control command."""
        self._action = [msg.steering, msg.throttle]

    def save_cost(self, msg):
        """Save the latest cost."""
        #self.cost = msg.data
        self._cost = msg.status

    def save_status(self, msg):
        """Save the expert status"""
        self._status = msg.status

    def autonomous(self):
        """Return the autonomous mode"""
        return self._autonomous

    def action(self, obs=None):
        """Return the latest control command"""
        return self._action

    def ready(self):
        """Return the expert status"""
        return self._status
        
    def cost(self):
        """Return the latest cost"""
        return self._cost
