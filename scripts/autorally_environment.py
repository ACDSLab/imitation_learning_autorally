#!/usr/bin/env python
"""Interface to the AutoRally system."""
import collections as col
import numpy as np
import rospy
import ros_numpy
from autorally_msgs.msg import chassisCommand, chassisState, wheelSpeeds, runstop
from imitation_learning.environment import Environment
from keyboard.msg import Key
from sensor_msgs.msg import Image, Imu


class RCAutonomous(object):
    """Use the AutoRally RC to set/unset autonomous mode."""
    def __init__(self):
        """Initialize the subscriber and status variable."""
        chassis_topic = rospy.get_param('~chassis_topic', '/chassisState')
        self.chassis_sub = rospy.Subscriber(chassis_topic, chassisState, self.chassis_callback)
        self.autonomous_mode = False
    
    def chassis_callback(self, msg):
        """Set the autonomous mode status."""
        self.autonomous_mode = msg.autonomousEnabled
    
    @property
    def autonomous_enabled(self):
        """Return True if autonomous mode is enabled."""
        return self.autonomous_mode


class KeyboardAutonomous(object):
    """Use the keyboard to set/unset autonomous mode."""
    def __init__(self):
        """Initialize the subscriber and status variable."""
        keyboard_topic = rospy.get_param('~keyboard_autonomous_topic', '/keyboard/keydown')
        self.keyboard_sub = rospy.Subscriber(keyboard_topic, Key, self.keyboard_callback)
        self.autonomous_mode = False
    
    def keyboard_callback(self, msg):
        """Set the autonomous mode status."""
        if msg.code == Key.KEY_a:
            self.autonomous_mode = True
        elif msg.code == Key.KEY_m:
            self.autonomous_mode = False
    
    @property
    def autonomous_enabled(self):
        """Return True if autonomous mode is enabled."""
        return self.autonomous_mode


class AutoRally(Environment):
    """AutoRally system interface."""
    
    #class
    Observation = col.namedtuple('Observation', ['img_left', 'img_right', 'imu', 'ws'])

    def __init__(self, expert):
        """Initialize the AutoRally interface."""
        self.expert = expert

        # pick an object to use to determine autonomous mode
        # we need this because alpha chassis & gazebo need keyboard
        # but beta chassis and gamma chassis can use RC
        use_keyboard = rospy.get_param('~use_keyboard', True)
        if use_keyboard:
            self.autonomous_setter = KeyboardAutonomous()
        else:
            self.autonomous_setter = RCAutonomous()
        
        # human expert warmup/warning time in seconds
        self.human_delay = float(max(rospy.get_param('~human_delay', 10), 0))

        # step update frequency upper limit
        rate_hz = float(rospy.get_param('~rate_hz', 40.0))
        self.wait_duration = rospy.Duration(nsecs=int(1.0e9 * (1.0 / rate_hz)))

        # control topic
        control_topic = rospy.get_param('~control_topic', 'chassisCommand')

        # control parameters
        throttle_limits = rospy.get_param('~throttle_limits', [-np.inf, np.inf])  # (min, max)
        steering_limits = rospy.get_param('~steering_limits', [-np.inf, np.inf])  # (min, max)
        self.min_throttle = float(throttle_limits[0])
        self.max_throttle = float(throttle_limits[1])
        self.min_steering = float(steering_limits[0])
        self.max_steering = float(steering_limits[1])

        # chassis state and emergency stop topics
        chassis_topic = rospy.get_param('~chassis_topic', '/chassisState')
        runstop_topic = rospy.get_param('~runstop_topic', '/runstop')

        # observation topics
        image_left_topic = rospy.get_param('~image_left_topic', '/left_camera/cropped_image_color')
        image_right_topic = rospy.get_param('~image_right_topic', '/right_camera/cropped_image_color')
        imu_topic = rospy.get_param('~imu_topic', '/imu/imu')
        wheelspeed_topic = rospy.get_param('~wheelspeed_topic', '/wheelSpeeds')

        # observation parameters
        dim_img = np.array(rospy.get_param('~dim_img', [80, 160, 3]))
        dim_imu = rospy.get_param('~dim_imu', 6)
        dim_ws = rospy.get_param('~dim_ws', 4)

        # initialize member data
        self.control_msg = chassisCommand()
        self.control_msg.sender = "imlearn"
        self.control_msg.header.frame_id = "imlearn"
        self.control_msg.frontBrake = -5.0
        self.cost = None
        self.emergency_stop = False
        self.img_left = np.empty(dim_img)
        self.img_right = np.empty(dim_img)
        self.imu = np.empty(dim_imu)
        self.ws = np.empty(dim_ws)
        self.obs = AutoRally.Observation(img_left=np.empty(dim_img), img_right=np.empty(dim_img),
                                         imu=np.empty(dim_imu), ws=np.empty(dim_ws))
        
        # initialize subscribers
        self.chassis_sub = rospy.Subscriber(chassis_topic, chassisState, self.chassis_callback)
        self.runstop_sub = rospy.Subscriber(runstop_topic, runstop, self.runstop_callback)
        self.image_left_sub = rospy.Subscriber(image_left_topic, Image, self.image_left_callback)
        self.image_right_sub = rospy.Subscriber(image_right_topic, Image, self.image_right_callback)
        self.imu_sub = rospy.Subscriber(imu_topic, Imu, self.imu_callback)
        self.wheelspeed_sub = rospy.Subscriber(wheelspeed_topic, wheelSpeeds, self.wheelspeed_callback)
        rospy.loginfo('Initialized all subscribers')

        # initialize publishers
        self.control_pub = rospy.Publisher(control_topic, chassisCommand, queue_size=1)
        rospy.loginfo('Initialized all publishers')
    
    def chassis_callback(self, msg):
        """Process chassis state messages."""
        pass
    
    def runstop_callback(self, msg):
        """Process emergency stop messages."""
        self.emergency_stop = not msg.motionEnabled
    
    def image_left_callback(self, msg):
        """Process images from the left camera."""
        self.img_left = ros_numpy.numpify(msg)

    def image_right_callback(self, msg):
        """Process images from the right camera."""
        self.img_right = ros_numpy.numpify(msg)

    def imu_callback(self, msg):
        """Process IMU data."""
        lx, ly, lz = msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z
        ax, ay, az = msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z
        self.imu = np.array([lx, ly, lz, ax, ay, az])

    def wheelspeed_callback(self, msg):
        """Process wheel speed data."""
        self.ws = np.array([msg.lfSpeed, msg.rfSpeed, msg.lbSpeed, msg.rbSpeed])

    def publish_control(self, timestamp, steering, throttle):
        """Clamp and publish control."""
        self.control_msg.header.stamp = timestamp
        self.control_msg.throttle = np.clip(throttle, self.min_throttle, self.max_throttle)
        self.control_msg.steering = np.clip(steering, self.min_steering, self.max_steering)
        self.control_pub.publish(self.control_msg)

    def sync_observation(self):
        return AutoRally.Observation(img_left=np.copy(self.img_left), img_right=np.copy(self.img_right),
                                     imu=np.copy(self.imu), ws=np.copy(self.ws))

    @property
    def motion_enabled(self):
        """Return True if the system is ready to move, False otherwise."""
        return self.expert.ready() is not None and not self.emergency_stop

    @property
    def autonomous_enabled(self):
        """Return the autonomous enabled status of the system."""
        return self.autonomous_setter.autonomous_enabled

    def step(self, command=None):
        """Apply the control command (if any) and return the resulting data."""
        start_time = rospy.get_rostime()
        if command is not None:
            self.publish_control(start_time, steering=command[0], throttle=command[1])
        
        # force a delay before returning to approximately match the desired step rate
        sleep_duration = self.wait_duration - (rospy.get_rostime() - start_time)
        rospy.sleep(sleep_duration)
        return (self.sync_observation(), self.expert.cost(), not self.motion_enabled)

    def wait_for_rollout(self, autonomous_control):
        """Wait until ready to perform a rollout and then return the latest data."""
        rospy.loginfo('Waiting until ready to roll out')

        # start by disabling autonomous mode for safety
        rospy.loginfo('Starting confirmation checks -- switch to MANUAL mode')
        r = rospy.Rate(1)  # 1 Hz
        while self.autonomous_enabled:
            r.sleep()
        
        # make sure the user is awake and paying attention
        rospy.loginfo('Hit RED on the runstop box')
        r = rospy.Rate(1)  # 1 Hz
        while self.motion_enabled:
            r.sleep()
        
        rospy.loginfo('Hit GREEN on the runstop box')
        r = rospy.Rate(1)  # 1 Hz
        while not self.motion_enabled:
            r.sleep()
        
        if autonomous_control:
            rospy.loginfo('Ready to roll out -- switch to AUTONOMOUS mode')
            r = rospy.Rate(1)  # 1 Hz
            while not self.autonomous_enabled:
                r.sleep()
        else:
            # we are still in manual mode and should stay in manual mode
            # give the delay warning and proceed
            rospy.loginfo("Roll out in {} seconds -- start driving now!".format(str(self.human_delay)))
            rospy.sleep(self.human_delay)
        
        # Return the current observation, cost, and whether we are done due
        # to motion error
        return self.step(command=None)
