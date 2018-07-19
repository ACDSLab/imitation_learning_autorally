import rospy
import logging

class ROSLogging(logging.Handler):
    """Taken from https://gist.github.com/nzjrs/8712011 for logging w/ ROS."""

    MAP = {logging.DEBUG: rospy.logdebug,
           logging.INFO: rospy.loginfo,
           logging.WARNING: rospy.logwarn,
           logging.ERROR: rospy.logerr,
           logging.CRITICAL: rospy.logfatal}

    def emit(self, record):
        """Handle a log message."""
        try:
            self.MAP[record.levelno]("%s: %s" % (record.name, record.msg))
        except KeyError:
            rospy.logerr("unknown log level %s LOG: %s: %s" %
                         (record.levelno, record.name, record.msg))
