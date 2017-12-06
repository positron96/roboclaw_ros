#!/usr/bin/env python
from math import pi, copysign

import diagnostic_msgs
import diagnostic_updater
import roboclaw_driver.roboclaw_driver as roboclaw
import rospy
import tf
from roboclaw.msg import MotorSpeeds
from geometry_msgs.msg import Quaternion, Twist
from nav_msgs.msg import Odometry


class Node:
    def __init__(self):

        self.ERRORS = {0x0000: (diagnostic_msgs.msg.DiagnosticStatus.OK, "Normal"),
                       0x0001: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "M1 over current"),
                       0x0002: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "M2 over current"),
                       0x0004: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Emergency Stop"),
                       0x0008: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Temperature1"),
                       0x0010: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Temperature2"),
                       0x0020: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Main batt voltage high"),
                       0x0040: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Logic batt voltage high"),
                       0x0080: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Logic batt voltage low"),
                       0x0100: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "M1 driver fault"),
                       0x0200: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "M2 driver fault"),
                       0x0400: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "Main batt voltage high"),
                       0x0800: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "Main batt voltage low"),
                       0x1000: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "Temperature1"),
                       0x2000: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "Temperature2"),
                       0x4000: (diagnostic_msgs.msg.DiagnosticStatus.OK, "M1 home"),
                       0x8000: (diagnostic_msgs.msg.DiagnosticStatus.OK, "M2 home")}

        rospy.init_node("roboclaw_node")
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo("Connecting to roboclaw")
        dev_name = rospy.get_param("~dev", "/dev/ttyACM0")
        baud_rate = int(rospy.get_param("~baud", "115200"))

        self.topic_timeout = int( rospy.get_param("~topic_timeout", "1") )
        self.TICKS_PER_RADIAN = float(rospy.get_param("~ticks_per_rotation", "663")) / (2*pi)
        self.MAX_SPEED = float(rospy.get_param("~max_speed", "0"))

        addresses = str(rospy.get_param("~addresses", "128")).split(",")
        self.addresses = []
        for addr in addresses:
            addr = int(addr)
            if addr > 0x87 or addr < 0x80:
                rospy.logfatal("Address out of range")
                rospy.signal_shutdown("Address out of range")
            self.addresses.append( addr )
        rospy.loginfo("Addresses: %s" % str(self.addresses) )

        # TODO need some way to check if address is correct
        try:
            roboclaw.Open(dev_name, baud_rate)
        except Exception as e:
            rospy.logfatal("Could not connect to Roboclaw serial device")
            rospy.logdebug(e)
            rospy.signal_shutdown("Could not connect to Roboclaw")
            return

        self.updater = diagnostic_updater.Updater()
        self.updater.setHardwareID("Roboclaw")
        self.updater.add(diagnostic_updater.
                         FunctionDiagnosticTask("Vitals", self.check_vitals))

        for addr in self.addresses:
            version = None
            try:
                version = roboclaw.ReadVersion(addr)
            except Exception as e:
                rospy.logwarn("Problem getting roboclaw version from address %d" % addr)
                rospy.logdebug(e)
                
            if version is None or not version[0]:
                rospy.logwarn("Could not get version from roboclaw address %d" % addr)
                rospy.signal_shutdown("Could not get version from roboclaw address %d" % addr)
                return
            
            rospy.logdebug("Controller %d version is %s" % (addr, repr(version[1]) ) )

            roboclaw.SpeedM1M2(addr, 0, 0)
            roboclaw.ResetEncoders(addr)

        self.last_set_speed_time = rospy.get_rostime()

        rospy.Subscriber("cmd_motors", MotorSpeeds, self.cmd_motors_callback)

        rospy.sleep(1)

        rospy.logdebug("dev %s", dev_name)
        rospy.logdebug("baud %d", baud_rate)
        rospy.logdebug("addresses %s", str(self.addresses) )
        rospy.logdebug("max_speed %f", self.MAX_SPEED)
        rospy.logdebug("ticks_per_radian: %f", self.TICKS_PER_RADIAN)

    def run(self):
        rospy.loginfo("Starting motor drive")
        r_time = rospy.Rate(10)
        while not rospy.is_shutdown():

            if (rospy.get_rostime() - self.last_set_speed_time).to_sec() > self.topic_timeout :
                rospy.loginfo("Did not get command for specified time, stopping")

                self.cmd([0,0]*len(self.addresses) )

            # TODO need find solution to the OSError11 looks like sync problem with serial
            # status1, enc1, crc1 = None, None, None
            # status2, enc2, crc2 = None, None, None

            # try:
            #     status1, enc1, crc1 = roboclaw.ReadEncM1(self.address)
            # except ValueError:
            #     pass
            # except OSError as e:
            #     rospy.logwarn("ReadEncM1 OSError: %d", e.errno)
            #     rospy.logdebug(e)

            # try:
            #     status2, enc2, crc2 = roboclaw.ReadEncM2(self.address)
            # except ValueError:
            #     pass
            # except OSError as e:
            #     rospy.logwarn("ReadEncM2 OSError: %d", e.errno)
            #     rospy.logdebug(e)

            # if enc1 is not None and enc2 is not None:
            #     rospy.logdebug(" Encoders %d %d" % (enc1, enc2))

            self.updater.update()
            r_time.sleep()

    def cmd_motors_callback(self, arr):
        self.cmd(arr.Speeds)
        rospy.loginfo( str(arr) )


    def cmd(self, arr):
        self.last_set_speed_time = rospy.get_rostime()

        if len(arr) != len(self.addresses)*2:
            rospy.logwarn( "node configured to control %d motors but %d speeds received" % (
                len(self.addresses), len(arr) )  ) 
            return 

        i=0
        for addr in self.addresses:
            # convert radians per sec to ticks per sec
            vr = int( arr[i]  * self.TICKS_PER_RADIAN )  
            vl = int( arr[i+1]* self.TICKS_PER_RADIAN )

            # clamp
            if self.MAX_SPEED != 0 :
                if abs(vr) > self.MAX_SPEED :  vr = copysign(self.MAX_SPEED, vr)
                if abs(vl) > self.MAX_SPEED :  vl = copysign(self.MAX_SPEED, vr)

            # now send
            try:                
                if vr is 0 and vl is 0:
                    roboclaw.ForwardM1(addr, 0)
                    roboclaw.ForwardM2(addr, 0)
                else:
                    roboclaw.SpeedM1M2(addr, vr, vl)
            except OSError as e:
                rospy.logwarn("SpeedM1M2 OSError: %d", e.errno)
                rospy.logdebug(e)

            i += 2



    # TODO: Need to make this work when more than one error is raised
    def check_vitals(self, stat):
        rospy.loginfo("checking vitals")
        for addr in self.addresses:
            try:
                status = roboclaw.ReadError(addr)[1]
            except OSError as e:
                rospy.logwarn("Diagnostics OSError: %d", e.errno)
                rospy.logdebug(e)
                continue
            state, message = self.ERRORS[status]
            stat.summary(state, message)
            try:
                stat.add("%d Main Batt V:" % addr, float(roboclaw.ReadMainBatteryVoltage(addr)[1] / 10))
                stat.add("%d Logic Batt V:" % addr, float(roboclaw.ReadLogicBatteryVoltage(addr)[1] / 10))
                stat.add("%d Temp1 C:" % addr, float(roboclaw.ReadTemp(addr)[1] / 10))
                stat.add("%d Temp2 C:" % addr, float(roboclaw.ReadTemp2(addr)[1] / 10))
            except OSError as e:
                rospy.logwarn("Diagnostics OSError: %d", e.errno)
                rospy.logdebug(e)
        return stat

    # TODO: need clean shutdown so motors stop even if new msgs are arriving
    def shutdown(self):
        rospy.loginfo("Shutting down")
        if roboclaw.IsOpened():
            for addr in self.addresses:
                try:
                    roboclaw.ForwardM1(addr, 0)
                    roboclaw.ForwardM2(addr, 0)
                except OSError:
                    rospy.logerr("Shutdown did not work trying again")
                    try:
                        roboclaw.ForwardM1(addr, 0)
                        roboclaw.ForwardM2(addr, 0)
                    except OSError as e:
                        rospy.logerr("Could not shutdown motors!!!!")
                        rospy.logdebug(e)


if __name__ == "__main__":
    try:
        node = Node()
        node.run()
    except rospy.ROSInterruptException:
        pass
    rospy.loginfo("Exiting")
