#!/usr/bin/env python

import rospy
from std_srvs.srv import Trigger, TriggerResponse
from raspicam_node.srv import TriggerContinuous, TriggerContinuousResponse

import RPi.GPIO as GPIO

class RaspiTrigger:
    def __init__(self):
        # ROS parameters
        self.trigger_channels = rospy.get_param("~trigger_channels")

        # set the GPIO mode
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.trigger_channels, GPIO.OUT, initial=GPIO.LOW)

        # Services
        rospy.Service('execute_trigger', Trigger, self.trigger)
        rospy.Service('execute_trigger_continuous', TriggerContinuous, self.trigger_continuous)

    def trigger(self, req):
        try:
            GPIO.output(self.trigger_channels, GPIO.HIGH)
            rospy.sleep(0.1)
            GPIO.output(self.trigger_channels, GPIO.LOW)
            trigger_response = "Successfull trigger"
        except:
            trigger_response = "Trigger failed"
            return TriggerResponse(success=False, message=trigger_response)

        return TriggerResponse(success=True, message=trigger_response)

    def trigger_continuous(self, req):
        rospy.wait_for_service("execute_trigger")
        for t in range(req.num_trigger):
            rospy.loginfo("Executing trigger {:2}/{:2}".format(t+1, req.num_trigger))
            try:
                execute_trigger = rospy.ServiceProxy("execute_trigger", Trigger)
                response = execute_trigger()
                rospy.loginfo(response)
            except rospy.ServiceException as e:
                rospy.logwarn("Service call failed: {}".format(e))
                trigger_continuous_response = "Trigger failed"
                return TriggerContinuousResponse(success=False, message=trigger_continuous_response)

            rospy.sleep(req.delay)

        return TriggerContinuousResponse(success=True, message="Continuous Trigger Successfull")

    def on_shutdown(self):
        # This causes weird stuff as it sets the GPIOs to IN
        # GPIO.cleanup()
        rospy.loginfo("Shutting down..")

def main():
    node_name = "raspicam_gpio_trigger"
    rospy.init_node(node_name, anonymous=False)

    rt = RaspiTrigger()

    # On shutdown
    rospy.on_shutdown(rt.on_shutdown)
    rospy.spin()

if __name__=="__main__":
    main()