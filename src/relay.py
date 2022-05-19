#!/usr/bin/python

import rospy
import serial
import time
import gpiozero
from arm_relay.msg import JointRateCommand, ActuatorFeedback, ABStatus
from arm_relay.srv import HomeArmBase, HomeArmBaseResponse, EnableActuators, EnableActuatorsResponse, ZeroAxis, ZeroAxisResponse
from std_msgs import *


class ArmRelay:
    def __init__(self):
        rospy.loginfo('Setting up node.')
        rospy.init_node('arm_base_relay')

        # Initialize serial port and connect to teensy microcontroller
        self.ser = None
        while (self.ser is None):
            port = rospy.get_param('~arm_teensy_serial_port', '/dev/serial0')
            try:
                self.ser = serial.Serial(port, 115200)
            except:
                rospy.loginfo(
                    "Unable to locate Teensy (@{0}), please ensure communications.".format(port))
                time.sleep(3)
        rospy.loginfo('Connected to teensy.')

        self.status_pub = rospy.Publisher('status', msg.String, queue_size=10)
        self.feedback_pub = rospy.Publisher(
            'feedback', ActuatorFeedback, queue_size=1)
        self.ab_status_pub = rospy.Publisher(
            'ab_status', ABStatus, queue_size=1)
        self.loop_count = 0

        self.cmd_rate_sub = rospy.Subscriber(
            "/joint_rate_command", JointRateCommand, self.process_rate_cmd)

        self.homing_service = rospy.Service(
            "/home_arm_base", HomeArmBase, self.home_arm_base)
        self.homing_in_progress = False
        self.homing_success = False

        self.enable_service = rospy.Service(
            "/enable_actuators", EnableActuators, self.enable_actuators)
        self.actuators_enabled = False
        self.enable_pin = gpiozero.OutputDevice(
            pin=4, active_high=True, initial_value=False)

        self.zero_service = rospy.Service(
            "/zero_actuators", ZeroAxis, self.zero_axis)
        self.axis_1_offset = 0
        self.set_offset = False

        self.topic_publisher_callback = {
            'status': self.process_status,
            'feedback': self.process_feedback,
            'response': self.process_status,
            'error': self.process_status,
            'homing_status': self.process_homing_status
        }

    def zero_axis(self, zero_srv):
        if zero_srv.axis == 1:
            self.set_offset = True
            return ZeroAxisResponse(True)
        else:
            return ZeroAxisResponse(False)

    def home_arm_base(self, home_srv):
        if home_srv.force or (not self.homing_in_progress and not self.homing_success):
            if home_srv.axis1Low:
                self.ser.write('l 0\n')
            else:
                self.ser.write('h 0\n')
            self.homing_in_progress = True

        return HomeArmBaseResponse(self.homing_success)

    def process_homing_status(self, status):
        self.homing_in_progress = False

        if "false" in status:
            self.homing_success = False
        elif "true" in status:
            self.homing_success = True

    def enable_actuators(self, enable_srv):
        if self.actuators_enabled and not enable_srv.enable:
            self.enable_pin.off()
            rospy.loginfo('Disabling Actuators')
        elif not self.actuators_enabled and enable_srv.enable:
            self.enable_pin.on()
            rospy.loginfo('Enabling Actuators')

        self.actuators_enabled = self.enable_pin.value
        return EnableActuatorsResponse(self.actuators_enabled)

    def process_rate_cmd(self, rate_cmd):
        if rate_cmd.axis > 4:
            return

        self.ser.write(str(rate_cmd.axis) + ',' +
                       str(rate_cmd.desiredRate) + "\n")

    def process_status(self, data):
        self.status_pub.publish(str(data))

    def process_ab_status(self):
        ab_status = ABStatus()
        ab_status.enabled = self.actuators_enabled
        ab_status.homed = self.homing_success
        self.ab_status_pub.publish(ab_status)

    def process_feedback(self, data):
        actuatorFeedback = ActuatorFeedback()

        # parse input data into keys for reference
        args = data.split(',')
        results = {}
        for pair in args:
            name, value = pair.split('=')
            results[name] = value

        actuatorFeedback.axis = int(results['x'])
        actuatorFeedback.angle = float(results['a'])
        if self.set_offset:
            self.axis_1_offset = actuatorFeedback.angle
            self.set_offset = False
        actuatorFeedback.angle -= self.axis_1_offset
        actuatorFeedback.actualRate = float(results['r'])

        self.feedback_pub.publish(actuatorFeedback)

    def process_message(self, message):
        args = message.split(";")
        topic = args[0]
        data = args[1]
        try:
            self.topic_publisher_callback[topic](data)
        except Exception as e:
            print(e)

    def run(self):
        rate = rospy.Rate(60)
        while not rospy.is_shutdown():
            while self.ser.in_waiting > 0:
                message = self.ser.read_until('\n')
                print(message)
                self.process_message(message)
            self.loop_count += 1
            if self.loop_count >= 60:
                self.loop_count = 0
                self.process_ab_status()
            rate.sleep()


if __name__ == '__main__':
    relay = ArmRelay()
    relay.run()
