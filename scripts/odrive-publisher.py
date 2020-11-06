#!/usr/bin/env python3
# license removed for brevity
import rospy
import std_msgs.msg
from odrive_controller.msg import ODriveMsg
import odrive


odrives = []

pos_pubs = []
vel_pubs = []
odc_pubs = []
odv_pubs = []
mcr_pubs = []
mvq_pubs = []
mvd_pubs = []
mcm_pubs = []
tqr_pubs = []
tqm_pubs = []
rqs_pubs = []

# Callbacks
def odrive_requested_state_callback(command):
    if (command.axis==0):
        odrives[command.odrv].axis0.requested_state = int(command.input_value)
    else:
        odrives[command.odrv].axis1.requested_state = int(command.input_value)


def odrive_position_input_callback(command):
    if (command.axis == 0):
        odrives[command.odrv].axis0.controller.input_position = command.input_value
    else:
        odrives[command.odrv].axis1.controller.input_position = command.input_value

def odrive_velocity_input_callback(command):
    if (command.axis == 0):
        odrives[command.odrv].axis0.controller.input_velocity = command.input_value
    else:
        odrives[command.odrv].axis1.controller.input_velocity = command.input_value

def odrive_torque_input_callback(command):
    if (command.axis == 0):
        odrives[command.odrv].axis0.controller.input_torque = command.input_value
    else:
        odrives[command.odrv].axis1.controller.input_torque = command.input_value

# Publish all function
def publish_all():
    odrive_counter = len(odrives)
    for i in range(odrive_counter):
        # axis0
        odrv = odrives[i]
        if (pos_pubs[2*i+0]!=None):
            pos_pubs[2 * i + 0].publish(odrv.axis0.encoder.pos_estimate)
            vel_pubs[2 * i + 0].publish(odrv.axis0.encoder.vel_estimate)
            mcr_pubs[2 * i + 0].publish(odrv.axis0.motor.current_control.Iq_setpoint)
            mvq_pubs[2 * i + 0].publish(odrv.axis0.motor.current_control.v_current_control_integral_q)
            mvd_pubs[2 * i + 0].publish(odrv.axis0.motor.current_control.v_current_control_integral_d)
            mcm_pubs[2 * i + 0].publish(odrv.axis0.motor.current_control.Iq_measured)
            tqr_pubs[2 * i + 0].publish(abs(odrv.axis0.motor.current_control.Iq_setpoint) * 8.27 / 140)
            tqm_pubs[2 * i + 0].publish(8.27 * abs(odrv.axis0.motor.current_control.Iq_measured) / 140)
            rqs_pubs[2 * i + 0].publish(odrv.axis0.requested_state)
        # axis1
        if (pos_pubs[2*i+1]!=None):
            pos_pubs[2 * i + 1].publish(odrv.axis1.encoder.pos_estimate)
            vel_pubs[2 * i + 1].publish(odrv.axis1.encoder.vel_estimate)
            mcr_pubs[2 * i + 1].publish(odrv.axis1.motor.current_control.Iq_setpoint)
            mvq_pubs[2 * i + 1].publish(odrv.axis0.motor.current_control.v_current_control_integral_q)
            mvd_pubs[2 * i + 1].publish(odrv.axis0.motor.current_control.v_current_control_integral_d)
            mcm_pubs[2 * i + 1].publish(odrv.axis1.motor.current_control.Iq_measured)
            tqr_pubs[2 * i + 1].publish(abs(odrv.axis1.motor.current_control.Iq_setpoint) * 8.27 / 140)
            tqm_pubs[2 * i + 1].publish(8.27 * abs(odrv.axis1.motor.current_control.Iq_measured) / 140)
            rqs_pubs[2 * i + 1].publish(odrv.axis1.requested_state)
        # odrive
        odc_pubs[i].publish(odrv.ibus)
        odv_pubs[i].publish(odrv.vbus_voltage)

def odrive_controller():

    all_odrives = {}

    serial_numbers = {"2081398A4D4D": [True, False], "206B39984D4D": [False, False]}

    for serial_number in serial_numbers:
        try:
            new_odrive = odrive.find_any(serial_number=serial_number, timeout=5)
        except TimeoutError:
            print("Can't find: 0x"+serial_number)
        else:
            all_odrives[new_odrive] = serial_numbers[serial_number]
            print("found new odrive: "+str(hex(new_odrive.serial_number)))

    for odrv in all_odrives:
        odrives.append(odrv)

    odrive_counter = len(odrives)
    print("found "+str(odrive_counter)+" odrives")

    print("Publising topics")
    queue_size = 1000
    for i in range(odrive_counter):
        if all_odrives[odrives[i]][0]:
            odrive_name = "odrive" + str(i) + "/axis0"
            print("Publising "+odrive_name)
            pos_pubs.append(rospy.Publisher(odrive_name + '/position/read', std_msgs.msg.Float64, queue_size=queue_size))
            vel_pubs.append(rospy.Publisher(odrive_name + '/velocity/read', std_msgs.msg.Float64, queue_size=queue_size))
            mcr_pubs.append(
                rospy.Publisher(odrive_name + '/motor_current/reference/read', std_msgs.msg.Float64, queue_size=queue_size))
            mvq_pubs.append(rospy.Publisher(odrive_name + '/motor_voltage_q/read', std_msgs.msg.Float64, queue_size=queue_size))
            mvd_pubs.append(rospy.Publisher(odrive_name + '/motor_voltage_d/read', std_msgs.msg.Float64, queue_size=queue_size))
            mcm_pubs.append(
                rospy.Publisher(odrive_name + '/motor_current/measured/read', std_msgs.msg.Float64, queue_size=queue_size))
            tqr_pubs.append(
                rospy.Publisher(odrive_name + '/torque/reference_from_current/read', std_msgs.msg.Float64, queue_size=queue_size))
            tqm_pubs.append(
                rospy.Publisher(odrive_name + '/torque/read', std_msgs.msg.Float64, queue_size=queue_size))
            rqs_pubs.append(rospy.Publisher(odrive_name + "/requested_state/read", std_msgs.msg.Int32, queue_size=1))
        else:
            pos_pubs.append(None)
            vel_pubs.append(None)
            mcr_pubs.append(None)
            mcm_pubs.append(None)
            tqr_pubs.append(None)
            tqm_pubs.append(None)
            rqs_pubs.append(None)

        if all_odrives[odrives[i]][1]:
            odrive_name = "odrive" + str(i) + "/axis1"
            print("Publising " + odrive_name)
            pos_pubs.append(rospy.Publisher(odrive_name + '/position/read', std_msgs.msg.Float64, queue_size=queue_size))
            vel_pubs.append(rospy.Publisher(odrive_name + '/velocity/read', std_msgs.msg.Float64, queue_size=queue_size))
            mcr_pubs.append(
                rospy.Publisher(odrive_name + '/motor_current_reference/read', std_msgs.msg.Float64, queue_size=queue_size))
            mvq_pubs.append(
                rospy.Publisher(odrive_name + '/motor_voltage_q/read', std_msgs.msg.Float64, queue_size=queue_size))
            mvd_pubs.append(
                rospy.Publisher(odrive_name + '/motor_voltage_d/read', std_msgs.msg.Float64, queue_size=queue_size))
            mcm_pubs.append(
                rospy.Publisher(odrive_name + '/motor_current_measured/read', std_msgs.msg.Float64, queue_size=queue_size))
            tqr_pubs.append(
                rospy.Publisher(odrive_name + '/torque_reference/read', std_msgs.msg.Float64, queue_size=queue_size))
            tqm_pubs.append(
                rospy.Publisher(odrive_name + '/torque_measured/read', std_msgs.msg.Float64, queue_size=queue_size))
            rqs_pubs.append(rospy.Publisher(odrive_name + "/requested_state/read", std_msgs.msg.Int32, queue_size=queue_size))
        else:
            pos_pubs.append(None)
            vel_pubs.append(None)
            mcr_pubs.append(None)
            mcm_pubs.append(None)
            tqr_pubs.append(None)
            tqm_pubs.append(None)
            rqs_pubs.append(None)

        odrive_name = "odrive" + str(i)
        odc_pubs.append(rospy.Publisher(odrive_name + '/odrive_current/read', std_msgs.msg.Float64, queue_size=queue_size))
        odv_pubs.append(rospy.Publisher(odrive_name + '/odrive_voltage/read', std_msgs.msg.Float64, queue_size=queue_size))

    print("Initiating node...")
    rospy.init_node('odrive_controller', anonymous=True)

    for i in range(odrive_counter):
        if all_odrives[odrives[i]][0]:
            odrive_name = "odrive" + str(i) + "/axis0"
            print("Subscribing " + odrive_name)
            rospy.Subscriber(odrive_name + "/position/write", ODriveMsg, odrive_position_input_callback)
            rospy.Subscriber(odrive_name + "/velocity/write", ODriveMsg, odrive_velocity_input_callback)
            rospy.Subscriber(odrive_name + "/torque/write", ODriveMsg, odrive_torque_input_callback)
            rospy.Subscriber(odrive_name + "/requested_state/write", ODriveMsg, odrive_requested_state_callback)

        if all_odrives[odrives[i]][1]:
            odrive_name = "odrive" + str(i) + "/axis1"
            print("Subscribing " + odrive_name)
            rospy.Subscriber(odrive_name + "/position/write", ODriveMsg, odrive_position_input_callback)
            rospy.Subscriber(odrive_name + "/velocity/write", ODriveMsg, odrive_velocity_input_callback)
            rospy.Subscriber(odrive_name + "/torque/write", ODriveMsg, odrive_torque_input_callback)
            rospy.Subscriber(odrive_name + "/requested_state/write", ODriveMsg, odrive_requested_state_callback)


    print("Node started.")
    rate = rospy.Rate(1000)

    while not rospy.is_shutdown():
        publish_all()

        rate.sleep()


if __name__ == '__main__':
    try:
        odrive_controller()
    except rospy.ROSInterruptException:
        pass
