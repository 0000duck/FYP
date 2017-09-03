#!/usr/bin/env python
import rospy
import message_filters
from dji_sdk.msg import Acceleration, AttitudeQuaternion
from sensor_msgs.msg import Imu

imu_m100 = Imu()

pub = rospy.Publisher("imu_m100", Imu, queue_size = 1000)
#Initialize imu_m100
imu_m100.orientation.w = 1.0
imu_m100.orientation_covariance[0] = 99999.9
imu_m100.orientation_covariance[4] = 99999.9
imu_m100.orientation_covariance[8] = 99999.9
imu_m100.header.frame_id = "imu4"#IDK why this is imu4. Just followed EuRoC.
"""
imu_m100.orientation_covariance[0] = 99999.9
Subscribe two topics: Acc and AttQuat. 
Publish one topic: imu_m100
Use two seperate callback funs.
If imu_m100.ts differs from incoming data, update header and data. If ts is the same with incoming data, update data and publish imu_m100.  
"""
#def update_header(data):
#	imu_m100.header.seq = data.header.seq
#	imu_m100.header.stamp.secs = data.header.stamp.secs
#	imu_m100.header.stamp.nsecs = data.header.stamp.nsecs

#def callback_acc(data):
#	imu_m100.linear_acceleration.x = data.ax * 9.8099
#	imu_m100.linear_acceleration.y = data.ay * 9.8099
#	imu_m100.linear_acceleration.z = data.az * 9.8099
	#if imu_m100.header.stamp.nsecs != data.header.stamp.nsecs:
#	update_header(data)
	#else:
#	pub.publish(imu_m100)#publish topic
		
#def callback_angvel(data):
#	imu_m100.angular_velocity.x = data.wx	
#	imu_m100.angular_velocity.y = data.wy	
#	imu_m100.angular_velocity.z = data.wz
	#if imu_m100.header.stamp.nsecs != data.header.stamp.nsecs:
#	update_header(data)
	#else:
#	pub.publish(imu_m100)#publish topic

def callback(Acceleration,AttitudeQuaternion):
	imu_m100.header.seq = Acceleration.header.seq
	imu_m100.header.stamp.secs = Acceleration.header.stamp.secs
	imu_m100.header.stamp.nsecs = Acceleration.header.stamp.nsecs
	imu_m100.linear_acceleration.x = Acceleration.ax * 9.8099
	imu_m100.linear_acceleration.y = Acceleration.ay * 9.8099
	imu_m100.linear_acceleration.z = Acceleration.az * 9.8099
	imu_m100.angular_velocity.x = AttitudeQuaternion.wx	
	imu_m100.angular_velocity.y = AttitudeQuaternion.wy	
	imu_m100.angular_velocity.z = AttitudeQuaternion.wz
	imu_m100.orientation.x=AttitudeQuaternion.q1
	imu_m100.orientation.y=AttitudeQuaternion.q2
	imu_m100.orientation.z=AttitudeQuaternion.q3
	imu_m100.orientation.w=AttitudeQuaternion.q0
	pub.publish(imu_m100)

def imu_m100_interface():
	rospy.init_node("imu_m100_interface",anonymous = True)
		
#	rospy.Subscriber("dji_sdk/acceleration", Acceleration, callback_acc)
#	rospy.Subscriber("dji_sdk/attitude_quaternion", AttitudeQuaternion, callback_angvel)
	Acc_sub = message_filters.Subscriber("dji_sdk/acceleration", Acceleration)
	Att_sub = message_filters.Subscriber("dji_sdk/attitude_quaternion", AttitudeQuaternion)
	ts = message_filters.TimeSynchronizer([Acc_sub, Att_sub], 10)
	ts.registerCallback(callback)
	rospy.spin()


if __name__ == '__main__':
	imu_m100_interface()
