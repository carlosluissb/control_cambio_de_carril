#!/usr/bin/env python

import rospy, math, numpy
from std_msgs.msg import Float64
from ackermann_msgs.msg import AckermannDriveStamped

current = 0.0
kind = 'steering'
reference_msg = AckermannDriveStamped()

def updateCurrentCB(current_msg):
  global kind
  global current
  if kind == "speed":
    current = current_msg.data # / 3.6 # from km/h to m/s
  if kind == "steering":
    current = current_msg.data * (math.pi/180) / 18 # from degrees to rad and steering wheel relation 

def updateReferenceCB(ref_ack):
  global reference_msg
  reference_msg = ref_ack   

if __name__ == '__main__':    
  try: 

    rospy.init_node('slope')
    reference_topic = rospy.get_param('~reference_topic', 'local_planner/ackermann_reference') 
    current_topic = rospy.get_param('~current_topic', 'can/current_steering') 
    output_topic = rospy.get_param('~control_topic', 'control/target_steering')
    
    slope = rospy.get_param('~slope', 2.5)
    hz = rospy.get_param('~hz', 20)
    vehicle = rospy.get_param('vehicle', 'ada')
    kind =  rospy.get_param('~kind', 'steering')

    frame_id = rospy.get_param('frame_id', '/rear_wheel_axis')
    frame_id = vehicle + frame_id

    rospy.Subscriber(reference_topic, AckermannDriveStamped, updateReferenceCB, queue_size=1)
    rospy.Subscriber(current_topic, Float64, updateCurrentCB, queue_size=1)

    cmd_pub = rospy.Publisher(output_topic, Float64, queue_size=1)

    reference = 0.0
    ix = 0
    output_cmd = Float64()
    output_publish_transf = Float64()
    reference_msg.drive.acceleration = 1
    reference_msg.drive.steering_angle_velocity = 0.1

    #rospy.loginfo("Slope Node started.\nListening to %s, publishing to %s. current: %s", reference_topic, output_topic, current_topic)
    
    rate = rospy.Rate(hz)

    while not rospy.is_shutdown():
      rate.sleep()

      tolerance = 0.1

      if kind == "speed":        
        slope = reference_msg.drive.acceleration
        reference = reference_msg.drive.speed
        tolerance = 0.1
      elif kind == "steering":
        slope = 1 * math.pi/ 180 #reference_msg.drive.steering_angle_velocity
        reference = reference_msg.drive.steering_angle
        tolerance = 1*math.pi/180

      print("reference, current, slop, kind", reference, current, slope, kind)
      
      if abs(reference - current) >= tolerance:
        At = abs((reference - current) / (75.0*math.pi/180/ 18))
        dy = (reference - current) / (At * hz)
        if (abs(output_cmd.data - reference)) >= tolerance:
          output_cmd.data = output_cmd.data + dy	
      else:
        print("already in reference")
	output_cmd.data = reference
      if kind == "speed":
	output_publish_transf.data = output_cmd.data # * 3.6 
      elif kind == "steering":    
	output_publish_transf.data = output_cmd.data * 180/math.pi *18
      
      cmd_pub.publish(output_publish_transf)

  except rospy.ROSInterruptException:
    pass

