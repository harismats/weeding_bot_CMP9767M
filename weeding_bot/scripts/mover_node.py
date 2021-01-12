#!/usr/bin/env python

import rospy
import sys
import tf
import math

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from scipy.spatial.transform import Rotation as R

from std_msgs.msg import Int16




class Mover:

  def __init__(self, robot_name):
    #get the name prefix
    self.robot_name = robot_name

    #publish crop row number we are at
    self.crop_row = Int16()
    self.crop_row_pub = rospy.Publisher("/crop_row", Int16, queue_size=10)


  #client that sends our goals to move base
  def movebase_client(self, wanted_x, wanted_y, wanted_yaw):

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    #create goal and fill it with the wanted info
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = wanted_x
    goal.target_pose.pose.position.y = wanted_y

    #initialise rotation from euler angles
    r = R.from_euler('xyz', [0, 0, wanted_yaw], degrees=True)
    #print(r.as_quat().shape)

    #fill our wanted orientation in quaternion 
    goal.target_pose.pose.orientation.x = r.as_quat()[0]
    goal.target_pose.pose.orientation.y = r.as_quat()[1]
    goal.target_pose.pose.orientation.z = r.as_quat()[2]
    goal.target_pose.pose.orientation.w = r.as_quat()[3]

    #send the goal
    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
      rospy.logerr("Action server not available!")
      rospy.signal_shutdown("Action server not available!")
    else:
      return client.get_result()



  #check if the server has done the motion and print the appropriate message
  def check_move_result(self, wanted_x, wanted_y, wanted_yaw, result):
    if result:
      print('Goal: x={}, y={}, yaw={}, has been reached!'.format(wanted_x, wanted_y, wanted_yaw))
    else:
      print('Problem executing motion to: x={}, y={}, yaw={}'.format(wanted_x, wanted_y, wanted_yaw))



  #publish the crop row we are currently at
  def publish_crop_row(self, crop_row_id):
    
    #update message with current row id
    self.crop_row.data = crop_row_id
    self.crop_row_pub.publish(self.crop_row)



  #send a new goal to the robot
  def sendNewGoal(self, x, y, yaw, crop_row_id):
    result = self.movebase_client(x, y, yaw)
    self.check_move_result(x, y, yaw, result)
    self.publish_crop_row(0)





def main(args):
  #initialize node
  rospy.init_node('mover_node', anonymous=True)

  #get arguments and instantiate object of class
  robot_name = sys.argv[1].decode('string-escape')

  #create object of class Mover
  mover_object = Mover(robot_name)


  ##-------------------------------------------##
  ##---------   Start sending goals  ----------##
  ##-------------------------------------------##

  #row 1 back and forth
  mover_object.sendNewGoal(5.25, -3, 180, 0)
  mover_object.sendNewGoal(-4.35, -3, 180, 0)

  #row 2 back and forth
  mover_object.sendNewGoal(-5.25, -1.95, 0, 1)
  mover_object.sendNewGoal(4.35, -1.95, 0, 1)

  #row 3 back and forth
  mover_object.sendNewGoal(5.25, 0.023, 180, 2)
  mover_object.sendNewGoal(4.35, 0.023, 180, 2)

  #row 4 back and forth
  mover_object.sendNewGoal(-5.25, 1.04, 0, 3)
  mover_object.sendNewGoal(4.35, 1.04, 0, 3)

  #row 5 back and forth
  mover_object.sendNewGoal(-5.25, 2.952, 180, 4)
  mover_object.sendNewGoal(4.35, 2.952, 180, 4)

  #row 6 back and forth
  mover_object.sendNewGoal(-5.25, 3.948, 0, 5)
  mover_object.sendNewGoal(4.35, 3.948, 0, 5)



  #start spinning
  try:
    rospy.spin()
  except rospy.ROSInterruptException:
    pass

        
if __name__ == '__main__':
  main(sys.argv)


