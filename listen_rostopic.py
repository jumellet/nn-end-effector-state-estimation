from time import sleep, time
from scipy import zeros
import rospy
from gazebo_msgs.srv import GetModelState, GetLinkState
from sensor_msgs.msg import Imu, MagneticField
from mavros_msgs.msg import RCIn, RCOut
import csv
import numpy as np

data_output = zeros(7)
data_input = zeros(18)
data_external = zeros(4)

tStart = time()
duration = 1    # sec

imu_acc = Imu().linear_acceleration
imu_gyro = Imu().angular_velocity
mag = MagneticField().magnetic_field
motor = RCOut().channels
joy = RCIn().channels

def imu_callback(data):
  global imu_acc, imu_gyro, data_input
  imu_acc = data.linear_acceleration
  imu_gyro = data.angular_velocity

  data_input[0] = time()
  data_input[1] = imu_acc.x
  data_input[2] = imu_acc.y
  data_input[3] = imu_acc.z
  data_input[4] = imu_gyro.x
  data_input[5] = imu_gyro.y
  data_input[6] = imu_gyro.z
  #print(data_input)

def mag_callback(data):
  global mag, data_input
  mag = data.magnetic_field

  data_input[7] = mag.x
  data_input[8] = mag.y
  data_input[9] = mag.z
  #print(data_input)

def motor_callback(data):
  global motor, data_input
  motor = data.channels

  data_input[10] = motor[0]
  data_input[11] = motor[1]
  data_input[12] = motor[2]
  data_input[13] = motor[3]
  #print(motor[0])

def joy_callback(data):
  global joy, data_input
  joy = data.channels

  data_input[14] = joy[0]
  data_input[15] = joy[1]
  data_input[16] = joy[2]
  data_input[17] = joy[3]
  #print(joy[0])

def listener():
    rospy.init_node('get_position', anonymous=True)
    get_link_state  = rospy.ServiceProxy("/gazebo/get_link_state", GetLinkState)
    rospy.wait_for_service("/gazebo/get_link_state")
    get_model_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
    rospy.wait_for_service("/gazebo/get_model_state")

    # Drone sensors measurements
    rospy.Subscriber('mavros/imu/data_raw', Imu, imu_callback)
    rospy.Subscriber('mavros/imu/mag', MagneticField, mag_callback)
    rospy.Subscriber('mavros/rc/out', RCOut, motor_callback)
    rospy.Subscriber('mavros/rc/in', RCIn, joy_callback)
    
    try:  
      endeffector_state = get_link_state("drone1::drone1::end-effector","drone1")
      drone_state = get_model_state('drone1', 'world')
      
      # Endeffector pose
      data_output[0] = time()
      data_output[1] = endeffector_state.link_state.pose.position.x
      data_output[2] = endeffector_state.link_state.pose.position.y
      data_output[3] = endeffector_state.link_state.pose.position.z
      data_output[4] = endeffector_state.link_state.pose.orientation.x
      data_output[5] = endeffector_state.link_state.pose.orientation.y
      data_output[6] = endeffector_state.link_state.pose.orientation.z

      # Drone pose
      data_external[0] = time()
      data_external[1] = drone_state.pose.position.x
      data_external[2] = drone_state.pose.position.y
      data_external[3] = drone_state.pose.position.z

    except Exception as e:
      rospy.logerr('Error on calling service: %s',str(e))
      return

if __name__ == '__main__':
  with open("/home/bismuth/Downloads/dataset-only-input-1.csv", 'w') as file_input, open("/home/bismuth/Downloads/dataset-only-output-1.csv", 'w') as file_output, open("/home/bismuth/Downloads/dataset-all-1.csv", 'w') as file_all, open("/home/bismuth/Downloads/dataset-external-1.csv", 'w') as file_external:
    
    writer_input = csv.writer(file_input)
    writer_output = csv.writer(file_output)
    writer_all = csv.writer(file_all)
    writer_external = csv.writer(file_external)

    # TODO: Add header row with column titles
    writer_input.writerow(['t', 'x_acc', 'y_acc', 'z_acc', 'x_gyro', 'y_gyro', 'z_gyro', 'x_mag', 'y_mag', 'z_mag', 'm_1', 'm_2', 'm_3', 'm_4', 'j_1', 'j_2', 'j_3', 'j_4'])
    writer_output.writerow(['t', 'x', 'y', 'z', 'r_x', 'r_y', 'r_z'])
    writer_all.writerow(['t_gz', 'x', 'y', 'z', 'r_x', 'r_y', 'r_z', 't_mavros', 'x_acc', 'y_acc', 'z_acc', 'x_gyro', 'y_gyro', 'z_gyro', 'x_mag', 'y_mag', 'z_mag', 'm_1', 'm_2', 'm_3', 'm_4', 'j_1', 'j_2', 'j_3', 'j_4'])
    writer_external.writerow(['t', 'x_w', 'y_w', 'z_w'])

    while (time() - tStart) < duration:
      try:
        listener()

        writer_input.writerow(data_input)
        writer_output.writerow(data_output)
        writer_all.writerow(np.concatenate((data_output, data_input)))
        writer_external.writerow(data_external)
        
        #print("----------------------------------")
        #print("NN input", data_input)
        #print("NN output", data_output)
      except rospy.ROSInterruptException:
        pass

      #sleep(.001)