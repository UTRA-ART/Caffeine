#!/usr/bin/python3
import os
import time
import paramiko #need to install paramiko Python package

import rospy
import tf
from std_msgs.msg import Bool
from sensor_msgs.msg import NavSatFix, LaserScan, Imu, Image, PointCloud2
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

class Scheduler:
    '''
    Class to enforce startup order of Caffeine ros nodes 
    '''
    def __init__(self):
        self.node = rospy.init_node('startup_scheduler')
        self.subs = []

        # Sensors  
        self.gps_started = False
        self.zed_started = False
        self.imu_started = False
        self.lidar_started = False
        self.assign_topic('/gps/fix', 'gps_started', NavSatFix)
        self.assign_topic('/zed_node/rgb/image_rect_color', 'zed_started', Image)
        self.assign_topic('/imu/data', 'imu_started', Imu)
        self.assign_topic('/scan', 'lidar_started', LaserScan)

        # Frames ???? How to do this 
        self.utm_published = False
        self.map_published = False
        # self.assign_topic('/imu/data', 'imu_started', None)
        # self.assign_topic('/scan', 'lidar_started', None)

        # Odometry 
        self.odom_global_published = False
        self.odom_gps_published = False
        self.odom_local_published = False
        self.odom_motor_published = False
        self.assign_topic('/odometry/global', 'odom_global_published', Odometry)
        self.assign_topic('/odometry/gps', 'odom_gps_published', Odometry)
        self.assign_topic('/odometry/local', 'odom_local_published', Odometry)
        self.assign_topic('/odom', 'odom_motor_published', Odometry)

        #Cartographer
        self.tracked_pose_set = False
        self.assign_topic('/tracked_pose', 'tracked_pose_set', PoseStamped)

        # Meta
        self.manual_default_set = False
        self.scan_override_set = False 
        #self.cv_cloud_set = False
        self.cv_lane_scan_set = False
        self.assign_topic('/pause_navigation', 'manual_default_set', Bool, True)
        self.assign_topic('/scan_modified', 'scan_override_set', LaserScan)
        #self.assign_topic('/cv/lane_detections_cloud','cv_cloud_set',PointCloud2)
        self.assign_topic('/cv/lane_detections_scan2','cv_lane_scan_set',LaserScan)

        #SSH
        self.raspberry_pi2 = "10.0.0.2" #IP Address
        self.raspberry_pi3 = "10.0.0.3" #IP Address
        self.username = "ubuntu"
        self.password = "utraart2021"
    
    def abort(self, msg):
        rospy.logerr("SOMETHING FAILED. ABORTING. THIS CAUSED IT:" + msg)
        # time.sleep(100000)
        nodes = os.popen("rosnode list").readlines()
        for i in range(len(nodes)):
            nodes[i] = nodes[i].replace("\n","")
        for node in nodes:
            os.system("rosnode kill "+node)

    def assign_topic(self, topic_name, bool_name, topic_type, expected_value=None):
        self.subs += [rospy.Subscriber(topic_name, topic_type, self.get_topic_callback(bool_name, expected_value))]

    def get_topic_callback(self, bool_name, expected_val=None):
        def ret(msg):
            if expected_val is not None:
                if msg.data == expected_val:
                    setattr(self, bool_name, True)
            else:
                setattr(self, bool_name, True)
            return 
        return ret

    def wait_for_condition(self, bool_name, timeout=60):
        start = time.time()
        while not getattr(self, bool_name) and time.time() - start < timeout:
            pass
        if time.time() - start >= timeout:
            self.abort(bool_name)
            raise RuntimeError(bool_name + 'condition not met in time.')
        return 

    def wait_for_transform(self, listener, base_frame, target_frame, timeout=60):
        start = time.time()
        while time.time() - start < timeout:
            try:
                _ = listener.waitForTransform(base_frame, target_frame, rospy.Time(0), rospy.Duration(3))
                break
            except:
                continue
        if time.time() - start >= timeout:
            self.abort(target_frame)
            raise RuntimeError(target_frame + 'frame not started in time.')

    def initiate_ssh(self, ip_address, username, password):
        if ip_address == self.raspberry_pi2:
            rospy.loginfo('Initiating SSH client to Raspberry Pi 2')
        elif ip_address == self.raspberry_pi3:
            rospy.loginfo('Initiating SSH client to Raspberry Pi 3')

        self.ssh_client = paramiko.client.SSHClient()
        self.ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        self.ssh_client.connect(ip_address, username=username, password=password)

        if ip_address == self.raspberry_pi2:
            rospy.loginfo('SSH client to Raspberry Pi 2 is active')
        elif ip_address == self.raspberry_pi3:
            rospy.loginfo('SSH client to Raspberry Pi 3 is active')

    def close_ssh(self):
        ip_address, port = self.ssh_client.get_transport().getpeername()
        if ip_address == self.raspberry_pi2:
            rospy.loginfo('Closing SSH client to Raspberry Pi 2')
        elif ip_address == self.raspberry_pi3:
            rospy.loginfo('Closing SSH client to Raspberry Pi 3')

        self.ssh_client.close()

        if ip_address == self.raspberry_pi2:
            rospy.loginfo('SSH client to Raspberry Pi 2 is closed')
        elif ip_address == self.raspberry_pi3:
            rospy.loginfo('SSH client to Raspberry Pi 3 is closed')

    def unsubscribe_all(self):
        for subscriber in self.subs:
            subscriber.unregister()
    
    def run(self):
        listener = tf.TransformListener()

        # Run manual override, wait for successful result 
        rospy.loginfo('Setting manual override...')
        os.system('rostopic pub /pause_navigation std_msgs/Bool true &> /dev/null &')
        self.wait_for_condition('manual_default_set', 5)
        rospy.loginfo('Manual mode set to True.')

        # Run rosbag 
        rospy.loginfo('Starting rosbag...')
        os.system('roslaunch description setup_rosbag.launch &> /dev/null &')
        rospy.loginfo('Rosbag started.')

        # Run tf startup 
        rospy.loginfo('Initializing state publisher...')
        os.system('roslaunch description state_publisher.launch &> /dev/null &')
        rospy.loginfo('State publisher succeeded.')

        # Launch sensors, wait for success
        rospy.loginfo('Starting up sensors...')
        os.system('roslaunch sensors sensors.launch launch_state:=IGVC frame_id:=gps_link &> /dev/null &')
        self.wait_for_condition('imu_started', 35)
        self.wait_for_condition('lidar_started', 35)
        self.wait_for_condition('gps_started', 90)
        rospy.loginfo('Sensors launched.')

        # override scan 
        rospy.loginfo('Launching scan override...')
        os.system('roslaunch filter_lidar_data filter_lidar_data.launch &> /dev/null &')
        self.wait_for_condition('scan_override_set', 20)
        rospy.loginfo('Scan overriden.')

        # Run cv pipeline, wait for zed 
        rospy.loginfo('Starting CV pipeline...')
        os.system('roslaunch cv pipeline.launch launch_state:=IGVC &> /dev/null &')
        self.wait_for_condition('zed_started', 35)
        #self.wait_for_condition('cv_cloud_set', 30)
        self.wait_for_condition('cv_lane_scan_set', 30)
        rospy.loginfo('CV pipeline launched.')
        
        # Run motor control, teleop and feedback, wait for /odom 
        rospy.loginfo('Starting motor controls...')
        self.initiate_ssh(self.raspberry_pi3, self.username, self.password)

        _stdin, _stdout, _stderr = self.ssh_client.exec_command('cd ~/caffeine_ws && source devel/setup.bash && roslaunch motor_control teleop_motor_control.launch &> /dev/null &')
        print(_stdout.read().decode()) #prints the stdout of the command

        #os.system('roslaunch description motor_control_pipeline.launch launch_state:=IGVC &> /dev/null &')
        # self.wait_for_transform(listener, 'base_link', 'odom')
        # self.wait_for_condition('odom_motor_published', 30)
        rospy.loginfo('Motor controls started.')
        self.close_ssh()

        #Run motor_odom_node, wait for /odom
        rospy.loginfo('Starting motor_odom_node...')
        os.system('roslaunch motor_odom motor_odom.launch launch_state:=IGVC &> /dev/null &')
        self.wait_for_condition('odom_motor_published', 30)
        rospy.loginfo('motor_odom_node launched.')

        # Run odom, wait for odom local, global, and /utm
        rospy.loginfo('Initializing odometry...')
        #self.initiate_ssh(self.raspberry_pi2, self.username, self.password)

        #_stdin, _stdout, _stderr = self.ssh_client.exec_command('cd ~/caffeine_ws && source devel/setup.bash && roslaunch odom odom.launch launch_state:=IGVC &> /dev/null &')
        #print(_stdout.read().decode()) #prints out the stdout of the command

        os.system('roslaunch odom odom.launch launch_state:=IGVC &> /dev/null &')
        self.wait_for_condition('odom_global_published', 35)
        self.wait_for_condition('odom_local_published', 35)
        # self.wait_for_condition('odom_gps_published', 25)

        rospy.loginfo('Odometry initialized.')
        #self.close_ssh()

        # Run utm frame transform
        rospy.loginfo('Initializing UTM...')
        os.system('roslaunch description utm.launch &> /dev/null &')
        self.wait_for_transform(listener, '/map', '/utm')
        rospy.loginfo('UTM initialized.')

        #Run cartographer
        rospy.loginfo('Starting cartographer...')
        os.system('roslaunch description cartographer.launch launch_state:=IGVC &> /dev/null &')
        self.wait_for_condition('tracked_pose_set', 60)
        self.wait_for_transform(listener, '/odom','/base_link')
        self.wait_for_transform(listener, '/map', '/odom')
        rospy.loginfo('Cartographer launched.')

        # Run nav stack --- load_waypoints out, wait for /map /scan_modified
        rospy.loginfo('Initializing navigation stack...')
        os.system('roslaunch nav_stack move_base.launch launch_state:=IGVC &> /dev/null &')
        self.wait_for_transform(listener, '/base_link', '/map')
        # self.wait_for_transform(listener, '/map', '/utm')
        rospy.loginfo('Navstack initialized.')
        
        # load waypoints 
        rospy.loginfo('Loading waypoints...')
        os.system('roslaunch load_waypoints load_waypoints.launch launch_state:=IGVC &> /dev/null &')
        rospy.loginfo('Waypoints loaded.') 

        # teleop 
        #rospy.loginfo('Starting teleop...')
        #os.system('roslaunch teleop_twist_keyboard keyboard_teleop.launch')
        #rospy.loginfo('Teleop launched.')

        return True


if __name__=='__main__':
    scheduler = Scheduler()
    if scheduler.run():
        rospy.loginfo("Scheduler succeeded. Caffeine is ready to rumble!")
        scheduler.unsubscribe_all()
        while not rospy.is_shutdown():
            pass
