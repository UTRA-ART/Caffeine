#!/usr/bin/env python3

import paramiko
import rospy

class SSHConnection:

    def __init__(self):
        self.node = rospy.init_node("SSH_Connection")
        #self.raspberry_pi2 = "10.0.0.2" #IP Address
        self.raspberry_pi2 = "192.168.171.155"
        self.raspberry_pi3 = "10.0.0.3" #IP Address
        self.username = "ubuntu"
        self.password = "utraart2021"

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

    def navigate_to_folder(self):
        _stdin, _stdout, _stderr = self.ssh_client.exec_command("cd ~/caffeine_ws")
        print(_stdout.read().decode()) #prints out the stdout of the command

        _stdin, _stdout, _stderr = self.ssh_client.exec_command("ls")
        print(_stdout.read().decode()) #prints out the stdout of the command

        _stdin, _stdout, _stderr = self.ssh_client.exec_command("source devel/setup.bash")
        print(_stdout.read().decode()) #prints out the stdout of the command

    def close_ssh(self):
        if ip_address == self.raspberry_pi2:
            rospy.loginfo('Closing SSH client to Raspberry Pi 2')
        elif ip_address == self.raspberry_pi3:
            rospy.loginfo('Closing SSH client to Raspberry Pi 3')

        self.ssh_client.close()

        if ip_address == self.raspberry_pi2:
            rospy.loginfo('SSH client to Raspberry Pi 2 is closed')
        elif ip_address == self.raspberry_pi3:
            rospy.loginfo('SSH client to Raspberry Pi 3 is closed')
        
    def run(self):
        self.initiate_ssh(self.raspberry_pi2, self.username, self.password)
        self.navigate_to_folder()
        self.close_ssh()

if __name__=='__main__':
    ssh_connection = SSHConnection()
    ssh_connection.run()
    
    while not rospy.is_shutdown():
        pass

