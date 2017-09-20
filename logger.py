#!/usr/bin/python
# Make sure that this is python 2

# ROS imports
import rospy
from sensor_msgs.msg import Joy 

# System imports
import subprocess
import psutil
import time
import signal
import sys
import os
from datetime import datetime

PRESSED = 1
RECORD = False
RECORDING = False

log_directory = "/home/tuckerhaydon/Desktop/logs/"
save_directory = ""

commands = []

class Command: 
    def __init__(self, command, filename):
        self.command = command
        self.filename = filename if filename != "" else None
        self.id = None
        self.file = None


    def start(self):
        if(self.filename is not None):
            self.file = open(save_directory + self.filename, "w+")

        self.id = subprocess.Popen(self.command, stdout=self.file, stdin=subprocess.PIPE, cwd=save_directory, shell=True)


    def stop(self):
        if self.id is not None:
            process = psutil.Process(self.id.pid)
            for sub_process in process.children(recursive=True):
                sub_process.send_signal(signal.SIGINT)

            self.id.wait()  # Wait for children to terminate
            self.id = None

        if self.file is not None:
            self.file.close()
            self.file = None


def joy_callback(joy_data):
    global RECORD

    if( joy_data.buttons[2] == PRESSED or 
        joy_data.buttons[3] == PRESSED or 
        joy_data.buttons[1] == PRESSED):
        RECORD = True
    if joy_data.buttons[0] == PRESSED:
        RECORD = False


def start_record_data():
    global save_directory
    rospy.loginfo("Start record data")
    now = datetime.now()
    save_directory = log_directory + "{0}-{1}-{2}-{3}-{4}-{5}/".format(now.year, now.month, now.day, now.hour, now.minute, now.second)
     
    rospy.loginfo("Saving in directory: " + save_directory)
    subprocess.call(["mkdir", save_directory])

    for command in commands:
        command.start()


def stop_record_data():
    rospy.loginfo("Stop record data")
    for command in commands:
        command.stop()

def sigint_handler(signal, frame):
    stop_record_data()
    sys.exit(0)

def main():
    global RECORDING, log_directory

    # Control-c handler
    signal.signal(signal.SIGINT, sigint_handler)

    # Node name and initialization
    rospy.init_node("logger", anonymous=True)

    # Subscribers
    rospy.Subscriber("/joy", Joy, joy_callback)

    log_directory = rospy.get_param("~log_directory")
    cmds = rospy.get_param("~commands")

    for cmd, fname in cmds.iteritems():
        commands.append(Command(cmd, fname))

    # commands.append(Command("ping google.com", filename="ping_test.txt"))
    # commands.append(Command("rosbag record -a -O data.bag", filename=None))
    
    # Main ROS thread
    while not rospy.is_shutdown():
        if(RECORD == True and RECORDING == False):
            RECORDING = True
            start_record_data()
        if(RECORD == False and RECORDING == True):
            RECORDING = False
            stop_record_data()

# Code entry point
if __name__ == '__main__':
    main()

