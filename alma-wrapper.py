#!/usr/bin/env python

import subprocess
#import rospy
import sys
import string
import time
import argparse
import random
import re
#from std_msgs.msg import String
#from actions import *

ap = argparse.ArgumentParser()
ap.add_argument("-a", "--alma", required=False, help="directory for ALMA")
args = vars(ap.parse_args())

alma_dir = "./src/julia/alma/"
if args["alma"]:
  alma_dir = args["alma"]

alma = subprocess.Popen(["./alma.x", "./demo/move.pl"], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, shell=False, cwd=alma_dir)

canDo = []
next_action = None

def main():
  #rospy.init_node('almabridge')
  #rospy.Subscriber("alma_in", String, add)

  # Print initial KB, before beginning
  output = alma.stdout.readline()
  while output != "\n":
    sys.stdout.write(string.replace(output, "alma: ", ""))
    output = alma.stdout.readline()
  sys.stdout.write("\n")

  while True:
    read_input()

    alma_step() 
    alma_print()
    
    read_KB()
    agent_process()
    send_output()

    sys.stdout.write("\n")
    time.sleep(1)


# Takes collected ROS input from Simulator-Wrapper, issues adds/deletes to ALMA based on
def read_input():
  # Currently feeds this without ROS, as simple start
  global next_action
  input = ["empty(left)", "empty(right)", "empty(up)", "empty(down)"]
  if next_action != None:
    input.append("not(doing(" + next_action + "))")
    next_action = None
  for line in input:
    alma_add(line)
    sys.stdout.write(alma.stdout.readline())
  return


# Parse KB obtained from ALMA, updating data structures
def read_KB():
  global canDo
  canDo = []

  regexp = re.compile("[0-9]+: canDo\((.*)\) \(parents.*$")
  output = alma.stdout.readline()
  while output != "\n":
    sys.stdout.write(string.replace(output, "alma: ", ""))

    search = regexp.search(output)
    if search:
      canDo.append(search.group(1))

    output = alma.stdout.readline()

# Processes ALMA KB contents
# Includes selection of action, if currently able to act
def agent_process():
  global next_action
  if len(canDo) > 0:
    pos = random.randint(0, len(canDo)-1)
    for idx, action in enumerate(canDo):
      if idx == pos:
        next_action = action
        alma_update("canDo(" + action + ")", "doing(" + action + ")")
        sys.stdout.write(alma.stdout.readline())
      else:
        alma_del("canDo(" + action + ")")
        sys.stdout.write(alma.stdout.readline())


# Pass ROS output to Simulator-Wrapper, especially chosen actions
def send_output():
  # TODO: pass along next_action
  return


# Functions for issuing ALMA commands:

def alma_print():
  alma.stdin.write("print\n")

def alma_step():
  alma.stdin.write("step\n")

def alma_add(data):
  alma.stdin.write("add " + data + ".\n")

def alma_del(data):
  alma.stdin.write("del " + data + ".\n")

def alma_update(target, update):
  alma.stdin.write("update " + target + ". " + update + ".\n")


if __name__ == '__main__':
  main()

