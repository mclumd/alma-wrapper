#!/usr/bin/env python

import subprocess
import rospy
import sys
import string
import time
import argparse
import random
import re
from threading import Lock
from std_msgs.msg import String

ap = argparse.ArgumentParser()
ap.add_argument("-a", "--alma", required=False, help="specify new directory for ALMA")
args = vars(ap.parse_args())

alma_dir = "./src/julia/alma/"
if args["alma"]:
  alma_dir = args["alma"]

alma = subprocess.Popen(["./alma.x", "./demo/move.pl"], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, shell=False, cwd=alma_dir)
lock = Lock()

def main():
  rospy.init_node('alma_wrapper')
  state = {}
  state["input"] = []
  rospy.Subscriber("agent_input", String, process_input, state["input"])
  state["output_topic"] = rospy.Publisher("agent_output", String, queue_size=1)
  state["action"] = None
  state["canDo"] = []

  # Print initial KB, before beginning
  output = alma.stdout.readline()
  while output != "\n":
    sys.stdout.write(string.replace(output, "alma: ", ""))
    output = alma.stdout.readline()
  sys.stdout.write("\n")

  while True:
    read_input(state)

    alma_step() 
    alma_print()
    
    read_KB(state)
    agent_process(state)
    send_output(state)

    sys.stdout.write("\n")
    time.sleep(1)


# Takes collected ROS input from Simulator-Wrapper, issues adds/deletes to ALMA based on
def read_input(state):
  # Currently feeds empty neighbors without ROS, as simple start
  lock.acquire()
  state["input"].append("empty(left)")
  state["input"].append("empty(right)")
  state["input"].append("empty(up)")
  state["input"].append("empty(down)")
  # Runs automatically without ROS input if following conditional is uncommented
  #if state["action"] is not None:
  #  state["input"].append("not(doing(" + state["action"] + "))")
  for line in state["input"]:
    alma_add(line)
    if line.startswith("not(doing("):
      state["action"] = None
    sys.stdout.write(alma.stdout.readline())
  del state["input"][:]
  lock.release()


# Parse KB obtained from ALMA, updating data structures
def read_KB(state):
  del state["canDo"][:]

  regexp = re.compile("[0-9]+: canDo\((.*)\) \(parents.*$")
  output = alma.stdout.readline()
  while output != "\n":
    sys.stdout.write(string.replace(output, "alma: ", ""))

    search = regexp.search(output)
    if search:
      state["canDo"].append(search.group(1))

    output = alma.stdout.readline()

# Processes ALMA KB contents
# Includes selection of action, if currently able to act
def agent_process(state):
  if len(state["canDo"]) > 0:
    pos = random.randint(0, len(state["canDo"])-1)
    for idx, action in enumerate(state["canDo"]):
      if idx == pos:
        state["action"] = action
        alma_update("canDo(" + action + ")", "doing(" + action + ")")
        sys.stdout.write(alma.stdout.readline())
      else:
        alma_del("canDo(" + action + ")")
        sys.stdout.write(alma.stdout.readline())


# Pass ROS output to Simulator-Wrapper, especially chosen actions
def send_output(state):
  if state["action"] is not None:
    state["output_topic"].publish(state["action"])


# Adds content from subscribed ROS topic to input buffer, for read_input to consume
def process_input(msg, args):
  lock.acquire()
  args.append(msg.data)
  lock.release()
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

