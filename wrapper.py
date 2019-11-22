#!/usr/bin/env python

"""
Basic script wrapping ALMA
"""

import subprocess
#import rospy
import sys
import string
import time
import argparse
import random
#from std_msgs.msg import String
#from actions import *

ap = argparse.ArgumentParser()
ap.add_argument("-a", "--alma", required=False, help="directory for ALMA")
args = vars(ap.parse_args())

alma_dir = "./src/julia/alma/"
if args["alma"] is not None:
  alma_dir = args["alma"]

alma = subprocess.Popen(["./alma.x", "./demo/move.pl"], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, shell=False, cwd=alma_dir)


def main():
  #rospy.init_node('almabridge')
  #rospy.Subscriber("alma_in", String, add)
  loop()


# Match up parentheses so we can figure out if a line is just a single ROS command
def find_parens(s):
  toret = {}
  pstack = []

  for i, c in enumerate(s):
    if c == '(':
      pstack.append(i)
    elif c == ')':
      toret[pstack.pop()] = i

  return toret


def loop():

  # Discard output from a prior command (i.e. add/del/observe currently)
  #while output != "\n" and not "added" in output and not "removed" in output and not "observed" in output:
  #  sys.stdout.write(output)
  #  output = alma.stdout.readline()
  #sys.stdout.write("\n")
  #sys.stdout.flush()

  while True:
    # print, read through for any ROS commands, then step
    alma.stdin.write('print\n')
    output = alma.stdout.readline()

    canDo = []
    while output != "\n":
      # Currently fails if following is removed; investigate
      sys.stdout.write(string.replace(output, "alma: ", ""))

      if len(output.split(": canDo(")) > 1:
        action = output.split(": canDo")[1].split(" (parents:")[0].strip()
        # This will only consider a ros command if the line is a single command
        # e.g. ros(speak())
        # Note that the commands need parens like a function would
        #if output[:find_parens(output)[0] + 1] == output:
          # We need to check if these are for the current time step
          # But now() is always the last printed, so hold onto these until we know what time it is
          #roscommands.append(output[1:-1])
        canDo.append(action[1:-1])
        alma.stdin.write("del canDo" + action + ".\n")

      # Find now()
      #if len(output.split(": now(")) > 1:
      #  t = output.split(": now(")[1].split(")")[0]
      #  for command in roscommands:
          # Only evaluate a ROS command if it's for the current time (actually time-1)
      #    if command.split("),")[1].strip() == str(int(t) - 1):
      #      print("ROS COMMAND: " + command.split(",")[0])
      #      eval(command.split(")")[0] + ")")

      output = alma.stdout.readline()

    sys.stdout.write("\n")
    sys.stdout.flush()

    if len(canDo) > 0:
      add("doing(" + canDo[random.randint(0, len(canDo)-1)] + ").")


    alma.stdin.write('step\n')    
    time.sleep(3)


def add(data):
  alma.stdin.write("add " + data + "\n")


if __name__ == '__main__':
  main()

