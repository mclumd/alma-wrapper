# ALMA-Wrapper
Wrapper code for creating ALMA-based agents

## Installation

Must have ROS installation and [ALMA 2.0](https://github.com/mclumd/alma-2.0)

From a directory named ``src``:

    git clone https://github.com/mclumd/alma-wrapper.git
    cd ../
    catkin_make

## Running
    source devel/setup.bash
    rosrun alma-wrapper alma-wrapper.py [flags]

## Python Usage
    alma-wrapper.py [-h] [-a ALMA]

optional arguments:  
  `-h, --help`            show help message and exit  
  `-a ALMA, --alma ALMA`  specify new directory of ALMA executable

## Communication

Publishes to ROS topic ``agent_output``
