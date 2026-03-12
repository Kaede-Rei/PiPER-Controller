#!/bin/bash

# 零点标定

from piper_sdk import *

joint = C_PiperInterface_V2()
joint.JointConfig(joint_num=5, set_zero=0xAE)
