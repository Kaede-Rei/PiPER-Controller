#!/bin/bash

# 零点标定

from piper_sdk import *

joint = C_PiperInterface_V2()
# 设置第 joint_num 颗电机的零点位置为当前位置
joint.JointConfig(joint_num=5, set_zero=0xAE)
