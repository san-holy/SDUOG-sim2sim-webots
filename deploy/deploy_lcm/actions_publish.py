import lcm
import numpy as np
from msg_lcm.robot_control import actions_lcmt
from deploy.deploy_lcm import robot_deploy

actions = robot_deploy.actions

msg = actions_lcmt()
msg.timestamp_us = 0
msg.torque = actions


lc = lcm.LCM()
while True:
    lc.publish("ACTIONS", msg.encode())


