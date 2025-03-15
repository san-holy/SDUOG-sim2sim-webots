import lcm
from msg_lcm.robot_control import obs_lcmt
from legged_gym.envs.base.legged_robot import LeggedRobot


def handler_obs_lcmt(channel, data):
    msg = obs_lcmt.decode(data)
    print("Received message on channel \"%s\"" % channel)
    print("   timestamp = %s" % str(msg.timestamp_us))
    print("   base_lin_vel = %s" % str(msg.base_lin_vel))
    print("   base_ang_vel = %s" % str(msg.base_ang_vel))
    print("   gravity: %s" % str(msg.gravity))
    print("   position = '%s'" % msg.pos)
    print("   default_position = '%s'" % msg.pos)
    print("   vel = %s" % str(msg.vel))
    print("   torque = %s" % str(msg.torque))
    print("")

lc = lcm.LCM()
subscription = lc.subscribe("OBS", handler_obs_lcmt)

try:
    while True:
        lc.handle()
except KeyboardInterrupt:
    pass