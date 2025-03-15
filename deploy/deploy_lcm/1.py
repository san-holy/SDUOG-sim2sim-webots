import torch
from deploy.deploy_lcm.msg_lcm.robot_control import obs_lcmt


msg = obs_lcmt()
base_lin_vel = msg.base_lin_vel
base_ang_vel = msg.base_ang_vel
projected_gravity = msg.gravity
commands = msg.commands
dof_pos = msg.pos
default_dof_pos = msg.dt_pos #默认位置
dof_vel = msg.vel
actions = msg.torque

obs_buf = torch.cat((   base_lin_vel * 2.0,
                        base_ang_vel  * 0.25,
                        projected_gravity,
                        commands[:, :4] * [2,2,0.25,1],
                        (dof_pos - default_dof_pos) * 1.0,
                        dof_vel * 0.05,
                        actions
                        ),dim=-1)

obs_buf = obs_buf.unsqueeze(0)

policy = torch.load('policy_model.pth')  # 需要替换为实际的文件路径
actions = policy(obs_buf.detach())