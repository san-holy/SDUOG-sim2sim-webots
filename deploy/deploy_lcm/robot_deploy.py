#define PY_SSIZE_T_CLEAN
import lcm
import torch
import threading
import time

from msg_lcm.robot_control import obs_lcmt
from msg_lcm.robot_control import actions_lcmt

# 数据清零
def zero_struct(msg):
    msg.base_lin_vel = [0, 0, 0]
    msg.base_ang_vel = [0, 0, 0]
    msg.gravity = [0, 0, 0]
    msg.commands = [0, 0, 0, 0]
    msg.pos = [0.0] * 12
    msg.vel = [0.0] * 12
    msg.torque = [0.0] * 12
    msg.terrain = [0.0] * 187
    return msg

# 共享状态封装
class SharedState:
    def __init__(self):
        self.lock = threading.Lock()
        self.latest_obs_msg = None
        self.no_data_count = 0
        self.last_timestamp = None

# 处理观测消息
def handler_obs_lcmt(channel, data, shared_state):
    msg = obs_lcmt.decode(data)
    with shared_state.lock:
        shared_state.latest_obs_msg = msg
        shared_state.no_data_count = 0

# 发布动作循环
def publish_actions(shared_state, policy, lc):
    interval = 0.020  # 50Hz
    max_missing_cycles = 10  # 允许的最大丢失周期

    while True:
        start_time = time.time()
        current_msg = None
        with shared_state.lock:
            if shared_state.latest_obs_msg is not None:
                current_msg = shared_state.latest_obs_msg
                shared_state.latest_obs_msg = None  # 清空缓冲区
                shared_state.no_data_count = 0     # 重置计数器
                shared_state.last_timestamp = current_msg.timestamp_us
            else:
                shared_state.no_data_count += 1    # 递增丢失计数器

        # 处理超时清零逻辑
        if shared_state.no_data_count >= max_missing_cycles:
            if shared_state.last_timestamp is not None:
                zero_msg = obs_lcmt()
                zero_msg.timestamp_us = shared_state.last_timestamp
                zero_struct(zero_msg)
                current_msg = zero_msg
                print("WARNING: Data timeout, zeroing commands!")

        if current_msg is not None:
            # 处理数据生成obs_buf
            # 使用指定设备创建所有张量
            device = next(policy.parameters()).device  # 自动获取模型所在设备
            
            # 每个部分添加unsqueeze(0)保证二维
            parts = [
                torch.tensor(current_msg.base_lin_vel, device=device).unsqueeze(0) * 2.0,          # 形状 [1,3]
                torch.tensor(current_msg.base_ang_vel, device=device).unsqueeze(0) * 0.25,         # 形状 [1,3]
                torch.tensor(current_msg.gravity, device=device).unsqueeze(0),                      # 形状 [1,3]
                torch.tensor(current_msg.commands[:4], device=device).unsqueeze(0) * torch.tensor([[2, 2, 0.25, 1]], device=device),                                # 形状 [1,4]
                torch.tensor(current_msg.pos, device=device).unsqueeze(0) * 1.0,          # 形状 [1,12]
                torch.tensor(current_msg.vel, device=device).unsqueeze(0) * 0.05,                  # 形状 [1,12]
                torch.tensor(current_msg.torque, device=device).unsqueeze(0),
                torch.tensor(current_msg.terrain, device=device).unsqueeze(0) * 5.0                        # 形状 [1,12]
            ]

            # # 拼接前检查维度
            # for i, p in enumerate(parts):
            #     print(f"部分 {i} 形状: {p.shape}")  # 调试输出

            # 横向拼接（dim=1）
            obs_buf = torch.cat(parts, dim=1) 

            # 生成动作
            with torch.no_grad():
                actions = policy(obs_buf)

            # 创建动作消息
            action_msg = actions_lcmt()
            action_msg.timestamp_us = current_msg.timestamp_us
            action_msg.torque = actions.numpy().flatten().tolist()
            print(f"动作: {action_msg.torque}")

            # 发布动作
            lc.publish("ACTIONS", action_msg.encode())

        # 控制频率
        elapsed = time.time() - start_time
        time.sleep(max(interval - elapsed, 0))

def run(policy):
    lc = lcm.LCM()
    shared_state = SharedState()

    # 订阅观测消息，绑定共享状态
    subscription = lc.subscribe("OBS", lambda c, d: handler_obs_lcmt(c, d, shared_state))

    # 启动发布线程
    publish_thread = threading.Thread(
        target=publish_actions, 
        args=(shared_state, policy, lc),
        daemon=True
    )
    publish_thread.start()

    try:
        while True:
            lc.handle()
    except KeyboardInterrupt:
        pass
    finally:
        lc.unsubscribe(subscription)

# 主程序入口
if __name__ == "__main__":
    # 修复后的模型加载方案
    # device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    device = torch.device('cpu')
    
    # 方案一：直接加载TorchScript模型（推荐）
    policy = torch.jit.load('../models/policy_1.pt', map_location=device)
    
    # 方案二：如果原始模型是state_dict格式（需要模型类定义）
    '''
    class PolicyModel(torch.nn.Module):
        def __init__(self):
            super().__init__()
            # 必须与训练时完全相同的模型结构
            
        def forward(self, x):
            # 原始前向逻辑
            
    policy = PolicyModel().to(device)
    policy.load_state_dict(torch.load('../models/policy_1.pt', map_location=device))
    policy.eval()
    '''
    
    # 检查模型有效性
    test_input = torch.randn(1, 236).to(device)  # 根据实际输入维度修改
    with torch.no_grad():
        print("测试输出:", policy(test_input))
    
    # 如果必须生成TorchScript格式（已有则不需要）
    # scripted_model = torch.jit.script(policy)
    # torch.jit.save(scripted_model, "policy_jit.pt")
    
    run(policy)