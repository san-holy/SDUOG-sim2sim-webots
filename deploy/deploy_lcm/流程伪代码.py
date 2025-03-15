def zero_buffer(msg):
    """结构体清零"""
    return msg=zero

def run(policy):
    """主程序"""
    msg = subscribe("obs")
    obs_buf = torch.cat(msg)
    actions = policy(obs_buf.detach())
    publish(actions)

if __name__ == "main":
    policy = torch.load("model.pt")
    thread(run, 0.002) # 需要一个开阻塞的线程，确保时间是0.002秒每个控制周期
    thread.start


