import torch

# 选择设备（自动检测是否有GPU）
device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

#-------------------------------------------------------
# 需要你修改的部分：
# 模型文件路径
MODEL_PATH = "../models/policy_1.pt"  # 替换为你的.pt文件路径
# 定义输入维度（根据模型要求修改）
input_shape = (1, 236)  # 示例形状：(batch_size, input_features)
#-------------------------------------------------------

# 直接加载TorchScript模型（无需定义模型类）
model = torch.jit.load(MODEL_PATH, map_location=device)
model.eval()  # 设置为评估模式

# 生成随机输入
random_input = torch.randn(input_shape).to(device)

# 执行推理
with torch.no_grad():
    output = model(random_input)

print("输入形状:", random_input.shape)
print("输出结果:")
print(output)
print("输出形状:", output.shape)