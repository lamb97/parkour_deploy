import torch

# 加载第一个 JIT 模型
image_encoder = torch.jit.load("/home/lamb97/下载/2024-07-21_21-48-46-12500-encoder_jit.pt")
image_encoder.eval()

# 创建第二个模型的示例输入（根据模型需求调整大小和类型）
image = torch.randn(1, 58, 87)  # 假设这是适合第二个模型的输入，调整维度以符合该模型
propri = torch.randn(1, 45)
hidden = torch.zeros(1, 1, 512)

# 加载第二个 JIT 模型
# 假设你有另一个模型文件 policy_mlp.jit
policy = torch.jit.load("/home/lamb97/下载/2024-07-21_21-48-46-12500-traced_policy_jit.pt")
policy.eval()

# 创建第一个模型的示例输入（根据模型需求调整大小和类型）
obs_buf = torch.randn(1, 665)  # 假设这是适合第一个模型的输入
depth_latent = torch.randn(1, 32)



# 导出第一个模型
torch.onnx.export(image_encoder, 
                  (image, propri, hidden),       # 第一个 JIT 模型
                  "encoder.onnx",  # 第一个模型的输出文件名
                  export_params=True,   # 导出模型的参数
                  opset_version=11,     # ONNX opset 版本
                  do_constant_folding=True,  # 优化常量折叠
                  input_names=['image', 'propri', 'hidden'],    # 输入名
                  output_names=['hidden_state,depth_latent'],  # 输出名
                  )

# 导出第二个模型
torch.onnx.export(policy,               # 第二个 JIT 模型
                  (obs_buf, depth_latent),      # 第二个模型的示例输入
                  "base_actor.onnx",    # 第二个模型的输出文件名
                  export_params=True,   # 导出模型的参数
                  opset_version=11,     # ONNX opset 版本
                  do_constant_folding=True,  # 优化常量折叠
                  input_names=['obs_buf','depth_latent'],    # 输入名
                  output_names=['output_policy'],  # 输出名
                  )
