# 💻 代码目录说明

本目录存放项目的所有源代码，按模块或功能组织。请遵循以下规范：

## 目录结构建议

```
src/
├── algorithm/          # 算法相关代码（视觉、控制、决策等）
├── embedded/           # 嵌入式底层代码（驱动、通信等）
├── hardware/           # 硬件相关代码（如有，如 FPGA 代码）
├── utils/              # 通用工具函数/脚本
├── tests/              # 单元测试/集成测试代码
└── README.md           # 本文件
```


## 代码管理规范
- **主分支**：`main` 分支始终保存稳定可运行的版本。
- **开发分支**：每个功能或模块请在独立分支上开发，完成后通过 **合并请求（Merge Request）** 合入主分支。
- **提交信息**：请写清晰、简洁的提交信息，说明“做了什么”和“为什么”。
- **注释**：关键算法和复杂逻辑请添加必要注释。

## 常用命令
```bash
# 克隆仓库
git clone <仓库地址>

# 创建并切换到新分支
git checkout -b feature/your-feature-name

# 提交更改
git add .
git commit -m "简要说明"
git push origin feature/your-feature-name
```

> 如有代码规范（如命名风格、格式化工具），请在此补充。

