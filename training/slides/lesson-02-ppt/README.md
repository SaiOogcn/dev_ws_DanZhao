# 第二次培训幻灯片（PPT 版）

2026 年 10 月 13 日第二次培训用的演示文稿，共 29 页。

`output/lesson-02-training.pptx` 是成品，用 PowerPoint 或 WPS 直接打开即可。12 页到 17 页是编译排错部分，对应这次培训的重点。

## 样式来源

配色和字体取自 `F:\学习资料\Pre PPT\PIDController.pptx`。具体数值如下。

| 用途 | 取值 |
| --- | --- |
| 封面与结束页底色 | `#3A3A3A` |
| 封面大标题 | `#FBE3D6` 奶油色 |
| 封面次级文字 | `#D9D9D9` |
| 书脊标题 | `#FBE3D6`，44 pt，竖排 |
| 书脊副题 | `#F2F2F2`，24 pt，竖排 |
| 正文标题 | `#000000`，PMingLiU-ExtB |
| 正文与表格 | `#262626`，等线 |
| 表头底色 | `#0E2841`，白字 |
| 命令块底色 | `#F2F2F2`，文字 `#0E2841` |

字体只用两个字族，都来自模板：等线用于正文和表格，PMingLiU-ExtB 用于标题。模板里还用到了 Batang 和 GungsuhChe，这两个韩文字体本机没有安装，用上面两个字族替代。

## 制作方式

演示文稿以模板为底稿生成：导入模板，保留封面和结束页，复制模板的内容页作为每一页的底（书脊和白色背景来自它），删掉与内容无关的插图，替换书脊与封面文字，再按内容排入标题、正文、命令块和表格。表格是原生表格，可以在 PowerPoint 里直接改。

## 页序

| 页 | 内容 |
| --- | --- |
| 1 | 封面 |
| 2 | 目录 |
| 3 至 8 | 工作空间与包的创建 |
| 9 至 11 | 编译的位置、命令与常用参数 |
| 12 至 16 | 三类典型报错：系统时间、编译错位置、忘记 source |
| 17 | 报错速查表 |
| 18 至 23 | 三条技术路线概览 |
| 24 至 28 | 技术栈与难度评估 |
| 29 | 结束页 |

## 校验结果

生成后做过这些检查：29 页齐全，13 张原生表格，全文只用到等线和 PMingLiU-ExtB，文字没有越过右侧书脊，没有内容触到页面边缘。版式校验 0 项缺陷，1 条提示：封面有一个对象越出页面边界 6 px 左右，这条来自模板本身，未做改动。

## 重新生成

`.build/` 里是生成脚本，`output/` 里是成品，`.codex-finalizer/` 里是校验报告。改内容只需要编辑 `build-deck.mjs` 里的 `SPECS` 数组再重新生成：

```powershell
$rt = 'C:\Users\zhich\.cache\codex-runtimes\codex-primary-runtime\dependencies'
$env:RUNTIME_NODE_MODULES = "$rt\node\node_modules"
$env:SKILL_DIR = 'C:\Users\zhich\.codex\plugins\cache\openai-primary-runtime\presentations\26.1011.11113\skills\presentations'
$env:RUNTIME_PYTHON = "$rt\python\python.exe"
$node = "$rt\node\bin\node.exe"

cd .build
& $node build-deck.mjs "$PWD"
& $node finalize.mjs (Split-Path $PWD) "$PWD\draft.pptx" `
  (Join-Path (Split-Path $PWD) 'output\lesson-02-training.pptx') `
  'F:\学习资料\Pre PPT\PIDController.pptx' `
  (Get-FileHash 'F:\学习资料\Pre PPT\PIDController.pptx' -Algorithm SHA256).Hash.ToLower()
```

成品文件名每改一次要换一个，校验脚本不会覆盖已有文件。
