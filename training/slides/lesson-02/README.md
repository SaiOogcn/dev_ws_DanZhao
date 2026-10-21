# 第二次培训幻灯片

2026 年 10 月 13 日，第二次培训。主题是工作空间与包的创建、编译与三类典型报错、三条技术路线概览、技术栈与难度评估。

`main.pdf` 是已经编译好的成品，共 32 页，可以直接投屏。`main.typ` 是源文件。

## 目录结构

```text
lesson-02/
  main.typ    幻灯片正文
  lib.typ     同济主题（来自 touying-Tongji，MIT）
  vi/         主题用的校徽和图片
  main.pdf    编译结果
```

## 怎么重新编译

需要 Typst 命令行。本机放在 `F:\remote_dev_ws\typst-bin\typst-x86_64-pc-windows-msvc\typst.exe`。

```powershell
cd F:\remote_dev_ws\remote_dev_ws\training\slides\lesson-02
& 'F:\remote_dev_ws\typst-bin\typst-x86_64-pc-windows-msvc\typst.exe' compile main.typ --root .
```

第一次编译会联网下载 `@preview/touying:0.6.1`，之后走本地缓存。

导出每页 PNG 方便检查排版：

```powershell
& <typst.exe> compile --format png --ppi 100 main.typ 'preview/p-{p}.png' --root .
```

## 字体

正文用 `Libertinus Serif` 排拉丁字符，中文按下面的顺序回退：

```typst
#let cjk-fonts = (
  "Libertinus Serif",
  "Microsoft YaHei",
  "Noto Sans CJK SC",
  "Noto Serif CJK SC",
)
```

Windows 上命中微软雅黑，Linux 上需要 `fonts-noto-cjk`。若报 `unknown font family`，说明该机器没有这个字体，属提示不属错误；把存在的字体换到列表前面即可。

## 内容结构

四个章节，共 28 页正文：

| 章节 | 页数 | 内容 |
| --- | --- | --- |
| 一、工作空间与包的创建 | 6 | 四个目录、包必须放在 `src/`、创建命令、两种包的最小结构、依赖声明、复制 demo 包 |
| 二、编译 | 11 | 唯一正确位置、三条命令的顺序、两个常用参数、三类典型报错与处理、排错总表 |
| 三、三条技术路线概览 | 6 | 共用硬件、对比表、三条路线各自的数据流与待补代码、公共约束 |
| 四、技术栈与难度评估 | 5 | 技术栈对照、难度评估、各自难在哪、评估依据、开工前自查 |

## 两处刻意调整

主题的章首页会把全篇小节标题铺成一张网格。这篇有 28 节，网格要占两页，第二页只剩两行。因此在 `main.typ` 里关掉了章首页：

```typst
config-common(new-section-slide-fn: none),
```

章节分组仍然由每页底部的 mini-slides 展示。想要章首页，把这行删掉，同时把每章的小节控制在 5 到 6 个。

正文每页内容量控制在接近页脚的位置，改动文字后重新编译并数一下页数：正常是 32 页，页数变多说明有内容被挤到了下一页。

## 主题来源

模板来自 <https://github.com/SaiOogcn/touying-Tongji>，基于 Touying 0.6.1，MIT 许可。`lib.typ` 和 `vi/` 是模板里的原文件，未做改动。
