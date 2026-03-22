# NanoEdge AI 部署说明

这个工程已经预留了一个可选的 `ENABLE_NANOEDGE_AI` 模式，用来把 NanoEdge AI Studio 导出的异常检测库接到 STM32H743 工程里。

## 1. 需要从 NanoEdge AI Studio 导出什么

根据 ST 官方文档，NanoEdge AI Studio 在 `Deployment` 页面编译后会导出：

- `libneai.a`
- `NanoEdgeAI.h`

当前工程约定把它们放到：

- `Middlewares/ST/NanoEdgeAI/Inc/NanoEdgeAI.h`
- `Middlewares/ST/NanoEdgeAI/Lib/libneai.a`

## 2. 如何编译 AI 版本

```powershell
cmake --preset Debug-NanoEdgeAI
cmake --build --preset Debug-NanoEdgeAI
```

如果你打开了 `ENABLE_NANOEDGE_AI` 但还没复制导出的文件，工程也能编译，只是会进入占位模式，UART 会提示 `missing_generated_files`。

## 3. 当前代码的硬件与数据假设

当前实现的假设是：

- 使用 `ADC2`
- 只把 `ADC2` 的输入通道送进 NanoEdge
- 只支持 `1-axis` 的异常检测库
- MCU 侧送给 NanoEdge 的是 `ADC 电压值（float，单位 V）`

如果你在 Studio 里训练时使用的采样率、窗口长度、轴数、量纲和这里不一致，结果会失真。

## 4. 运行流程

1. `TIM3` 触发 `ADC2 + DMA`
2. DMA 半缓冲/满缓冲把采样块喂给 `NanoEdgeAI_ProcessAdcBlock()`
3. 先执行学习
4. 达到最小学习次数后自动切换到检测
5. `UART5` 周期性输出学习进度、相似度和错误状态

## 5. 如果你的库不是单轴

当前代码只接了一路 ADC 输入。

如果你的库是在 Studio 里按 2 轴或 3 轴训练的，需要修改 `Core/Src/main.c` 的 `HandleAdcDmaBlock()`，按 NanoEdge 训练时的轴顺序重新打包数据。
