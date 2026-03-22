# NanoEdge AI Drop-In Folder

把 NanoEdge AI Studio 在 `Deployment` 页面导出的文件放到这里：

- `Middlewares/ST/NanoEdgeAI/Inc/NanoEdgeAI.h`
- `Middlewares/ST/NanoEdgeAI/Lib/libneai.a`

当前工程里的接入代码默认按“单轴 ADC 输入”工作。

- 如果你的库是 1-axis anomaly detection，直接使用即可。
- 如果你的库是 2 轴或 3 轴，需要修改 `Core/Src/main.c` 中的数据打包逻辑。
- 如果你后续要做知识库存储，可以继续基于链接脚本里的 `.neai` 段扩展。
