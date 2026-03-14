项目名称
========

多工程 STM32H7 示例集合 — 包含 00_H7_test、01_USB_test、02_DM02_test 等示例工程，用于基于 STM32H723 的开发、编译、烧录与调试。

目录结构概览
----------------
- 00_H7_test/ — 基本 H7 演示工程
- 01_USB_test/ — USB CDC 设备示例
- 02_DM02_test/ — 其他示例和模块
- Core/, Drivers/, Middlewares/ — 各工程共用的 HAL 与中间件代码

依赖
-----
- CMake (推荐 >= 3.16)
- Ninja（或选择的构建生成器）
- arm-none-eabi 工具链（gcc-arm-none-eabi）
- OpenOCD 或 ST-Link 工具用于烧录与调试

快速构建 (以 01_USB_test 为例)
---------------------------
1. 打开终端，进入示例目录：

   `cd 01_USB_test`

2. 创建并进入构建目录：

   `mkdir -p build && cd build`

3. 运行 CMake 配置（示例使用工作区内的 toolchain 文件）：

   `cmake -S .. -B . -DCMAKE_TOOLCHAIN_FILE=../cmake/gcc-arm-none-eabi.cmake -G Ninja -DCMAKE_BUILD_TYPE=Debug`

4. 编译：

   `ninja`

烧录与调试
-----------
- 使用 OpenOCD：

  `openocd -f ../openocd.cfg -f interface/stlink.cfg`

- VSCode 已包含调试配置，直接在调试面板选择对应配置并启动即可（见 .vscode/launch.json）。

常见注意项
-----------
- 请确保系统 PATH 中包含 arm-none-eabi 工具链和 ninja。
- 若使用不同的 toolchain，请修改 cmake 命令的 CMAKE_TOOLCHAIN_FILE。

贡献与联系
-----------
如需添加示例或改进文档，请提交 PR 或在仓库中创建 issue。

许可证
------
各第三方驱动与中间件遵循其各自许可（例如 STM32 HAL、CMSIS 等）。
