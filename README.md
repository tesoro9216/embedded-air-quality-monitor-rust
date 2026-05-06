# embedded-air-quality-monitor-rust
Rust-Based Design of an Air Quality Detection System for Single Chip Microcomputers

# Embedded Air Quality Monitor (Rust)

基于 Rust 的嵌入式空气质量监测系统，适用于 STM32F103C8T6 微控制器。

## 系统编译与复现说明

### 环境配置及依赖项下载

1. **安装 ST-Link 驱动**  
   确保 ST-Link 调试器驱动已正确安装。

2. **安装 Rust 工具链**  
   从 [Rust 官方网站](https://www.rust-lang.org/tools/install) 下载并安装 `rustup`。

3. **添加交叉编译目标**  
   在终端执行以下命令，添加适用于 STM32F103C8T6 的交叉编译目标：
   ```bash
   rustup target add thumbv7m-none-eabi
   ```

4. **安装必要工具**  
   - `probe-rs-tools`：提供与 ST-Link 通信、烧录固件、日志打印和调试支持。  
   - `cargo-generate`：用于从提供的源码模板生成项目。  
   执行以下命令安装：
   ```bash
   cargo install probe-rs-tools cargo-generate
   ```

### 克隆项目

1. **创建并进入工程文件夹**  
   打开终端（如 PowerShell），执行：
   ```bash
   mkdir my_project
   cd my_project
   ```
   > 注意：路径中应避免包含中文字符。

2. **克隆仓库**  
   方式一（推荐，使用 `cargo-generate`）：
   ```bash
   cargo generate --git https://github.com/tesoro9216/embedded-air-quality-monitor-rust
   ```
   然后按照提示输入项目名称，如 `project1`。

   方式二（使用 `git clone`）：
   ```bash
   git clone https://github.com/tesoro9216/embedded-air-quality-monitor-rust project1
   ```

### 编译、烧录与运行

1. **连接硬件**  
   通过 ST-Link 调试器将计算机与目标单片机（STM32F103C8T6）连接。

2. **一键编译烧录运行**  
   在项目目录下执行以下命令：
   ```bash
   cargo run --release
   ```
   该命令会依次完成固件编译、代码烧录并开始运行程序。

## 说明

- 确保 ST-Link 驱动安装正确且设备连接稳定。
- `cargo run --release` 以优化模式编译，如需调试信息可移除 `--release` 选项（但建议在调试阶段使用）。
- 更多详细信息请参考项目源码或联系维护者。
