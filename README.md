# embedded-air-quality-monitor-rust
Rust-Based Design of an Air Quality Detection System for Single Chip Microcomputers
基于 Rust 的嵌入式空气质量监测系统，适用于 STM32F103C8T6 微控制器。

📚 [中文](#中文) | [English](#english)

---

## <a id="中文"></a>中文

### 系统编译与复现说明

#### 环境配置及依赖项下载

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

#### 克隆项目

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

#### 编译、烧录与运行

1. **连接硬件**  
   通过 ST-Link 调试器将计算机与目标单片机（STM32F103C8T6）连接。

2. **一键编译烧录运行**  
   在项目目录下执行以下命令：
   ```bash
   cargo run --release
   ```
   该命令会依次完成固件编译、代码烧录并开始运行程序。

---

## <a id="english"></a>English

### Build & Reproduction Instructions

#### Environment Setup and Dependencies

1. **Install ST-Link Driver**  
   Ensure the ST-Link debugger driver is properly installed.

2. **Install Rust Toolchain**  
   Download and install `rustup` from the [official Rust website](https://www.rust-lang.org/tools/install).

3. **Add Cross-Compilation Target**  
   Run the following command to add the cross-compilation target for STM32F103C8T6:
   ```bash
   rustup target add thumbv7m-none-eabi
   ```

4. **Install Required Tools**  
   - `probe-rs-tools`: Provides communication with ST-Link, firmware flashing, logging, and debugging support.  
   - `cargo-generate`: Used to generate a project from the provided source template.  
   Install them with:
   ```bash
   cargo install probe-rs-tools cargo-generate
   ```

#### Cloning the Project

1. **Create and Enter a Project Folder**  
   Open a terminal (e.g., PowerShell) and run:
   ```bash
   mkdir my_project
   cd my_project
   ```
   > Note: Avoid using Chinese characters in the path.

2. **Clone the Repository**  
   Option 1 (recommended, using `cargo-generate`):
   ```bash
   cargo generate --git https://github.com/tesoro9216/embedded-air-quality-monitor-rust
   ```
   Then enter a project name, e.g., `project1`.

   Option 2 (using `git clone`):
   ```bash
   git clone https://github.com/tesoro9216/embedded-air-quality-monitor-rust project1
   ```

#### Compiling, Flashing, and Running

1. **Connect the Hardware**  
   Connect your computer to the target MCU (STM32F103C8T6) via an ST-Link debugger.

2. **One-Command Build and Flash**  
   Run the following command in the project directory:
   ```bash
   cargo run --release
   ```
   This will compile the firmware, flash it to the board, and start execution.

---

## License

This project is open-source. See the [LICENSE](LICENSE) file for details.
```
