#![no_std]
#![no_main]

// 引入字符串格式化和无堆栈字符串
use core::fmt::Write;
use heapless::String;

// panic
use defmt_rtt as _;
use panic_probe as _;
use defmt::Debug2Format;

// nb阻塞
use nb::block;

// HAL 库
use cortex_m_rt::entry;
use stm32f1xx_hal::{
    i2c::{BlockingI2c, DutyCycle, Mode}, 
    pac, //外设访问层(寄存器)
    prelude::*, //拓展特性库
    rcc::Config, 
    timer::pwm_input::QeiOptions, //qei 正交编码器模块
    serial::{Config as SerialConfig, Serial}, //串口 UART 通信
};

// 引入 RefCell 和总线复用机制
use core::cell::RefCell;
use embedded_hal_bus::i2c::RefCellDevice;

// 引入 SHT3x 传感器驱动
use sht31::prelude::*;

// 引入屏幕和图形绘图库
use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306};
use embedded_graphics::{
    mono_font::{ascii::{FONT_6X10, FONT_4X6}, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};

//真时间戳
defmt::timestamp!("{=u32:us}", {
    // 因为配置的系统主频是 72MHz，所以周期数除以 72 就是微秒 (us)
    cortex_m::peripheral::DWT::cycle_count() / 72
});

// 定义
// 定义传感器健康状态
#[derive(Clone, Copy, PartialEq)]
enum SensorStatus {
    Init,
    Ok,
    Error,
}
// 定义 TempHumi 缓存结构体
#[derive(Clone, Copy)]
struct TempHumi {
    status: SensorStatus,
    fail_count: u8, // 连续失败计数器
    is_temp_neg: bool,
    temp_whole: u32,
    temp_frac: u32,
    humi_whole: u32,
    humi_frac: u32,
}
impl Default for TempHumi {
    fn default() -> Self {
        Self {
            status: SensorStatus::Init,
            fail_count: 0,
            is_temp_neg: false,
            temp_whole: 0,
            temp_frac: 0,
            humi_whole: 0,
            humi_frac: 0,
        }
    }
}
// 定义 PM2.5 缓存结构体
#[derive(Clone, Copy)]
struct Pm25Cache {
    status: SensorStatus,
    pm2_5: u16,
    pm10: u16,
}
impl Default for Pm25Cache {
    fn default() -> Self {
        Self {
            status: SensorStatus::Init,
            pm2_5: 0,
            pm10: 0
        }
    }
}
// 定义 HCHO 缓存结构体
#[derive(Clone, Copy)]
struct HchoCache {
    status: SensorStatus,
    ug_m3: u16,      // 浓度：微克/立方米 (ug/m3) -> 1000 ug/m3 = 1 mg/m3
    ppb: u16,        // 浓度：十亿分之一 (ppb)
}
impl Default for HchoCache {
    fn default() -> Self {
        Self {
            status: SensorStatus::Init,
            ug_m3: 0,
            ppb: 0
        }
    }
}
// 定义 UI 页面枚举
#[derive(Clone, Copy, PartialEq)]
enum Page {
    TempHumi,
    Pm25,
    Hcho
}
impl Page {
    fn next(self) -> Self {
        match self {
            Page::TempHumi => Page::Pm25,
            Page::Pm25 => Page::Hcho,
            Page::Hcho => Page::TempHumi,
        }
    }
    fn prev(self) -> Self {
        match self {
            Page::TempHumi => Page::Hcho,
            Page::Hcho => Page::Pm25,
            Page::Pm25 => Page::TempHumi,
        }
    }
}

// 常量
const SW_DEBOUNCE_THRES: u8 = 2; // 按键消抖 tick 数
const SW_LONG_PRESS_THRES: u8 = 30; // 按键长按 tick 数
const ENCODER_STEP: i16 = 4;         // 编码器触发一次翻页所需的步长

const TICK_1_SEC: u8 = 50;           // 1秒钟对应的 Tick 数 (主心跳 50Hz, 20ms) ，也是温湿度自动刷新的周期
const TEMPHUMI_TICK_MEASURE: u8 = TICK_1_SEC - 2;         // 下发索要温湿度数据指令：每秒倒数第 2 个 Tick
const PM25_TICK_CYCLE: u16 = 9000; // PM2.5测量周期：每 9000 个 Tick = 3分钟
const PM25_TICK_MEASURE: u16 = 1000; // 下发索要PM2.5数据指令：周期第 1000 个 Tick = 第 20 秒
const HCHO_TICK_MEASURE: u8 = TICK_1_SEC / 2;      // 下发索要 HCHO 数据指令：每秒第一半个 Tick

// PM2.5 PMS7003 通信指令集
const CMD_PMS_PASSIVE: &[u8] = &[0x42, 0x4D, 0xE1, 0x00, 0x00, 0x01, 0x70]; // 设置被动模式
const CMD_PMS_WAKEUP: &[u8]  = &[0x42, 0x4D, 0xE4, 0x00, 0x01, 0x01, 0x74]; // 唤醒风扇与激光
const CMD_PMS_READ: &[u8]    = &[0x42, 0x4D, 0xE2, 0x00, 0x00, 0x01, 0x71]; // 发送被动读取请求
const CMD_PMS_SLEEP: &[u8]   = &[0x42, 0x4D, 0xE4, 0x00, 0x00, 0x01, 0x73]; // 进入低功耗休眠
//  HCHO ZE08 通信指令集
const CMD_ZE08_QA_MODE: &[u8] = &[0xFF, 0x01, 0x78, 0x41, 0x00, 0x00, 0x00, 0x00, 0x46]; // 切换至问答模式
const CMD_ZE08_READ: &[u8]    = &[0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79]; // 索要数据指令

// 报警阈值
const PM25_ALARM_THRES: u16 = 115; // PM2.5 报警阈值 (ug/m3)
const HCHO_ALARM_THRES: u16 = 80; // HCHO 报警阈值 (ug/m3, 即 0.08 mg/m3)

#[entry]
fn main() -> ! {
    // 1. 初始化cp dp
    let mut cp = cortex_m::Peripherals::take().unwrap();
    let dp = pac::Peripherals::take().unwrap();
    //defmt::info!("cp dp初始化完毕");
    cp.DCB.enable_trace();
    cp.DWT.enable_cycle_counter();
    
    // 2. 配置时钟
    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.freeze(Config::hse(8.MHz()).sysclk(72.MHz()), &mut flash.acr);
    let clocks = rcc.clocks;
    defmt::info!("时钟配置完毕");

    // 3. 配置引脚
    //let mut afio = dp.AFIO.constrain(&mut rcc);
    // 3.1. 配置 GPIOA
    let mut gpioa = dp.GPIOA.split(&mut rcc);
    // 3.1.1. GPIOA : 配置按键引脚：PA5
    let sw = gpioa.pa5.into_pull_up_input(&mut gpioa.crl);
    // 3.1.2. GPIOA : 编码器引脚：PA6 (A相), PA7 (B相)
    let enc_a = gpioa.pa6.into_floating_input(&mut gpioa.crl);
    let enc_b = gpioa.pa7.into_floating_input(&mut gpioa.crl);
    // 3.1.3.  GPIOA : UART1 ( PM2.5 传感器 )引脚：PA9 (TX), PA10 (RX)
    let pm25_tx_pin = gpioa.pa9.into_alternate_push_pull(&mut gpioa.crh);
    let pm25_rx_pin = gpioa.pa10;

    // 3.3. 配置 GPIOB
    let mut gpiob = dp.GPIOB.split(&mut rcc);
    // 3.3.1 GPIOB : I2C1 ( OLED 屏 + 温湿度传感器) 引脚：SCL = PB6, SDA = PB7
    let i2c1_scl = gpiob.pb6.into_alternate_open_drain(&mut gpiob.crl);
    let i2c1_sda = gpiob.pb7.into_alternate_open_drain(&mut gpiob.crl);
    // 3.3.2. GPIOB : UART3 ( HCHO 传感器 ) 引脚：PB10 (TX), PB11 (RX)
    let hcho_tx_pin = gpiob.pb10.into_alternate_push_pull(&mut gpiob.crh);
    let hcho_rx_pin = gpiob.pb11;
    // 3.3.3. GPIOB : 蜂鸣器引脚：PB12
    let mut buzzer = gpiob.pb12.into_push_pull_output(&mut gpiob.crh);
    buzzer.set_high(); // 默认静音

    // 3.4. 配置 GPIOC
    let mut gpioc = dp.GPIOC.split(&mut rcc);
    // 3.4.1 GPIOC : LED 灯引脚：PC13 ，配置为推挽输出
    let mut led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);
    defmt::info!("引脚配置完毕");

    // 4.启动外设/初始化外设
    // 4.1. 初始化 TIM3 (QEI编码器) 
    let qei = dp.TIM3.qei(
        (enc_a, enc_b),
        QeiOptions::default(),
        &mut rcc
    );
    // 4.2. UART1 (PM2.5 传感器)
    // 4.2.1. 配置 UART1 (PMS7003) 波特率为 9600，分离TX RX
    let pm25_serial = Serial::new(
    dp.USART1,
    (pm25_tx_pin, pm25_rx_pin),
    SerialConfig::default()
        .baudrate(9600.bps()),
    &mut rcc,
    );
    let (mut pm25_tx, mut pm25_rx) = pm25_serial.split();
    // 4.2.2. 定义发送函数，并设置为被动模式和睡眠状态
    let mut send_pms_cmd = |cmd: &[u8]| {
        for &b in cmd {
            let _ = block!(pm25_tx.write(b));
        }
        let _ = block!(pm25_tx.flush());
    };
    send_pms_cmd(CMD_PMS_PASSIVE); // 被动模式
    send_pms_cmd(CMD_PMS_SLEEP); // 休眠
    defmt::info!("PM2.5 传感器初始化完毕，已休眠");
    // 4.3. I2C1 (OLED屏幕 + 温湿度传感器)
    // 4.3.1.延时 50ms 等待 OLED 上电稳定
    cp.SYST.delay(&clocks).delay_ms(50_u16);
    // 4.3.2. 配置 I2C1 硬件外设
    let i2c = BlockingI2c::new(
        dp.I2C1,
        (i2c1_scl, i2c1_sda),
        Mode::Fast {
            frequency: 400.kHz(),
            duty_cycle: DutyCycle::Ratio2to1,
        },
        &mut rcc, 
        //1000, 10, 10, 10, //START 信号超时 (start_timeout), STOP 信号超时 (start_stop_timeout), 地址应答超时 (address_timeout), 单字节数据超时 (byte_timeout)
        //100, 100, 100, 1000, //推荐数值
        1000, 100, 100, 1000,
    );
    //defmt::info!("I2C1 硬件外设启动完毕");
    // 4.3.3. 利用 RefCell 将 I2C 包装为共享总线
    let i2c_bus = RefCell::new(i2c);
    // 4.3.4. 为 OLED 和 SHT30 分别创建虚拟的 I2C 设备代理
    let oled_i2c = RefCellDevice::new(&i2c_bus);
    let sht_i2c = RefCellDevice::new(&i2c_bus);
    // 4.3.5. 初始化 OLED 屏幕
    let oled_interface = I2CDisplayInterface::new(oled_i2c);
    let mut display = Ssd1306::new(
        oled_interface,
        DisplaySize128x64,
        DisplayRotation::Rotate0,
    ).into_buffered_graphics_mode(); // 开启显存缓冲模式
    display.init().unwrap();
    display.clear(BinaryColor::Off).unwrap(); // 显式清空屏幕
    // 4.3.6. 初始化 SHT30 温湿度传感器
    let mut sht30 = SHT31::single_shot(sht_i2c, SingleShot::new())
        .with_unit(TemperatureUnit::Celsius);
    // 4.4.1. 配置 UART3 波特率为 9600，分离TX RX
    let hcho_serial = Serial::new(
        dp.USART3,
        (hcho_tx_pin, hcho_rx_pin),
        SerialConfig::default().baudrate(9600.bps()),
        &mut rcc,
    );
    let (mut hcho_tx, mut hcho_rx) = hcho_serial.split();
    // 4.4.2. 定义发送函数，并设置为被动模式和睡眠状态
    let mut send_hcho_cmd = |cmd: &[u8]| {
        for &b in cmd {
            let _ = block!(hcho_tx.write(b));
        }
        let _ = block!(hcho_tx.flush());
    };
    send_hcho_cmd(CMD_ZE08_QA_MODE); // 问答模式
    defmt::info!("HCHO 传感器初始化完毕，已切换至问答模式");
    // 4.5. LED 灯默认熄灭
    led.set_high();
    defmt::info!("所有外设初始化完毕");

    // 5. 初始化主循环节拍器
    let mut timer = dp.TIM2.counter_hz(&mut rcc);
    timer.start(50.Hz()).unwrap();
    //defmt::info!("主循环节拍器初始化完毕");

    // 6. 变量
    // 6.1. 字体样式
    let text_style_normal = MonoTextStyleBuilder::new()
        .font(&FONT_6X10)
        .text_color(BinaryColor::On)
        .build();
    let text_style_small = MonoTextStyleBuilder::new()
        .font(&FONT_4X6)
        .text_color(BinaryColor::On)
        .build();
    // 6.2. 其他变量
    let mut current_page = Page::TempHumi; //目前页码
    let mut need_redraw = true; // 是否需要推送到屏幕

    let mut refresh_ticks: u8 = 0; // 用于 1 秒定时刷新的计数器

    let mut sw_hold_ticks: u8 = 0; // 按键按下计数器
    let mut sw_long_pressed_triggered = false; // 按键是否按下

    let mut enc_last_count = qei.count(); // 记录编码器的初始位置

    let mut led_on_ticks: u8 = 0; // LED 维持亮起的 Tick 计数（20ms 一个 Tick）

    let mut is_temphumi_measuring = false; // 记录是否正在测量 TempHumi
    let mut temphumi_cache = TempHumi::default(); //实例化 TempHumi 缓存

    let mut pm25_cache = Pm25Cache::default(); //实例化 PM2.5 缓存
    let mut pm25_schedule_ticks: u16 = 0;

    let mut hcho_cache = HchoCache::default(); //实例化 HCHO 缓存

    let mut is_muted = false; // 全局静音标志
    //defmt::info!("变量初始化完毕");

    //主循环
    defmt::info!("初始化完成，主循环开始");
    loop {
        // 阻塞等待，20ms 心跳
        block!(timer.wait()).unwrap();

        // ==========================================
        // 任务 A: 自动刷新、数据测量
        // ==========================================
        // 1. 自动刷新逻辑
        refresh_ticks = refresh_ticks.saturating_add(1);

        // 2. 温湿度测量逻辑
        // 2.1. 下发测量指令
        if refresh_ticks == TEMPHUMI_TICK_MEASURE && current_page == Page::TempHumi {
            let _ = sht30.measure();
            is_temphumi_measuring = true;
        }
        // 2.2. 读取数据，进行运算，并更新到 Cache
        if refresh_ticks >= TICK_1_SEC {
            refresh_ticks = 0;
            if is_temphumi_measuring {
                match sht30.read() {
                    Ok(reading) => {
                        temphumi_cache.fail_count = 0; 
                        temphumi_cache.status = SensorStatus::Ok;
                        // 温度
                        let temp = reading.temperature;
                        temphumi_cache.is_temp_neg = temp < 0.0;                        
                        let temp_abs = if temphumi_cache.is_temp_neg { -temp } else { temp };
                        let temp_scaled = (temp_abs * 10.0 + 0.5) as u32;
                        temphumi_cache.temp_whole = temp_scaled / 10;
                        temphumi_cache.temp_frac = temp_scaled % 10;
                        // 湿度
                        let humi = reading.humidity;
                        let humi_scaled = (humi * 10.0 + 0.5) as u32;
                        temphumi_cache.humi_whole = humi_scaled / 10;
                        temphumi_cache.humi_frac = humi_scaled % 10;
                        // 重绘界面
                        if current_page == Page::TempHumi { need_redraw = true; }
                    }
                    Err(e) => {
                        // 错误状态处理
                        defmt::error!("SHT30 Read Fail: {:?}", Debug2Format(&e));
                        temphumi_cache.fail_count = temphumi_cache.fail_count.saturating_add(1);
                        // 连续 3 次读不到数据，判定传感器报错
                        if temphumi_cache.fail_count >= 3 && temphumi_cache.status != SensorStatus::Error {
                            temphumi_cache.status = SensorStatus::Error;
                            if current_page == Page::TempHumi { need_redraw = true; }
                        }
                    }
                }
                is_temphumi_measuring = false;
            }
            if current_page == Page::Pm25 {
                need_redraw = true;
            }
        }

         // 3. PM2.5 测量
        pm25_schedule_ticks = pm25_schedule_ticks.wrapping_add(1);
        
        // 3.1. 唤醒传感器，风扇开始抽气
        if pm25_schedule_ticks == 1 {
            send_pms_cmd(CMD_PMS_WAKEUP); // 唤醒指令
            defmt::info!("PM2.5 唤醒");
        }

        // 3.2. 阻塞读取处理帧，并立刻休眠
        if pm25_schedule_ticks == PM25_TICK_MEASURE {
            loop {
                match pm25_rx.read() {
                    Ok(_) => {},
                    _ => { break; }
                }
            } // 丢弃旧帧
            send_pms_cmd(CMD_PMS_READ);
            defmt::info!("PM2.5 被动读取");
            // 3.2.1. 寻找包头            
            let mut sync = 0;
            for _ in 0..100 {
                match block!(pm25_rx.read()) {
                    Ok(b) => {
                        if sync == 0 && b == 0x42 { sync = 1; } // 寻找包头1：0x42
                        else if sync == 1 && b == 0x4D { sync = 2; break; } // 寻找包头2：0x4D
                        else { sync = 0; }
                    },
                    Err(_) => continue,
                }
            }
            // 3.2.2. 确认包头后，读取处理数据帧
            if sync == 2 {
                let mut buf = [0u8; 32];
                buf[0] = 0x42;
                buf[1] = 0x4D;
                for i in 2..32 {
                    loop {
                        match pm25_rx.read() {
                            Ok(b) => {
                                buf[i] = b;
                                break;
                            },
                            Err(nb::Error::WouldBlock) => {
                                continue; 
                            },
                            Err(_) => {
                                buf[i] = 0;
                                break;
                            }
                        }
                    }
                }
                // 3.2.2.1 计算校验和 (0x42 + 0x4D + buf[0..27])
                let mut sum: u16 = 0;
                for i in 0..30 { sum += buf[i] as u16; }
                let expected_sum = ((buf[30] as u16) << 8) | (buf[31] as u16);
                if sum == expected_sum {
                    pm25_cache.status = SensorStatus::Ok;
                    // PM2.5 大气环境下浓度: 高八位 buf[12], 低八位 buf[13]
                    pm25_cache.pm2_5 = ((buf[12] as u16) << 8) | (buf[13] as u16);
                    // PM10 大气环境下浓度: 高八位 buf[14], 低八位 buf[15]
                    pm25_cache.pm10 = ((buf[14] as u16) << 8) | (buf[15] as u16);
                    if current_page == Page::Pm25 { need_redraw = true; }
                    defmt::info!("PM2.5 读取成功! PM2.5: {} ug/m3, PM10: {} ug/m3", pm25_cache.pm2_5, pm25_cache.pm10);
                } else {
                    pm25_cache.status = SensorStatus::Error;
                    defmt::error!("PM2.5 校验和错误: 计算 {}, 期望 {}", sum, expected_sum);
                }
            } else {
                pm25_cache.status = SensorStatus::Error;
                defmt::error!("PM2.5 等待包头超时！");
            }
            // 3.2.4. 不管成功失败，立刻休眠风扇保命
            send_pms_cmd(CMD_PMS_SLEEP); // 休眠指令
            defmt::info!("PM2.5 休眠");
            if current_page == Page::Pm25 { need_redraw = true; }
        }
        
        // 3.3. 测量周期重置
        if pm25_schedule_ticks >= PM25_TICK_CYCLE {
            pm25_schedule_ticks = 0;
        }

        // 4. HCHO 测量
        // 4.2. 阻塞读取处理帧
        if refresh_ticks == HCHO_TICK_MEASURE {
            loop {
                match hcho_rx.read() {
                    Ok(_) => {},
                    _ => { break; }
                } 
            } // 丢弃旧帧
            send_hcho_cmd(CMD_ZE08_READ);
            // 4.2.1. 寻找包头            
            let mut sync = 0;
            for _ in 0..30 {
                match block!(hcho_rx.read()) {
                    Ok(b) => {
                        if sync == 0 && b == 0xFF { sync = 1; } // 寻找包头1：0xFF
                        else if sync == 1 && b == 0x86 { sync = 2; break; } // 寻找包头2：0x86
                        else { sync = 0; }
                    },
                    Err(_) => continue,
                }
            }
            // 4.2.2. 确认包头后，读取解析数据帧
            if sync == 2 {
                let mut buf = [0u8; 9];
                buf[0] = 0xFF;
                buf[1] = 0x86;
                for i in 2..9 {
                    loop {
                        match hcho_rx.read() {
                            Ok(b) => {
                                buf[i] = b;
                                break;
                            },
                            Err(nb::Error::WouldBlock) => {
                                continue; 
                            },
                            Err(_) => {
                                buf[i] = 0;
                                break;
                            }
                        }
                    }
                }
                // 4.2.2.1. 计算校验和 取反(Byte1 + ... + Byte7)) + 1 = Byte 8
                let mut sum: u8 = 0;
                for i in 1..8 { sum = sum.wrapping_add(buf[i]); }
                let expected_sum = (!sum).wrapping_add(1);
                if expected_sum == buf[8] {
                    hcho_cache.status = SensorStatus::Ok;
                    // 气体浓度 ug/m3 是Byte 2 和 Byte 3
                    hcho_cache.ug_m3 = ((buf[2] as u16) << 8) | (buf[3] as u16);
                    // 气体浓度 ppb 是 Byte 6 和 Byte 7
                    hcho_cache.ppb = ((buf[6] as u16) << 8) | (buf[7] as u16);
                    if current_page == Page::Hcho { need_redraw = true; }
                } else {
                    hcho_cache.status = SensorStatus::Error;
                    defmt::error!("HCHO 校验和错误: 收到 {}, 期望 {}", buf[8], expected_sum);
                }
            } else {
                hcho_cache.status = SensorStatus::Error;
                defmt::error!("HCHO 读取超时");
            }
        }

        // ==========================================
        // 任务 B: 处理编码器旋转
        // ==========================================
        let enc_current_count = qei.count();
        let enc_diff = enc_current_count.wrapping_sub(enc_last_count) as i16;
        let mut enc_page_changed = false;
        
        if enc_diff >= ENCODER_STEP { // 过滤抖动
            current_page = current_page.next();
            enc_page_changed = true;
        } else if enc_diff <= -ENCODER_STEP {
            current_page = current_page.prev();
            enc_page_changed = true;
        }

        // 编码器触发翻页后的逻辑
        if enc_page_changed {
            match current_page { //换页刷新
                Page::TempHumi => refresh_ticks = TEMPHUMI_TICK_MEASURE - 1,
                Page::Hcho => refresh_ticks = HCHO_TICK_MEASURE - 1,
                _ => {} //保护 PM2.5 寿命
            }
            enc_last_count = enc_current_count;
            need_redraw = true;
            led.set_low();
            led_on_ticks = 1;
        }

        // ==========================================
        // 任务 C: 侦测按钮按下
        // ==========================================
        if sw.is_low() {
            sw_hold_ticks = sw_hold_ticks.saturating_add(1);
            led.set_low();
            led_on_ticks = 2;
            if sw_hold_ticks >= SW_LONG_PRESS_THRES && !sw_long_pressed_triggered {
                // 长按触发：切换静音
                is_muted = !is_muted;
                sw_long_pressed_triggered = true;
                need_redraw = true;
                defmt::info!("用户动作：长按 - 静音 {}", is_muted);
            }
        } else {
            if sw_hold_ticks >= SW_DEBOUNCE_THRES && sw_hold_ticks < SW_LONG_PRESS_THRES {
                // 短按触发：强制刷新当前页面数据
                match current_page {
                    Page::Pm25 => {
                        pm25_schedule_ticks = 0; 
                        defmt::info!("用户按键：PM2.5 强制刷新");
                    },
                    Page::TempHumi => {
                        refresh_ticks = TEMPHUMI_TICK_MEASURE - 1; 
                        defmt::info!("用户按键：温湿度强制刷新");
                    },
                    Page::Hcho => {
                        refresh_ticks = HCHO_TICK_MEASURE - 1; 
                        //hcho_cache.status = SensorStatus::Init;
                        defmt::info!("用户按键：HCHO强制刷新");
                    },
                }
                need_redraw = true;
                defmt::info!("用户动作：短按 - 刷新");
            }
            // 状态复位
            sw_hold_ticks = 0;
            sw_long_pressed_triggered = false;
        }

        // ==========================================
        // 空气安全判定
        // ==========================================
        let is_pm25_alarm = pm25_cache.status == SensorStatus::Ok && pm25_cache.pm2_5 > PM25_ALARM_THRES;
        let is_hcho_alarm = hcho_cache.status == SensorStatus::Ok && hcho_cache.ug_m3 > HCHO_ALARM_THRES;
        
        // ==========================================
        // 任务 D: 渲染 UI 画面
        // ==========================================
        if need_redraw {
            display.clear(BinaryColor::Off).unwrap();
            
            // 绘制顶部标题
            Text::with_baseline("=== Air Monitor ===", Point::new(10, 0), text_style_normal, Baseline::Top).draw(&mut display).unwrap();

            // 绘制翻页内容
            match current_page {
                Page::TempHumi => { 
                    Text::with_baseline("> PAGE 1: Temp/Humi", Point::new(10, 12), text_style_normal, Baseline::Top).draw(&mut display).unwrap(); 
                    match temphumi_cache.status {
                        SensorStatus::Ok => {
                            // 温度
                            let sign_str = if temphumi_cache.is_temp_neg { "-" } else { "" };
                            let mut temp_str = String::<32>::new();
                            write!(&mut temp_str, "Temp: {}{}.{} C", sign_str, temphumi_cache.temp_whole, temphumi_cache.temp_frac).unwrap();
                            Text::with_baseline(&temp_str, Point::new(10, 26), text_style_normal, Baseline::Top).draw(&mut display).unwrap();
                            // 湿度
                            let mut humi_str = String::<32>::new();
                            write!(&mut humi_str, "Humi: {}.{} %", temphumi_cache.humi_whole, temphumi_cache.humi_frac).unwrap();
                            Text::with_baseline(&humi_str, Point::new(10, 40), text_style_normal, Baseline::Top).draw(&mut display).unwrap();
                        },
                        SensorStatus::Init => {
                            Text::with_baseline("Loading data...", Point::new(10, 26), text_style_normal, Baseline::Top).draw(&mut display).unwrap();
                        },
                        SensorStatus::Error => {
                            // 传感器错误状态处理
                            Text::with_baseline("SENSOR ERROR!", Point::new(10, 26), text_style_normal, Baseline::Top).draw(&mut display).unwrap();
                            Text::with_baseline("Check I2C Wire.", Point::new(10, 40), text_style_normal, Baseline::Top).draw(&mut display).unwrap();
                        }
                    }
                },
                Page::Pm25 => { 
                    Text::with_baseline("> PAGE 2: PM 2.5", Point::new(10, 12), text_style_normal, Baseline::Top).draw(&mut display).unwrap(); 
                    match pm25_cache.status {
                        SensorStatus::Ok => {
                            // PM2.5
                            let mut pm2_5_str = String::<32>::new();
                            write!(&mut pm2_5_str, "PM2.5: {} ug/m3", pm25_cache.pm2_5).unwrap();
                            Text::with_baseline(&pm2_5_str, Point::new(10, 26), text_style_normal, Baseline::Top).draw(&mut display).unwrap();
                            // PM10
                            let mut pm10_str = String::<32>::new();
                            write!(&mut pm10_str, "PM10 : {} ug/m3", pm25_cache.pm10).unwrap();
                            Text::with_baseline(&pm10_str, Point::new(10, 40), text_style_normal, Baseline::Top).draw(&mut display).unwrap();
                            // 休眠提示
                            let pm25_ok_wait_sec = if pm25_schedule_ticks < PM25_TICK_MEASURE {
                                (PM25_TICK_MEASURE - pm25_schedule_ticks) / (TICK_1_SEC as u16)
                            } else {
                                (PM25_TICK_CYCLE - pm25_schedule_ticks + PM25_TICK_MEASURE) / (TICK_1_SEC as u16)
                            };
                            let pm25_status_2 = if pm25_schedule_ticks < PM25_TICK_MEASURE { "Run" } else { "Zzz" };
                            let mut pm25_tip_str = String::<32>::new();
                            write!(&mut pm25_tip_str, "[{}{}s]", pm25_status_2, pm25_ok_wait_sec).unwrap();
                            Text::with_baseline(&pm25_tip_str, Point::new(85, 56), text_style_small, Baseline::Top).draw(&mut display).unwrap();
                        },
                        SensorStatus::Init => {
                            // 计算剩余唤醒时间
                            let pm25_init_wait_sec = if pm25_schedule_ticks < PM25_TICK_MEASURE { (PM25_TICK_MEASURE / TICK_1_SEC as u16) - (pm25_schedule_ticks / TICK_1_SEC as u16) } else { 0 };
                            let mut pm25_init_wait_str = String::<32>::new();
                            write!(&mut pm25_init_wait_str, "Warming up... {}s", pm25_init_wait_sec).unwrap();
                            Text::with_baseline(&pm25_init_wait_str, Point::new(10, 26), text_style_normal, Baseline::Top).draw(&mut display).unwrap();
                        },
                        SensorStatus::Error => {
                            Text::with_baseline("SENSOR ERROR!", Point::new(10, 26), text_style_normal, Baseline::Top).draw(&mut display).unwrap();
                            Text::with_baseline("Check UART Wire.", Point::new(10, 40), text_style_normal, Baseline::Top).draw(&mut display).unwrap();
                        }
                    }
                },
                Page::Hcho => { 
                    Text::with_baseline("> PAGE 3: HCHO", Point::new(10, 12), text_style_normal, Baseline::Top).draw(&mut display).unwrap(); 
                    match hcho_cache.status {
                        SensorStatus::Ok => {
                            // 屏幕显示 1：微克/立方米
                            let mut ug_str = String::<32>::new();
                            write!(&mut ug_str, "Conc: {} ug/m3", hcho_cache.ug_m3).unwrap();
                            Text::with_baseline(&ug_str, Point::new(10, 26), text_style_normal, Baseline::Top).draw(&mut display).unwrap();
                            // 屏幕显示 2：毫克/立方米 (极其简单的数学，直接分离整数和小数)
                            let whole = hcho_cache.ug_m3 / 1000;
                            let frac = hcho_cache.ug_m3 % 1000;
                            let mut mg_str = String::<32>::new();
                            write!(&mut mg_str, "Eqv : {}.{:03} mg/m3", whole, frac).unwrap();
                            Text::with_baseline(&mg_str, Point::new(10, 40), text_style_normal, Baseline::Top).draw(&mut display).unwrap();
                        },
                        SensorStatus::Init => {
                            Text::with_baseline("Warming up...", Point::new(10, 26), text_style_normal, Baseline::Top).draw(&mut display).unwrap();
                            // 提示用户电化学需要预热 (手册第6页写了初次上电需24-48小时预热)
                            Text::with_baseline("Need 24h to stable", Point::new(10, 40), text_style_normal, Baseline::Top).draw(&mut display).unwrap();
                        },
                        SensorStatus::Error => {
                            Text::with_baseline("SENSOR ERROR!", Point::new(10, 26), text_style_normal, Baseline::Top).draw(&mut display).unwrap();
                            Text::with_baseline("Check PB10/PB11", Point::new(10, 40), text_style_normal, Baseline::Top).draw(&mut display).unwrap();
                        }
                    }
                },
            }

            // 绘制底部全局警报与静音标志
            // 绘制静音标志
            if is_muted {
                Text::with_baseline("[Mute]", Point::new(0, 56), text_style_small, Baseline::Top).draw(&mut display).unwrap();
            }
            // 绘制警报
            // 采用 Y=54 配合 text_style_normal 可以勉强显示， Y=56 配合 text_style_small 完美
            if is_pm25_alarm {
                Text::with_baseline("PM2.5 Warn!", Point::new(42, 56), text_style_small, Baseline::Top).draw(&mut display).unwrap();
            } else if is_hcho_alarm {
                Text::with_baseline("HCHO Warn!", Point::new(44, 56), text_style_small, Baseline::Top).draw(&mut display).unwrap();
            }

            // 一次性推送到 OLED
            display.flush().unwrap();
            need_redraw = false;
        }

        // ==========================================
        // 任务 D: LED 计数器
        // ==========================================
        if led_on_ticks > 0 {
            led_on_ticks -= 1;
            if led_on_ticks == 0 {
                led.set_high();
            }
        }

        // ==========================================
        // 任务 E: 蜂鸣器报警
        // ==========================================

        if (is_pm25_alarm || is_hcho_alarm) && !is_muted {
            if refresh_ticks < 5 || (refresh_ticks >= 10 && refresh_ticks < 15) {
                buzzer.set_low();  // 报警
            } else {
                buzzer.set_high(); // 停止报警
            }
        } else {
            buzzer.set_high(); // 停止报警
        }

    }
}