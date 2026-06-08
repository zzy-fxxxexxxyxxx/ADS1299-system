#include <SPI.h>
#include "esp_timer.h"

// ================== 宏定义区 ==================
// ---------- 片数定义，每片ADS的数据格式定义 ---------- 
#define NUM 4

#define SERIAL_BAUD 921600
#define PRINT_RAW_FRAME 0
#define STREAM_BINARY_FRAME 1

#define BIN_SYNC1 0xA5
#define BIN_SYNC2 0x5A
#define BIN_PROTOCOL_VERSION 0x01

#define CHANNELS_PER_ADS 8
#define STATUS_BYTES_PER_ADS 3
#define BYTES_PER_CHANNEL 3
#define BYTES_PER_ADS_FRAME (STATUS_BYTES_PER_ADS + CHANNELS_PER_ADS * BYTES_PER_CHANNEL)

#define TOTAL_CHANNELS (CHANNELS_PER_ADS * NUM)
#define TOTAL_FRAME_BYTES (BYTES_PER_ADS_FRAME * NUM)

#define ADS_REG_COUNT 24                      // 每片ADS要读取的寄存器数量

// ---------- 采样率配置 ----------
#define CONFIG1_SINGLE        0b11110110      // 单片模式，250 SPS，内部时钟输出
#define CONFIG1_MASTER_DAISY  0b10110110      // 菊花链模式主片，250 SPS，内部时钟输出
#define CONFIG1_SLAVE_DAISY   0b10010110      // 菊花链模式从片，250 SPS，使用外部时钟

#define CONFIG1_MASTER_REG ((NUM > 1) ? CONFIG1_MASTER_DAISY : CONFIG1_SINGLE)    //REG 是 register 的缩写，意思是“寄存器值”
#define CONFIG1_SLAVE_REG  CONFIG1_SLAVE_DAISY

#define CONFIG2_REG_TEST 0b11010101  // 内部产生测试信号，±0.00375V，1.95Hz
#define CONFIG2_REG 0b11000000  // 不使用内部测试信号
#define CONFIG3_REG 0b11101100  // 内部参考缓冲开启、BIASIN不接入模拟端

// ---------- 通道配置 ----------
// #define CHnSET_REG 0x60   // 增益 24x，普通电极输入
// #define CHnSET_REG 0b00000000   // 增益 ×1，普通电极输入
#define CHnSET_REG 0b01000000   // 增益 ×8，普通电极输入
#define CHnSET_TEST 0b01000101  //8倍增益，自检信号输入

const double vPerLSB = 4.5 / (8.0 * 8388607.0);   // ADS1299电压转换系数，对应8倍增益，单位：V

#define ENABLE_SRB1 0b00100000  // MISC1: 启用 SRB1
#define BIAS_SENSP 0xFF   // BIAS 正端全开
#define BIAS_SENSN 0xFF   // BIAS 负端全开

// ---------- 阻抗测量配置 ----------
#define LOFF_CONFIG 0x06  // 选择 6nA，FLEAD_OFF=10(31.2Hz)
#define LOFF_SENSP 0xFF   // 开启所有通道正端注入
#define LOFF_SENSN 0xFF   // 开启所有通道负端注入

// ---------- 模式选择 ----------
#define MODE_IDLE 0               // 空闲模式，是 上电->完成初始化 后的默认模式
#define MODE_CONTINUOUS_READ 1
#define MODE_IMPEDANCE_MEASURE 2
#define MODE_SELF_TEST 3

// ================== 引脚定义 ==================
#define CS_PIN1 A4
#define CS_PIN2 A3
#define SCLK_PIN SCK
#define MOSI_PIN MOSI
#define MISO_PIN MISO
#define DRDY_PIN A0
#define START_PIN A2
#define RESET_PIN A1

// ================== SPI 单字节命令列表 ==================
#define WAKEUP 0x02         // 从待机模式唤醒 ADS1299
#define STANDBY 0x04        // 让 ADS1299 进入低功耗待机模式
#define RESET 0x06          // 软件复位 ADS1299，类似拉低再释放 RESET 引脚
#define START 0x08          // 启动 ADC 转换（使用这个命令之后ADC就会工作，DRDY就会不断跳变）
#define STOP 0x0A           // 停止 ADC 转换
#define RDATAC 0x10         // 进入连续读取数据模式（该指令负责把START指令采样的值放到MISO上面，所以必须和START指令同时使用才有意义）
#define SDATAC 0x11         // 停止连续读取数据模式（注意只是停止连续读取，没有停止采样，如果ADC还在采样，DRDY还是在跳变，还是会触发中断）
#define RDATA 0x12          // 单次读取一帧数据
#define RREG 0b00100000     //  这是 ADS1299 的 RREG 读寄存器命令（的前缀），格式是 001r rrrr 后五位是寄存器地址
#define WREG 0b01000000     //  这是 ADS1299 的 WREG 写寄存器命令（的前缀），格式是 010r rrrr 后五位是寄存器地址

// ================== 全局变量 ==================
volatile bool dataReady = false;  // DRDY 中断标志
int currentMode = MODE_IDLE;
uint16_t binaryFrameCounter = 0;
// double channelDataBuffer[9];  // 0=STATUS，其余 8 个通道
uint32_t statusBuffer[NUM];                 // 每片 ADS1299 一个 STATUS
double channelDataBuffer[TOTAL_CHANNELS];   // 所有通道电压值，现在不把STATUS一股脑塞进channelDataBuffer了

enum AdsTarget { ADS_MASTER, ADS_SLAVES, ADS_ALL };

// ================== 函数声明 ==================
void IRAM_ATTR onDRDYInterrupt();                           // 中断，现在的中断只负责设置标志位
void initADS1299();                                         // 只写入寄存器
void printAllRegisters();                                   // 读取initADS1299执行后寄存器的值，验证spi通信是否正常，能否正常写入寄存器
void startContinuousReadMode();                             // 将连续读取模式的配置写入寄存器，并进入连续读取模式
void startImpedanceMeasurementMode();                       // 将阻抗检测模式的配置写入寄存器，并进入阻抗检测模式
void startSelfTestMode();                                   // 将自检模式的配置写入寄存器，并进入自检模式
void readData();                                            // 从MISO读取一帧原始数据，并通过串口发送二进制帧
void convertData(byte *data, uint32_t *statusData, double *channelData);          // 调试备用：解析数据并打印文本电压值
byte readRegister(byte reg);                                // 拉片选，读取指定寄存器的值，注意没有写SDATAC和恢复连续读取

void selectADS(AdsTarget target);
void deselectADS(AdsTarget target);
void sendCommand(AdsTarget target, byte cmd);               // 发命令：拉片选，发命令
void writeRegister(AdsTarget target, byte reg, byte value); // 写寄存器：拉片选，发要写的寄存器和要写的值
void readRegistersDaisy(byte startReg, byte regCount, byte *values);
void writeBinaryFrame(const byte *payload, uint16_t payloadLen);


// ================== setup (硬件初始化模块) ==================
void setup() {
  delay(1000);  
  Serial.begin(SERIAL_BAUD);

  // 初始化引脚
  pinMode(CS_PIN1, OUTPUT);
  pinMode(CS_PIN2, OUTPUT);

  pinMode(SCLK_PIN, OUTPUT);
  pinMode(MOSI_PIN, OUTPUT);
  pinMode(MISO_PIN, INPUT);
  pinMode(DRDY_PIN, INPUT);
  pinMode(START_PIN, OUTPUT);
  pinMode(RESET_PIN, OUTPUT);

  digitalWrite(CS_PIN1, HIGH);  // CS 默认拉高
  digitalWrite(CS_PIN2, HIGH);

  digitalWrite(START_PIN, LOW);
  digitalWrite(RESET_PIN, HIGH);
  delay(100);

  digitalWrite(RESET_PIN, LOW);
  delay(4);
  digitalWrite(RESET_PIN, HIGH);
  delay(100);
  
  // 初始化 SPI
  SPI.begin(SCLK_PIN, MISO_PIN, MOSI_PIN, CS_PIN1);    // CS 手动控制，CS_PIN1 仅作为 SPI 默认 SS
  // SPI.beginTransaction(SPISettings(SPI_CLOCK_DIV8, MSBFIRST, SPI_MODE1));
  SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE1));
  
  // 初始化 ADS1299
  initADS1299();
  printAllRegisters(); // 在这里调用，查看 init 后的状态
  Serial.println("ADS1299 初始化完成");
  
  // 配置外部中断（DRDY 下降沿触发）
  attachInterrupt(digitalPinToInterrupt(DRDY_PIN), onDRDYInterrupt, FALLING);

  currentMode = MODE_IDLE;
}

// ================== loop (主循环) ==================
void loop() {
  // 串口命令切换模式
  if (Serial.available()) {
    char cmd = Serial.read();
    if (cmd == '1') {
      currentMode = MODE_CONTINUOUS_READ;
      startContinuousReadMode();
    } else if (cmd == '2') {
      currentMode = MODE_IMPEDANCE_MEASURE;
      startImpedanceMeasurementMode();
    } else if (cmd == '3') {
      currentMode = MODE_SELF_TEST;
      startSelfTestMode();
    }
  }

  // 有数据时读取
  if (dataReady) {
    dataReady = false;
    readData();
  }


  // initADS1299();
  // printAllRegisters(); // 在这里调用，查看 init 后的状态
  // Serial.println("ADS1299 初始化完成");
  // delay(100);

}

// ================== 中断服务函数 ==================
void IRAM_ATTR onDRDYInterrupt() {
  dataReady = true;
}

// ================== 初始化及SPI检查 ==================
void initADS1299() {
  Serial.print("\n--- initADS1299开始执行 ---");
  sendCommand(ADS_ALL, RESET);   // 复位
  delay(100);
  sendCommand(ADS_ALL, STOP);  // 停止ADC转换
  sendCommand(ADS_ALL, SDATAC); // 停止连续读取

  writeRegister(ADS_MASTER, 0x01, CONFIG1_MASTER_REG);
  if (NUM > 1) writeRegister(ADS_SLAVES, 0x01, CONFIG1_SLAVE_REG);
  writeRegister(ADS_ALL, 0x02, CONFIG2_REG);
  writeRegister(ADS_ALL, 0x03, CONFIG3_REG);
  for (int i = 0x05; i <= 0x0C; i++) writeRegister(ADS_ALL, i, CHnSET_REG);
  writeRegister(ADS_ALL, 0x0D, BIAS_SENSP);
  writeRegister(ADS_ALL, 0x0E, BIAS_SENSN);
  writeRegister(ADS_ALL, 0x15, ENABLE_SRB1);

  // 默认关闭导联检测
  writeRegister(ADS_ALL, 0x04, 0x00);
  writeRegister(ADS_ALL, 0x0F, 0x00);
  writeRegister(ADS_ALL, 0x10, 0x00);

  Serial.print("\n--- initADS1299执行结束 ---");
}

void printAllRegisters() {
  Serial.println("\n--- ADS1299 Daisy Register Map ---");

  if (ADS_REG_COUNT != 24) {
    Serial.println("ADS_REG_COUNT mismatch");
    return;
  }

  byte regValues[ADS_REG_COUNT * NUM];

  // SDATAC 命令在 readRegistersDaisy 中会发送
  readRegistersDaisy(0x00, ADS_REG_COUNT, regValues);

  byte regAddresses[] = {
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B,
    0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17
  };

  const char* regNames[] = {
    "ID", "CONFIG1", "CONFIG2", "CONFIG3", "LOFF", "CH1SET", "CH2SET", "CH3SET",
    "CH4SET", "CH5SET", "CH6SET", "CH7SET", "CH8SET", "BIAS_SENSP", "BIAS_SENSN", "LOFF_SENSP",
    "LOFF_SENSN", "LOFF_FLIP", "LOFF_STATP", "LOFF_STATN", "GPIO", "MISC1", "MISC2", "CONFIG4"
  };

  for (int chip = 0; chip < NUM; chip++) {
    if (chip == 0) {
      Serial.println("\n[MASTER]");
    } else {
      Serial.print("\n[SLAVE ");
      Serial.print(chip);
      Serial.println("]");
    }

    for (int i = 0; i < ADS_REG_COUNT; i++) {
      byte val = regValues[chip * ADS_REG_COUNT + i];

      Serial.print("Reg 0x");
      if (regAddresses[i] < 0x10) Serial.print("0");
      Serial.print(regAddresses[i], HEX);
      Serial.print(" (");
      Serial.print(regNames[i]);
      Serial.print("): 0x");
      if (val < 0x10) Serial.print("0");
      Serial.println(val, HEX);
    }
  }

  Serial.println("----------------------------\n");
}

// ================== 模式配置 ==================
void startContinuousReadMode() {
  sendCommand(ADS_ALL, RESET);
  delay(100);
  sendCommand(ADS_ALL, SDATAC);

  writeRegister(ADS_MASTER, 0x01, CONFIG1_MASTER_REG);
  if (NUM > 1) {
    writeRegister(ADS_SLAVES, 0x01, CONFIG1_SLAVE_REG);
  }
  writeRegister(ADS_ALL, 0x02, CONFIG2_REG);
  writeRegister(ADS_ALL, 0x03, CONFIG3_REG);
  for (int i = 0x05; i <= 0x0C; i++) writeRegister(ADS_ALL, i, CHnSET_REG);
  writeRegister(ADS_ALL, 0x0D, BIAS_SENSP);
  writeRegister(ADS_ALL, 0x0E, BIAS_SENSN);
  writeRegister(ADS_ALL, 0x15, ENABLE_SRB1);

  sendCommand(ADS_ALL, START);
  sendCommand(ADS_ALL, RDATAC);
}

void startImpedanceMeasurementMode() {
  sendCommand(ADS_ALL, RESET);
  delay(100);
  sendCommand(ADS_ALL, SDATAC);

  writeRegister(ADS_MASTER, 0x01, CONFIG1_MASTER_REG);
  if (NUM > 1) {
    writeRegister(ADS_SLAVES, 0x01, CONFIG1_SLAVE_REG);
  }
  writeRegister(ADS_ALL, 0x02, CONFIG2_REG);
  writeRegister(ADS_ALL, 0x03, CONFIG3_REG);
  writeRegister(ADS_ALL, 0x0D, BIAS_SENSP);
  writeRegister(ADS_ALL, 0x0E, BIAS_SENSN);
  writeRegister(ADS_ALL, 0x15, ENABLE_SRB1);

  writeRegister(ADS_ALL, 0x04, LOFF_CONFIG);
  writeRegister(ADS_ALL, 0x0F, LOFF_SENSP);
  writeRegister(ADS_ALL, 0x10, LOFF_SENSN);
  for (int i = 0x05; i <= 0x0C; i++) writeRegister(ADS_ALL, i, CHnSET_REG);
  sendCommand(ADS_ALL, START);
  sendCommand(ADS_ALL, RDATAC);
  Serial.println("阻抗测量模式已启用（注意需解调导联频率信号）");
}

void startSelfTestMode() {
  sendCommand(ADS_ALL, RESET);
  delay(100);
  sendCommand(ADS_ALL, SDATAC); // 停止连续读取
  
  writeRegister(ADS_MASTER, 0x01, CONFIG1_MASTER_REG); 
  if (NUM > 1) { writeRegister(ADS_SLAVES, 0x01, CONFIG1_SLAVE_REG); }
  writeRegister(ADS_ALL, 0x02, CONFIG2_REG_TEST); 
  writeRegister(ADS_ALL, 0x03, CONFIG3_REG); 

  for (int i = 0x05; i <= 0x0C; i++) writeRegister(ADS_ALL, i, CHnSET_TEST); 

  sendCommand(ADS_ALL, START);
  delay(10);
  sendCommand(ADS_ALL, RDATAC); // 恢复连续读取模式
  delay(10);
  Serial.println("自检模式：250Hz, 最大幅值已启用");
}

void readData() {
  byte data[TOTAL_FRAME_BYTES];

  selectADS(ADS_ALL);
  for (int i = 0; i < TOTAL_FRAME_BYTES; i++) data[i] = SPI.transfer(0x00);   // 读取一个字节
  deselectADS(ADS_ALL);

#if PRINT_RAW_FRAME
  // 输出全部27*num个字节，以十六进制显示（每个字节两位，空格分隔）
  for (int i = 0; i < TOTAL_FRAME_BYTES; i++) {
    if (data[i] < 0x10) Serial.print("0");   // 保证输出两位
    Serial.print(data[i], HEX);
    if (i < TOTAL_FRAME_BYTES - 1) Serial.print(" ");           // 字节之间加空格
  }
  Serial.println();   // 换行
#endif

#if STREAM_BINARY_FRAME
  writeBinaryFrame(data, TOTAL_FRAME_BYTES);
#else
  convertData(data, statusBuffer, channelDataBuffer);
#endif
}

void convertData(byte *data, uint32_t *statusData, double *channelData) {
  for (int chip = 0; chip < NUM; chip++) {
    int chipBase = chip * BYTES_PER_ADS_FRAME;

    uint32_t statusValue = ((uint32_t)data[chipBase] << 16) | ((uint32_t)data[chipBase + 1] << 8) | data[chipBase + 2];

    statusData[chip] = statusValue;

    for (int ch = 0; ch < CHANNELS_PER_ADS; ch++) {
      int byteIndex = chipBase + STATUS_BYTES_PER_ADS + ch * BYTES_PER_CHANNEL;

      int32_t raw = ((int32_t)data[byteIndex] << 16) | ((int32_t)data[byteIndex + 1] << 8) | data[byteIndex + 2];

      if (raw & 0x800000) raw |= 0xFF000000;

      int globalChannel = chip * CHANNELS_PER_ADS + ch;
      channelData[globalChannel] = (double)raw * vPerLSB;
    }
  }

  Serial.print("FLAG:");
  for (int chip = 0; chip < NUM; chip++) {
    uint32_t topThree = statusData[chip] >> 12;
    Serial.print(topThree, HEX);
    if (chip < NUM - 1) Serial.print(",");
  }

  Serial.print("channel:");
  for (int i = 0; i < TOTAL_CHANNELS; i++) {
    Serial.print(channelData[i], 6);
    if (i < TOTAL_CHANNELS - 1) Serial.print(",");
  }
  Serial.println();
}

// ================== 底层 SPI 操作 ==================
void selectADS(AdsTarget target) {
  if (target == ADS_MASTER || target == ADS_ALL) { digitalWrite(CS_PIN1, LOW); }
  if (target == ADS_SLAVES || target == ADS_ALL) { digitalWrite(CS_PIN2, LOW); }
}

void deselectADS(AdsTarget target) {
  if (target == ADS_MASTER || target == ADS_ALL) { digitalWrite(CS_PIN1, HIGH); }
  if (target == ADS_SLAVES || target == ADS_ALL) { digitalWrite(CS_PIN2, HIGH); }
}

void sendCommand(AdsTarget target, byte cmd) {
  selectADS(target);
  SPI.transfer(cmd);
  delayMicroseconds(2);
  deselectADS(target);
}

void writeRegister(AdsTarget target, byte reg, byte value) {
  selectADS(target);
  SPI.transfer(WREG | reg);
  SPI.transfer(0x00);
  SPI.transfer(value);
  deselectADS(target);
}

void writeBinaryFrame(const byte *payload, uint16_t payloadLen) {
  byte checksum = 0;
  uint16_t seq = binaryFrameCounter++;

  byte header[] = {
    BIN_SYNC1,
    BIN_SYNC2,
    BIN_PROTOCOL_VERSION,
    (byte)NUM,
    (byte)(payloadLen & 0xFF),
    (byte)((payloadLen >> 8) & 0xFF),
    (byte)(seq & 0xFF),
    (byte)((seq >> 8) & 0xFF)
  };

  for (int i = 2; i < (int)sizeof(header); i++) checksum += header[i];
  for (uint16_t i = 0; i < payloadLen; i++) checksum += payload[i];

  Serial.write(header, sizeof(header));
  Serial.write(payload, payloadLen);
  Serial.write(checksum);
}

byte readRegister(byte reg) {
  digitalWrite(CS_PIN1, LOW);
  SPI.transfer(RREG | reg);
  SPI.transfer(0x00);
  byte val = SPI.transfer(0x00);
  digitalWrite(CS_PIN1, HIGH);
  return val;
}

void readRegistersDaisy(byte startReg, byte regCount, byte *values) {
  sendCommand(ADS_ALL, SDATAC);
  delay(10);

  selectADS(ADS_ALL);
  SPI.transfer(RREG | startReg);
  SPI.transfer(regCount - 1);

  for (int i = 0; i < regCount * NUM; i++) values[i] = SPI.transfer(0x00); 

  // selectADS(ADS_ALL);
  // SPI.transfer(RREG | startReg);
  // SPI.transfer(regCount - 1);

  // for (int i = 0; i < regCount; i++) values[i] = SPI.transfer(0x00); 

  // SPI.transfer(RREG | startReg);
  // SPI.transfer(regCount - 1);

  // for (int i = regCount; i < 2*regCount; i++) values[i] = SPI.transfer(0x00); 

  deselectADS(ADS_ALL);
}
