#include <Arduino.h>
#include <LovyanGFX.hpp>
#include <vector>
#include <DFRobotDFPlayerMini.h>
#include <HardwareSerial.h>
#include <Wire.h>        // MPU6050 的 I2C 接口

// ===== CONFIG =====
namespace Config {
  // 展示
  constexpr uint8_t DISPLAY_ROTATION = 0;  // 调整 ST7789 1.3 英寸的旋转角度
  constexpr uint8_t FRAME_DELAY_MS = 100;
  
  // GPIO 和传感器
  constexpr uint8_t BUTTON_PIN = 1;
  constexpr uint8_t DEBOUNCE_TIME_MS = 15;       // 降低设置以加快响应速度。
  constexpr uint16_t HOLDING_TIME_MS = 400;
  
  // 添加输入处理分离的配置。
  constexpr bool PRIORITIZE_INPUT = true;          // 优先处理输入数据。
  constexpr unsigned long INPUT_CHECK_INTERVAL = 5; // 更频繁地检查输入内容。
  
  // Audio
  constexpr uint8_t AUDIO_VOLUME = 28; // 音量（0-30）
  
  // MPU6050 的配置
  constexpr uint8_t SDA_PIN = 8;          // Chân SDA cho I2C
  constexpr uint8_t SCL_PIN = 9;          // I2C 的 SCL 引脚
  constexpr uint8_t MPU6050_I2C_ADDR = 0x68; // MPU6050 I2C 地址
  constexpr float SHAKE_THRESHOLD = 1.2;     // 摇晃检测阈值（克）
  constexpr unsigned long SHAKE_DETECTION_TIME = 1000; // 检测摇晃的时间 (ms)
  constexpr unsigned long SHAKE_COOLDOWN_TIME = 1000;  // 冷却时间，防止连续检测
}

// ===== 数据类型 =====
typedef struct _VideoInfo {
  const uint8_t* const* frames;
  const uint16_t* frames_size;
  uint16_t num_frames;
  uint8_t audio_idx;
} VideoInfo;

// ===== 视频声明 =====
// Tạm thời comment các video không có
// #include "video01.h"
// #include "video02.h"
// #include "video03.h"
// #include "video04.h"
// #include "video05.h"
// #include "video06.h"
// #include "video07.h"
// #include "video08.h"
// #include "video09.h"
// #include "video10.h"
// #include "video11.h"
// #include "video12.h"
// #include "video13.h"
// #include "video14.h"
// #include "video15.h"
// #include "video16.h"
// #include "video17.h"
// #include "video18.h"
#include "full1.h"
#include "chongmat1.h"
#include "xoadau1.h"

// ===== 按钮状态机 =====
enum class ButtonState {
  IDLE,           // 没有按钮
  PRESSED,        // 按钮刚被按下，正在等待判断是短按还是长按
  HELD,           // 按钮正在被保持
  RELEASED_SHORT, // 按钮在短按后被释放
  RELEASED_HOLD   // 按钮在长按后被释放
};

// ===== 玩家模式 =====
enum class PlayerMode {
  STOPPED,        // 停止模式（黑屏）
  PLAYING,        // 发放视频和音频
  PAUSED          // 暂停模式 (保持当前屏幕，不播放音频)
};

// ===== MPU6050 震动检测管理器 =====
class MPU6050Manager {
private:
  bool _initialized = false;
  uint8_t _i2cAddress = Config::MPU6050_I2C_ADDR;
  unsigned long _lastReadTime = 0;
  float _accelX = 0, _accelY = 0, _accelZ = 0;
  bool _shakeDetected = false;
  unsigned long _shakeStartTime = 0;
  unsigned long _lastShakeTime = 0; // 最后一次摇晃的时间
  
  // 用于计算绝对加速度的滤波器
  float _lastMagnitude = 1.0; // 从1g（重力）开始
  int _shakeCount = 0; // 计数 shake 次数在时间窗口内
  unsigned long _shakeWindowStart = 0; // 发现 shake 的窗口开始时间
  
public:
  MPU6050Manager() {}
  
  void init() {
    // I2C初始化
    Wire.begin(Config::SDA_PIN, Config::SCL_PIN);
    
    // 检查与 MPU6050 的连接。
    Wire.beginTransmission(_i2cAddress);
    uint8_t error = Wire.endTransmission();
    
    if (error == 0) {
      // 唤醒 MPU6050（退出睡眠模式）
      writeRegister(0x6B, 0x00);
      
      // 加速度计量程配置：±2g
      writeRegister(0x1C, 0x00);
      
      // 低通滤波器配置
      writeRegister(0x1A, 0x03);
      
      delay(100);
      _initialized = true;
      Serial.println("MPU6050 initialized successfully");
    } else {
      Serial.printf("Failed to initialize MPU6050, error: %d\n", error);
    }
  }
  
  void update() {
    if (!_initialized) {
      return;
    }
    
    unsigned long currentTime = millis();
    
    // 每 20 毫秒读取一次数据
    if (currentTime - _lastReadTime >= 20) {
      _lastReadTime = currentTime;
      
      // 读取加速度计数据
      readAccelData();
      
      // 计算总加速度的大小。
      float magnitude = sqrt(_accelX * _accelX + _accelY * _accelY + _accelZ * _accelZ);
      
      // 计算加速度的变化（抖动检测）。
      float deltaAccel = abs(magnitude - _lastMagnitude);
      _lastMagnitude = magnitude;
      
      // 检查冷却时间。
      if (currentTime - _lastShakeTime < Config::SHAKE_COOLDOWN_TIME) {
        return; // 仍处于搁置状态
      }
      
      // 如果加速度变化超过阈值，则会检测到震动
      if (deltaAccel > Config::SHAKE_THRESHOLD) {
        // 如果还没有 shake 窗口，开始新的窗口
        if (_shakeCount == 0) {
          _shakeWindowStart = currentTime;
        }
        
        _shakeCount++;
        Serial.printf("Shake event %d! Delta: %.2f g\n", _shakeCount, deltaAccel);
        
        // 检查在规定时间内是否有足够的摇晃。
        if (_shakeCount >= 3 && (currentTime - _shakeWindowStart) <= Config::SHAKE_DETECTION_TIME) {
          if (!_shakeDetected) {
            _shakeDetected = true;
            _lastShakeTime = currentTime;
            Serial.printf("STRONG SHAKE DETECTED! %d shakes in %lu ms\n", 
                         _shakeCount, currentTime - _shakeWindowStart);
          }
        }
      }
      
      // 如果指定时间已过，则重置摇晃窗口。
      if (_shakeCount > 0 && (currentTime - _shakeWindowStart) > Config::SHAKE_DETECTION_TIME) {
        _shakeCount = 0;
        Serial.println("Shake window reset");
      }
    }
  }
  
  bool isShakeDetected() {
    bool detected = _shakeDetected;
    if (detected) {
      _shakeDetected = false; // 读取后重置
    }
    return detected;
  }
  
  void getAcceleration(float &x, float &y, float &z) {
    x = _accelX;
    y = _accelY;
    z = _accelZ;
  }
  
private:
  void writeRegister(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(_i2cAddress);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
  }
  
  uint8_t readRegister(uint8_t reg) {
    Wire.beginTransmission(_i2cAddress);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(_i2cAddress, (uint8_t)1);
    return Wire.read();
  }
  
  void readAccelData() {
    Wire.beginTransmission(_i2cAddress);
    Wire.write(0x3B); // ACCEL_XOUT_H register
    Wire.endTransmission(false);
    Wire.requestFrom(_i2cAddress, (uint8_t)6);
    
    int16_t ax = (Wire.read() << 8) | Wire.read();
    int16_t ay = (Wire.read() << 8) | Wire.read();
    int16_t az = (Wire.read() << 8) | Wire.read();
    
    // 换算成克（误差范围±2克）
    _accelX = ax / 16384.0;
    _accelY = ay / 16384.0;
    _accelZ = az / 16384.0;
  }
  
public:
  static MPU6050Manager& getInstance() {
    static MPU6050Manager instance;
    return instance;
  }
};

// 适用于 ESP32-C3 和 ST7735 的 LovyanGFX 配置，分辨率为 128x160
class LGFX : public lgfx::LGFX_Device
{
  lgfx::Panel_ST7789 _panel_instance;
  lgfx::Bus_SPI _bus_instance;

public:
  LGFX(void)
  {
    // SPI总线配置
    {
      auto cfg = _bus_instance.config();
      cfg.spi_host = SPI2_HOST;
      cfg.spi_mode = 3;
      cfg.freq_write = 40000000;    // 提高ST7789的SPI速度
      cfg.freq_read  = 20000000;
      cfg.spi_3wire  = true;
      cfg.use_lock   = true;
      cfg.dma_channel = SPI_DMA_CH_AUTO; // 启用DMA
      cfg.pin_sclk = 4;    // SCK (TFT_SCLK)
      cfg.pin_mosi = 6;    // SDA (TFT_MOSI)
      cfg.pin_miso = -1;   // 请勿使用MISO引脚，因为ST7789不需要它
      cfg.pin_dc   = 3;    // DC/RS (TFT_DC)
      _bus_instance.config(cfg);
      _panel_instance.setBus(&_bus_instance);
    }

    // ST7789 1.3英寸 240x240显示器的面板配置
    {
      auto cfg = _panel_instance.config();
      cfg.pin_cs           = -1;     // 使用 CS 可以获得更好的稳定性，但需要额外的引脚。对于简单连接，保持 -1。
      cfg.pin_rst          = 10;    // RST (TFT_RST)
      cfg.pin_busy         = -1;
      cfg.panel_width      = 240;   // 物理尺寸 240x240
      cfg.panel_height     = 240;   // 物理尺寸 240x240
      cfg.offset_x         = 0;     // ST7789 1.3” 的偏移量通常为 0。
      cfg.offset_y         = 0;     // ST7789 1.3” 的偏移量通常为 0。
      cfg.offset_rotation  = 0;
      cfg.dummy_read_pixel = 8;
      cfg.dummy_read_bits  = 1;
      cfg.readable         = false;
      cfg.invert           = true;  // ST7789 通常需要颜色反转。
      cfg.rgb_order        = false;  // RGB
      cfg.dlen_16bit       = false;
      cfg.bus_shared       = true;

      _panel_instance.config(cfg);
    }

    setPanel(&_panel_instance);
  }
};

// ===== 显示管理=====
class DisplayManager {
private:
  LGFX _tft;
  
public:
  DisplayManager() : _tft() {}
  
  void init() {
    // 背光配置为 GPIO 引脚 7。
    pinMode(7, OUTPUT);
    digitalWrite(7, HIGH); // BLK ON
    
    _tft.init();
    _tft.setRotation(Config::DISPLAY_ROTATION);
    _tft.fillScreen(TFT_BLACK);
  }
  
  LGFX* getTft() { return &_tft; }
  
  // 显示屏背光控制（GPIO 7）
  void setBacklight(bool on) {
    digitalWrite(7, on ? HIGH : LOW);
  }
  
  void drawFrame(const VideoInfo* video, uint16_t frameIndex) {
    const uint8_t* jpg_data = (const uint8_t*)pgm_read_ptr(&video->frames[frameIndex]);
    uint16_t jpg_size = pgm_read_word(&video->frames_size[frameIndex]);
  
    _tft.drawJpg(jpg_data, jpg_size, 0, 0);
  }
  
  static DisplayManager& getInstance() {
    static DisplayManager instance;
    return instance;
  }
};

// ===== 音频管理 =====
class AudioManager {
private:
  DFRobotDFPlayerMini _dfPlayer;
  HardwareSerial _dfSerial;
  bool _isPlaying = false;
  uint8_t _currentTrack = 0;
  unsigned long _lastCommandTime = 0;
  static constexpr unsigned long MIN_COMMAND_INTERVAL = 50;

public:
  AudioManager() : _dfSerial(1) {} // ESP32-C3 使用硬件串口 1
  
  void init() {
    // 初始化 DFPlayer 的串口（接收：GPIO 21，发送：GPIO 20）
    _dfSerial.begin(9600, SERIAL_8N1, 21, 20);
    
    Serial.println("Initializing DFPlayer...");
    if (!_dfPlayer.begin(_dfSerial)) {
      Serial.println("Failed to initialize DFPlayer!");
    } else {
      Serial.println("DFPlayer Mini initialized!");
      _dfPlayer.setTimeOut(500);
      _dfPlayer.volume(Config::AUDIO_VOLUME);
      _dfPlayer.EQ(DFPLAYER_EQ_NORMAL);
    }
  }
  
  void play(uint8_t trackNumber) {
    if (millis() - _lastCommandTime < MIN_COMMAND_INTERVAL) {
      delay(MIN_COMMAND_INTERVAL);
    }
    
    // 播放新音频前，请暂停当前音频
    if (_isPlaying) {
      _dfPlayer.stop();
      delay(50);
    }
    _dfPlayer.play(trackNumber);
    _currentTrack = trackNumber;
    _isPlaying = true;
    _lastCommandTime = millis();
    
    Serial.printf("Playing audio track: %d\n", trackNumber);
  }
  
  void stop() {
    if (!_isPlaying) {
      return;
    }
    
    if (millis() - _lastCommandTime < MIN_COMMAND_INTERVAL) {
      delay(MIN_COMMAND_INTERVAL);
    }
    
    _dfPlayer.stop();
    _isPlaying = false;
    _lastCommandTime = millis();
  }
  
  static AudioManager& getInstance() {
    static AudioManager instance;
    return instance;
  }
};

// ===== 输入管理器=====
class InputManager {
private:
  ButtonState _buttonState = ButtonState::IDLE;
  unsigned long _buttonPressTime = 0;
  unsigned long _lastButtonReleaseTime = 0;
  int _lastReading = HIGH;
  unsigned long _lastDebounceTime = 0;
  unsigned long _lastUpdateTime = 0;
  
  // 物理按钮状态
  bool _physicalButtonState = false; // LOW（按下）= true，HIGH（释放）= false
  
public:
  void init() {
    pinMode(Config::BUTTON_PIN, INPUT_PULLUP);
    _lastUpdateTime = millis();
  }
  
  // 快速更新 - 只读按钮状态
  void quickUpdate() {
    unsigned long now = millis();
    if (now - _lastUpdateTime >= Config::INPUT_CHECK_INTERVAL) {
      _lastUpdateTime = now;
      
      // 读取物理按钮的状态（由于 INPUT_PULLUP 的原因，状态会反转）
      bool newButtonState = (digitalRead(Config::BUTTON_PIN) == LOW);
      
      // 如果状态发生变化，则更新防抖时间。
      if (newButtonState != _physicalButtonState) {
        _lastDebounceTime = now;
      }
      _physicalButtonState = newButtonState;
    }
  }
  
  // 完整更新 - 状态机处理
  void update() {
    quickUpdate(); // 请确保您拥有最新数据。
    processButton();
    // updateDoubleClickState() 部分已被移除。
  }
  
  ButtonState getButtonState() {
    ButtonState currentState = _buttonState;
    
    // Reset transient states
    if (_buttonState == ButtonState::RELEASED_SHORT || _buttonState == ButtonState::RELEASED_HOLD) {
      _buttonState = ButtonState::IDLE;
    }
    
    return currentState;
  }

private:
  // 根据 _physicalButtonState 处理按钮状态
  void processButton() {
    unsigned long now = millis();
    
    // 仅在反弹时间过后进行处理。
    if (now - _lastDebounceTime <= Config::DEBOUNCE_TIME_MS) {
      return;
    }
    
    // 根据 _physicalButtonState 处理按钮状态
    if (_physicalButtonState) {  // 按钮目前处于按下状态。
      if (_buttonState == ButtonState::IDLE) {
        _buttonState = ButtonState::PRESSED;
        _buttonPressTime = now;
        Serial.println("Button pressed");
      }
      else if (_buttonState == ButtonState::PRESSED && 
               now - _buttonPressTime >= Config::HOLDING_TIME_MS) {
        _buttonState = ButtonState::HELD;
        Serial.println("Button hold detected");
      }
    }
    else {  // 按钮正在松开
      if (_buttonState == ButtonState::PRESSED) {
        if (now - _buttonPressTime < Config::HOLDING_TIME_MS) {
          _buttonState = ButtonState::RELEASED_SHORT;
          _lastButtonReleaseTime = now;
          Serial.println("Button released (short press)");
        } else {
          _buttonState = ButtonState::RELEASED_HOLD;
          _lastButtonReleaseTime = now;
          Serial.println("Button released (after hold)");
        }
      }
      else if (_buttonState == ButtonState::HELD) {
        _buttonState = ButtonState::RELEASED_HOLD;
        _lastButtonReleaseTime = now;
        Serial.println("Button released (after hold)");
      }
    }
  }
  
  //updateDoubleClickState() 方法已被移除。

public:
  static InputManager& getInstance() {
    static InputManager instance;
    return instance;
  }
};

// ===== 视频播放器 =====
class VideoPlayer {
private:
  std::vector<VideoInfo*> _videos;
  uint8_t _currentIndex = 0;
  uint16_t _currentFrame = 0;
  PlayerMode _currentMode = PlayerMode::STOPPED; // 从静止状态开始
  unsigned long _lastFrameTime = 0;
  bool _playingSpecialVideo = false; // 标记以观看特别视频
  uint16_t _savedFrame = 0; // 中断时保存帧位置。
  bool _playingButtonHoldVideo = false; // 标记以监控视频，按住按钮
  
public:
  VideoPlayer() {
    // 使用 full 1（主视频）、chongmat 1（摇晃视频）和 xoadau 1（按住按钮视频）
    _videos = {
      &full1,      // Index 0: 主视频
      &chongmat1,  // Index 1: Video shake
      &xoadau1     // Index 2: Video button hold
    };
    
    // 请先观看第一个视频
    _currentIndex = 0;
    
    Serial.printf("Loaded %d video files\n", _videos.size());
    for(int i = 0; i < _videos.size(); i++) {
      Serial.printf("Video %d: %d frames, audio track %d\n", 
                   i, _videos[i]->num_frames, _videos[i]->audio_idx);
    }
  }
  
  void init() {
    // 开头是黑屏，然后自动开始播放视频。
    DisplayManager::getInstance().getTft()->fillScreen(TFT_BLACK);
    
    // 开机自动播放视频，无需按任何按钮。
    _currentMode = PlayerMode::PLAYING;
    _currentIndex = 0; // 从视频 17（正常）开始。
    _currentFrame = 0;
    _lastFrameTime = millis();
    DisplayManager::getInstance().setBacklight(true); // 打开背光。
    playCurrentAudio(); // 如有音频，请播放。
    
    Serial.println("Player initialized and automatically started playback");
  }
  
  void update() {
    // 在进行任何其他操作之前，请务必快速更新输入。
    InputManager::getInstance().quickUpdate();
    
    // 检查 MPU6050 的摇晃检测（最高优先级）
    handleShakeDetection();
    
    // 按键输入（优先级低于摇晃操作）
    handleButtonInput();
    
    // 只有在游戏模式下才能更新帧。
    if (_currentMode == PlayerMode::PLAYING) {
      unsigned long currentTime = millis();
      
      if (currentTime - _lastFrameTime >= Config::FRAME_DELAY_MS) {
        _lastFrameTime = currentTime;
        
        // 显示当前帧
        DisplayManager::getInstance().drawFrame(getCurrentVideo(), _currentFrame);
        
        // 提高帧速率
        _currentFrame++;
        if (_currentFrame >= getCurrentVideo()->num_frames) {
          // 视频结束时
          if (_playingSpecialVideo) {
            // 如果您正在播放特殊视频（chongmat1），请返回主视频。
            _playingSpecialVideo = false;
            _currentIndex = 0; // 返回完整版1
            _currentFrame = _savedFrame; // 返回到已保存的帧位置。
            playCurrentAudio();
            Serial.printf("视频 chongmat1 结束，返回视频 full1 的帧 %d\n", _savedFrame);
          } else if (_playingButtonHoldVideo) {
            // 如果当前正在播放带有按住按钮 (xoadau1) 的视频，则再次循环播放此视频。
            _currentFrame = 0;
            playCurrentAudio();
            Serial.println("Video xoadau1 loop lại");
          } else {
            // 如果主视频正在播放，则循环播放到开头。
            _currentFrame = 0;
            playCurrentAudio();
          }
        }
        
        // 绘制帧后快速更新输入。
        InputManager::getInstance().quickUpdate();
      }
    }
  }
  
private:
  // 手柄摇晃检测（优先级最高）
  void handleShakeDetection() {
    if (MPU6050Manager::getInstance().isShakeDetected()) {
      // 摇晃功能优先级最高——按住按钮甚至可以中断视频播放。
      if (!_playingSpecialVideo && _currentMode == PlayerMode::PLAYING) {
        // 保存当前帧位置
        if (_playingButtonHoldVideo) {
          // 如果视频播放时按住按钮，主视频的帧仍会被保存
          // （按键按住开始时帧已保存）
          Serial.println("摇晃手机暂停视频，按住按钮暂停视频");
        } else {
          // 如果主视频正在播放，则保存当前帧。
          _savedFrame = _currentFrame;
        }
        
        // 如果启用了旗帜按钮按住功能，请将其关闭并切换到摇晃模式。
        _playingButtonHoldVideo = false;
        _playingSpecialVideo = true;
        _currentIndex = 1; // 切换到 chongmat1
        _currentFrame = 0;
        playCurrentAudio();
        Serial.printf("检测到摇晃! 保存帧 %d, 切换到视频 chongmat1\n", _savedFrame);
      }
    }
  }
  
  // 按键输入（优先级低于摇晃操作）
  void handleButtonInput() {
    auto& inputManager = InputManager::getInstance();
    ButtonState buttonState = inputManager.getButtonState();
    
    // 操作锁定按钮（仅在摇晃视频未播放时）。
    if (buttonState == ButtonState::HELD) {
      // 如果 xoadau1 视频尚未播放，并且您当前没有播放 shake 视频，请切换到 xoadau1 视频。
      if (!_playingButtonHoldVideo && !_playingSpecialVideo && _currentMode == PlayerMode::PLAYING) {
        // 保存主视频的当前帧位置。
        _savedFrame = _currentFrame;
        
        _playingButtonHoldVideo = true;
        _currentIndex = 2; // 切换到 xoadau1
        _currentFrame = 0;
        playCurrentAudio();
        Serial.printf("检测到按键保持 Lưu frame %d, chuyển sang video xoadau1\n", _savedFrame);
      }
    }
    
    // 从保持功能中处理释放按钮（仅当摇晃视频未播放时）。
    if (buttonState == ButtonState::RELEASED_HOLD) {
      // 如果您当前正在播放按住按钮的视频而不是摇晃视频，请返回主视频。
      if (_playingButtonHoldVideo && !_playingSpecialVideo && _currentMode == PlayerMode::PLAYING) {
        _playingButtonHoldVideo = false;
        _currentIndex = 0; // 返回完整版1
        _currentFrame = _savedFrame; // 返回到已保存的帧位置
        playCurrentAudio();
        Serial.printf("松开按钮! 返回完整视频 1 的帧 %d\n", _savedFrame);
      }
    }
  }
  
  // 输入处理
  void handleInput() {
    auto& inputManager = InputManager::getInstance();
    ButtonState buttonState = inputManager.getButtonState();
    
    // 使用简单模型处理按钮按下事件。
    if (buttonState == ButtonState::RELEASED_SHORT) {
      // 按钮按下处理遵循一个简单的模式：停止 -> 播放 -> 停止
      switch (_currentMode) {
        case PlayerMode::STOPPED:
          // 暂停播放时，按下按钮即可恢复播放。
          _currentMode = PlayerMode::PLAYING;
          _currentIndex = 0; // 始终从视频 17（正常）开始
          _currentFrame = 0; // 从第一帧开始
          _lastFrameTime = millis(); // 重置时间，立即开始游戏。
          DisplayManager::getInstance().setBacklight(true); //打开背光。
          playCurrentAudio(); // 如有音频，请播放。
          Serial.println("开始播放视频");
          break;
          
        case PlayerMode::PLAYING:
          // 播放过程中，按下按钮即可停止播放。
          _currentMode = PlayerMode::STOPPED;
          AudioManager::getInstance().stop(); // 停止音频
          clearScreen(); // 将屏幕清空至黑色。
          Serial.println("停止播放视频");
          break;
          
        case PlayerMode::PAUSED:
          // 作为备用方案，以防以后需要暂停模式。
          _currentMode = PlayerMode::STOPPED;
          clearScreen();
          Serial.println("停止播放视频");
          break;
      }
    }
  }
  
  // 播放当前视频的音频
  void playCurrentAudio() {
    uint8_t audioTrack = getCurrentVideo()->audio_idx;
    // 仅当 audio_idx 不为 0 时才播放音频。
    if (audioTrack != 0) {
      AudioManager::getInstance().play(audioTrack);
    } else {
      //如果 audio_idx = 0，则停止当前正在播放的音频（如果有）。
      AudioManager::getInstance().stop();
      Serial.println("没有音频可播放（audio_idx = 0）");
    }
  }
  
  // 获取最新视频
  VideoInfo* getCurrentVideo() const {
    return _videos[_currentIndex];
  }
  
  // 将屏幕清空至黑色并关闭背光
  void clearScreen() {
    DisplayManager::getInstance().getTft()->fillScreen(TFT_BLACK);
    DisplayManager::getInstance().setBacklight(false); // 屏幕变黑时关闭背光
  }


public:
  static VideoPlayer& getInstance() {
    static VideoPlayer instance;
    return instance;
  }
};

// ===== 项目入口点 =====
void setup() {
  Serial.begin(115200);
  Serial.println("初始化...");
  
  // 在屏幕上添加调试代码。
  Serial.println("初始化显示...");
  DisplayManager::getInstance().init();
  
  LGFX* tft = DisplayManager::getInstance().getTft();
  Serial.printf("显示已初始化: %dx%d pixels\n", tft->width(), tft->height());
  
  AudioManager::getInstance().init();
  InputManager::getInstance().init();
  VideoPlayer::getInstance().init();
  
  // Khởi tạo MPU6050 cho phát hiện shake
  MPU6050Manager::getInstance().init();
  
  Serial.println("初始化完成 - 系统已准备就绪，处于停止模式");  
}

void loop() {
  // 请保持输入更新功能启用，以便其余代码正常运行
  InputManager::getInstance().quickUpdate();
  InputManager::getInstance().update();

  // 更新 MPU6050 以检测摇晃。
  MPU6050Manager::getInstance().update();

  // 更新视频播放器（包括摇晃检测处理）
  VideoPlayer::getInstance().update();
}