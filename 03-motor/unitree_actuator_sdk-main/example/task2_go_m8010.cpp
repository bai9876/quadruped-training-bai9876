#include <iostream>
#include <cmath>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <iomanip>
#include <string>
#include <algorithm>
#include "unitreeMotor/unitreeMotor.h"
#include "serialPort/SerialPort.h"

// ==================== 非阻塞键盘输入 ====================
class NonBlockingKeyboard {
public:
    NonBlockingKeyboard() {
        tcgetattr(STDIN_FILENO, &old_term_);
        new_term_ = old_term_;
        new_term_.c_lflag &= ~(ICANON | ECHO);
        new_term_.c_cc[VMIN] = 0;
        new_term_.c_cc[VTIME] = 0;
        tcsetattr(STDIN_FILENO, TCSANOW, &new_term_);
    }
    ~NonBlockingKeyboard() { tcsetattr(STDIN_FILENO, TCSANOW, &old_term_); }
    int getChar() {
        unsigned char ch;
        int n = read(STDIN_FILENO, &ch, 1);
        return (n > 0) ? ch : -1;
    }
private:
    struct termios old_term_, new_term_;
};

// ==================== 平滑插值器 ====================
class SmoothInterpolator {
public:
    SmoothInterpolator() : start_(0), target_(0), duration_(0), elapsed_(0), active_(false) {}
    void start(float start_pos, float target_pos, float duration_sec) {
        start_ = start_pos;
        target_ = target_pos;
        duration_ = duration_sec;
        elapsed_ = 0.0f;
        active_ = true;
    }
    float update(float dt) {
        if (!active_) return target_;
        elapsed_ += dt;
        if (elapsed_ >= duration_) {
            active_ = false;
            return target_;
        }
        float ratio = elapsed_ / duration_;
        float smooth_ratio = ratio * ratio * (3.0f - 2.0f * ratio);
        return start_ + (target_ - start_) * smooth_ratio;
    }
    bool isActive() const { return active_; }
    float target() const { return target_; }
private:
    float start_, target_, duration_, elapsed_;
    bool active_;
};

// ==================== 主程序 ====================
int main() {
    // 1. 初始化串口与电机
    SerialPort serial("/dev/ttyUSB0"); // 如果串口不对，请修改这里
    MotorCmd cmd;
    MotorData data;

    cmd.motorType = MotorType::GO_M8010_6;
    data.motorType = MotorType::GO_M8010_6;
    cmd.mode = queryMotorMode(MotorType::GO_M8010_6, MotorMode::FOC);
    cmd.id = 0; // 电机ID

    // ===== 核心参数 =====
    const float GEAR_RATIO = 6.33f; // 物理减速比（用于键盘输入输出端角度）
    const float kp_output = 2.0f; // 位置刚度（起步用小值防震）
    const float kd_output = 0.1f; // 阻尼

    // ========== 2. 安全启动：清空脏数据并读取初始位置 ==========
    cmd.kp = 0.0f; cmd.kd = 0.0f; cmd.tau = 0.0f; cmd.dq = 0.0f; cmd.q = 0.0f;
    for (int i = 0; i < 10; i++) {
        serial.sendRecv(&cmd, &data);
        usleep(10000); 
    }

    float raw_rotor_angle = 0.0f;
    for (int i = 0; i < 20; i++) {
        serial.sendRecv(&cmd, &data);
        if (data.merror == 0 && std::abs(data.q) < 50.0f) { 
            raw_rotor_angle = data.q; 
            break;
        }
        usleep(5000);
    }

    // ========== 3. 计算初始软件位置（无偏移，直接换算输出端） ==========
    float current_soft_pos = raw_rotor_angle / GEAR_RATIO;
    std::cout << "Software current position: " << current_soft_pos << " rad\n";

    // ========== 4. 原地锁死，防止抽动 ==========
    cmd.q = current_soft_pos * GEAR_RATIO;
    cmd.kp = kp_output;
    cmd.kd = kd_output;
    for (int i = 0; i < 20; i++) {
        serial.sendRecv(&cmd, &data);
        usleep(2000); // 原地稳住
    }

    // ========== 5. 平滑回归零点 ==========
    const float ZERO_SPEED = 0.4f; // 回零速度
    std::cout << "Moving to zero position slowly...\n";
    SmoothInterpolator interpolator;
    float duration_zero = std::max(0.5f, std::abs(current_soft_pos) / ZERO_SPEED);
    interpolator.start(current_soft_pos, 0.0f, duration_zero);
    
    NonBlockingKeyboard keyboard;
    const float dt = 0.002f; // 2ms控制周期
    float cmd_pos = current_soft_pos;

    while (interpolator.isActive()) {
        cmd_pos = interpolator.update(dt);
        cmd.q = cmd_pos * GEAR_RATIO; 
        serial.sendRecv(&cmd, &data);
        usleep(static_cast<int>(dt * 1000000));
    }
    cmd.q = 0.0f;
    serial.sendRecv(&cmd, &data);
    std::cout << "Zero position reached.\n";

    // ========== 6. 键盘输入目标角度（单位：弧度） ==========
    std::cout << "\nEnter target angle in radians (e.g., 0.523 for 30 deg), or 'q' to quit:\n";
    float target_pos = 0.0f;
    int print_counter = 0;
    std::string input_buffer = "";

    while (true) {
        int ch = keyboard.getChar();
        
        // 安全刹车退出
        if (ch == 'q' || ch == 'Q') {
            std::cout << "\nSafe braking at current position...\n";
            cmd.q = data.q; // 物理锁死当前位置
            for (int i = 0; i < 200; i++) {
                serial.sendRecv(&cmd, &data);
                usleep(2000); 
            }
            cmd.kp = 0.0f; cmd.kd = 0.0f; cmd.tau = 0.0f;
            serial.sendRecv(&cmd, &data);
            std::cout << "Motor safely stopped. Exiting.\n";
            break;
        }

        // 键盘缓冲逻辑（输入数字并按下回车后才执行）
        if (ch != -1) {
            if (ch == '\n' || ch == '\r') {
                if (!input_buffer.empty()) {
                    try {
                        float angle_input = std::stof(input_buffer);
                        angle_input = std::max(-3.14f, std::min(3.14f, angle_input)); // 软件限位
                        std::cout << "\nMoving to " << angle_input << " rad ...\n";
                        target_pos = angle_input;
                        float duration = std::max(0.5f, std::abs(target_pos - cmd_pos) / 0.8f);
                        interpolator.start(cmd_pos, target_pos, duration);
                    } catch (...) {
                        std::cout << "\n输入无效，请重新输入！\n";
                    }
                    input_buffer.clear();
                }
            } 
            else if ((ch >= '0' && ch <= '9') || ch == '.' || ch == '-') {
                input_buffer += (char)ch;
                std::cout << (char)ch << std::flush; // 屏幕回显
            }
        }

        // 插值器更新与指令下发
        if (interpolator.isActive()) {
            cmd_pos = interpolator.update(dt);
        } else {
            cmd_pos = interpolator.target();
        }
        
        cmd.q = cmd_pos * GEAR_RATIO; 
        serial.sendRecv(&cmd, &data);

        // 每5秒打印一次状态（500Hz * 5秒 = 2500次循环）
        print_counter++;
        if (print_counter >= 2500) {
            printf("merror: %d, temp: %d, tau: %.3f | 软件位置: %.3f rad\n", 
                   data.merror, data.temp, data.tau, cmd_pos);
            print_counter = 0; 
        }

        usleep(static_cast<int>(dt * 1000000));
    }

    return 0;
}

