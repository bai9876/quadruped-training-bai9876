#include <iostream>
#include <cmath>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <iomanip>
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
        float smooth_ratio = ratio * ratio * (3.0f - 2.0f * ratio); // 三次平滑
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
    // 1. 初始化串口与电机（根据实际情况修改端口）
    SerialPort serial("/dev/ttyUSB0");
    MotorCmd cmd;
    MotorData data;
 
    cmd.motorType = MotorType::GO_M8010_6;
    data.motorType = MotorType::GO_M8010_6;
    cmd.mode = queryMotorMode(MotorType::GO_M8010_6, MotorMode::FOC);
    cmd.id = 0; 
    
    const float kp_output = 2.0f;  // 先保持小一点，防止抖动
    const float kd_output = 0.1f;
    const float GEAR_RATIO = 6.33f;
    const float OFFSET_OUTPUT = 30.0f * 3.1415926f / 180.0f;
// ========== 1. 清空脏数据并准备 ==========
cmd.kp = 0.0f; cmd.kd = 0.0f; cmd.tau = 0.0f; cmd.dq = 0.0f; cmd.q = 0.0f;
for (int i = 0; i < 10; i++) {
    serial.sendRecv(&cmd, &data);
    usleep(10000); 
}

// ========== 2. 读取当前真实位置 ==========
float current_soft_pos = 0.0f;
for (int i = 0; i < 20; i++) {
    serial.sendRecv(&cmd, &data);
    if (data.merror == 0 && std::abs(data.q) < 10.0f) { 
        current_soft_pos = (data.q / GEAR_RATIO) - OFFSET_OUTPUT; 
        break;
    }
    usleep(5000);
}
std::cout << "Software current position: " << current_soft_pos << " rad\n";

// ========== 3. 原地锁死，防止抽动 ==========
cmd.q = (current_soft_pos + OFFSET_OUTPUT) * GEAR_RATIO; // 目标就是现在的位置
cmd.kp = kp_output;  // 加上刚度
cmd.kd = kd_output;  // 加上阻尼
for (int i = 0; i < 20; i++) {
    serial.sendRecv(&cmd, &data);
    usleep(2000); // 原地稳一下，此时绝不会抽
}

// ========== 4. 开始慢慢回零 ==========
const float ZERO_SPEED = 0.4f; 
std::cout << "Moving to zero position slowly...\n";
SmoothInterpolator interpolator;
float duration_zero = std::max(0.5f, std::abs(current_soft_pos) / ZERO_SPEED);
interpolator.start(current_soft_pos, 0.0f, duration_zero);

NonBlockingKeyboard keyboard;
const float dt = 0.002f;
float cmd_pos = current_soft_pos;

    while (interpolator.isActive()) {
        cmd_pos = interpolator.update(dt);
        cmd.q = (cmd_pos + OFFSET_OUTPUT) * GEAR_RATIO;                     
        serial.sendRecv(&cmd, &data);
        usleep(static_cast<int>(dt * 1000000));
    }
    cmd.q = 0.0f;
    serial.sendRecv(&cmd, &data);
    std::cout << "Zero position reached.\n";

    // 5. 第二阶段：键盘输入目标角度，平滑转动
    std::cout << "\nEnter target angle in radians (e.g., 1.57), or 'q' to quit:\n";
    float target_pos = 0.0f;
int print_counter = 0;
    while (true) {
        int ch = keyboard.getChar();
        if (ch == 'q' || ch == 'Q') {
    std::cout << "\nBraking at current position...\n";
    
    // 1. 读取当前实际位置，作为目标位置，准备原地锁死
    cmd.q = data.q; 
    
    // 2. 保持原有的控制参数，不要改变 kp 和 kd
    // 让电机用刚才转动的参数，死死抱住当前位置
    
    // 3. 持续发送指令一小段时间，让电机稳定下来
    for (int i = 0; i < 200; i++) {
        serial.sendRecv(&cmd, &data);
        usleep(2000); // 保持约0.4秒
    }
    
    // 4. 稳定后，直接把 kp 和 kd 全部设 0，然后退出程序
    // 此时电机断电（失去力矩），不会有振动
    cmd.kp = 0.0f;
    cmd.kd = 0.0f;
    cmd.tau = 0.0f;
    serial.sendRecv(&cmd, &data);
    
    std::cout << "Motor safely stopped. Exiting.\n";
    break;
}

        static std::string input_buffer = ""; // 用来暂存你输入的数字
if (ch != -1) {
    if (ch == '\n' || ch == '\r') { // 按下回车键
        if (!input_buffer.empty()) {
            try {
                float angle_input = std::stof(input_buffer);
                
                // ===== 下面是原来的移动逻辑 =====
                // 限幅保护（根据电机行程调整）
                angle_input = std::max(-3.14f, std::min(3.14f, angle_input));
                std::cout << "\nMoving to " << angle_input << " rad ...\n";
                target_pos = angle_input;
                float duration = std::max(0.5f, std::abs(target_pos - cmd_pos) / 0.8f);
                interpolator.start(cmd_pos, target_pos, duration);
                // ===== 移动逻辑结束 =====

            } catch (...) {
                std::cout << "\n输入无效，请重新输入！\n";
            }
            input_buffer.clear();
        }
    } 
    else if ((ch >= '0' && ch <= '9') || ch == '.' || ch == '-') {
        input_buffer += (char)ch; // 把数字、小数点、负号存起来
        std::cout << (char)ch << std::flush; // 在屏幕上回显，让你看到自己敲了什么
    }
}

        if (interpolator.isActive()) {
            cmd_pos = interpolator.update(dt);
        } else {
            cmd_pos = interpolator.target();
        }
        cmd.q = (cmd_pos + OFFSET_OUTPUT) * GEAR_RATIO;                     
        serial.sendRecv(&cmd, &data);
        print_counter++;
        if (print_counter >= 2500) {
        printf("merror: %d, temp: %d, tau: %.3f |软件位置: %.3f\n", data.merror, data.temp, data.tau, cmd_pos);
        print_counter = 0;
        }
        // 可选：实时打印当前角度
         float physical_output = data.q / GEAR_RATIO;
         std::cout << "\r物理输出端: " << std::fixed << std::setprecision(3) 
                   << physical_output << " rad " << std::flush;

        usleep(static_cast<int>(dt * 1000000));
    }

    // 6. 安全退出：发送刹车指令
    std::cout << "\nProgram exied safely.\n";
    return 0;
}
