#include <nlohmann/json.hpp> // 引入JSON库

using json = nlohmann::json;
using SensorData = std::unordered_map<int, std::vector<std::vector<int>>>;

json constructSensorJson(const SensorData& sensor_data) 
{
    // 创建JSON对象
    json result = json::object();
    
    // 遍历传感器数据（假设只处理第一个端口）
    for (const auto& [port_id, points] : sensor_data) {
        // 设置端口ID字段
        result["id"] = std::to_string(port_id);
        
        // 创建FingerTip数组
        json finger_tip = json::array();
        
        // 转换点数据（假设每个点的坐标需要除以10.0）
        for (const auto& point : points) {
            if (point.size() == 3) {
                finger_tip.push_back(json::array({
                    point[0] / 10.0,  // X坐标转换
                    point[1] / 10.0,  // Y坐标转换
                    point[2] / 10.0   // Z坐标转换
                }));
            }
        }
        
        // 添加FingerTip字段
        result["FingerTip"] = finger_tip;
        break; // 只处理第一个端口
    }
    
    return result;
}




#include "Hand.h"

int main() {
    // 创建左手实例
    Hand leftHand("left_hand");
    
    // 添加所有关节
    const std::vector<std::string> jointNames = {
        "thumbMCP", "thumbIP", "IndexMCP", "IndexPIP", "IndexDIP",
        "MiddleMCP", "MiddlePIP", "MiddleDIP", "RingMCP", "RingPIP",
        "RingDIP", "LittleMCP", "LittlePIP", "LittleDIP"
    };
    
    for (const auto& name : jointNames) {
        leftHand.addJoint(name);
    }
    
    // 设置控制参数（位置模式，速度0.5，力度150）
    leftHand.setControlParams(ControlMode::POSITION, 0.5, 150);
    
    // 设置单个关节位置
    leftHand.setJoint("thumbMCP", 0.8);
    
    // 设置所有关节到0.5位置
    leftHand.setAllJoints(0.5);
    
    // 生成控制指令（模拟发送到API）
    std::string controlCommand = leftHand.executeMovement();
    // std::cout << controlCommand << std::endl;
    
    return 0;
}




#include "hand.hpp"
#include "serial_port.hpp" // 串口实现

int main() {
    // 创建串口实例
    auto joint_serial = createSerialPort();  // 关节串口
    auto sensor_serial = createSerialPort(); // 传感器串口

    // 创建 Hand 实例
    Hand hand("left_hand", joint_serial, sensor_serial);

    // 初始化串口
    if (!hand.initJointSerial("/dev/ttyUSB0", 115200)) {
        // 处理初始化失败
    }
    if (!hand.initSensorSerial("/dev/ttyUSB1", 9600)) {
        // 处理初始化失败
    }

    // 添加关节和传感器
    hand.addJoint("thumbMCP");
    hand.addSensor(SensorUtils::SensorType::THUMB, 1);

    // 设置关节角度
    hand.setJoint("thumbMCP", 0.5);

    // 获取传感器数据
    auto sensor = hand.getSensorArray().getSensor(SensorUtils::SensorType::THUMB);
    if (sensor) {
        sensor->setData({{10, 20, 30}, {40, 50, 60}});
        std::cout << sensor->toJson().dump(4) << std::endl;
    }

    return 0;
}