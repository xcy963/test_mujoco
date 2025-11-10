#include "include/serial_com.h"
#include <filesystem>
#include <iostream>
namespace hitcrt {

bool SerialCom::findDevice(std::string& result) {
    namespace fs = std::filesystem;

    std::string device;
    for (const auto& devFile : fs::directory_iterator("/dev/")) {
        if (!fs::is_character_file(devFile))
            continue; // 串口属于字符设备

        device = devFile.path().string();
        if (device.find("ttyUSB") != std::string::npos) {
            result = device;
            return true;
        }
        if (device.find("ttyCH341USB") != std::string::npos) {
            result = device;
            return true;
        }
        if (device.find("ttyACM") != std::string::npos) {//typeC的
            result = device;
            return true;
        }
    }
    return false;
}

/// str为"auto"时,自动查找并打开第一个找到的设备
auto SerialCom::init(const std::string& device_name, int baudRate)
    -> INIT_STATE {
    if (device_name == "auto") {
        if (!findDevice(device)) {
            // std::cout << "[init: serial] Auto find ttyUSB* fail\n";
            m_isOpen = false;
            return INIT_STATE::AUTOFIND_FAILED;
        }
    } else {
        device = "/dev/" + device_name;
        if (!std::filesystem::is_character_file(
                device)) { // 包括文件不存在的情况
            // std::cerr << fmt::format("[init: serial] No Such File: {}\n ",
            //                          device);
            m_isOpen = false;
            return INIT_STATE::SPECIFY_FAILED;
        }
    }
    std::cout<<"找到的设备是"<<device<<std::endl;
    // fmt::print(stderr, "[serial] Use serial device \"{}\"", device);

    m_serialBase_ = std::make_unique<SerialBase>(device, baudRate);
    m_isOpen = true;
    return INIT_STATE::SUCCEEDED;
}
} // namespace hitcrt