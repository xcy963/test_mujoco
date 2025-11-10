// 处理串口设备的查找，负责多线程收发的锁管理
// TODO: 锁管理
#include "serialBase.h"
#include <shared_mutex>

namespace hitcrt {
class SerialCom {
  protected:
    bool m_isOpen;
    std::unique_ptr<SerialBase> m_serialBase_;

    using WriteLock_t = std::unique_lock<std::shared_mutex>;
    using ReadLock_t = std::shared_lock<std::shared_mutex>;
    std::shared_mutex send_mtx, receive_mtx; //分开锁了

  public:
    std::string device;

  public:
    enum class INIT_STATE { SUCCEEDED, AUTOFIND_FAILED, SPECIFY_FAILED };

    // example: init("auto", 460800); / init("ttyUSB0", 921600);
    INIT_STATE init(const std::string& device, int baudRate);

    void send(unsigned char* ch, size_t length) {
        WriteLock_t wlck(send_mtx);
        m_serialBase_->send(ch, length);
    }
    void receive(unsigned char* buff, size_t& length) {
        ReadLock_t rlck(receive_mtx);
        m_serialBase_->receive(buff, length);
    }

    bool isOpened() const { return m_isOpen; }

  protected:
    bool findDevice(std::string& result);
};

} // namespace hitcrt