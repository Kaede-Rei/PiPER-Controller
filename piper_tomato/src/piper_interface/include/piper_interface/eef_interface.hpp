#ifndef _piper_interface_eef_interface_hpp_
#define _piper_interface_eef_interface_hpp_

#include "tl_optional/optional.hpp"

#include "piper_interface/interface_module.hpp"
#include "piper_controller/eef_interface.hpp"
#include "piper_commander/cmd_dispatcher.hpp"
#include "piper_msgs2/CommandEef.h"

namespace piper {

// ! ========================= 接 口 变 量 / 结 构 体 / 枚 举 声 明 ========================= ! //



// ! ========================= 接 口 类 / 函 数 声 明 ========================= ! //

class EefCmdService : public ROSModuleInterface {
public:
    EefCmdService(ros::NodeHandle& nh, std::shared_ptr<EndEffector> eef, std::shared_ptr<EefCmdDispatcher> dispatcher, std::string service_name);
    ~EefCmdService() = default;

    EefCmdService(const EefCmdService&) = delete;
    EefCmdService& operator=(const EefCmdService&) = delete;
    EefCmdService(EefCmdService&&) = delete;
    EefCmdService& operator=(EefCmdService&&) = delete;

private:
    bool on_request(piper_msgs2::CommandEef::Request& req, piper_msgs2::CommandEef::Response& res);
    tl::optional<EefCmdRequest> convert_srvreq_to_eefreq(const piper_msgs2::CommandEef::Request& srv_req);

private:
    std::unique_ptr<ros::ServiceServer> _srv_;
    std::shared_ptr<EndEffector> _eef_;
    std::shared_ptr<EefCmdDispatcher> _dispatcher_;
};

// ! ========================= 模 版 方 法 实 现 ========================= ! //



}

#endif
