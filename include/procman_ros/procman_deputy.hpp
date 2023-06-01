#ifndef PROCMAN_PROCMAN_DEPUTY_HPP__
#define PROCMAN_PROCMAN_DEPUTY_HPP__

#include <set>
#include <string>
#include <variant>

#include "rclcpp/rclcpp.hpp"

#include "procman_ros/socket_monitor.hpp"
#include "procman/procman.hpp"
#include <procman_msgs/msg/procman_orders.hpp>
#include <procman_msgs/msg/procman_discovery.hpp>
#include <procman_msgs/msg/procman_deputy_info.hpp>
#include <procman_msgs/msg/procman_output.hpp>


namespace procman {

struct DeputyCommand;

struct DeputyOptions {
  static DeputyOptions Defaults();

  std::string deputy_id;
  bool verbose;
};

class ProcmanDeputy : public rclcpp::Node{
  public:
    ProcmanDeputy(const DeputyOptions& options);
    ~ProcmanDeputy();

    void Run();

  private:
    void OrdersReceived(const procman_msgs::msg::ProcmanOrders::SharedPtr orders);
    void DiscoveryReceived(const procman_msgs::msg::ProcmanDiscovery::SharedPtr msg);
    void InfoReceived(const procman_msgs::msg::ProcmanDeputyInfo::SharedPtr msg);

    void OnDiscoveryTimer();

    void OnOneSecondTimer();

    void OnIntrospectionTimer();

    void OnQuitTimer();

    void OnPosixSignal(int signum);

    void OnProcessOutputAvailable(DeputyCommand* deputy_cmd);

    void UpdateCpuTimes();

    void CheckForStoppedCommands();

    void TransmitProcessInfo();

    void MaybeScheduleRespawn(DeputyCommand *deputy_cmd);

    int StartCommand(DeputyCommand* deputy_cmd, int desired_runid);

    int StopCommand(DeputyCommand* deputy_cmd);

    void TransmitStr(const std::string& command_id, const char* str);

    void PrintfAndTransmit(const std::string& command_id, const char *fmt, ...);

    void MaybePublishOutputMessage();

    void ProcessSockets();

    DeputyOptions options_;

    Procman* pm_;

    SocketMonitor event_loop_;

    std::string deputy_id_;

    SystemInfo current_system_status;
    SystemInfo previous_system_status;
    float cpu_load_;

    int64_t deputy_start_time_;
    pid_t deputy_pid_;

    rclcpp::Subscription<procman_msgs::msg::ProcmanDiscovery>::SharedPtr discovery_sub_;
    rclcpp::Subscription<procman_msgs::msg::ProcmanDeputyInfo>::SharedPtr info_sub_;
    rclcpp::Subscription<procman_msgs::msg::ProcmanOrders>::SharedPtr orders_sub_;

    rclcpp::Publisher<procman_msgs::msg::ProcmanDeputyInfo>::SharedPtr info_pub_;
    rclcpp::Publisher<procman_msgs::msg::ProcmanDiscovery>::SharedPtr discover_pub_;
    rclcpp::Publisher<procman_msgs::msg::ProcmanOutput>::SharedPtr output_pub_;

    rclcpp::TimerBase::SharedPtr discovery_timer_;
    rclcpp::TimerBase::SharedPtr one_second_timer_;
    rclcpp::TimerBase::SharedPtr introspection_timer_;
    rclcpp::TimerBase::SharedPtr quit_timer_;
    rclcpp::TimerBase::SharedPtr check_output_msg_timer_;
    rclcpp::Clock clock_;

    std::map<ProcmanCommandPtr, DeputyCommand*> commands_;

    bool exiting_;

    int64_t last_output_transmit_utime_;
    int output_buf_size_;
    procman_msgs::msg::ProcmanOutput output_msg_;
};

}  // namespace procman

#endif  // PROCMAN_PROCMAN_DEPUTY_HPP__
