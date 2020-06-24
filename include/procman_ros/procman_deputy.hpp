#ifndef PROCMAN_PROCMAN_DEPUTY_HPP__
#define PROCMAN_PROCMAN_DEPUTY_HPP__

#include <set>
#include <string>
#include <ros/ros.h>

#include "procman_ros/event_loop.hpp"
#include "procman/procman.hpp"
#include <procman_ros/ProcmanOrders.h>
#include <procman_ros/ProcmanDiscovery.h>
#include <procman_ros/ProcmanDeputyInfo.h>
#include <procman_ros/ProcmanOutput.h>
#include "std_msgs/String.h"

namespace procman {

struct DeputyCommand;

struct DeputyOptions {
  static DeputyOptions Defaults();

  std::string deputy_id;
  std::string lcm_url;
  bool verbose;
};

class ProcmanDeputy {
  public:
    ProcmanDeputy(const DeputyOptions& options);
    ~ProcmanDeputy();

    void Run();

  private:
    void OrdersReceived(const procman_ros::ProcmanOrdersConstPtr& orders);
    void DiscoveryReceived(const procman_ros::ProcmanDiscoveryConstPtr& msg);
    void InfoReceived(const procman_ros::ProcmanDeputyInfoConstPtr& msg);

    void OnDiscoveryTimer(const ros::WallTimerEvent& event);

    void OnOneSecondTimer(const ros::WallTimerEvent& event);

    void OnIntrospectionTimer(const ros::WallTimerEvent& event);

    void OnQuitTimer(const ros::WallTimerEvent& event);

    void OnPosixSignal(int signum);

    void OnProcessOutputAvailable(DeputyCommand* mi);

    void UpdateCpuTimes();

    void CheckForStoppedCommands();

    void TransmitProcessInfo();

    void MaybeScheduleRespawn(DeputyCommand *mi);

    int StartCommand(DeputyCommand* mi, int desired_runid);

    int StopCommand(DeputyCommand* mi);

    void TransmitStr(const std::string& command_id, const char* str);

    void PrintfAndTransmit(const std::string& command_id, const char *fmt, ...);

    void MaybePublishOutputMessage(const ros::WallTimerEvent& event);

    void ProcessSockets();

    DeputyOptions options_;

    Procman* pm_;

    EventLoop event_loop_;

    std::string deputy_id_;

    SystemInfo cpu_time_[2];
    float cpu_load_;

    int64_t deputy_start_time_;
    pid_t deputy_pid_;

    ros::Subscriber discovery_sub_;
    ros::Subscriber info_sub_;
    ros::Subscriber orders_sub_;

    ros::Publisher info_pub_;
    ros::Publisher discover_pub_;
    ros::Publisher output_pub_;


    ros::NodeHandle nh_;

    ros::WallTimer discovery_timer_;
    ros::WallTimer one_second_timer_;
    ros::WallTimer introspection_timer_;
    ros::WallTimer quit_timer_;
    ros::WallTimer check_output_msg_timer_;

    std::map<ProcmanCommandPtr, DeputyCommand*> commands_;

    bool exiting_;

    int64_t last_output_transmit_utime_;
    int output_buf_size_;
    procman_ros::ProcmanOutput output_msg_;
};

}  // namespace procman

#endif  // PROCMAN_PROCMAN_DEPUTY_HPP__
