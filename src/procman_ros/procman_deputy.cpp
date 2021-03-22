#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <inttypes.h>
#include <libgen.h>
#include <signal.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/poll.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>

#include <map>
#include <set>

#include "procman_ros/procman_deputy.hpp"
#include <ros/master.h>

using procman_ros::ProcmanCmdDesired;
using procman_ros::ProcmanCmdDesiredConstPtr;
using procman_ros::ProcmanDeputyInfo;
using procman_ros::ProcmanDeputyInfoConstPtr;
using procman_ros::ProcmanDiscovery;
using procman_ros::ProcmanDiscoveryConstPtr;
using procman_ros::ProcmanOrders;
using procman_ros::ProcmanOrdersConstPtr;
using procman_ros::ProcmanOutputConstPtr;

namespace procman {

#define ESTIMATED_MAX_CLOCK_ERROR_RATE 1.001

#define MIN_RESPAWN_DELAY_MS 10
#define MAX_RESPAWN_DELAY_MS 1000
#define RESPAWN_BACKOFF_RATE 2
#define DISCOVERY_TIME_MS 500

#define DEFAULT_STOP_SIGNAL 2
#define DEFAULT_STOP_TIME_ALLOWED 7

#define PROCMAN_MAX_MESSAGE_AGE_USEC 60000000LL

static int64_t timestamp_now() {
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return (int64_t)tv.tv_sec * 1000000 + tv.tv_usec;
}

struct DeputyCommand {
  ProcmanCommandPtr cmd;

  std::string cmd_id;

  SocketNotifierPtr stdout_notifier;

  // Each time the command is started, it's assigned a runid. The main purpose
  // of runid is to enable
  int32_t actual_runid;

  bool should_be_running;

  ProcessInfo previous_status;
  ProcessInfo current_status;
  float cpu_usage;

  std::string group;
  bool auto_respawn;

  ros::WallTimer respawn_timer;

  int64_t last_start_time;
  int respawn_backoff_ms;

  int stop_signal;
  float stop_time_allowed;

  int num_kills_sent;
  int64_t first_kill_time;

  // If true, the command should be stopped and removed from the deputy.
  bool remove_requested;
};

DeputyOptions DeputyOptions::Defaults() {
  DeputyOptions result;

  char buf[256];
  memset(buf, 0, sizeof(buf));
  gethostname(buf, sizeof(buf) - 1);
  result.deputy_id = buf;

  result.verbose = false;
  return result;
}

ProcmanDeputy::ProcmanDeputy(const DeputyOptions &options)
    : options_(options), event_loop_(), deputy_id_(options.deputy_id),
      cpu_load_(-1), deputy_start_time_(), deputy_pid_(getpid()), commands_(),
      exiting_(false), last_output_transmit_utime_(0), output_buf_size_(0),
      output_msg_() {
  // Do not initialise start time in construction initialisation list, because
  // it will give a false start time if roscore is not running
  deputy_start_time_ = timestamp_now();
  pm_ = new Procman();
  // Queue sizes are important because messages are not immediately processed -
  // if queue size is very small some deputies will never receive orders
  info_sub_ = nh_.subscribe("/procman/info", 10, &ProcmanDeputy::InfoReceived, this);
  discovery_sub_ =
      nh_.subscribe("/procman/discover", 10, &ProcmanDeputy::DiscoveryReceived, this);
  info_pub_ = nh_.advertise<procman_ros::ProcmanDeputyInfo>("/procman/info", 10);
  discover_pub_ =
      nh_.advertise<procman_ros::ProcmanDiscovery>("/procman/discover", 10);
  output_pub_ = nh_.advertise<procman_ros::ProcmanOutput>("/procman/output", 100);
  orders_sub_ =
      nh_.subscribe("/procman/orders", 10, &ProcmanDeputy::OrdersReceived, this);
  // Setup timers

  // must create this before calling onDiscoveryTimer, otherwise it can be
  // uninitialised and cause a segfault. Is not started when created
  one_second_timer_ =
      nh_.createWallTimer(ros::WallDuration(1),
                          &ProcmanDeputy::OnOneSecondTimer, this, false, false);

  // When the deputy is first created, periodically send out discovery messages
  // to see what other procman deputy processes are active
  discovery_timer_ = nh_.createWallTimer(
      ros::WallDuration(0.2), &ProcmanDeputy::OnDiscoveryTimer, this);
  OnDiscoveryTimer(ros::WallTimerEvent());

  // periodically check memory usage. This is never used?
  introspection_timer_ = nh_.createWallTimer(
      ros::WallDuration(120), &ProcmanDeputy::OnIntrospectionTimer, this, false,
      false);

  check_output_msg_timer_ = nh_.createWallTimer(
      ros::WallDuration(0.02), &ProcmanDeputy::MaybePublishOutputMessage, this);

  event_loop_.SetPosixSignals(
      {SIGINT, SIGHUP, SIGQUIT, SIGTERM, SIGCHLD},
      std::bind(&ProcmanDeputy::OnPosixSignal, this, std::placeholders::_1));

  output_msg_.deputy_id = deputy_id_;
  output_msg_.num_commands = 0;

}

ProcmanDeputy::~ProcmanDeputy() {
  for (auto item : commands_) {
    delete item.second;
  }
  delete pm_;
}

void ProcmanDeputy::Run() {
  while (ros::ok()) {
    ros::spinOnce();
    event_loop_.IterateOnce();
  }
}

void ProcmanDeputy::TransmitStr(const std::string &command_id,
                                const char *str) {
  bool found = false;
  for (int i = 0; i < output_msg_.num_commands && !found; ++i) {
    if (command_id != output_msg_.command_ids[i]) {
      continue;
    }
    output_msg_.text[i].append(str);
    output_buf_size_ += strlen(str);
    found = true;
  }
  if (!found) {
    output_msg_.num_commands++;
    output_msg_.command_ids.push_back(command_id);
    output_msg_.text.push_back(str);
    output_buf_size_ += strlen(str);
  }

  MaybePublishOutputMessage(ros::WallTimerEvent());
}

void ProcmanDeputy::PrintfAndTransmit(const std::string &command_id,
                                      const char *fmt, ...) {
  char buf[256];
  va_list ap;
  va_start(ap, fmt);

  const int len = vsnprintf(buf, sizeof(buf), fmt, ap);
  if (options_.verbose) {
    fputs(buf, stderr);
  }
  if (len) {
    TransmitStr(command_id, buf);
  }
}

void ProcmanDeputy::MaybePublishOutputMessage(
    const ros::WallTimerEvent &event) {
  if (output_buf_size_ == 0) {
    return;
  }

  const int64_t ms_since_last_transmit =
      abs(static_cast<int>(timestamp_now() - last_output_transmit_utime_)) /
      1000;

  if (output_buf_size_ > 4096 || (ms_since_last_transmit >= 10)) {
    output_msg_.timestamp = ros::Time::now();
    output_pub_.publish(output_msg_);
    // clear the message after publishing
    output_msg_.num_commands = 0;
    output_msg_.command_ids.clear();
    output_msg_.text.clear();
    output_buf_size_ = 0;

    last_output_transmit_utime_ = output_msg_.timestamp.toNSec() * 1e-3;
  }
}

// invoked when a child process writes something to its stdout/stderr fd
void ProcmanDeputy::OnProcessOutputAvailable(DeputyCommand *deputy_cmd) {
  ProcmanCommandPtr cmd = deputy_cmd->cmd;
  char buf[1024];
  const int bytes_read = read(cmd->StdoutFd(), buf, sizeof(buf) - 1);
  if (bytes_read > 0) {
    buf[bytes_read] = '\0';
    TransmitStr(deputy_cmd->cmd_id, buf);
  }
}

void ProcmanDeputy::MaybeScheduleRespawn(DeputyCommand *deputy_cmd) {
  if (deputy_cmd->auto_respawn && deputy_cmd->should_be_running) {
    deputy_cmd->respawn_timer.setPeriod(
        ros::WallDuration(deputy_cmd->respawn_backoff_ms / 1000));
    deputy_cmd->respawn_timer.start();
  }
}

int ProcmanDeputy::StartCommand(DeputyCommand *deputy_cmd, int desired_runid) {
  if (exiting_) {
    return -1;
  }
  ProcmanCommandPtr cmd = deputy_cmd->cmd;

  ROS_DEBUG("[%s] start\n", deputy_cmd->cmd_id.c_str());

  int status;
  deputy_cmd->should_be_running = true;
  deputy_cmd->respawn_timer.stop();

  // update the respawn backoff counter, to throttle how quickly a
  // process respawns
  int ms_since_started = (timestamp_now() - deputy_cmd->last_start_time) / 1000;
  if (ms_since_started < MAX_RESPAWN_DELAY_MS) {
    deputy_cmd->respawn_backoff_ms =
        std::min(MAX_RESPAWN_DELAY_MS / 1000,
                 deputy_cmd->respawn_backoff_ms * RESPAWN_BACKOFF_RATE / 1000);
  } else {
    int d = ms_since_started / MAX_RESPAWN_DELAY_MS;
    deputy_cmd->respawn_backoff_ms = std::max(MIN_RESPAWN_DELAY_MS / 1000,
                                      deputy_cmd->respawn_backoff_ms / 1000 >> d);
  }
  deputy_cmd->last_start_time = timestamp_now();

  pm_->StartCommand(cmd);

  fcntl(cmd->StdoutFd(), F_SETFL, O_NONBLOCK);
  deputy_cmd->stdout_notifier = event_loop_.AddSocket(
      cmd->StdoutFd(), SocketMonitor::kRead,
      std::bind(&ProcmanDeputy::OnProcessOutputAvailable, this, deputy_cmd));

  deputy_cmd->actual_runid = desired_runid;
  deputy_cmd->num_kills_sent = 0;
  deputy_cmd->first_kill_time = 0;
  return 0;
}

int ProcmanDeputy::StopCommand(DeputyCommand *mi) {
  ProcmanCommandPtr cmd = mi->cmd;

  if (!cmd->Pid()) {
    return 0;
  }

  mi->should_be_running = false;
  mi->respawn_timer.stop();

  int64_t now = timestamp_now();
  int64_t sigkill_time =
      mi->first_kill_time + (int64_t)(mi->stop_time_allowed * 1000000);
  bool okay;
  if (!mi->first_kill_time) {
    ROS_DEBUG("[%s] stop (signal %d)\n", mi->cmd_id.c_str(), mi->stop_signal);
    okay = pm_->KillCommand(cmd, mi->stop_signal);
    mi->first_kill_time = now;
    mi->num_kills_sent++;
  } else if (now > sigkill_time) {
    ROS_DEBUG("[%s] stop (signal %d)\n", mi->cmd_id.c_str(), SIGKILL);
    okay = pm_->KillCommand(cmd, SIGKILL);
  } else {
    return 0;
  }

  if (!okay) {
    PrintfAndTransmit(mi->cmd_id, "failed to send kill signal to command\n");
    return 1;
  }
  return 0;
}

void ProcmanDeputy::CheckForStoppedCommands() {
  ProcmanCommandPtr cmd = pm_->CheckForStoppedCommands();

  while (cmd) {
    DeputyCommand *mi = commands_[cmd];

    // check the stdout pipes to see if there is anything from stdout /
    // stderr.
    struct pollfd pfd = {cmd->StdoutFd(), POLLIN, 0};
    const int poll_status = poll(&pfd, 1, 0);
    if (pfd.revents & POLLIN) {
      OnProcessOutputAvailable(mi);
    }

    // did the child terminate with a signal?
    const int exit_status = cmd->ExitStatus();

    if (WIFSIGNALED(exit_status)) {
      const int signum = WTERMSIG(exit_status);
      ROS_DEBUG("[%s] terminated by signal %d (%s)\n", mi->cmd_id.c_str(),
                signum, strsignal(signum));
    } else if (exit_status != 0) {
      ROS_DEBUG("[%s] exited with status %d\n", mi->cmd_id.c_str(),
                WEXITSTATUS(exit_status));
    } else {
      ROS_DEBUG("[%s] exited\n", mi->cmd_id.c_str());
    }

    if (WIFSIGNALED(exit_status)) {
      int signum = WTERMSIG(exit_status);

      PrintfAndTransmit(mi->cmd_id, "%s\n", strsignal(signum), signum);
      if (WCOREDUMP(exit_status)) {
        PrintfAndTransmit(mi->cmd_id, "Core dumped.\n");
      }
    }

    if (mi->stdout_notifier) {
      mi->stdout_notifier.reset();
      pm_->CleanupStoppedCommand(cmd);
    }

    // remove ?
    if (mi->remove_requested) {
      ROS_DEBUG("[%s] remove\n", mi->cmd_id.c_str());
      // cleanup the private data structure used
      commands_.erase(cmd);
      pm_->RemoveCommand(cmd);
      delete mi;
    } else {
      MaybeScheduleRespawn(mi);
    }

    cmd = pm_->CheckForStoppedCommands();
    TransmitProcessInfo();
  }
}

void ProcmanDeputy::OnQuitTimer(const ros::WallTimerEvent &event) {
  for (auto &item : commands_) {
    DeputyCommand *mi = item.second;
    ProcmanCommandPtr cmd = item.first;
    if (cmd->Pid()) {
      ROS_DEBUG("[%s] stop (signal %d)\n", mi->cmd_id.c_str(), SIGKILL);
      pm_->KillCommand(cmd, SIGKILL);
    }
    commands_.erase(cmd);
    pm_->RemoveCommand(cmd);
    delete mi;
  }
}

void ProcmanDeputy::TransmitProcessInfo() {
  // build a deputy info message
  ProcmanDeputyInfo msg;
  msg.timestamp = ros::Time::now();
  msg.deputy_id = deputy_id_;
  msg.cpu_load = cpu_load_;
  msg.phys_mem_total_bytes = current_system_status.memtotal;
  msg.phys_mem_free_bytes = current_system_status.memfree;
  msg.swap_total_bytes = current_system_status.swaptotal;
  msg.swap_free_bytes = current_system_status.swapfree;

  msg.ncmds = commands_.size();
  msg.cmds.resize(msg.ncmds);

  int cmd_index = 0;
  for (auto &item : commands_) {
    ProcmanCommandPtr cmd = item.first;
    DeputyCommand *mi = item.second;

    msg.cmds[cmd_index].cmd.exec_str = cmd->ExecStr();
    msg.cmds[cmd_index].cmd.command_id = mi->cmd_id;
    msg.cmds[cmd_index].cmd.group = mi->group;
    msg.cmds[cmd_index].cmd.auto_respawn = mi->auto_respawn;
    msg.cmds[cmd_index].cmd.stop_signal = mi->stop_signal;
    msg.cmds[cmd_index].cmd.stop_time_allowed = mi->stop_time_allowed;
    msg.cmds[cmd_index].actual_runid = mi->actual_runid;
    msg.cmds[cmd_index].pid = cmd->Pid();
    msg.cmds[cmd_index].exit_code = cmd->ExitStatus();
    msg.cmds[cmd_index].cpu_usage = mi->cpu_usage;
    msg.cmds[cmd_index].mem_vsize_bytes = mi->current_status.vsize;
    msg.cmds[cmd_index].mem_rss_bytes = mi->current_status.rss;
    cmd_index++;
  }

  if (options_.verbose) {
    ROS_DEBUG("transmitting deputy info!\n");
  }
  info_pub_.publish(msg);
}

void ProcmanDeputy::UpdateCpuTimes() {
  if (!ReadSystemInfo(&current_system_status)) {
    return;
  }

  uint64_t elapsed_jiffies =
      current_system_status.user - previous_system_status.user +
      current_system_status.user_low - previous_system_status.user_low +
      current_system_status.system - previous_system_status.system +
      current_system_status.idle - previous_system_status.idle;

  uint64_t loaded_jiffies =
      current_system_status.user - previous_system_status.user +
      current_system_status.user_low - previous_system_status.user_low +
      current_system_status.system - previous_system_status.system;

  if (!elapsed_jiffies || loaded_jiffies > elapsed_jiffies) {
    cpu_load_ = 0;
  } else {
    cpu_load_ = (double)loaded_jiffies / elapsed_jiffies;
  }

  for (auto &item : commands_) {
    ProcmanCommandPtr sheriff_cmd = item.first;
    DeputyCommand *deputy_cmd = item.second;

    if (sheriff_cmd->Pid()) {
      if (!ReadProcessInfoWithChildren(sheriff_cmd->Pid(), &deputy_cmd->current_status)) {
        deputy_cmd->cpu_usage = 0;
        deputy_cmd->current_status.vsize = 0;
        deputy_cmd->current_status.rss = 0;
        perror("update_cpu_times - procinfo_read_proc_cpu_mem");
        // TODO handle this error
      } else {
        uint64_t used_jiffies = deputy_cmd->current_status.user -
                                deputy_cmd->previous_status.user +
                                deputy_cmd->current_status.system -
                                deputy_cmd->previous_status.system;

        if (!elapsed_jiffies || previous_system_status.user == 0 ||
            previous_system_status.system == 0 ||
            used_jiffies > elapsed_jiffies) {
          deputy_cmd->cpu_usage = 0;
        } else {
          deputy_cmd->cpu_usage = (double)used_jiffies / elapsed_jiffies;
        }
      }
    } else {
      deputy_cmd->cpu_usage = 0;
      deputy_cmd->current_status.vsize = 0;
      deputy_cmd->current_status.rss = 0;
    }

    deputy_cmd->previous_status = deputy_cmd->current_status;
  }

  previous_system_status = current_system_status;
}

void ProcmanDeputy::OnOneSecondTimer(const ros::WallTimerEvent &event) {
  UpdateCpuTimes();
  TransmitProcessInfo();
}

void ProcmanDeputy::OnIntrospectionTimer(const ros::WallTimerEvent &event) {
  int mypid = getpid();
  ProcessInfo pinfo;
  int status = ReadProcessInfo(mypid, &pinfo);
  if (0 != status) {
    perror("introspection_timeout - procinfo_read_proc_cpu_mem");
  }

  int nrunning = 0;
  for (ProcmanCommandPtr cmd : pm_->GetCommands()) {
    if (cmd->Pid()) {
      nrunning++;
    }
  }

  ROS_DEBUG(
      "MARK - rss: %" PRId64 " kB vsz: %" PRId64 " kB procs: %d (%d alive)\n",
      pinfo.rss / 1024, pinfo.vsize / 1024, (int)commands_.size(), nrunning);
}

void ProcmanDeputy::OnPosixSignal(int signum) {
  if (signum == SIGCHLD) {
    // a child process died.  check to see which one, and cleanup its
    // remains.
    CheckForStoppedCommands();
  } else {
    // quit was requested.  kill all processes and quit
    ROS_DEBUG("received signal %d (%s).  stopping all processes\n", signum,
              strsignal(signum));

    float max_stop_time_allowed = 1;

    // first, send everything a SIGINT to give them a chance to exit
    // cleanly.
    for (auto &item : commands_) {
      StopCommand(item.second);
    }
    exiting_ = true;

    // set a timer, after which everything will be more forcefully
    // terminated.
    quit_timer_ = nh_.createWallTimer(ros::WallDuration(max_stop_time_allowed),
                                      &ProcmanDeputy::OnQuitTimer, this);
  }

  if (exiting_) {
    // if we're exiting, and all child processes are dead, then exit.
    bool all_dead = true;
    for (ProcmanCommandPtr cmd : pm_->GetCommands()) {
      if (cmd->Pid()) {
        all_dead = false;
        break;
      }
    }
    if (all_dead) {
      ROS_DEBUG("all child processes are dead, exiting.\n");
      ros::shutdown();
    }
  }
}

static const ProcmanCmdDesired *
OrdersFindCmd(const ProcmanOrdersConstPtr &orders,
              const std::string &command_id) {
  for (int i = 0; i < orders->ncmds; i++) {
    if (command_id == orders->cmds[i].cmd.command_id) {
      return &orders->cmds[i];
    }
  }
  return nullptr;
}

void ProcmanDeputy::OrdersReceived(
    const procman_ros::ProcmanOrdersConstPtr &orders) {
  // ignore orders if we're exiting
  if (exiting_) {
    return;
  }
  // ignore orders for other deputies
  if (orders->deputy_id != deputy_id_) {
    if (options_.verbose)
      ROS_DEBUG("ignoring orders for other deputy %s\n",
                orders->deputy_id.c_str());
    return;
  }

  // ignore stale orders (where utime is too long ago)
  int64_t now = timestamp_now();
  if (now - orders->timestamp.toNSec() / 1000 > PROCMAN_MAX_MESSAGE_AGE_USEC) {
    for (int i = 0; i < orders->ncmds; i++) {
      const ProcmanCmdDesired &cmd_msg = orders->cmds[i];
      PrintfAndTransmit(
          cmd_msg.cmd.command_id,
          "ignoring stale orders (utime %d seconds ago). You may want to check "
          "the system clocks!\n",
          (int)((now - orders->timestamp.toNSec() / 1000) / 1000000));
    }
    return;
  }

  // attempt to carry out the orders
  int action_taken = 0;
  int i;
  if (options_.verbose)
    ROS_DEBUG("orders for me received with %d commands\n", orders->ncmds);
  for (i = 0; i < orders->ncmds; i++) {
    const ProcmanCmdDesired &cmd_msg = orders->cmds[i];

    if (options_.verbose)
      ROS_DEBUG("order %d: %s (%d, %d)\n", i, cmd_msg.cmd.exec_str.c_str(),
                cmd_msg.desired_runid, cmd_msg.force_quit);

    // do we already have this command somewhere?
    DeputyCommand *deputy_cmd = nullptr;
    for (auto &item : commands_) {
      if (item.second->cmd_id == cmd_msg.cmd.command_id) {
        deputy_cmd = item.second;
        break;
      }
    }
    ProcmanCommandPtr cmd;

    if (deputy_cmd) {
      cmd = deputy_cmd->cmd;
    } else {
      // if not, then create it.
      cmd = pm_->AddCommand(cmd_msg.cmd.exec_str);

      // allocate a private data structure
      deputy_cmd = new DeputyCommand();
      deputy_cmd->cmd_id = cmd_msg.cmd.command_id;
      deputy_cmd->group = cmd_msg.cmd.group;
      deputy_cmd->auto_respawn = cmd_msg.cmd.auto_respawn;
      deputy_cmd->stop_signal = cmd_msg.cmd.stop_signal;
      deputy_cmd->stop_time_allowed = cmd_msg.cmd.stop_time_allowed;
      deputy_cmd->last_start_time = 0;
      deputy_cmd->respawn_backoff_ms = MIN_RESPAWN_DELAY_MS;
      deputy_cmd->stdout_notifier.reset();
      deputy_cmd->actual_runid = 0;

      deputy_cmd->respawn_timer = nh_.createWallTimer(
          ros::WallDuration(MIN_RESPAWN_DELAY_MS / 1000),
          [this, deputy_cmd](const ros::WallTimerEvent &event) {
            if (deputy_cmd->auto_respawn && deputy_cmd->should_be_running &&
                !exiting_) {
              StartCommand(deputy_cmd, deputy_cmd->actual_runid);
            }
          },
          false, false);

      deputy_cmd->cmd = cmd;
      commands_[cmd] = deputy_cmd;
      action_taken = 1;

      ROS_DEBUG("[%s] new command [%s]\n", deputy_cmd->cmd_id.c_str(),
                cmd->ExecStr().c_str());
    }

    // check if the command needs to be started or stopped
    CommandStatus cmd_status = pm_->GetCommandStatus(cmd);

    // rename a command?  does not kill a running command, so effect does
    // not apply until command is restarted.
    if (cmd->ExecStr() != cmd_msg.cmd.exec_str) {
      ROS_DEBUG("[%s] exec str -> [%s]\n", deputy_cmd->cmd_id.c_str(),
                cmd_msg.cmd.exec_str.c_str());
      pm_->SetCommandExecStr(cmd, cmd_msg.cmd.exec_str);

      action_taken = 1;
    }

    // has auto-respawn changed?
    if (cmd_msg.cmd.auto_respawn != deputy_cmd->auto_respawn) {
      ROS_DEBUG("[%s] auto-respawn -> %d\n", deputy_cmd->cmd_id.c_str(),
                cmd_msg.cmd.auto_respawn);
      deputy_cmd->auto_respawn = cmd_msg.cmd.auto_respawn;
    }

    // change the group of a command?
    if (cmd_msg.cmd.group != deputy_cmd->group) {
      ROS_DEBUG("[%s] group -> [%s]\n", deputy_cmd->cmd_id.c_str(),
                cmd_msg.cmd.group.c_str());
      deputy_cmd->group = cmd_msg.cmd.group;
      action_taken = 1;
    }

    // change the stop signal of a command?
    if (deputy_cmd->stop_signal != cmd_msg.cmd.stop_signal) {
      ROS_DEBUG("[%s] stop signal -> [%d]\n", deputy_cmd->cmd_id.c_str(),
                cmd_msg.cmd.stop_signal);
      deputy_cmd->stop_signal = cmd_msg.cmd.stop_signal;
    }

    // change the stop time allowed of a command?
    if (deputy_cmd->stop_time_allowed != cmd_msg.cmd.stop_time_allowed) {
      ROS_DEBUG("[%s] stop time allowed -> [%f]\n", deputy_cmd->cmd_id.c_str(),
                cmd_msg.cmd.stop_time_allowed);
      deputy_cmd->stop_time_allowed = cmd_msg.cmd.stop_time_allowed;
    }

    deputy_cmd->should_be_running = !cmd_msg.force_quit;

    if (PROCMAN_CMD_STOPPED == cmd_status &&
        (deputy_cmd->actual_runid != cmd_msg.desired_runid) &&
        deputy_cmd->should_be_running) {
      StartCommand(deputy_cmd, cmd_msg.desired_runid);
      action_taken = 1;
    } else if (PROCMAN_CMD_RUNNING == cmd_status &&
               ((!deputy_cmd->should_be_running) ||
                (cmd_msg.desired_runid != deputy_cmd->actual_runid &&
                 cmd_msg.desired_runid != 0))) {
      StopCommand(deputy_cmd);
      action_taken = 1;
    } else if (cmd_msg.desired_runid != 0) {
      deputy_cmd->actual_runid = cmd_msg.desired_runid;
    }
  }

  // if there are any commands being managed that did not appear in the
  // orders, then stop and remove those commands
  std::vector<DeputyCommand *> toremove;
  for (auto &item : commands_) {
    DeputyCommand *mi = item.second;
    ProcmanCommandPtr cmd = item.first;
    const ProcmanCmdDesired *cmd_msg = OrdersFindCmd(orders, mi->cmd_id);

    if (!cmd_msg) {
      // push the orphaned command into a list first.  remove later, to
      // avoid corrupting the linked list (since this is a borrowed data
      // structure)
      toremove.push_back(mi);
    }
  }

  // cull orphaned commands
  for (DeputyCommand *mi : toremove) {
    ProcmanCommandPtr cmd = mi->cmd;

    if (cmd->Pid()) {
      ROS_DEBUG("[%s] scheduling removal\n", mi->cmd_id.c_str());
      mi->remove_requested = 1;
      StopCommand(mi);
    } else {
      ROS_DEBUG("[%s] remove\n", mi->cmd_id.c_str());
      // cleanup the private data structure used
      commands_.erase(cmd);
      pm_->RemoveCommand(cmd);
      delete mi;
    }

    action_taken = 1;
  }

  if (action_taken) {
    TransmitProcessInfo();
  }
}

void ProcmanDeputy::DiscoveryReceived(
    const procman_ros::ProcmanDiscoveryConstPtr &msg) {
  const int64_t now = timestamp_now();
  if (now < deputy_start_time_ + DISCOVERY_TIME_MS * 1000) {
    // received a discovery message while still in discovery mode.  Check to
    // see if it's from a conflicting deputy.
    if (msg->transmitter_id == deputy_id_ && msg->nonce != deputy_pid_) {
      ROS_DEBUG("ERROR.  Detected another deputy [%s].  Aborting to avoid "
                "conflicts.\n",
                msg->transmitter_id.c_str());
      exit(1);
    }
  } else {
    // received a discovery message while not in discovery mode.  Respond by
    // transmitting deputy info.
    TransmitProcessInfo();
  }
}

void ProcmanDeputy::InfoReceived(const ProcmanDeputyInfoConstPtr &msg) {
  int64_t now = timestamp_now();
  if (now < deputy_start_time_ + DISCOVERY_TIME_MS * 1000) {
    // A different deputy has reported while we're still in discovery mode.
    // Check to see if the deputy names are in conflict.
    if (msg->deputy_id == deputy_id_) {
      ROS_DEBUG("ERROR.  Detected another deputy [%s].  Aborting to avoid "
                "conflicts.\n",
                msg->deputy_id.c_str());
      exit(2);
    }
  } else {
    ROS_DEBUG("WARNING:  Still processing info messages while not in discovery "
              "mode??\n");
  }
}

void ProcmanDeputy::OnDiscoveryTimer(const ros::WallTimerEvent &event) {
  const int64_t now = timestamp_now();
  if (now < deputy_start_time_ + DISCOVERY_TIME_MS * 1000) {
    // Publish a discover message to check for conflicting deputies
    ProcmanDiscovery msg;
    msg.timestamp = ros::Time::now();
    msg.transmitter_id = deputy_id_;
    msg.nonce = deputy_pid_;
    discover_pub_.publish(msg);
  } else {
    //    ROS_DEBUG("Discovery period finished. Activating deputy.");

    // Discovery period is over. Stop subscribing to deputy info messages, and
    // start subscribing to sheriff orders.
    discovery_timer_.stop();

    info_sub_.shutdown();
    ROS_INFO("Subscribing to pm orders");

    // Start the timer to periodically transmit status information
    one_second_timer_.start();
    OnOneSecondTimer(ros::WallTimerEvent());
  }
}

static void usage() {
  fprintf(
      stderr,
      "usage: procman-deputy [options]\n"
      "\n"
      "  -h, --help        shows this help text and exits\n"
      "  -v, --verbose     verbose output\n"
      "  -i, --id NAME   use deputy id NAME instead of hostname\n"
      "  -l, --log PATH    dump messages to PATH instead of stdout\n"
      "  -s, --start-roscore  If there is no roscore, start one\n"
      "  -p, --persist-roscore  If set, persist roscore after the deputy shuts down, if it started one"
      "\n"
      "DEPUTY ID\n"
      "  The deputy id must be unique from other deputies.  On startup,\n"
      "  if another deputy with the same id is detected, the newly started\n"
      "  deputy will self-terminate.\n"
      "\n"
      "EXIT STATUS\n"
      "  0   Clean exit on SIGINT, SIGTERM\n"
      "  1   OS or other networking error\n"
      "  2   Conflicting deputy detected on the network\n");
}

} // namespace procman

using namespace procman;

int main(int argc, char **argv) {
  const char *optstring = "hvfl:i:sp";
  int c;
  bool start_roscore = false;
  bool persist_roscore = false;
  struct option long_opts[] = {
      {"help", no_argument, nullptr, 'h'},      {"verbose", no_argument, nullptr, 'v'},
      {"log", required_argument, nullptr, 'l'}, {"start-roscore", no_argument, nullptr, 's'},
      {"id", required_argument, nullptr, 'i'}, {"persist-roscore", no_argument, nullptr, 'p'},
      {nullptr, 0, nullptr, 0}};

  DeputyOptions dep_options = DeputyOptions::Defaults();
  char *logfilename = NULL;
  std::string deputy_id_override;

  while ((c = getopt_long(argc, argv, optstring, long_opts, 0)) >= 0) {
    switch (c) {
    case 'v':
      dep_options.verbose = true;
      break;
    case 'l':
      free(logfilename);
      logfilename = strdup(optarg);
      break;
    case 's':
      start_roscore = true;
      break;
    case 'p':
      persist_roscore = true;
      break;
    case 'i':
      deputy_id_override = optarg;
      break;
    case 'h':
    default:
      usage();
      return 1;
    }
  }

  // Add the directory containing procamn_deputy to PATH. This is mostly
  // for convenience.
  if (argc <= 0) {
    return 1;
  }
  char *argv0 = strdup(argv[0]);
  std::string newpath = std::string(dirname(argv0)) + ":" + getenv("PATH");
  printf("setting PATH to %s\n", newpath.c_str());
  setenv("PATH", newpath.c_str(), 1);
  free(argv0);

  // redirect stdout and stderr to a log file if the -l command line flag
  // was specified.
  if (logfilename) {
    int fd = open(logfilename, O_WRONLY | O_APPEND | O_CREAT, 0644);
    if (fd < 0) {
      perror("open");
      fprintf(stderr, "couldn't open logfile %s\n", logfilename);
      return 1;
    }
    close(1);
    close(2);
    if (dup2(fd, 1) < 0) {
      return 1;
    }
    if (dup2(fd, 2) < 0) {
      return 1;
    }
    close(fd);
    setlinebuf(stdout);
    setlinebuf(stderr);
  }

  // set deputy hostname to the system hostname
  if (!deputy_id_override.empty()) {
    dep_options.deputy_id = deputy_id_override;
  }

  try {
    ros::init(argc, argv, "procman_ros_deputy_" + dep_options.deputy_id);
  } catch (ros::InvalidNameException e) {
    ros::init(argc, argv, "procman_ros_deputy", ros::init_options::AnonymousName);
    ROS_WARN("Deputy name %s is not valid as a node name. Node will be anonymised.", dep_options.deputy_id.c_str());
  }

  if (start_roscore && !ros::master::check()) {
    pid_t pid = fork();
    if (pid == 0) {
      // redirect output so that it doesn't show on the terminal  
      std::string roscore_cmd = "roscore > /dev/null 2>&1";

      // If we want to persist the roscore, the child process changes its
      // process group and then runs a roscore. Changing the process group is
      // necessary so that it doesn't receive sigint from the terminal
      if (persist_roscore) {
        setpgid(0, 0);
        // background the process so that this child process of deputy exits
        // immediately after spawning the roscore
        roscore_cmd += " &";
      }

      int ignored = system(roscore_cmd.c_str());
      exit(0);
    }
    // parent process continues and runs the deputy, but the two processes are
    // no longer connected. Wait until core is started before continuing
    while (!ros::master::check()) {
      sleep(1);
    }
  }

  ProcmanDeputy pmd(dep_options);

  pmd.Run();

  return 0;
}
