#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <libgen.h>
#include <signal.h>
#include <stdarg.h>
#include <inttypes.h>
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

#include <ros/ros.h>

#include "procman_deputy.hpp"
using procman_ros::ProcmanCmdDesired;
using procman_ros::ProcmanCmdDesiredConstPtr;
using procman_ros::ProcmanDeputyInfoConstPtr;
using procman_ros::ProcmanOutputConstPtr;
using procman_ros::ProcmanDiscovery;
using procman_ros::ProcmanDiscoveryConstPtr;
using procman_ros::ProcmanDeputyInfo;
using procman_ros::ProcmanDeputyInfoConstPtr;
using procman_ros::ProcmanOrders;
using procman_ros::ProcmanOrdersConstPtr;

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
  gettimeofday (&tv, NULL);
  return (int64_t) tv.tv_sec * 1000000 + tv.tv_usec;
}

struct DeputyCommand {
  ProcmanCommandPtr cmd;

  std::string cmd_id;

  SocketNotifierPtr stdout_notifier;

  // Each time the command is started, it's assigned a runid. The main purpose
  // of runid is to enable
  int32_t actual_runid;

  bool should_be_running;

  ProcessInfo cpu_time[2];
  float cpu_usage;

  std::string group;
  bool auto_respawn;

  TimerPtr respawn_timer;

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

ProcmanDeputy::ProcmanDeputy(const DeputyOptions& options) :
  options_(options),
  event_loop_(),
  deputy_id_(options.deputy_id),
  cpu_load_(-1),
  deputy_start_time_(timestamp_now()),
  deputy_pid_(getpid()),
  discovery_timer_(),
  one_second_timer_(),
  introspection_timer_(),
  quit_timer_(),
  commands_(),
  exiting_(false),
  last_output_transmit_utime_(0),
  output_buf_size_(0),
  output_msg_() {
  pm_ = new Procman();

  info_sub_ = nh_.subscribe("pm_info", 1, &ProcmanDeputy::InfoReceived, this);
  discovery_sub_ = nh_.subscribe("pm_discover", 1, &ProcmanDeputy::DiscoveryReceived, this);
  info_pub_ = nh_.advertise<procman_ros::ProcmanDeputyInfo>("pm_info", 10);
  discover_pub_ = nh_.advertise<procman_ros::ProcmanDiscovery>("pm_discover", 10);
  output_pub_ = nh_.advertise<procman_ros::ProcmanOutput>("pm_output", 10);
  orders_sub_ = nh_.subscribe("pm_orders", 1, &ProcmanDeputy::OrdersReceived, this);
  // Setup timers

  // must create this before calling onDiscoveryTimer, otherwise it can be
  // uninitialised and cause a segfault
  one_second_timer_ = event_loop_.AddTimer(1000, EventLoop::kRepeating, false,
      std::bind(&ProcmanDeputy::OnOneSecondTimer, this));

  // When the deputy is first created, periodically send out discovery messages
  // to see what other procman deputy processes are active
  discovery_timer_ = event_loop_.AddTimer(200, EventLoop::kRepeating, true,
      std::bind(&ProcmanDeputy::OnDiscoveryTimer, this));
  OnDiscoveryTimer();

  // periodically check memory usage
  introspection_timer_ = event_loop_.AddTimer(120000, EventLoop::kRepeating, false,
      std::bind(&ProcmanDeputy::OnIntrospectionTimer, this));

  check_output_msg_timer_ = event_loop_.AddTimer(10, EventLoop::kRepeating, true,
      std::bind(&ProcmanDeputy::MaybePublishOutputMessage, this));

  event_loop_.SetPosixSignals({ SIGINT, SIGHUP, SIGQUIT, SIGTERM, SIGCHLD },
      std::bind(&ProcmanDeputy::OnPosixSignal, this, std::placeholders::_1));

  output_msg_.deputy_id = deputy_id_;
  output_msg_.num_commands = 0;

  memset(cpu_time_, 0, sizeof(SystemInfo) * 2);
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

void ProcmanDeputy::TransmitStr(const std::string& command_id, const char* str) {
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

  MaybePublishOutputMessage();
}

void ProcmanDeputy::PrintfAndTransmit(const std::string& command_id, const char *fmt, ...) {
  char buf[256];
  va_list ap;
  va_start(ap, fmt);

  const int len = vsnprintf(buf, sizeof(buf), fmt, ap);
  if(options_.verbose) {
    fputs(buf, stderr);
  }
  if (len) {
    TransmitStr(command_id, buf);
  }
}

void ProcmanDeputy::MaybePublishOutputMessage() {
  if (output_buf_size_ == 0) {
    return;
  }

  const int64_t ms_since_last_transmit =
    abs(static_cast<int>(timestamp_now() - last_output_transmit_utime_)) / 1000;

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
void ProcmanDeputy::OnProcessOutputAvailable(DeputyCommand* mi) {
  ProcmanCommandPtr cmd = mi->cmd;
  char buf[1024];
  const int bytes_read = read(cmd->StdoutFd(), buf, sizeof(buf) - 1);
  if (bytes_read > 0) {
    buf[bytes_read] = '\0';
    TransmitStr(mi->cmd_id, buf);
  }
}

void ProcmanDeputy::MaybeScheduleRespawn(DeputyCommand *mi) {
  if(mi->auto_respawn && mi->should_be_running) {
    mi->respawn_timer->SetInterval(mi->respawn_backoff_ms);
    mi->respawn_timer->Start();
  }
}

int ProcmanDeputy::StartCommand(DeputyCommand* mi, int desired_runid) {
  if(exiting_) {
    return -1;
  }
  ProcmanCommandPtr cmd = mi->cmd;

  ROS_DEBUG("[%s] start\n", mi->cmd_id.c_str());

  int status;
  mi->should_be_running = true;
  mi->respawn_timer->Stop();

  // update the respawn backoff counter, to throttle how quickly a
  // process respawns
  int ms_since_started = (timestamp_now() - mi->last_start_time) / 1000;
  if(ms_since_started < MAX_RESPAWN_DELAY_MS) {
    mi->respawn_backoff_ms = std::min(MAX_RESPAWN_DELAY_MS,
        mi->respawn_backoff_ms * RESPAWN_BACKOFF_RATE);
  } else {
    int d = ms_since_started / MAX_RESPAWN_DELAY_MS;
    mi->respawn_backoff_ms = std::max(MIN_RESPAWN_DELAY_MS,
        mi->respawn_backoff_ms >> d);
  }
  mi->last_start_time = timestamp_now();

  pm_->StartCommand(cmd);

  fcntl(cmd->StdoutFd(), F_SETFL, O_NONBLOCK);
  mi->stdout_notifier = event_loop_.AddSocket(cmd->StdoutFd(),
      EventLoop::kRead,
      std::bind(&ProcmanDeputy::OnProcessOutputAvailable, this, mi));

  mi->actual_runid = desired_runid;
  mi->num_kills_sent = 0;
  mi->first_kill_time = 0;
  return 0;
}

int ProcmanDeputy::StopCommand(DeputyCommand* mi) {
  ProcmanCommandPtr cmd = mi->cmd;

  if (!cmd->Pid()) {
    return 0;
  }

  mi->should_be_running = false;
  mi->respawn_timer->Stop();

  int64_t now = timestamp_now();
  int64_t sigkill_time = mi->first_kill_time + (int64_t)(mi->stop_time_allowed * 1000000);
  bool okay;
  if(!mi->first_kill_time) {
    ROS_DEBUG("[%s] stop (signal %d)\n", mi->cmd_id.c_str(), mi->stop_signal);
    okay = pm_->KillCommand(cmd, mi->stop_signal);
    mi->first_kill_time = now;
    mi->num_kills_sent++;
  } else if(now > sigkill_time) {
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
    struct pollfd pfd = {
      cmd->StdoutFd(),
      POLLIN,
      0
    };
    const int poll_status = poll(&pfd, 1, 0);
    if (pfd.revents & POLLIN) {
      OnProcessOutputAvailable(mi);
    }

    // did the child terminate with a signal?
    const int exit_status = cmd->ExitStatus();

    if (WIFSIGNALED (exit_status)) {
      const int signum = WTERMSIG (exit_status);
      ROS_DEBUG("[%s] terminated by signal %d (%s)\n",
          mi->cmd_id.c_str(), signum, strsignal (signum));
    } else if (exit_status != 0) {
      ROS_DEBUG("[%s] exited with status %d\n",
          mi->cmd_id.c_str(), WEXITSTATUS (exit_status));
    } else {
      ROS_DEBUG("[%s] exited\n", mi->cmd_id.c_str());
    }

    if (WIFSIGNALED (exit_status)) {
      int signum = WTERMSIG (exit_status);

      PrintfAndTransmit(mi->cmd_id,
          "%s\n",
          strsignal (signum), signum);
      if (WCOREDUMP (exit_status)) {
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

void ProcmanDeputy::OnQuitTimer() {
  for (auto& item : commands_) {
    DeputyCommand* mi = item.second;
    ProcmanCommandPtr cmd = item.first;
    if (cmd->Pid()) {
      ROS_DEBUG("[%s] stop (signal %d)\n", mi->cmd_id.c_str(), SIGKILL);
      pm_->KillCommand(cmd, SIGKILL);
    }
    commands_.erase(cmd);
    pm_->RemoveCommand(cmd);
    delete mi;
  }

  ROS_DEBUG("stopping deputy main loop\n");
  event_loop_.Quit();
}

void ProcmanDeputy::TransmitProcessInfo() {
  // build a deputy info message
  ProcmanDeputyInfo msg;
  msg.timestamp = ros::Time::now();
  msg.deputy_id = deputy_id_;
  msg.cpu_load = cpu_load_;
  msg.phys_mem_total_bytes = cpu_time_[1].memtotal;
  msg.phys_mem_free_bytes = cpu_time_[1].memfree;
  msg.swap_total_bytes = cpu_time_[1].swaptotal;
  msg.swap_free_bytes = cpu_time_[1].swapfree;

  msg.ncmds = commands_.size();
  msg.cmds.resize(msg.ncmds);

  int cmd_index = 0;
  for (auto& item : commands_) {
    ProcmanCommandPtr cmd = item.first;
    DeputyCommand* mi = item.second;

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
    msg.cmds[cmd_index].mem_vsize_bytes = mi->cpu_time[1].vsize;
    msg.cmds[cmd_index].mem_rss_bytes = mi->cpu_time[1].rss;
    cmd_index++;
  }

  if (options_.verbose) {
    ROS_DEBUG("transmitting deputy info!\n");
  }
  info_pub_.publish(msg);
}

void ProcmanDeputy::UpdateCpuTimes() {
  if(! ReadSystemInfo(&cpu_time_[1])) {
    return;
  }

  SystemInfo *a = &cpu_time_[1];
  SystemInfo *b = &cpu_time_[0];

  uint64_t elapsed_jiffies = a->user - b->user +
    a->user_low - b->user_low +
    a->system - b->system +
    a->idle - b->idle;
  uint64_t loaded_jiffies = a->user - b->user +
    a->user_low - b->user_low +
    a->system - b->system;
  if (! elapsed_jiffies || loaded_jiffies > elapsed_jiffies) {
    cpu_load_ = 0;
  } else {
    cpu_load_ = (double)loaded_jiffies / elapsed_jiffies;
  }

  for (auto& item : commands_) {
    ProcmanCommandPtr cmd = item.first;
    DeputyCommand* mi = item.second;

    if (cmd->Pid()) {
      if (! ReadProcessInfo(cmd->Pid(), &mi->cpu_time[1])) {
        mi->cpu_usage = 0;
        mi->cpu_time[1].vsize = 0;
        mi->cpu_time[1].rss = 0;
        perror("update_cpu_times - procinfo_read_proc_cpu_mem");
        // TODO handle this error
      } else {
        ProcessInfo *pa = &mi->cpu_time[1];
        ProcessInfo *pb = &mi->cpu_time[0];

        uint64_t used_jiffies = pa->user - pb->user +
          pa->system - pb->system;

        if (! elapsed_jiffies || pb->user == 0 || pb->system == 0 ||
            used_jiffies > elapsed_jiffies) {
          mi->cpu_usage = 0;
        } else {
          mi->cpu_usage = (double)used_jiffies / elapsed_jiffies;
        }
      }
    } else {
      mi->cpu_usage = 0;
      mi->cpu_time[1].vsize = 0;
      mi->cpu_time[1].rss = 0;
    }

    mi->cpu_time[0] = mi->cpu_time[1];
  }

  cpu_time_[0] = cpu_time_[1];
}

void ProcmanDeputy::OnOneSecondTimer() {
  UpdateCpuTimes();
  TransmitProcessInfo();
}

void ProcmanDeputy::OnIntrospectionTimer() {
  int mypid = getpid();
  ProcessInfo pinfo;
  int status = ReadProcessInfo(mypid, &pinfo);
  if(0 != status)  {
    perror("introspection_timeout - procinfo_read_proc_cpu_mem");
  }

  int nrunning = 0;
  for (ProcmanCommandPtr cmd : pm_->GetCommands()) {
    if (cmd->Pid()) {
      nrunning++;
    }
  }

  ROS_DEBUG("MARK - rss: %" PRId64 " kB vsz: %" PRId64
      " kB procs: %d (%d alive)\n",
      pinfo.rss / 1024, pinfo.vsize / 1024,
      (int) commands_.size(),
      nrunning
      );
}

void ProcmanDeputy::OnPosixSignal(int signum) {
  if (signum == SIGCHLD) {
    // a child process died.  check to see which one, and cleanup its
    // remains.
    CheckForStoppedCommands();
  } else {
    // quit was requested.  kill all processes and quit
    ROS_DEBUG("received signal %d (%s).  stopping all processes\n", signum,
        strsignal (signum));

    float max_stop_time_allowed = 1;

    // first, send everything a SIGINT to give them a chance to exit
    // cleanly.
    for (auto& item : commands_) {
      StopCommand(item.second);
    }
    exiting_ = true;

    // set a timer, after which everything will be more forcefully
    // terminated.
    quit_timer_ = event_loop_.AddTimer(max_stop_time_allowed * 1000,
        EventLoop::kSingleShot, true,
        std::bind(&ProcmanDeputy::OnQuitTimer, this));
  }

  if(exiting_) {
    // if we're exiting, and all child processes are dead, then exit.
    bool all_dead = true;
    for (ProcmanCommandPtr cmd : pm_->GetCommands()) {
      if (cmd->Pid()) {
        all_dead = false;
        break;
      }
    }
    if(all_dead) {
      ROS_DEBUG("all child processes are dead, exiting.\n");
      event_loop_.Quit();
    }
  }
}

static const ProcmanCmdDesired* OrdersFindCmd (const ProcmanOrdersConstPtr& orders,
    const std::string& command_id) {
  for (int i=0; i<orders->ncmds; i++) {
    if (command_id == orders->cmds[i].cmd.command_id) {
      return &orders->cmds[i];
    }
  }
  return nullptr;
}

void ProcmanDeputy::OrdersReceived(const procman_ros::ProcmanOrdersConstPtr& orders) {
  // ignore orders if we're exiting
  if (exiting_) {
    return;
  }
  // ignore orders for other deputies
  if (orders->deputy_id != deputy_id_) {
    if (options_.verbose)
      ROS_DEBUG("ignoring orders for other deputy %s\n", orders->deputy_id.c_str());
    return;
  }

  // ignore stale orders (where utime is too long ago)
  int64_t now = timestamp_now ();
  if (now - orders->timestamp.toNSec()/1000 > PROCMAN_MAX_MESSAGE_AGE_USEC) {
    for (int i=0; i<orders->ncmds; i++) {
      const ProcmanCmdDesired& cmd_msg = orders->cmds[i];
      PrintfAndTransmit(cmd_msg.cmd.command_id,
          "ignoring stale orders (utime %d seconds ago). You may want to check the system clocks!\n",
          (int) ((now - orders->timestamp.toNSec()/1000) / 1000000));
    }
    return;
  }

  // attempt to carry out the orders
  int action_taken = 0;
  int i;
  if (options_.verbose)
    ROS_DEBUG("orders for me received with %d commands\n", orders->ncmds);
  for (i=0; i<orders->ncmds; i++) {
    const ProcmanCmdDesired& cmd_msg = orders->cmds[i];

    if (options_.verbose)
      ROS_DEBUG("order %d: %s (%d, %d)\n",
          i, cmd_msg.cmd.exec_str.c_str(),
          cmd_msg.desired_runid, cmd_msg.force_quit);

    // do we already have this command somewhere?
    DeputyCommand *mi = nullptr;
    for (auto& item : commands_) {
      if (item.second->cmd_id == cmd_msg.cmd.command_id) {
        mi = item.second;
        break;
      }
    }
    ProcmanCommandPtr cmd;

    if (mi) {
      cmd = mi->cmd;
    } else {
      // if not, then create it.
      cmd = pm_->AddCommand(cmd_msg.cmd.exec_str);

      // allocate a private data structure
      mi = new DeputyCommand();
      mi->cmd_id = cmd_msg.cmd.command_id;
      mi->group = cmd_msg.cmd.group;
      mi->auto_respawn = cmd_msg.cmd.auto_respawn;
      mi->stop_signal = cmd_msg.cmd.stop_signal;
      mi->stop_time_allowed = cmd_msg.cmd.stop_time_allowed;
      mi->last_start_time = 0;
      mi->respawn_backoff_ms = MIN_RESPAWN_DELAY_MS;
      mi->stdout_notifier.reset();
      mi->actual_runid = 0;

      mi->respawn_timer = event_loop_.AddTimer(MIN_RESPAWN_DELAY_MS,
          EventLoop::kSingleShot, false,
          [this, mi]() {
          if(mi->auto_respawn && mi->should_be_running && !exiting_) {
            StartCommand(mi, mi->actual_runid);
          }
          });

      mi->cmd = cmd;
      commands_[cmd] = mi;
      action_taken = 1;

      ROS_DEBUG("[%s] new command [%s]\n", mi->cmd_id.c_str(),
          cmd->ExecStr().c_str());
    }

    // check if the command needs to be started or stopped
    CommandStatus cmd_status = pm_->GetCommandStatus(cmd);

    // rename a command?  does not kill a running command, so effect does
    // not apply until command is restarted.
    if (cmd->ExecStr() != cmd_msg.cmd.exec_str) {
      ROS_DEBUG("[%s] exec str -> [%s]\n", mi->cmd_id.c_str(),
          cmd_msg.cmd.exec_str.c_str());
      pm_->SetCommandExecStr(cmd, cmd_msg.cmd.exec_str);

      action_taken = 1;
    }

    // has auto-respawn changed?
    if (cmd_msg.cmd.auto_respawn != mi->auto_respawn) {
      ROS_DEBUG("[%s] auto-respawn -> %d\n", mi->cmd_id.c_str(),
          cmd_msg.cmd.auto_respawn);
      mi->auto_respawn = cmd_msg.cmd.auto_respawn;
    }

    // change the group of a command?
    if (cmd_msg.cmd.group != mi->group) {
      ROS_DEBUG("[%s] group -> [%s]\n", mi->cmd_id.c_str(),
          cmd_msg.cmd.group.c_str());
      mi->group = cmd_msg.cmd.group;
      action_taken = 1;
    }

    // change the stop signal of a command?
    if(mi->stop_signal != cmd_msg.cmd.stop_signal) {
      ROS_DEBUG("[%s] stop signal -> [%d]\n", mi->cmd_id.c_str(),
          cmd_msg.cmd.stop_signal);
      mi->stop_signal = cmd_msg.cmd.stop_signal;
    }

    // change the stop time allowed of a command?
    if(mi->stop_time_allowed != cmd_msg.cmd.stop_time_allowed) {
      ROS_DEBUG("[%s] stop time allowed -> [%f]\n", mi->cmd_id.c_str(),
          cmd_msg.cmd.stop_time_allowed);
      mi->stop_time_allowed = cmd_msg.cmd.stop_time_allowed;
    }

    mi->should_be_running = !cmd_msg.force_quit;

    if (PROCMAN_CMD_STOPPED == cmd_status &&
        (mi->actual_runid != cmd_msg.desired_runid) &&
         mi->should_be_running) {
      StartCommand(mi, cmd_msg.desired_runid);
      action_taken = 1;
    } else if (PROCMAN_CMD_RUNNING == cmd_status &&
        ((!mi->should_be_running) ||
         (cmd_msg.desired_runid != mi->actual_runid &&
          cmd_msg.desired_runid != 0))) {
      StopCommand(mi);
      action_taken = 1;
    } else if (cmd_msg.desired_runid != 0) {
      mi->actual_runid = cmd_msg.desired_runid;
    }
  }

  // if there are any commands being managed that did not appear in the
  // orders, then stop and remove those commands
  std::vector<DeputyCommand*> toremove;
  for (auto& item : commands_) {
    DeputyCommand* mi = item.second;
    ProcmanCommandPtr cmd = item.first;
    const ProcmanCmdDesired *cmd_msg = OrdersFindCmd (orders, mi->cmd_id);

    if (! cmd_msg) {
      // push the orphaned command into a list first.  remove later, to
      // avoid corrupting the linked list (since this is a borrowed data
      // structure)
      toremove.push_back(mi);
    }
  }

  // cull orphaned commands
  for (DeputyCommand* mi : toremove) {
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

void ProcmanDeputy::DiscoveryReceived(const procman_ros::ProcmanDiscoveryConstPtr& msg) {
  const int64_t now = timestamp_now();
  if(now < deputy_start_time_ + DISCOVERY_TIME_MS * 1000) {
    // received a discovery message while still in discovery mode.  Check to
    // see if it's from a conflicting deputy.
    if(msg->transmitter_id == deputy_id_ && msg->nonce != deputy_pid_) {
      ROS_DEBUG("ERROR.  Detected another deputy [%s].  Aborting to avoid conflicts.\n",
          msg->transmitter_id.c_str());
      exit(1);
    }
  } else {
    // received a discovery message while not in discovery mode.  Respond by
    // transmitting deputy info.
    TransmitProcessInfo();
  }
}

void ProcmanDeputy::InfoReceived(const ProcmanDeputyInfoConstPtr& msg) {
  int64_t now = timestamp_now();
  if(now < deputy_start_time_ + DISCOVERY_TIME_MS * 1000) {
    // A different deputy has reported while we're still in discovery mode.
    // Check to see if the deputy names are in conflict.
    if(msg->deputy_id == deputy_id_) {
      ROS_DEBUG("ERROR.  Detected another deputy [%s].  Aborting to avoid conflicts.\n",
          msg->deputy_id.c_str());
      exit(2);
    }
  } else {
    ROS_DEBUG("WARNING:  Still processing info messages while not in discovery mode??\n");
  }
}

void ProcmanDeputy::OnDiscoveryTimer() {
  const int64_t now = timestamp_now();
  if(now < deputy_start_time_ + DISCOVERY_TIME_MS * 1000) {
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
    discovery_timer_->Stop();

    info_sub_.shutdown();
    ROS_INFO("Subscribing to pm orders");


    // Start the timer to periodically transmit status information
    one_second_timer_->Start();
    OnOneSecondTimer();
  }
}

static void usage() {
    fprintf (stderr, "usage: procman-deputy [options]\n"
            "\n"
            "  -h, --help        shows this help text and exits\n"
            "  -v, --verbose     verbose output\n"
            "  -i, --id NAME   use deputy id NAME instead of hostname\n"
            "  -l, --log PATH    dump messages to PATH instead of stdout\n"
            "  -u, --lcmurl URL  use specified LCM URL for procman messages\n"
            "\n"
            "DEPUTY ID\n"
            "  The deputy id must be unique from other deputies.  On startup,\n"
            "  if another deputy with the same id is detected, the newly started\n"
            "  deputy will self-terminate.\n"
            "\n"
            "EXIT STATUS\n"
            "  0   Clean exit on SIGINT, SIGTERM\n"
            "  1   OS or other networking error\n"
            "  2   Conflicting deputy detected on the network\n"
          );
}

}  // namespace procman

using namespace procman;

int main (int argc, char **argv) {
  ros::init(argc, argv, "procman_ros_deputy");
  const char *optstring = "hvfl:i:u:";
  int c;
  struct option long_opts[] = {
    { "help", no_argument, 0, 'h' },
    { "verbose", no_argument, 0, 'v' },
    { "log", required_argument, 0, 'l' },
    { "lcmurl", required_argument, 0, 'u' },
    { "id", required_argument, 0, 'i' },
    { 0, 0, 0, 0 }
  };

  DeputyOptions dep_options = DeputyOptions::Defaults();
  char *logfilename = NULL;
  std::string deputy_id_override;

  while ((c = getopt_long (argc, argv, optstring, long_opts, 0)) >= 0) {
    switch (c) {
      case 'v':
        dep_options.verbose = true;
        break;
      case 'l':
        free(logfilename);
        logfilename = strdup (optarg);
        break;
      case 'u':
        dep_options.lcm_url = optarg;
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
  char* argv0 = strdup(argv[0]);
  std::string newpath = std::string(dirname(argv0)) + ":" + getenv("PATH");
  printf("setting PATH to %s\n", newpath.c_str());
  setenv("PATH", newpath.c_str(), 1);
  free(argv0);

  // redirect stdout and stderr to a log file if the -l command line flag
  // was specified.
  if (logfilename) {
    int fd = open (logfilename, O_WRONLY | O_APPEND | O_CREAT, 0644);
    if (fd < 0) {
      perror ("open");
      fprintf (stderr, "couldn't open logfile %s\n", logfilename);
      return 1;
    }
    close(1);
    close(2);
    if (dup2(fd, 1) < 0) { return 1; }
    if (dup2(fd, 2) < 0) { return 1; }
    close (fd);
    setlinebuf (stdout);
    setlinebuf (stderr);
  }

  // set deputy hostname to the system hostname
  if (!deputy_id_override.empty()) {
    dep_options.deputy_id = deputy_id_override;
  }

  ProcmanDeputy pmd(dep_options);

  pmd.Run();

  return 0;
}
