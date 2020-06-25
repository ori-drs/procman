#include "procman_ros/event_loop.hpp"

#include <ros/ros.h>
#include <assert.h>
#include <fcntl.h>
#include <poll.h>
#include <signal.h>
#include <sys/time.h>
#include <unistd.h>

#include <algorithm>
#include <stdexcept>

namespace procman {

static int g_signal_fds[2] = { -1, -1 };

static int64_t Now() {
    struct timeval tv;
    gettimeofday (&tv, NULL);
    return (int64_t) tv.tv_sec * 1000000 + tv.tv_usec;
}

class SocketNotifier {
  public:
    ~SocketNotifier();

  private:
    SocketNotifier(int fd, EventLoop::EventType event_type,
        std::function<void()> callback,
        EventLoop* loop);

    friend class EventLoop;

    int fd_;
    EventLoop::EventType event_type_;
    std::function<void()> callback_;
    std::weak_ptr<SocketNotifier> weak_;
    EventLoop* loop_;
};

SocketNotifier::SocketNotifier(int fd, EventLoop::EventType event_type,
        std::function<void()> callback,
        EventLoop* loop) :
  fd_(fd),
  event_type_(event_type),
  callback_(callback),
  loop_(loop) {
}

SocketNotifier::~SocketNotifier() {
  ROS_DEBUG("Destroying socket notifier %p for %d\n", this, fd_);

  auto iter = std::find(loop_->sockets_.begin(), loop_->sockets_.end(), this);
  if (iter != loop_->sockets_.end()) {
    ROS_DEBUG("found in sockets_\n");
    loop_->sockets_.erase(iter);
  }

  // If the SocketNotifier being destroyed is a socket queued up for callback,
  // then zero out its place in the queue, but don't remove it to avoid messing
  // with queue iteration.
  auto ready_iter = std::find(loop_->sockets_ready_.begin(),
      loop_->sockets_ready_.end(), this);
  if (iter != loop_->sockets_ready_.end()) {
    *ready_iter = nullptr;
  }
  fd_ = -1;
}

EventLoop::EventLoop() :
  quit_(false) {}

EventLoop::~EventLoop() {
  if (g_signal_fds[0] != -1) {
    close(g_signal_fds[0]);
    close(g_signal_fds[1]);
    g_signal_fds[0] = -1;
    g_signal_fds[1] = -1;
  }
}

SocketNotifierPtr EventLoop::AddSocket(int fd,
    EventType event_type, std::function<void()> callback) {
  if (event_type != kRead &&
      event_type != kWrite &&
      event_type != kError) {
    throw std::invalid_argument("Invalid socket event type");
  }
  SocketNotifier* notifier = new SocketNotifier(fd, event_type, callback, this);
  sockets_.push_back(notifier);
  return SocketNotifierPtr(notifier);
}

static void signal_handler (int signum) {
  int wstatus = write(g_signal_fds[1], &signum, sizeof(int));
  (void) wstatus;
}

void EventLoop::SetPosixSignals(const std::vector<int>& signums,
    std::function<void(int signum)> callback) {
  if (g_signal_fds[0] != -1) {
    throw std::runtime_error("EventLoop POSIX signals already set");
  }

  if (0 != pipe(g_signal_fds)) {
    throw std::runtime_error("Error initializing internal pipe for POSIX signals");
  }

  const int flags = fcntl(g_signal_fds[1], F_GETFL);
  fcntl(g_signal_fds[1], F_SETFL, flags | O_NONBLOCK);

  for (int signum : signums) {
    signal(signum, signal_handler);
  }

  posix_signal_notifier_ = AddSocket(g_signal_fds[0], kRead,
      [this, callback]() {
        int signum;
        const int unused = read(g_signal_fds[0], &signum, sizeof(int));
        (void) unused;
        callback(signum);
      });
}

void EventLoop::IterateOnce() {
  // Prepare pollfd structure
  const int num_sockets = sockets_.size();
  struct pollfd* pfds = new struct pollfd[num_sockets];
  for (int index = 0; index < num_sockets; ++index) {
    pfds[index].fd = sockets_[index]->fd_;
    switch (sockets_[index]->event_type_) {
      case kRead:
        pfds[index].events = POLLIN;
        break;
      case kWrite:
        pfds[index].events = POLLOUT;
        break;
      case kError:
        pfds[index].events = POLLERR;
        break;
      default:
        pfds[index].events = POLLIN;
        break;
    }
    pfds[index].revents = 0;
  }

  // poll sockets for the maximum wait time.
  const int num_sockets_ready = poll(pfds, num_sockets, 300);

  // Check which sockets are ready, and queue them up for invoking callbacks.
  if (num_sockets_ready) {
    for (int index = 0; index < num_sockets; ++index) {
      struct pollfd* pfd = &pfds[index];
      if (pfd->revents & pfd->events) {
        ROS_DEBUG("marking socket notifier %p (%d) for callback",
            sockets_[index], pfd->fd);
        sockets_ready_.push_back(sockets_[index]);
      }
    }
  }
  // Call callbacks for sockets that are ready
  for (int index = 0; index < sockets_ready_.size(); ++index) {
    SocketNotifier* notifier = sockets_ready_[index];
    if (!notifier) {
      continue;
    }
    notifier->callback_();
  }
  sockets_ready_.clear();

  delete[] pfds;

}

}  // namespace procman
