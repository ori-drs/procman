#ifndef PROCMAN_EVENT_LOOP_HPP__
#define PROCMAN_EVENT_LOOP_HPP__

#include <memory>
#include <set>
#include <vector>
#include <functional>

namespace procman {

class SocketMontior;

class SocketNotifier;


typedef std::shared_ptr<SocketNotifier> SocketNotifierPtr;

class SocketMonitor {
  public:
    enum EventType {
      kRead,
      kWrite,
      kError
    };

    SocketMonitor();

    ~SocketMonitor();

    SocketNotifierPtr AddSocket(int fd, EventType event_type,
        std::function<void()> callback);

    void SetPosixSignals(const std::vector<int>& signums,
        std::function<void(int signum)> callback);

    void Run();

    void Quit();

    bool isQuitting() const {
      return quit_;
    }

    void IterateOnce();

  private:
    friend class SocketNotifier;

    bool quit_;

    std::vector<SocketNotifier*> sockets_;

    std::vector<SocketNotifier*> sockets_ready_;

    SocketNotifierPtr posix_signal_notifier_;
};

}  // namespace procman

#endif  // PROCMAN_EVENT_LOOP_HPP__
