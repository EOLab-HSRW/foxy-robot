#ifndef __QUADRATURE_ENCODER_HPP__
#define __QUADRATURE_ENCODER_HPP__

#include <linux/gpio.h>

#include <atomic>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <poll.h>
#include <string>
#include <thread>

#include <errno.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

class QuadratureEncoder
{
public:
  QuadratureEncoder() = default;
  ~QuadratureEncoder() { stop(); close(); }

  const std::string & last_error() const { return last_error_; }
  int64_t count() const { return count_.load(std::memory_order_relaxed); }

  // chip_dev: "/dev/gpiochip0"
  // line_a/line_b: line offsets
  bool open(const std::string & chip_dev,
            unsigned int line_a,
            unsigned int line_b,
            bool invert = false,
            uint32_t debounce_us = 0)
  {
    last_error_.clear();
    invert_ = invert;
    debounce_us_ = debounce_us;
    line_a_ = line_a;
    line_b_ = line_b;

    if (line_a_ == line_b_) return fail_msg("line_a and line_b must be different");

    chip_fd_ = ::open(chip_dev.c_str(), O_RDONLY | O_CLOEXEC);
    if (chip_fd_ < 0) return fail_sys(("open(" + chip_dev + ")").c_str());

    // Validate line offsets
    gpiochip_info cinfo{};
    if (::ioctl(chip_fd_, GPIO_GET_CHIPINFO_IOCTL, &cinfo) < 0) {
      close();
      return fail_sys("GPIO_GET_CHIPINFO_IOCTL");
    }
    if (line_a_ >= cinfo.lines || line_b_ >= cinfo.lines) {
      std::string msg = "line offset out of range; chip has " + std::to_string(cinfo.lines) +
                        " lines, requested A=" + std::to_string(line_a_) +
                        " B=" + std::to_string(line_b_);
      close();
      return fail_msg(msg);
    }

    // 1) Read initial levels using a TEMP handle, then release it.
    int a0 = 0, b0 = 0;
    if (!read_initial_levels(a0, b0)) {
      close();
      return false; // last_error_ already set
    }
    a_level_ = a0;
    b_level_ = b0;
    prev_state_ = pack_state(a_level_, b_level_);

    // 2) Request edge events (ONLY) for each line.
    if (!request_event_fd(line_a_, "diffbot_enc_a", event_a_fd_)) { close(); return false; }
    if (!request_event_fd(line_b_, "diffbot_enc_b", event_b_fd_)) { close(); return false; }

    count_.store(0, std::memory_order_relaxed);
    last_ts_ns_.store(0, std::memory_order_relaxed);
    return true;
  }

  void start()
  {
    if (running_.exchange(true)) return;
    thread_ = std::thread([this]() { run(); });
  }

  void stop()
  {
    if (!running_.exchange(false)) return;
    if (thread_.joinable()) thread_.join();
  }

  void close()
  {
    if (event_a_fd_ >= 0) { ::close(event_a_fd_); event_a_fd_ = -1; }
    if (event_b_fd_ >= 0) { ::close(event_b_fd_); event_b_fd_ = -1; }
    if (chip_fd_ >= 0)    { ::close(chip_fd_);    chip_fd_ = -1; }
  }

  // Optional helper: ask kernel who "owns" the line (if it's already in use)
  bool get_line_info(unsigned int line, gpioline_info & out) const
  {
    if (chip_fd_ < 0) return false;
    std::memset(&out, 0, sizeof(out));
    out.line_offset = line;
    return (::ioctl(chip_fd_, GPIO_GET_LINEINFO_IOCTL, &out) == 0);
  }

private:
  struct Pending {
    bool valid{false};
    gpioevent_data ev{};
    bool is_a{false};
  };

  bool fail_sys(const char * what)
  {
    last_error_ = std::string(what) + ": errno=" + std::to_string(errno) +
                  " (" + std::strerror(errno) + ")";
    return false;
  }
  bool fail_msg(const std::string & msg) { last_error_ = msg; return false; }

  static uint8_t pack_state(int a, int b)
  {
    return static_cast<uint8_t>(((a & 1) << 1) | (b & 1)); // A=bit1, B=bit0
  }

  static int8_t quad_delta(uint8_t prev, uint8_t curr)
  {
    static constexpr int8_t table[16] = {
      0, +1, -1,  0,
     -1,  0,  0, +1,
     +1,  0,  0, -1,
      0, -1, +1,  0
    };
    return table[(prev << 2) | curr];
  }

  bool read_initial_levels(int & a, int & b)
  {
    gpiohandle_request hreq{};
    hreq.lines = 2;
    hreq.lineoffsets[0] = line_a_;
    hreq.lineoffsets[1] = line_b_;
    hreq.flags = GPIOHANDLE_REQUEST_INPUT;
    std::snprintf(hreq.consumer_label, sizeof(hreq.consumer_label), "diffbot_enc_init");

    if (::ioctl(chip_fd_, GPIO_GET_LINEHANDLE_IOCTL, &hreq) < 0) {
      // If this is EBUSY, it means *another* consumer already owns the line(s)
      return fail_sys("GPIO_GET_LINEHANDLE_IOCTL (initial read)");
    }

    gpiohandle_data data{};
    const int rc = ::ioctl(hreq.fd, GPIOHANDLE_GET_LINE_VALUES_IOCTL, &data);
    ::close(hreq.fd);

    if (rc < 0) return fail_sys("GPIOHANDLE_GET_LINE_VALUES_IOCTL (initial read)");

    a = data.values[0] ? 1 : 0;
    b = data.values[1] ? 1 : 0;
    return true;
  }

  bool request_event_fd(unsigned int line, const char * label, int & out_fd)
  {
    gpioevent_request ereq{};
    ereq.lineoffset = line;
    ereq.handleflags = GPIOHANDLE_REQUEST_INPUT;
    ereq.eventflags = GPIOEVENT_REQUEST_BOTH_EDGES;
    std::snprintf(ereq.consumer_label, sizeof(ereq.consumer_label), "%s", label);

    if (::ioctl(chip_fd_, GPIO_GET_LINEEVENT_IOCTL, &ereq) < 0) {
      return fail_sys(("GPIO_GET_LINEEVENT_IOCTL (line " + std::to_string(line) + ")").c_str());
    }
    out_fd = ereq.fd;

    const int flags = ::fcntl(out_fd, F_GETFL, 0);
    if (flags >= 0) ::fcntl(out_fd, F_SETFL, flags | O_NONBLOCK);
    return true;
  }

  bool debounce_ok(uint64_t ts_ns)
  {
    if (debounce_us_ == 0) return true;
    const uint64_t last = last_ts_ns_.load(std::memory_order_relaxed);
    const uint64_t min_dt = static_cast<uint64_t>(debounce_us_) * 1000ULL;
    if (last != 0 && ts_ns - last < min_dt) return false;
    last_ts_ns_.store(ts_ns, std::memory_order_relaxed);
    return true;
  }

  bool try_read_event(int fd, Pending & p, bool is_a)
  {
    if (p.valid) return true;
    gpioevent_data ev{};
    const ssize_t n = ::read(fd, &ev, sizeof(ev));
    if (n == static_cast<ssize_t>(sizeof(ev))) {
      p.valid = true;
      p.ev = ev;
      p.is_a = is_a;
      return true;
    }
    return false;
  }

  void process(const Pending & p)
  {
    // Update the software level for the line that changed
    int new_level = 0;
    if (p.ev.id == GPIOEVENT_EVENT_RISING_EDGE) new_level = 1;
    else if (p.ev.id == GPIOEVENT_EVENT_FALLING_EDGE) new_level = 0;
    else return;

    if (!debounce_ok(p.ev.timestamp)) return;

    if (p.is_a) a_level_ = new_level;
    else        b_level_ = new_level;

    const uint8_t curr = pack_state(a_level_, b_level_);
    const int8_t d = quad_delta(prev_state_, curr);
    prev_state_ = curr;

    if (d != 0) count_.fetch_add(invert_ ? -d : d, std::memory_order_relaxed);
  }

  void run()
  {
    pollfd fds[2]{};
    fds[0].fd = event_a_fd_; fds[0].events = POLLIN;
    fds[1].fd = event_b_fd_; fds[1].events = POLLIN;

    Pending pa, pb;

    while (running_.load())
    {
      const int ret = ::poll(fds, 2, 50);
      if (ret <= 0) continue;

      // Merge events by timestamp so A/B ordering is correct if both have pending events.
      while (true)
      {
        (void)try_read_event(event_a_fd_, pa, true);
        (void)try_read_event(event_b_fd_, pb, false);

        if (!pa.valid && !pb.valid) break;

        if (pa.valid && (!pb.valid || pa.ev.timestamp <= pb.ev.timestamp)) {
          process(pa);
          pa.valid = false;
        } else {
          process(pb);
          pb.valid = false;
        }
      }
    }
  }

  int chip_fd_{-1};
  int event_a_fd_{-1};
  int event_b_fd_{-1};

  unsigned int line_a_{0}, line_b_{0};

  std::atomic<bool> running_{false};
  std::thread thread_;

  std::atomic<int64_t> count_{0};
  std::atomic<uint64_t> last_ts_ns_{0};

  int a_level_{0};
  int b_level_{0};
  uint8_t prev_state_{0};

  bool invert_{false};
  uint32_t debounce_us_{0};

  std::string last_error_;
};



#endif
