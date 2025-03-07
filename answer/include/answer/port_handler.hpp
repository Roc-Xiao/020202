#ifndef _PORT_HANDLER_HPP_
#define _PORT_HANDLER_HPP_

#include <fcntl.h>
#include <termios.h>
#include <string>
#include <iostream>
#include <unistd.h>
#include <functional>

class PortHandler {
public:
    PortHandler() = delete;
    PortHandler(const std::string& port_path, speed_t baud_rate);
    ~PortHandler();

    bool is_ok() const { return m_is_ok; }
    bool is_open() const { return m_is_open; }
    void set_baud_rate(speed_t baud_rate);
    speed_t baud_rate() const { return m_baud_rate; }
    bool set_port_config(std::function<void(struct termios&)> option_fn);
    std::function<void(struct termios&)> get_default_port_config_fn();

protected:
    void default_port_config(struct termios& options);

private:
    std::string m_port_path;
    speed_t m_baud_rate;
    int m_fd;
    bool m_is_ok;
    bool m_is_open;
    struct termios m_options;
};

#endif // _PORT_HANDLER_HPP_