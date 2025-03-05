#include "port_handler.hpp"

PortHandler::PortHandler(const std::string& port_path, speed_t baud_rate)
    : m_port_path(port_path), m_baud_rate(baud_rate)
{
    m_fd = open(m_port_path.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (m_fd < 0) {
        std::cerr << "fail to open serial port " << m_port_path << "\n";
        m_is_ok = false;
        m_is_open = false;
    }
    else {
        m_is_open = true;
        fcntl(m_fd, F_SETFL, 0);
        if (!set_port_config(std::bind(&PortHandler::default_port_config, this, std::placeholders::_1))) {
            std::cerr << "fail to init port " << m_port_path << " 's config\n";
            m_is_open = false;
            close(m_fd);
        }
        else {
            m_is_ok = true;
        }
    }
}

PortHandler::~PortHandler() {
    if (m_is_open) {
        close(m_fd);
    }
}

void PortHandler::set_baud_rate(speed_t baud_rate) {
    m_baud_rate = baud_rate;
    cfsetispeed(&m_options, m_baud_rate);
    cfsetospeed(&m_options, m_baud_rate);
}

bool PortHandler::set_port_config(std::function<void(struct termios&)> option_fn) {
    if (tcgetattr(m_fd, &m_options) != 0) {
        std::cerr << "Failed to get serial port attributes\n";
        return false;
    }

    option_fn(m_options);
    set_baud_rate(m_baud_rate);

    if (tcsetattr(m_fd, TCSANOW, &m_options) != 0) {
        std::cerr << "Failed to set serial port attributes\n";
        return false;
    }

    return true;
}

std::function<void(struct termios&)> PortHandler::get_default_port_config_fn() {
    return std::bind(&PortHandler::default_port_config, this, std::placeholders::_1);
}

void PortHandler::default_port_config(struct termios& options) {
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag |= CREAD | CLOCAL;
    options.c_cflag &= ~CRTSCTS;
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_oflag &= ~OPOST;
    options.c_cc[VMIN] = 1;
    options.c_cc[VTIME] = 0;
} 