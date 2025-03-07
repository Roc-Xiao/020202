#ifndef _DATA_PROTOCOL_HPP_
#define _DATA_PROTOCOL_HPP_

#include <cstdint>

namespace protocol {
    const uint64_t CMD_WRITE = 0x00;
    const uint64_t CMD_READ = 0x01;

    template<typename T>
    struct FrameHeader {
        size_t length;
        int cmd_id;
    };

    template<typename T>
    struct FrameChecker {
        bool check_header(const T& header);
        bool check_tail(const T& tail);
        size_t get_length(const T& header);
        int get_cmd_id(const T& header);
        void set_length(T& header, size_t length);
        void set_cmd_id(T& header, int cmd_id);
    };
}

#endif // _DATA_PROTOCOL_HPP_