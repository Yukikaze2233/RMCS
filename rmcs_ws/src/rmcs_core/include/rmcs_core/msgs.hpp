#pragma once

#include <cstdint>

namespace rmcs_core::msgs {

enum class Switch : uint8_t { UNKNOWN = 0, UP = 1, DOWN = 2, MIDDLE = 3 };

struct __attribute__((packed)) Mouse {
    bool left  : 1;
    bool right : 1;
};

struct __attribute__((packed)) Keyboard {
    bool w     : 1;
    bool s     : 1;
    bool a     : 1;
    bool d     : 1;
    bool shift : 1;
    bool ctrl  : 1;
    bool q     : 1;
    bool e     : 1;
    bool r     : 1;
    bool f     : 1;
    bool g     : 1;
    bool z     : 1;
    bool x     : 1;
    bool c     : 1;
    bool v     : 1;
    bool b     : 1;
};

enum class GameStage : uint8_t {
    NOT_START     = 0,
    PREPARATION   = 1,
    REFEREE_CHECK = 2,
    COUNTDOWN     = 3,
    STARTED       = 4,
    SETTLING      = 5
};

} // namespace rmcs_core::msgs