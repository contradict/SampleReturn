#include <stdint.h>

namespace gpuimageproc {
struct ConnectedTopics {
    union {
        uint32_t _val;
        struct {
            uint32_t DebayerMono : 1;
            uint32_t DebayerColor : 1;
            uint32_t RectifyMono : 1;
            uint32_t RectifyColor : 1;
            uint32_t Disparity : 1;
            uint32_t Pointcloud : 1;
        };
    };
    ConnectedTopics(): _val(0) {};
    void operator|=(ConnectedTopics &other)
    {
        _val |= other._val;
    };
    int level()
    {
        return _val==0?0:32-__builtin_clz(_val);
    };
};

}
