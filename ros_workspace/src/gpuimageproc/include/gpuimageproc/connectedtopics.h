#include <stdint.h>

namespace gpuimageproc {
struct ConnectedTopics {
    union {
        uint32_t _val;
        struct {
            uint32_t DebayerMonoLeft : 1;
            uint32_t DebayerMonoRight : 1;
            uint32_t DebayerColorLeft : 1;
            uint32_t DebayerColorRight : 1;
            uint32_t RectifyMonoLeft : 1;
            uint32_t RectifyMonoRight : 1;
            uint32_t RectifyColorLeft : 1;
            uint32_t RectifyColorRight : 1;
            uint32_t Disparity : 1;
            uint32_t DisparityVis : 1;
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
