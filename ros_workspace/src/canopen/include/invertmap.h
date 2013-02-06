#ifndef __INVERT_MAP__
#define __INVERT_MAP__

#include <map>

template <typename keytype, typename valuetype>
static std::map< valuetype, keytype > InvertMap(const std::map< keytype, valuetype > m)
{
    std::map< valuetype, keytype > invmap;
    typename std::map< keytype, valuetype >::const_iterator mapit;
    for(mapit=m.begin(); mapit != m.end(); mapit++) {
        invmap[(*mapit).second] = (*mapit).first;
    }
    return invmap;
};

#endif // __INVERT_MAP__
