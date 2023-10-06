#include <vector>

#include "types.h"

#pragma once

#ifndef _STRUCTS_H_
#define _STRUCTS_H_


template<typename T1, typename T2> struct pair {
    T1 f;
    T2 s;
};


struct coordinates {
    std::vector<pair<double, double>> cords;
    u64 pos;

    bool set_next_pos() {
        if (++pos >= cords.size()) {
            pos = 0;
            return false;
        }
        return true;
    }
};


#endif
