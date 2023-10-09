#pragma once

#ifndef _STRUCTS_H_
#define _STRUCTS_H_

#include <vector>

#include "types.h"


template<typename T1, typename T2> struct pair {
    T1 f;
    T2 s;
};


struct coordinates {
    std::vector<pair<double, double>> cords;
    i64 pos;

    bool set_next_pos() {
        if (++pos >= cords.size()) {
            pos = 0;
            return false;
        }
        return true;
    }
};


#endif
