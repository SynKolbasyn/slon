#pragma once

#ifndef _STRUCTS_H_
#define _STRUCTS_H_


#include <vector>
#include <utility>

#include "types.h"


using std::pair;
using std::vector;


namespace sk {


struct Coordinates {
  vector<pair<f64, f64>> cords;
  i64 pos;


  bool set_next_pos() {
    if (++pos >= cords.size()) {
      pos = 0;
      return false;
    }
    return true;
  }


  u64 count() {
    return cords.size();
  }
};



} // namespace sk


#endif
