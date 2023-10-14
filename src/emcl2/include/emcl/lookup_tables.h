#ifndef EMCL2_LOOKUP_TABLES_H
#define EMCL2_LOOKUP_TABLES_H
#include <array>
#include <cmath>


namespace emcl2::lut {

extern std::array<double, 1<<16> sin_lut;
extern std::array<double, 1<<16> cos_lut;

namespace detail {
struct LutInitializer {
  LutInitializer();
};
extern LutInitializer lut_initializer;
}

}


#endif
