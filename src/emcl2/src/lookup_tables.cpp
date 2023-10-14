#include "emcl/lookup_tables.h"


namespace emcl2::lut {

std::array<double, 1<<16> sin_lut;
std::array<double, 1<<16> cos_lut;

namespace detail {

LutInitializer::LutInitializer() {
  for(int i=0;i<(1<<16);i++){
		cos_lut[i] = std::cos(M_PI*i/(1<<15));
		sin_lut[i] = std::sin(M_PI*i/(1<<15));
	}
}

LutInitializer lut_initializer{};

}

}