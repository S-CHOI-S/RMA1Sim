//----------------------------//
// This file is part of RaiSim//
// Copyright 2020, RaiSim Tech//
//----------------------------//


#ifndef RAISIM_STIFFWIRE_HPP
#define RAISIM_STIFFWIRE_HPP

#include "Wire.hpp"

namespace raisim {

class StiffWire : public Wire {
 public:
  StiffWire(Object* obj1, size_t localIdx1, Vec<3> pos1_b, Object* obj2, size_t localIdx2, Vec<3> pos2_b, double length);
 private:

};

}

#endif //RAISIM_STIFFWIRE_HPP
