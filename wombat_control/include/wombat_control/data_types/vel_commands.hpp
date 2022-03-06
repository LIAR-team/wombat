// Copyright 2021 Azzollini Ilario, Gentilini Lorenzo, Soragna Alberto, Tazzari Roberto.
// All Rights Reserved.

#ifndef WOMBAT_CONTROL__DATA_TYPES__VEL_COMMANDS_HPP_
#define WOMBAT_CONTROL__DATA_TYPES__VEL_COMMANDS_HPP_

#include <iostream>

struct VelCommands
{
  double v;
  double omega;
};

// inline if you define a function in the .hpp
inline std::ostream & operator<<(std::ostream & outstr, const VelCommands & val)
{
  return outstr << "v: " << val.v << ", " << "omega: " << val.omega;
}

inline bool operator==(const VelCommands & comm1, const VelCommands & comm2)
{
  return (comm1.v == comm2.v) && (comm1.omega == comm2.omega);
}

#endif  // WOMBAT_CONTROL__DATA_TYPES__VEL_COMMANDS_HPP_
