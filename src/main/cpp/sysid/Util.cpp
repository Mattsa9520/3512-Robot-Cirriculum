// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "sysid/Util.h"

#include <sstream>
#include <string>
#include <vector>

std::vector<std::string> sysid::Split(const std::string& s, char c) {
  std::vector<std::string> result;
  std::stringstream ss(s);
  std::string item;

  while (std::getline(ss, item, c)) {
    result.push_back(item);
  }

  return result;
}
