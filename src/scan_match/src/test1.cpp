#include <iostream>

#include "../include/test1.h"

Mammal::Mammal() {};
Mammal::~Mammal() {};

void Mammal::speak()
{
  std::cout << "kalimera" << std::endl;
}
