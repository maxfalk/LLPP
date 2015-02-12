#include <cstdio>
#include <iostream>
#include "ped_crowd.h"
#include <cuda_runtime.h>
using namespace Ped;
// For testing purposes
void __global__ dummyKernel() 
{
  printf("ON CUDA!\n" );
}


void whereToGoCUDA()
{
  std::cout << "If you read this, makefile system assumes CUDA is working" << std::endl;
}
