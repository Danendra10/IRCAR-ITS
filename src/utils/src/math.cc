#include "math/math.hh"

unsigned int DivideBy2(unsigned int in)
{
    return in >> 1;
}

unsigned int DivideBy3(unsigned int in)
{
    unsigned int constant = 0x55555556; // (2^32 - 1) / 3

    unsigned long long product = (unsigned long long)in * constant;

    return (unsigned int)(product >> 32);
}

unsigned int DivideBy4(unsigned int in)
{
    return in >> 2;
}