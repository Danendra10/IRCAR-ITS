#ifndef __MATH_HH_
#define __MATH_HH_

#include "opencv2/core/core.hpp"
#include <iostream>

using namespace std;
using namespace cv;

unsigned int DivideBy2(unsigned int in);
unsigned int DivideBy3(unsigned int in);
unsigned int DivideBy4(unsigned int in);
float pixel_to_real(float pix);

class PolynomialRegression
{
public:
    PolynomialRegression(int degree);
    void fit(const vector<double> &x, const vector<double> &y);
    double predict(double x);
    void print();
    vector<double> getW();

private:
    int degree;
    vector<double> w;
};

#endif