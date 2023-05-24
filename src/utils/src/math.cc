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

PolynomialRegression::PolynomialRegression(int degree)
{
    this->degree = degree;
    w = vector<double>(degree + 1, 0);
}

void PolynomialRegression::fit(const vector<double> &x, const vector<double> &y)
{
    int n = x.size();
    Mat X = Mat::zeros(n, degree + 1, CV_64FC1);
    Mat Y = Mat::zeros(n, 1, CV_64FC1);
    Mat W = Mat::zeros(degree + 1, degree + 1, CV_64FC1);
    Mat W_inv = Mat::zeros(degree + 1, degree + 1, CV_64FC1);
    Mat W_inv_X = Mat::zeros(degree + 1, n, CV_64FC1);
    Mat W_inv_X_Y = Mat::zeros(degree + 1, 1, CV_64FC1);

    for (int i = 0; i < n; i++)
    {
        Y.at<double>(i, 0) = y[i];
        for (int j = 0; j < degree + 1; j++)
        {
            X.at<double>(i, j) = pow(x[i], j);
        }
    }

    W = X.t() * X;
    W_inv = W.inv();
    W_inv_X = W_inv * X.t();
    W_inv_X_Y = W_inv_X * Y;

    for (int i = 0; i < degree + 1; i++)
    {
        w[i] = W_inv_X_Y.at<double>(i, 0);
    }
}

double PolynomialRegression::predict(double x)
{
    double y = 0;
    for (int i = 0; i < degree + 1; i++)
    {
        y += w[i] * pow(x, i);
    }
    return y;
}

void PolynomialRegression::print()
{
    cout << "y = ";
    for (int i = degree; i >= 0; i--)
    {
        cout << w[i] << "x^" << i;
        if (i != 0)
            cout << " + ";
    }
    cout << endl;
}

vector<double> PolynomialRegression::getW()
{
    return w;
}