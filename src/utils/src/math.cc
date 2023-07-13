#include "math/math.hh"

vector<double> regresi;

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

float pixel_to_real(float pix)
{
    int max_orde = 6;
    double orde[] = { 1.6025170549750258e+000,
        -8.0261626712776948e-003,
        1.9085792942752757e-004,
        -5.1360872046995370e-007,
        6.6860526863516786e-010,
        -2.8823572300900999e-013 };

    // double orde[] = {-4.8848247487963425e+002,
    //                  1.0955158773901486e+001,
    //                  -9.5262780549231291e-002,
    //                  4.0493765879295917e-004,
    //                  -8.4429904418805596e-007,
    //                  6.9449433208931844e-010};
    double result = 0;
    for (int i = 0; i <= max_orde; i++) {
        result += (orde[i] * pow(pix, i));
    }

    return result;
}

PolynomialRegression::PolynomialRegression(int degree)
{
    this->degree = degree;
    w = vector<double>(degree + 1, 0);
}

void PolynomialRegression::fit(const vector<double>& x, const vector<double>& y)
{
    int n = x.size();
    Mat X = Mat::zeros(n, degree + 1, CV_64FC1);
    Mat Y = Mat::zeros(n, 1, CV_64FC1);
    Mat W = Mat::zeros(degree + 1, degree + 1, CV_64FC1);
    Mat W_inv = Mat::zeros(degree + 1, degree + 1, CV_64FC1);
    Mat W_inv_X = Mat::zeros(degree + 1, n, CV_64FC1);
    Mat W_inv_X_Y = Mat::zeros(degree + 1, 1, CV_64FC1);

    for (int i = 0; i < n; i++) {
        Y.at<double>(i, 0) = y[i];
        for (int j = 0; j < degree + 1; j++) {
            X.at<double>(i, j) = pow(x[i], j);
        }
    }

    W = X.t() * X;
    W_inv = W.inv();
    W_inv_X = W_inv * X.t();
    W_inv_X_Y = W_inv_X * Y;

    for (int i = 0; i < degree + 1; i++) {
        w[i] = W_inv_X_Y.at<double>(i, 0);
    }
}

double PolynomialRegression::predict(double x)
{
    double y = 0;
    for (int i = 0; i < degree + 1; i++) {
        y += w[i] * pow(x, i);
    }
    return y;
}

void PolynomialRegression::print()
{
    cout << "y = ";
    for (int i = degree; i >= 0; i--) {
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

int SizeOfLane(vector<Lane> lanes, int start_idx, int end_idx)
{
    int size = 0;
    for (int i = start_idx; i < end_idx; i++) {
        if (lanes[i].x != NULL && lanes[i].y != NULL)
            size++;
    }
    if (!size)
        return -1;
    return size;
}

void CalcTrapezoidalMotion(double distance, double max_velocity, double acceleration, double deceleration)
{
    printf("Distance: %.2lf\n", distance);
    std::vector<double> time;
    std::vector<double> velocity;
    std::vector<double> pos;

    double acceleration_time = max_velocity / acceleration;                                    // a_time = v_max / a
    double deceleration_time = max_velocity / deceleration;                                    // d_time = v_max / d
    double acceleration_distance = 0.5 * acceleration * acceleration_time * acceleration_time; // a_dist = 0.5 * a * a_time^2
    double deceleration_distance = 0.5 * deceleration * deceleration_time * deceleration_time; // d_dist = 0.5 * d * d_time^2
    double remaining_distance = distance - acceleration_distance - deceleration_distance;      // r_dist = dist - a_dist - d_dist || remaining distance is the distance covered at max velocity

    if (remaining_distance < 0)
    {
        acceleration_distance = (distance * acceleration) / (acceleration + deceleration); // this will calculate the distance covered during acceleration and deceleration
        deceleration_distance = acceleration_distance;
        acceleration_time = sqrt(2 * acceleration_distance / acceleration); // this will calculate the time taken for acceleration and deceleration
        deceleration_time = sqrt(2 * deceleration_distance / deceleration);
        remaining_distance = 0;
    }

    double total_time = acceleration_time + deceleration_time + (remaining_distance / max_velocity);

    double current_time = 0;
    while (current_time <= total_time)
    {
        double current_velocity;
        if (current_time < acceleration_time)
            current_velocity = acceleration * current_time; // this will calculate the velocity during acceleration before reaching max velocity
        else if (current_time < (total_time - deceleration_time))
            current_velocity = max_velocity; // this is the moment when the max velocity is reached before deceleration
        else
            current_velocity = max_velocity - deceleration * (current_time - (total_time - deceleration_time)); // this will calculate the velocity during deceleration after reaching max velocity

        double current_position;

        time.push_back(current_time);
        velocity.push_back(current_velocity);

        current_time += 0.1; // Step size for plotting
    }
}

void CalculateTargetToTurnLeft90Degree(CarPose current_car_pose, Pose3D *target_to_turn_left_90_degree)
{
    double x = current_car_pose.x;
    double y = current_car_pose.y;
    double theta = current_car_pose.th;

    /**
     * This is the formula to calculate the target to turn left 90 degree
     * x1 = x + 0.5 * cos(theta)
     * y1 = y + 0.5 * sin(theta)
     *
     * its the equation of
     */
    double x1 = x + 0.5 * cos(theta);
    double y1 = y + 0.5 * sin(theta);

    double x2 = x1 + 0.5 * cos(theta + M_PI / 2);
    double y2 = y1 + 0.5 * sin(theta + M_PI / 2);

    target_to_turn_left_90_degree->x = x2;
    target_to_turn_left_90_degree->y = y2;
    target_to_turn_left_90_degree->th = theta + M_PI / 2;
}