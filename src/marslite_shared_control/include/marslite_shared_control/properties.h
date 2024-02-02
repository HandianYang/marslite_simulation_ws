#ifndef PROPERTIES_H_
#define PROPERTIES_H_

#include <cmath>
#include <vector>

namespace marslite {

/**
 * @brief Generate a vector of evenly spaced points between `begin` and `end`.
 * @param begin The first element of vector
 * @param end The last element of vector
 * @param number The number of the elements
 * @return The vector (in `std::vector<float>` type)
*/
static std::vector<float> linspace(const float& begin, const float& end, const int& number)
{
    const float space = (end - begin) / (number - 1);

    std::vector<float> ret(number);
    ret[0] = begin;
    for (int i = 1; i < number; ++i)
        ret[i] = ret[i-1] + space;

    return ret;
}

// size of the laser and the virtual zones
const static size_t LASER_SIZE = 1440;

// vector of the angle within [-pi, pi]
const static std::vector<float> THETA = linspace(-M_PI, M_PI, LASER_SIZE);

} // namespace marslite

#endif // #define PROPERTIES_H_