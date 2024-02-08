#ifndef ARITHMETICS_H_
#define ARITHMETICS_H_

#include <vector>
#include <stdint.h>
#include <iostream>

#include "marslite_properties/Exceptions.h"
using marslite::math::DataNumberLessThan2Exception;
using marslite::math::MismatchSizeException;

namespace marslite {

namespace math {

/**
 * @brief Generate a vector of evenly spaced points between `begin` and `end`.
 * @param begin the first element of vector
 * @param end the last element of vector
 * @param number the number of the elements
 * @return the vector (in `std::vector<float>` type)
*/
template <class dataT>
static std::vector<dataT> linspace(const dataT& begin, const dataT& end, const unsigned int& dataNum)
{
    if (dataNum < 2)
        throw DataNumberLessThan2Exception(dataNum);

    std::vector<dataT> ret(dataNum, 0);
    ret[0] = begin;

    const dataT space = (end - begin) / (dataNum - 1);
    for (unsigned int i = 1; i < dataNum; ++i)
        ret[i] = ret[i-1] + space;

    return ret;
}

/**
 * @brief Calculate the definite integral of func(x)
 * @param x the domain of the function
 * @param func the codomain of the function
 * @return the definite integral result
*/
template <class dataT>
static dataT integral(const std::vector<dataT>& x, const std::vector<dataT>& func)
{
    if (x.size() != func.size())
        throw MismatchSizeException(x.size(), func.size());
    if (x.size() < 2)
        throw DataNumberLessThan2Exception(x.size());
    
    dataT result = 0;
    const dataT space = x[1] - x[0];
    const unsigned int dataNum = x.size();
    for (unsigned int i = 1; i < dataNum; ++i)
        result += (func[i] - func[i-1]) / space;

    return result;
}

} // namespace math

} // namespace marslite

#endif  // #ifndef ARITHMETICS_H_