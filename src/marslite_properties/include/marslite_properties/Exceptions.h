#ifndef EXCEPTIONS_H_
#define EXCEPTIONS_H_

#include <string>
#include <exception>

namespace marslite {

namespace math {

/**
 * @brief Exception for vectors with size less than 2.
*/
class DataNumberLessThan2Exception : public std::exception {
public:
    explicit DataNumberLessThan2Exception(const long& dataNum = -1) : dataNum_(dataNum) {}

    const char* what() const noexcept override {
        const std::string msg = "[Error] number of data (" + std::to_string(dataNum_) + " was given) less than 2";
        return msg.c_str();
    }

private:
    long dataNum_;
};

/**
 * @brief Exception for mismatch size between two vectors.
*/
class MismatchSizeException : public std::exception {
public:
    explicit MismatchSizeException(const long& aSize = -1, const long& bSize = -1) : aSize_(aSize), bSize_(bSize) {}

    const char* what() const noexcept override {
        const std::string msg = "[Error] mismatch size of two vectors (" + std::to_string(aSize_)
            + " and " + std::to_string(bSize_) + " were given)";
        return msg.c_str();
    }

private:
    long aSize_, bSize_;
};

} // namespace math

} // namespace marslite



#endif  // #ifndef EXCEPTIONS_H_