#ifndef PROPERTIES_H_
#define PROPERTIES_H_

#include <vector>
#include "marslite_properties/Arithmetics.h"

namespace marslite {

// size of the laser and the virtual zones
const static size_t LASER_SIZE = 1440;

// vector of the angle within [-pi, pi]
const static std::vector<float> THETA = marslite::math::linspace<float>(-M_PI, M_PI, LASER_SIZE);

namespace move_base {

/**
 * @struct marslite_navigation::move_base::Pose
 * @brief Pose of the robot's base
*/
struct Pose {
    float x;
    float y;
    float theta;
};

/**
 * @struct marslite_navigation::move_base::Velocity
 * @brief Velocity (linear/angular) of the robot
*/
struct Velocity {
    /* Member variables */
    float velocity;
    struct {
        float max;
        float min;
    } limit;
    
    /* Member functions */
    Velocity(const float& velocity = 0.0, const float& max = 1.0, const float& min = -1.0) {
        this->velocity = velocity;
        this->limit.max = max;
        this->limit.min = min;
    }

    /**
     * @brief Increase the velocity of the robot
     * @param amount The increase of the velocity (in `float` type)
    */
    Velocity& operator+=(const float& amount) {
        this->velocity += amount;
        if (this->velocity >= this->limit.max)  this->velocity = this->limit.max;
        return *this;
    }

    /**
     * @brief Increase the velocity of the robot
     * @param vel The increase of the velocity (in `Velocity` type)
    */
    Velocity& operator+=(const Velocity& vel) {
        this->velocity += vel.velocity;
        if (this->velocity >= this->limit.max)  this->velocity = this->limit.max;
        return *this;
    }

    /**
     * @brief Decrease the velocity of the robot
     * @param amount The decrease of the velocity (in `float` type)
    */
    Velocity& operator-=(const float& amount) {
        this->velocity -= amount;
        if (this->velocity <= this->limit.min)  this->velocity = this->limit.min;
        return *this;
    }

    /**
     * @brief Decrease the velocity of the robot
     * @param amount The decrease of the velocity (in `Velocity` type)
    */
    Velocity& operator-=(const Velocity& vel) {
        this->velocity -= vel.velocity;
        if (this->velocity <= this->limit.min)  this->velocity = this->limit.min;
        return *this;
    }

    /**
     * @brief Make the robot gradually decelerates
     * @param amount The decrease of the velocity (in `float` type)
    */
    void slowDown(const float& amount) {
        if (this->velocity > 1e-03 && this->velocity - amount > 1e-03)
            this->velocity -= amount;
        else if (this->velocity < -1e-03 && this->velocity + amount < -1e-03)
            this->velocity += amount;
        else
            this->velocity = 0.0;
    }
};

} // namespace move_base

} // namespace marslite

#endif // #define PROPERTIES_H_