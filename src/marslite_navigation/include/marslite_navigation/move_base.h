/**
 * @file move_base.h
 * @author Handian Yang
 * @copyright Released under the terms of the GPLv3.0 or later
 * @date 2024
 * 
 * @brief The header file for all navigation properties related to marslite robots.
 * @note `move_base.h` is part of `marslite_simulation_ws`.
 * 
 * `marslite_simulation_ws` is free software: you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License as published
 *  by the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 * 
 * `marslite_simulation_ws` is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 *  Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License along
 *  with `marslite_simulation_ws`. If not, see <https://www.gnu.org/licenses/>.
*/

#ifndef MARSLITE_NAVIGATION_MOVEBASE_H_
#define MARSLITE_NAVIGATION_MOVEBASE_H_

#include <vector>
#include "marslite_properties/Arithmetic.h"

/**
 * @namespace marslite operation namespace
*/
namespace marslite {

// size of the laser and the virtual zones
const static size_t LASER_SIZE = 1440;

// vector of the angle within [-pi, pi]
const static std::vector<float> THETA = marslite::math::linspace<float>(-M_PI, M_PI, LASER_SIZE);


/**
 * @namespace MoveBase namespace for marslite robots. Relationship: `marslite::move_base`
*/
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

#endif // #define MARSLITE_NAVIGATION_MOVEBASE_H_