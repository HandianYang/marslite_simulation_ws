#ifndef MOVE_BASE_H_
#define MOVE_BASE_H_

namespace marslite_navigation {

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
    
    Velocity() {}

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

} // namespace marslite_navigation



#endif // #define MOVE_BASE_H_