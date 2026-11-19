#pragma once

#include"../config.h"
#include"../physics/Rocket.h"
#include"../physics/Balloon.h"


// 13 - dimensional space 
// 2 => target position (x,y)
// 2 => rocket position (x,y)
// 1 => square distance between rocket and target
// 2 => rocket velocity (vx, vy)
// 2 => target velocity (vx, vy)
// 1 => rocket orientation
// 1 => rocket angular velocity
// 1 => rocket thrust 
// 1 => rocket gimbal angle 

struct State {

        double targetX, targetY;
        double rocketX, rocketY;
        double squareDistance;
        double rocketVx, rocketVy;
        double targetVx, targetVy;
        double orientation;
        double omega;
        double thrust, gimbalAngle;
};



// This action returns the angle between the rocket and the balloon
struct Action{

    Action(double thrust, double gimbalAngle) : _desiredThrust(thrust), _desiredAngle{gimbalAngle} {}
    Action() : _desiredThrust(0.0), _desiredAngle(0.0) {}

    double _desiredThrust;      // Desired rocket thrust
    double _desiredAngle;       // Desired rocket gimbal angle
};



class Controller {  

    public:
        Controller() = default;
        virtual ~Controller() = default;
        virtual Action computeAction(const State& data) const = 0;
};
