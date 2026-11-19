#pragma once

#include"../config.h"
#include"../physics/Rocket.h"
#include"../physics/Balloon.h"
#include"Controller.h"



// This is a simple controller, in the next version this controller will be replaced by a reinfocement learning model 

class SimpleController : public Controller {

    public:
        SimpleController(){}

        Action computeAction(const State& state) const override;
};

