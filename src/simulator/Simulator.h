#pragma once

#include<cmath>
#include<chrono>

#include"../physics/Rocket.h"
#include"../physics/Balloon.h"
#include"../controller/SimpleController.h"
#include"../controller/RL_Controller.h"
#include"../config.h"

#include<random>

// This struct is used to compute the forces to apply to the rocket
struct RocketForces {

    RocketForces(QPointF f, double t) : force{f}, torque{t}{}

    QPointF force;
    double torque;
};

// Used in simulator step function to capture the step output
struct StepResult {
    State state;
    bool terminated;
};


class Simulator {

    public:
        Simulator();

        void initialise();
        void simulate(double dt, const Controller& controller);
        void respawnBalloon();
        // void manageRocketBoundariesCollisions();
        void manageBalloonBoundariesCollisions();
        bool checkRocketBalloonCollision() const;
        bool checkRocketBoundariesCollisions() const;
        void reset();

        size_t numberEvents() const;
        size_t numberSuccess() const;
        double successRatio() const;
        void addEvent(bool result);


        RocketForces computeRocketForces(const Action& action);
        State getState() const;
        StepResult step(const Action& action, double dt);
        
        const Rocket& rocket() const;
        const Balloon& balloon() const;

        void setRandomSeed(unsigned int seed);

    private:

        // Simulation boundaries
        static constexpr double WORLD_WIDTH = Config::WINDOW_WIDTH;        // we use constexpr instead of const so the variable is initialised at compile time and not runtime
        static constexpr double WORLD_HEIGHT = Config::WINDOW_HEIGHT;
        
        Rocket m_rocket;
        Balloon m_balloon;
        std::mt19937 m_random_generator;

        size_t m_number_events;
        size_t m_number_success;
};