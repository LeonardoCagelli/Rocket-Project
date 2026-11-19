#pragma once



namespace Config
{

    // Window / world
    constexpr double WINDOW_WIDTH = 1000.0;
    constexpr double WINDOW_HEIGHT = 800.0;
    constexpr double TIMER_INTERVAL = 16; // in milliseconds 

    // Rocket
    constexpr double ROCKET_WIDTH = 80.0;
    constexpr double ROCKET_HEIGHT = 20.0;
    constexpr double ROCKET_INITIAL_X = 400.0;
    constexpr double ROCKET_INITIAL_Y = 740.0;
    constexpr double ROCKET_INITIAL_VELOCITY_X = 0.0; //80.0;
    constexpr double ROCKET_INITIAL_VELOCITY_Y = 0.0; //100.0;
    constexpr double ROCKET_INITIAL_ORIENTATION = -3.141592653/2.0;
    constexpr double ROCKET_MASS = 100.0;
    constexpr double ROCKET_INERTIA = 100.0; 
    constexpr double MAX_GIMBAL_ANGLE = 0.0005;
    constexpr double MAX_THRUST = 5000.0;

    // Balloon
    constexpr double BALLOON_RADIUS = 15.0;
    constexpr double BALLOON_INITIAL_X = 100.0;
    constexpr double BALLOON_INITIAL_Y = 100.0;
    constexpr double BALLOON_INITIAL_VELOCITY_X = 0.0;//80.0;
    constexpr double BALLOON_INITIAL_VELOCITY_Y = 0.0;//80.0;

    // Simulation
    constexpr double SIMULATION_FREQUENCY = 60.0;

    // Physics
    constexpr double GRAVITY = 9.81;

    namespace SimpleController {

        // desired acceleration
        constexpr double TARGET_NET_ACCELERATION = 50.0; // m/s^2

    }

    namespace RL_DQN_Controller {

        constexpr double INITIAL_THRUST = 0.0; // ROCKET_MASS * GRAVITY;
        constexpr double INITIAL_GIMBAL_ANGLE = 0.0;
        constexpr double DELTA_THRUST_PERCENTAGE = 0.01;
        constexpr double DELTA_GIMBAL_PERCENTAGE = 0.05;
        constexpr double DELTA_THRUST = MAX_THRUST * DELTA_THRUST_PERCENTAGE; // 10;
        constexpr double DELTA_GIMBAL_ANGLE = MAX_GIMBAL_ANGLE * DELTA_GIMBAL_PERCENTAGE; // 0.0005;

    }

}


