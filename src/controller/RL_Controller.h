#pragma once

#include"Controller.h"
#include <onnxruntime_cxx_api.h>



// ##########################################
// #### SAC Reinforcement Learning agent ####
// ##########################################

// class RL_SAC_Controller : public Controller {

//     public:

//         RL_SAC_Controller();
//         std::array<float, 12> getRL_State(const State& state) const;
//         Action computeAction(const State& state) const override;

//     private:

//         Ort::Env env_;                          // Initialise runtime
//         Ort::SessionOptions sessionOptions_;    // Configure inference
//         mutable Ort::Session session_;                  // Loads the neural network and executes it 
//         // The keyword mutable is used to let this instance change in a const marked method without causing and error


// };



// ##########################################
// #### DQN Reinforcement Learning agent ####
// ##########################################

class RL_DQN_Controller : public Controller {

    public:
        RL_DQN_Controller();
        std::array<float, 15> getRL_State(const State& state) const;
        Action computeAction(const State& state) const override;

    private:

        Ort::Env env_;                          // Initialise runtime
        Ort::SessionOptions sessionOptions_;    // Configure inference
        mutable Ort::Session session_;                  // Loads the neural network and executes it 
        // The keyword mutable is used to let this instance change in a const marked method without causing and error

        // these values are defined into config.h
        double _delta_thrust;
        double _delta_gimbal_angle;



        const int8_t _ACTION_TABLE[9][2] = {{1, 1},  {1, 0},  {1, -1}, 
                                            {0, 1},  {0, 0},  {0, -1},
                                            {-1, 1}, {-1, 0}, {-1, -1}};


};

