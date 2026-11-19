#include"RL_Controller.h"

#include <QCoreApplication>

#include <algorithm>
#include <array>
#include <cmath>
#include <string>
#include<iostream>



// ##########################################
// #### SAC Reinforcement Learning agent ####
// ##########################################


// RL_SAC_Controller::RL_SAC_Controller() : env_(ORT_LOGGING_LEVEL_WARNING, "RocketController"),
//       sessionOptions_(),
//       session_(
//           env_,
//           (QCoreApplication::applicationDirPath()
//            + "/dqn_rocket_actor.onnx").toStdString().c_str(),
//           sessionOptions_){}



// std::array<float, 12> RL_SAC_Controller::getRL_State(const State& state) const {

//     double velocity_normalisation{100.0};
//     double omega_normalisation{10.0};

//     // Build exactly the same 12 observations used in Python.
//     const double relativeX = state.targetX - state.rocketX;
//     const double relativeY = state.targetY - state.rocketY;

//     const double relativeVx = state.targetVx - state.rocketVx;
//     const double relativeVy = state.targetVy - state.rocketVy;

//     std::array<float, 12> observation = {
//         static_cast<float>(relativeX / Config::WINDOW_WIDTH),
//         static_cast<float>(relativeY / Config::WINDOW_HEIGHT),
//         static_cast<float>(state.squareDistance / (Config::WINDOW_WIDTH * Config::WINDOW_WIDTH + Config::WINDOW_HEIGHT * Config::WINDOW_HEIGHT)),
//         static_cast<float>(relativeVx / velocity_normalisation),
//         static_cast<float>(relativeVy / velocity_normalisation),
//         static_cast<float>(state.rocketVx / velocity_normalisation),
//         static_cast<float>(state.rocketVy / velocity_normalisation),
//         static_cast<float>(state.targetVx / velocity_normalisation),
//         static_cast<float>(state.targetVy / velocity_normalisation),
//         static_cast<float>(std::sin(state.orientation)),
//         static_cast<float>(std::cos(state.orientation)),
//         static_cast<float>(state.omega / omega_normalisation)
//     };

//     return observation;
// }


// Action RL_SAC_Controller::computeAction(const State& state) const {

//     std::array<float, 12> observation = getRL_State(state);

//     // Create the input tensor: one observation with 12 features.
//     const std::array<int64_t, 2> inputShape = {1, 12};

//     Ort::MemoryInfo memoryInfo = Ort::MemoryInfo::CreateCpu(
//         OrtArenaAllocator,
//         OrtMemTypeDefault
//     );

//     Ort::Value inputTensor = Ort::Value::CreateTensor<float>(
//         memoryInfo,
//         observation.data(),
//         observation.size(),
//         inputShape.data(),
//         inputShape.size()
//     );

//     // These names must match export_model.py.
//     const char* inputNames[] = {"observation"};
//     const char* outputNames[] = {"action"};

//     // Run the neural network.
//     auto outputs = session_.Run(
//         Ort::RunOptions{nullptr},
//         inputNames,
//         &inputTensor,
//         1,
//         outputNames,
//         1
//     );

//     const float* actionValues = outputs[0].GetTensorData<float>();

//     // The SAC actor returns two normalized actions in [-1, 1].
//     const double normalizedThrust =
//         std::clamp(static_cast<double>(actionValues[0]), -1.0, 1.0);

//     const double normalizedGimbal =
//         std::clamp(static_cast<double>(actionValues[1]), -1.0, 1.0);

//     // Convert normalized actions to the physical controls.
//     const double thrust = (normalizedThrust + 1.0) * 0.5 * Config::MAX_THRUST;  // the current model has been trained on 5000
//     const double gimbalAngle = normalizedGimbal * Config::MAX_GIMBAL_ANGLE;     // the current model has been trained on 0.2

//     return Action(thrust, gimbalAngle);
// }






// ##########################################
// #### DQN Reinforcement Learning agent ####
// ##########################################

RL_DQN_Controller::RL_DQN_Controller() : env_(ORT_LOGGING_LEVEL_WARNING, "RocketController"),
      sessionOptions_(),
      session_(
          env_,
          (QCoreApplication::applicationDirPath()
           + "/dqn_rocket_actor.onnx").toStdString().c_str(),
          sessionOptions_),
          _delta_thrust{Config::RL_DQN_Controller::DELTA_THRUST},
          _delta_gimbal_angle{Config::RL_DQN_Controller::DELTA_GIMBAL_ANGLE}{}





std::array<float, 15> RL_DQN_Controller::getRL_State(const State& state) const {

    double velocity_normalisation{100.0};
    double omega_normalisation{10.0};

    // Build exactly the same 12 observations used in Python.
    const double relativeX = state.targetX - state.rocketX;
    const double relativeY = state.targetY - state.rocketY;

    const double relativeVx = state.targetVx - state.rocketVx;
    const double relativeVy = state.targetVy - state.rocketVy;

    std::array<float, 15> observation = {
        static_cast<float>(relativeX / Config::WINDOW_WIDTH),
        static_cast<float>(relativeY / Config::WINDOW_HEIGHT),
        static_cast<float>(state.squareDistance / (Config::WINDOW_WIDTH * Config::WINDOW_WIDTH + Config::WINDOW_HEIGHT * Config::WINDOW_HEIGHT)),
        static_cast<float>(relativeVx / velocity_normalisation),
        static_cast<float>(relativeVy / velocity_normalisation),
        static_cast<float>(state.rocketVx / velocity_normalisation),
        static_cast<float>(state.rocketVy / velocity_normalisation),
        static_cast<float>(state.targetVx / velocity_normalisation),
        static_cast<float>(state.targetVy / velocity_normalisation),
        static_cast<float>(std::sin(state.orientation)),
        static_cast<float>(std::cos(state.orientation)),
        static_cast<float>(state.omega / omega_normalisation),
        static_cast<float>(state.thrust / Config::MAX_THRUST),
        static_cast<float>(std::sin(state.gimbalAngle / Config::MAX_GIMBAL_ANGLE)),
        static_cast<float>(std::cos(state.gimbalAngle / Config::MAX_GIMBAL_ANGLE)),
    };

    return observation;
}



Action RL_DQN_Controller::computeAction(const State& state) const {

    std::array<float, 15> observation = getRL_State(state);

    // Create the input tensor: one observation with 15 features.
    const std::array<int64_t, 2> inputShape = {1, 15};

    Ort::MemoryInfo memoryInfo = Ort::MemoryInfo::CreateCpu(
        OrtArenaAllocator,
        OrtMemTypeDefault
    );

    Ort::Value inputTensor = Ort::Value::CreateTensor<float>(
        memoryInfo,
        observation.data(),
        observation.size(),
        inputShape.data(),
        inputShape.size()
    );

    // These names must match export_model.py.
    const char* inputNames[] = {"observation"};
    const char* outputNames[] = {"action"};

    // Run the neural network.
    auto outputs = session_.Run(
        Ort::RunOptions{nullptr},
        inputNames,
        &inputTensor,
        1,
        outputNames,
        1
    );

    // Get the output tensor.
    Ort::Value& outputTensor = outputs.front();

    // Check that the output is a tensor.
    if (!outputTensor.IsTensor()) {
        throw std::runtime_error("DQN output is not a tensor.");
    }

    // Check the output data type.
    ONNXTensorElementDataType outputType =
        outputTensor.GetTensorTypeAndShapeInfo().GetElementType();

    if (outputType != ONNX_TENSOR_ELEMENT_DATA_TYPE_INT64) {
        throw std::runtime_error("Unexpected DQN output type.");
    }

    // Retrieve the selected action ID.
    const int64_t* actionData =
        outputTensor.GetTensorData<int64_t>();

    const int actionID = static_cast<int>(actionData[0]);
    // std::cout<<"\n\nID: "<<actionID<<"\n\n";

    // Check that the action is valid.
    if (actionID < 0 || actionID >= 9) {
        throw std::runtime_error("Invalid DQN action ID.");
    }

    const auto action = _ACTION_TABLE[actionID];        // from 0 to 8
    const int thrust_direction = action[0];          // -1, 0 or 1
    const int gimbal_angle_direction = action[1];    // -1, 0 or 1

    // std::cout<<"\nAction ====> ["<<thrust_direction<<" , "<<gimbal_angle_direction<<"]"<<std::endl;

    double thrust = state.thrust + (thrust_direction * _delta_thrust);
    double gimbal_angle = state.gimbalAngle + (gimbal_angle_direction * _delta_gimbal_angle);

    // std::cout<<"thrust - angle ====> ["<<thrust<<" , "<<gimbal_angle<<"]\n"<<std::endl;

    thrust = std::clamp(thrust, 0.0, Config::MAX_THRUST);
    gimbal_angle = std::clamp(gimbal_angle, - Config::MAX_GIMBAL_ANGLE, Config::MAX_GIMBAL_ANGLE);

    return Action(thrust, gimbal_angle);
}
