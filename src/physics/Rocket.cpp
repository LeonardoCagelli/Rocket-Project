#include"Rocket.h"
#include"../config.h"


Rocket::Rocket(const QPointF& position) : 
        MovingObject(position, QPointF(Config::ROCKET_INITIAL_VELOCITY_X, Config::ROCKET_INITIAL_VELOCITY_Y)),
        m_width(Config::ROCKET_WIDTH),
        m_height(Config::ROCKET_HEIGHT),
        m_mass(Config::ROCKET_MASS),
        m_inertia(Config::ROCKET_INERTIA),
        m_orientation(Config::ROCKET_INITIAL_ORIENTATION),
        // m_orientation(std::atan2(Config::ROCKET_INITIAL_VELOCITY_Y, Config::ROCKET_INITIAL_VELOCITY_X)),
        m_omega(0.0),
        m_thrust(0.0),
        m_gimbalAngle(0.0),
        m_maxGimbalAngle(Config::MAX_GIMBAL_ANGLE),
        m_maxThrust(Config::MAX_THRUST){}


uint32_t Rocket::width() const {
    return m_width;
}

uint32_t Rocket::height() const {
    return m_height;
}

double Rocket::mass() const {
    return m_mass;
}

double Rocket::inertia() const {
    return m_inertia;
}

double Rocket::orientation() const {
    return m_orientation;
}

double Rocket::omega() const {
    return m_omega;
}

double Rocket::thrust() const {
    return m_thrust;
}

double Rocket::gimbalAngle() const {
    return m_gimbalAngle;
}

double Rocket::maxGimbalAngle() const {
    return m_maxGimbalAngle;
}

double Rocket::maxThrust() const {
    return m_maxThrust;
}

void Rocket::setThrust(double thrust) {
    m_thrust = std::clamp(thrust, 0.0, m_maxThrust);
}

void Rocket::setGimbalAngle(double angle) {
    m_gimbalAngle = std::clamp(angle, -m_maxGimbalAngle, m_maxGimbalAngle);
}

void Rocket::setOrientation(double orientation) {
    m_orientation = orientation;
}
void Rocket::setOmega(double omega) {
    m_omega = omega;
}

void Rocket::reset() {
    m_position = QPointF(Config::ROCKET_INITIAL_X, Config::ROCKET_INITIAL_Y);       
    m_velocity = QPointF(Config::ROCKET_INITIAL_VELOCITY_X, Config::ROCKET_INITIAL_VELOCITY_Y);   

    m_orientation = Config::ROCKET_INITIAL_ORIENTATION;    
    m_omega = 0.0;                  
    m_thrust = 0.0;                 
    m_gimbalAngle = 0.0;
    m_thrust = Config::RL_DQN_Controller::INITIAL_THRUST;
    m_gimbalAngle = Config::RL_DQN_Controller::INITIAL_GIMBAL_ANGLE;
}

// New step function 
void Rocket::integrate(double dt, const QPointF& linearAcceleration, double angularAcceleration) {
    // Integrate linear motion
    m_velocity += linearAcceleration * dt;
    m_position += m_velocity * dt;

    // Integrate angular motion 
    m_omega += angularAcceleration * dt;
    m_orientation += m_omega * dt;
}