#include"Simulator.h"
#include <cmath>
#include<iostream>

Simulator::Simulator() : 
        m_rocket(QPointF(
                Config::ROCKET_INITIAL_X,
                Config::ROCKET_INITIAL_Y)),    
        m_balloon(QPointF(
                Config::BALLOON_INITIAL_X,
                Config::BALLOON_INITIAL_Y),
            QPointF(
                Config::BALLOON_INITIAL_VELOCITY_X,
                Config::BALLOON_INITIAL_VELOCITY_Y),
            Config::BALLOON_RADIUS),
        m_number_events{}, m_number_success{}
        {

    initialise(); 
    setRandomSeed(32);
}



void Simulator::initialise(){
    
    m_rocket.setPosition(QPointF(Config::ROCKET_INITIAL_X, Config::ROCKET_INITIAL_Y));

    m_rocket.setVelocity(QPointF(Config::ROCKET_INITIAL_VELOCITY_X, Config::ROCKET_INITIAL_VELOCITY_Y));

    m_balloon.setPosition(QPointF(Config::BALLOON_INITIAL_X, Config::BALLOON_INITIAL_Y));
}



void Simulator::respawnBalloon()
{
    std::uniform_real_distribution<double> xDistribution(
        Config::BALLOON_RADIUS,
        Config::WINDOW_WIDTH - Config::BALLOON_RADIUS
    );

    std::uniform_real_distribution<double> yDistribution(
        Config::BALLOON_RADIUS,
        Config::WINDOW_HEIGHT/2 - Config::BALLOON_RADIUS
    );

    std::uniform_real_distribution<double> thetaDistribution(0.0, 2 * M_PI);

    double speed = std::hypot(Config::BALLOON_INITIAL_VELOCITY_X, Config::BALLOON_INITIAL_VELOCITY_Y);
    double new_theta = thetaDistribution(m_random_generator);
    double x_velocity = speed * std::cos(new_theta);
    double y_velocity = speed * std::sin(new_theta);

    double x = xDistribution(m_random_generator);
    double y = yDistribution(m_random_generator);

    m_balloon.setPosition(QPointF(x, y));
    m_balloon.setVelocity(QPointF(x_velocity, y_velocity));
}



void Simulator::manageBalloonBoundariesCollisions(){
    QPointF position = m_balloon.position();
    QPointF velocity = m_balloon.velocity();
    double radius = m_balloon.radius();

    if(position.x() < radius){
        position.setX(radius);
        velocity.setX(-velocity.x());
    }

    if(position.x() > WORLD_WIDTH - radius){
        position.setX(WORLD_WIDTH - radius);
        velocity.setX(-velocity.x());
    }

    if(position.y() < radius){
        position.setY(radius);
        velocity.setY(-velocity.y());
    }

    if(position.y() > WORLD_HEIGHT - radius){
        position.setY(WORLD_HEIGHT - radius);
        velocity.setY(-velocity.y());
    }

    // Update parameters
    m_balloon.setPosition(position);
    m_balloon.setVelocity(velocity);

    return;
}



bool Simulator::checkRocketBoundariesCollisions() const {

    QPointF position = m_rocket.position();

    // Parameters to manage the boundaries collisions 
    double half_width = m_rocket.width()/2.0;
    double half_height = m_rocket.height()/2.0;

    // theta is the angle between 
    double theta = m_rocket.orientation();

    double extentX = abs(half_width * std::cos(theta)) +
                        abs(half_height * std::sin(theta));

    double extentY = abs(half_height * std::cos(theta)) + 
                        abs(half_width * std::sin(theta));

    if(
        position.x() < extentX or 
        position.y() < extentY or 
        position.x() > WORLD_WIDTH - extentX or
        position.y() > WORLD_HEIGHT - extentY
    ) return true;

    return false;
}



// this simple function approximate the rocket as a circle with radius the half diagonal segment
bool Simulator::checkRocketBalloonCollision() const {

    double rocket_radius = m_rocket.width()/2;  // this is an approximation of this model 
    double balloon_radius = m_balloon.radius();

    double leverArm = m_rocket.width()/2;

    QPointF headPosition =
        m_rocket.position() +
        QPointF(leverArm * std::cos(m_rocket.orientation()),
                leverArm * std::sin(m_rocket.orientation()));

    QPointF v_distance = headPosition - m_balloon.position();

    double sigma{20};
    double distance = pow(pow(v_distance.x(), 2) + pow(v_distance.y(), 2), 0.5) + sigma;

    if(distance < rocket_radius + balloon_radius) return true;
    else return false;
}



RocketForces Simulator::computeRocketForces(const Action& action){

    // Here the controller generates the desired thrust magnitude and the gimbalAngle to generate the force vector, then 
    // we compute the torque according to the formula 

    double thrustMagnitude = action._desiredThrust;
    double gimbalAngle = action._desiredAngle;

    double thrustAngle = m_rocket.orientation() + gimbalAngle;

    //double gimbalAngle = std::clamp(action._desiredAngle, -m_rocket.maxGimbalAngle(), m_rocket.maxGimbalAngle());
    
    // rocket.width() returns the "height" of the rocket
    double torque = thrustMagnitude * (m_rocket.width()/2.0) * std::sin(gimbalAngle);

    QPointF thrust(
            action._desiredThrust * std::cos(thrustAngle), 
            action._desiredThrust * std::sin(thrustAngle)
    );

    // here i compute all the external forces acting on the rocket 
    QPointF gravity(0.0, m_rocket.mass() * Config::GRAVITY);

    // here i should add all the other forces
    QPointF total_force = gravity + thrust;

    // update the rocket parameters: gimbalAngle -> the function shouldn't have to be marked as const
    return RocketForces(total_force, torque);
}



void Simulator::reset(){

    m_rocket.reset();
    respawnBalloon();
}



StepResult Simulator::step(const Action& action, double dt) {
    m_balloon.step(dt);
    manageBalloonBoundariesCollisions();

    const RocketForces forces = computeRocketForces(action);

    // Update rocket information
    m_rocket.setThrust(action._desiredThrust);
    m_rocket.setGimbalAngle(action._desiredAngle);

    const double angularAcceleration = forces.torque / m_rocket.inertia();
    const QPointF linearAcceleration = forces.force / m_rocket.mass();

    m_rocket.integrate(dt, linearAcceleration, angularAcceleration);

    const bool success = checkRocketBalloonCollision();
    const bool failure = checkRocketBoundariesCollisions();
    const bool terminated = success || failure;

    if (terminated) addEvent(success);

    return StepResult{getState(), terminated};
}



// 13 - dimensional space 
State Simulator::getState() const {

    auto rocketPosition = m_rocket.position();
    auto balloonPosition = m_balloon.position();
    auto rocketVelocity = m_rocket.velocity();
    auto balloonVelocity = m_balloon.velocity();

    double deltaX = rocketPosition.x() - balloonPosition.x();
    double deltaY = rocketPosition.y() - balloonPosition.y();
    double squareDistance = deltaX*deltaX + deltaY*deltaY;

    State state;
    state.targetX = balloonPosition.x();
    state.targetY = balloonPosition.y();
    state.rocketX = rocketPosition.x();
    state.rocketY = rocketPosition.y();
    state.squareDistance = squareDistance;
    state.rocketVx = rocketVelocity.x();
    state.rocketVy = rocketVelocity.y();
    state.targetVx = balloonVelocity.x();
    state.targetVy = balloonVelocity.y();
    state.orientation = m_rocket.orientation();
    state.omega = m_rocket.omega();
    state.thrust = m_rocket.thrust();
    state.gimbalAngle = m_rocket.gimbalAngle();

    return state;
}



void Simulator::simulate(double dt, const Controller& controller) {
    const State state = getState();
    const Action action = controller.computeAction(state);

    const StepResult result = step(action, dt);

    if (result.terminated) reset();
}



const Rocket& Simulator::rocket() const {
    
    return m_rocket; // chat said it should be " return m_rocket; "
}

const Balloon& Simulator::balloon() const {

    return m_balloon;
}


size_t Simulator::numberEvents() const {
    return m_number_events;
}

size_t Simulator::numberSuccess() const {
    return m_number_success;
}

double Simulator::successRatio() const {
    if(m_number_events == 0) return 0.0;
    return static_cast<double>(m_number_success) / static_cast<double>(m_number_events);
}

void Simulator::addEvent(bool result) {
    if(result){
        m_number_events++;
        m_number_success++;
    } else {
        m_number_events++;
    }
    return;
}


void Simulator::setRandomSeed(unsigned int seed){
    m_random_generator.seed(seed);
}
