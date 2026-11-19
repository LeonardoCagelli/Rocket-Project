#include"SimpleController.h"



Action SimpleController::computeAction(const State& state) const
{

    constexpr double PI = 3.14159265358979323846;
    constexpr double EPSILON = 1e-9;

    // PD and alignment gains.
    constexpr double K_target = 4.0;
    constexpr double K_velocity = 1.0;
    constexpr double Kd = 4.0;

    // Below this speed, velocity direction is unreliable.
    constexpr double MIN_ALIGNMENT_SPEED = 1.0;

    auto wrapAngle = [](double angle) {
        constexpr double PI_LOCAL =
            3.14159265358979323846;

        while (angle > PI_LOCAL)
            angle -= 2.0 * PI_LOCAL;

        while (angle < -PI_LOCAL)
            angle += 2.0 * PI_LOCAL;

        return angle;
    };

    // 1. Calculate the direction toward the target.

    const double leverArm = Config::ROCKET_WIDTH / 2.0;

    const QPointF rocketPosition = QPointF(state.rocketX, state.rocketY);
    const QPointF targetPosition = QPointF(state.targetX, state.targetY);

    const QPointF headPosition =
        rocketPosition +
        QPointF(leverArm * std::cos(state.orientation),
                leverArm * std::sin(state.orientation));

    const QPointF dir = targetPosition - headPosition;

    const double targetHeading =
        std::atan2(dir.y(), dir.x());

    const double targetAngleError =
        wrapAngle(targetHeading - state.orientation);

    // 2. Calculate the velocity-alignment error.
    const QPointF velocity = QPointF(state.rocketVx, state.rocketVy);

    const double speed =
        std::hypot(velocity.x(), velocity.y());

    double velocityAngleError = 0.0;

    if (speed > EPSILON)
    {
        const double velocityHeading =
            std::atan2(velocity.y(), velocity.x());

        velocityAngleError =
            wrapAngle(velocityHeading - state.orientation);
    }

    // Fade in velocity alignment smoothly as speed increases.
    const double alignmentWeight = std::clamp(
        (speed - MIN_ALIGNMENT_SPEED) / MIN_ALIGNMENT_SPEED,
        0.0,
        1.0
    );

    // 3. Combine target tracking, velocity alignment,
    // and angular-velocity damping.
    const double desiredAngularAcceleration =
        K_target * targetAngleError
        + K_velocity * alignmentWeight * velocityAngleError
        - Kd * state.omega;

    // 4. Convert desired angular acceleration to a gimbal angle.
    const double maxThrust = Config::MAX_THRUST;

    if (maxThrust <= EPSILON || leverArm <= EPSILON)
        return Action(0.0, 0.0);

    const double desiredTorque =
        Config::ROCKET_INERTIA * desiredAngularAcceleration;

    const double maxSinGimbal =
        std::sin(Config::MAX_GIMBAL_ANGLE);

    const double sinGimbal = std::clamp(
        desiredTorque / (maxThrust * leverArm),
        -maxSinGimbal,
         maxSinGimbal
    );

    const double desiredGimbalAngle =
        std::asin(sinGimbal);

    // 5. Compute thrust direction in world coordinates.
    const double theta =
        state.orientation + desiredGimbalAngle;

    const QPointF thrustDirection(
        std::cos(theta),
        std::sin(theta)
    );

    // Positive Y points downward in this coordinate convention.
    const QPointF gravityForce(
        0.0,
        Config::ROCKET_MASS * Config::GRAVITY
    );

    // Desired magnitude of the total force.
    const double targetForceMagnitude =
        Config::ROCKET_MASS * Config::SimpleController::TARGET_NET_ACCELERATION;

    const double d =
        QPointF::dotProduct(gravityForce, thrustDirection);

    const double c =
        QPointF::dotProduct(gravityForce, gravityForce)
        - targetForceMagnitude * targetForceMagnitude;

    const double discriminant = d * d - c;

    double desiredThrust = 0.0;

    if (discriminant >= 0.0)
    {
        // Solve |gravityForce + T * thrustDirection|
        // = targetForceMagnitude.
        desiredThrust =
            -d + std::sqrt(discriminant);
    }
    else
    {
        // Exact target magnitude is unreachable in this direction.
        // Select the thrust that minimizes the net force magnitude.
        desiredThrust = -d;
    }

    // Respect actuator limits.
    desiredThrust = std::clamp(
        desiredThrust,
        0.0,
        maxThrust
    );

    return Action(desiredThrust, desiredGimbalAngle);
}

