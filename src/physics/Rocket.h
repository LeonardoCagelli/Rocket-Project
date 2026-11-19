#pragma once

#include<QPointF>
#include <stdint.h>
#include<algorithm>
#include"MovingObject.h"
#include<iostream>

class Rocket : public MovingObject {
    public:
        Rocket(const QPointF& position);

        uint32_t width() const;
        uint32_t height() const;
        double mass() const;
        double inertia() const;
        double orientation() const;
        double omega() const;
        double thrust() const;
        double gimbalAngle() const;
        double maxGimbalAngle() const;
        double maxThrust() const;


        void setThrust(double thrust);
        void setGimbalAngle(double angle);
        void setOrientation(double orientation);
        void setOmega(double omega);

        void reset();

        void integrate(double dt, const QPointF& linearAcceleration, double angularAcceleration);
        void step(double dt) override {
            throw std::logic_error("step() should not be called on a Rocket instance");
        }

    private:

        // the position of the rocket represents the position of the center of mass
        uint32_t m_height;
        uint32_t m_width;
        double m_mass;
        double m_inertia;

        double m_orientation; // heading angle, different from the velocity derived angle. It's the direction the rocket is pointing
        double m_omega;
        double m_thrust;     // engine magnitude 
        double m_gimbalAngle;   // the angle between the rocket axis and the engine axis

        double m_maxGimbalAngle;
        double m_maxThrust;

}; 

