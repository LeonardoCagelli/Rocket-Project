#include"Balloon.h"

Balloon::Balloon(const QPointF& position, double radius)
        : MovingObject(position, QPointF(0,0)), m_radius(radius) {}

Balloon::Balloon(const QPointF& position, const QPointF& velocity, double radius)
        : MovingObject(position, velocity), m_radius(radius) {}

double Balloon::radius() const {
    return m_radius;
}