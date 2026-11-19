#include"MovingObject.h"


MovingObject::MovingObject() : m_position(QPointF(0.0, 0.0)), m_velocity(QPointF(0.0, 0.0)) {}

MovingObject::MovingObject(const QPointF& position, const QPointF& velocity) :
        m_position(position), m_velocity(velocity) {}

void MovingObject::step(double dt){
    m_position += m_velocity * dt;
}

void MovingObject::setVelocity(const QPointF& velocity){
    m_velocity = velocity;
}

void MovingObject::setPosition(const QPointF& position){
    m_position = position;
}

QPointF MovingObject::velocity() const {
    return m_velocity;
}

QPointF MovingObject::position() const {
    return m_position;
}