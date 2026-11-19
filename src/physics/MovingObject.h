#pragma once

#include<QPointF>

class MovingObject {

    public:
        MovingObject();
        MovingObject(const QPointF& position, const QPointF& velocity);

        virtual void step(double dt);
        void setVelocity(const QPointF& velocity);
        void setPosition(const QPointF& position);

        QPointF velocity() const;
        QPointF position() const;

    protected:  
        QPointF m_position;
        QPointF m_velocity;
};