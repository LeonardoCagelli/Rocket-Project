#pragma once

#include"MovingObject.h"
#include<QPointF>

class Balloon : public MovingObject{
    public:

        Balloon(const QPointF& position, double radius);
        Balloon(const QPointF& position, const QPointF& velocity, double radius);
        double radius() const;

    private:
        double m_radius;

};