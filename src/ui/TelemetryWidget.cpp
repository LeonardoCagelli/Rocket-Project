#include"TelemetryWidget.h"
#include"../simulator/Simulator.h"

#include<QTimer>
#include<QLabel>
#include<QVBoxLayout>
#include <QCloseEvent>


TelemetryWidget::TelemetryWidget(Simulator& simulator , QWidget* parent) :
        QWidget(parent),
        m_simulator(simulator),
        m_timer(new QTimer(this))
{
    setWindowTitle("Telemetry");
    setWindowFlags(Qt::Tool
                | Qt::CustomizeWindowHint
                | Qt::WindowTitleHint);   // title bar, but no close button
    resize(300, 400);

    // Create the labels

    m_number_events = new QLabel(this);
    m_number_success = new QLabel(this);

    m_positionXLabel = new QLabel(this);
    m_positionYLabel = new QLabel(this);

    m_velocityXLabel = new QLabel(this);
    m_velocityYLabel = new QLabel(this);
    m_velocityMagnitudeLabel = new QLabel(this);

    m_headingLabel = new QLabel(this);
    m_angularVelocityLabel = new QLabel(this);

    m_distanceLabel = new QLabel(this);

    m_balloon_speed = new QLabel(this);

    m_thrust_label = new QLabel(this);
    m_gimbalAngle_label = new QLabel(this);


    // Create the structure to store the labels in a vertical order
    QVBoxLayout* layout = new QVBoxLayout(this);

    layout->addWidget(new QLabel("<b>Simulation Events</b>", this));
    layout->addWidget(m_number_events);
    layout->addWidget(m_number_success);

    layout->addWidget(new QLabel("<b>Position</b>", this));
    layout->addWidget(m_positionXLabel);
    layout->addWidget(m_positionYLabel);

    layout->addWidget(new QLabel("<b>Velocity</b>", this));
    layout->addWidget(m_velocityXLabel);
    layout->addWidget(m_velocityYLabel);
    layout->addWidget(m_velocityMagnitudeLabel);

    layout->addWidget(new QLabel("<b>Rotation</b>", this));
    layout->addWidget(m_headingLabel);
    layout->addWidget(m_angularVelocityLabel);

    layout->addWidget(new QLabel("<b>Target</b>", this));
    layout->addWidget(m_distanceLabel);

    layout->addWidget(new QLabel("<b>Balloon speed</b>", this));
    layout->addWidget(m_balloon_speed);

    layout->addWidget(new QLabel("<b>Controller Commands</b>", this));
    layout->addWidget(m_thrust_label);
    layout->addWidget(m_gimbalAngle_label);


    connect(m_timer,
            &QTimer::timeout,
            this,
            &TelemetryWidget::updateTelemetry);

    m_timer->start(100);

    updateTelemetry();
}

void TelemetryWidget::updateTelemetry()
{
    const Rocket& rocket = m_simulator.rocket();
    const Balloon& balloon = m_simulator.balloon();

    const QPointF position = rocket.position();
    const QPointF velocity = rocket.velocity();

    const double thrust = m_simulator.rocket().thrust();
    const double gimbalAngle = m_simulator.rocket().gimbalAngle();

    const double velocityMagnitude =
        std::hypot(velocity.x(), velocity.y());

    const double distance =
        std::hypot(
            balloon.position().x() - position.x(),
            balloon.position().y() - position.y()
        );

    const double balloon_speed = std::hypot(balloon.velocity().x(), balloon.velocity().y());

    m_number_events->setText(
        QString("Total Events: %1").arg(m_simulator.numberEvents())
    );

    m_number_success->setText(
        QString("Successful events: %1").arg(m_simulator.numberSuccess())
    );

    m_positionXLabel->setText(
        QString("X: %1").arg(position.x(), 0, 'f', 2)
    );

    m_positionYLabel->setText(
        QString("Y: %1").arg(position.y(), 0, 'f', 2)
    );

    m_velocityXLabel->setText(
        QString("X: %1").arg(velocity.x(), 0, 'f', 2)
    );

    m_velocityYLabel->setText(
        QString("Y: %1").arg(velocity.y(), 0, 'f', 2)
    );

    m_velocityMagnitudeLabel->setText(
        QString("Velocity Magnitude: %1").arg(velocityMagnitude, 0, 'f', 2)
    );

    m_headingLabel->setText(
        QString("Heading: %1 rad").arg(rocket.orientation(), 0, 'f', 3)
    );

    m_angularVelocityLabel->setText(
        QString("Angular velocity: %1 rad/s")
            .arg(rocket.omega(), 0, 'f', 3)
    );

    m_distanceLabel->setText(
        QString("Distance: %1").arg(distance, 0, 'f', 2)
    );

    m_balloon_speed->setText(
        QString("Balloon Speed: %1").arg(balloon_speed, 0, 'f', 3)
    );

    m_thrust_label->setText(
        QString("Thrust: %1").arg(thrust, 0, 'f', 2)
    );

    m_gimbalAngle_label->setText(
        QString("Gimbal Angle : %1").arg(gimbalAngle, 0, 'f', 7)
    );
}


void TelemetryWidget::closeEvent(QCloseEvent* event)
{
    event->ignore();
}

