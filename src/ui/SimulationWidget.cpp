#include"SimulationWidget.h"

SimulationWidget::SimulationWidget(const Controller& controller, QWidget* parent) : QWidget(parent), m_controller(controller) {

    setFixedSize(Config::WINDOW_WIDTH, Config::WINDOW_HEIGHT);

    m_timer.setInterval(Config::TIMER_INTERVAL); // 16 milliseconds 

    connect(&m_timer, 
            &QTimer::timeout, 
            this, 
            [this](){
                m_simulator.simulate(1.0/Config::SIMULATION_FREQUENCY, m_controller); // 60Hz => it's almost 16 milliseconds
                update();   // this function tells the widget that the window has to be redrawn -> it calls PaintEvent() function automatically
            });

    m_timer.start();
}

Simulator& SimulationWidget::simulator() {
    return m_simulator;
}

void SimulationWidget::paintEvent(QPaintEvent *event) {
    Q_UNUSED(event);

    QPainter painter(this);

    const Balloon& balloon = m_simulator.balloon();
    const Rocket& rocket = m_simulator.rocket();

    // Balloon
    painter.setPen(Qt::black);
    painter.setBrush(Qt::blue);
    
    painter.drawEllipse(
            balloon.position(),
            balloon.radius(), 
            balloon.radius()
    );

    painter.save();

    QPointF rocketPosition = rocket.position();

    painter.translate(rocketPosition);
    painter.rotate(rocket.orientation() * 180.0 / M_PI);

    painter.setBrush(Qt::red);

    // The rectangle is created extending the center of the rocket by half-width and half-height
    painter.drawRect(
        -Config::ROCKET_WIDTH / 2.0,
        -Config::ROCKET_HEIGHT / 2.0,
        Config::ROCKET_WIDTH,
        Config::ROCKET_HEIGHT
    );

    painter.setBrush(Qt::black);
    painter.drawEllipse(
    QPointF(Config::ROCKET_WIDTH / 2.0 - 8.0, 0.0),
    4.0,
    4.0
        );

    painter.restore();
   
}


void SimulationWidget::closeEvent(QCloseEvent* event)
{
    QWidget::closeEvent(event);
    QApplication::exit(0);
}