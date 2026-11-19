#pragma once

#include<QWidget>

class Simulator; // we can tell the compiler that there is a class called Simulator and include the Simulator.h file into the .cpp file
class QLabel;
class QTimer;
class QCloseEvent;


class TelemetryWidget : public QWidget {

    public:
        explicit TelemetryWidget(Simulator& simulator , QWidget* parent = nullptr);

    private:

        bool m_allowClose = false;

        void updateTelemetry();

        const Simulator& m_simulator;     // This class has direct access to the 
        QTimer* m_timer;

        QLabel* m_number_events;
        QLabel* m_number_success;

        QLabel* m_positionXLabel;
        QLabel* m_positionYLabel;

        QLabel* m_velocityXLabel;
        QLabel* m_velocityYLabel;
        QLabel* m_velocityMagnitudeLabel;

        QLabel* m_headingLabel;
        QLabel* m_angularVelocityLabel;
        QLabel* m_angularAccelerationLabel;

        QLabel* m_distanceLabel;

        QLabel* m_balloon_speed;

        QLabel* m_thrust_label;
        QLabel* m_gimbalAngle_label;
        QLabel* m_orientation_label;
        QLabel* m_omega_label;

    protected:
        void closeEvent(QCloseEvent* event) override;
        
};

