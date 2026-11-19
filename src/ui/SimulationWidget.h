#pragma one

#include<QWidget>
#include<QTimer>
#include<QPainter>
#include<QCloseEvent>
#include<QApplication>

#include"../simulator/Simulator.h"
#include"../controller/Controller.h"
#include"../config.h"



class SimulationWidget : public QWidget {

    public:
        explicit SimulationWidget(const Controller& controller, QWidget* parent = nullptr);
        Simulator& simulator();

    protected:
        void paintEvent(QPaintEvent *event) override;
        void closeEvent(QCloseEvent* event) override;

    private:
        Simulator m_simulator;
        const Controller& m_controller;
        QTimer m_timer;

};
