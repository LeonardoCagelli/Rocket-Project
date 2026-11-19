#include<iostream>
#include<QApplication>
#include<QScreen>

#include"ui/SimulationWidget.h"
#include"ui/TelemetryWidget.h"



int main(int argc, char *argv[]){
    QApplication app(argc, argv);

    //std::cout<<"Step 1"<<std::endl;

    RL_DQN_Controller controller;

    //std::cout<<"Step 2"<<std::endl;

    SimulationWidget window(controller);
    window.setWindowTitle("Balloon Interceptor");

    //std::cout<<"Step 3"<<std::endl;

    TelemetryWidget telemetry(window.simulator(), &window);     // in this case, we have created telemetry as a child of window
    telemetry.setWindowTitle("Telemetry");

    //std::cout<<"Step 4"<<std::endl;

    // Move the windows 
    QScreen *screen = QGuiApplication::primaryScreen();
    QRect screenGeometry = screen->availableGeometry();

    //std::cout<<"Step 4"<<std::endl;

    window.move(
        screenGeometry.center().x() - window.width() / 2,
        screenGeometry.center().y() - window.height() / 2
    );

    //std::cout<<"Step 5"<<std::endl;

    telemetry.move(
        screenGeometry.left(),
        screenGeometry.center().y() - telemetry.height()/2
    );

    //std::cout<<"Step 6"<<std::endl;

    window.show();
    telemetry.show();

    //std::cout<<"Step 7"<<std::endl;

    return app.exec();
}