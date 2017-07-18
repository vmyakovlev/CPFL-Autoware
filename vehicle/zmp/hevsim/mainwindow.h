#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>

#include <iostream>
#include <fstream>
#include <string>
#include <queue>
#include <sys/types.h>
#include <sys/time.h>
#include <unistd.h>
#include <math.h>

using std::cout;
using std::endl;
using std::ios;
using std::ofstream;
using std::string;
using std::queue;

#define HEVSIM

namespace Ui {
class MainWindow;
}

namespace zmp {
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

    /*======Autoware Socket=================*/ 

    // actuation
    void SteeringControl(double current_steering_angle, double cmd_steering_angle);
    void StrokeControl(double current_velocity, double cmd_velocity);
    void VelocityControl(double current_velocity, double cmd_velocity);

    // cmd
    static void* CMDGetterEntry(void *a);

    // mode
    static void* ModeSetterEntry(void *a);
    void SetStrMode(int mode);
    void SetDrvMode(int mode);

    // gear
    static void* GearSetterEntry(void *a);
    void SetGear(int gear);

    // can
    void SendCAN(void);

    // common
    bool ConfigSocket(void);
    void UpdateState(void);
    void ClearCntDiag(void);

    static void *LogThreadEntry(void *);
    void logThread(void);


    double GetAccelStroke(void) {
      return _accel_stroke;
    }
    double GetBrakeStroke(void) {
      return _brake_stroke;
    }
    double GetSteerTorque(void) {
      return _steer_torque;
    }
    double GetSteerAngle(void) {
      return _steer_angle;
    }
    double GetVelocity(void) {
      return _velocity;
    }
    int GetDriveMode(void) {
      return _drive_mode;
    }
    int GetSteerMode(void) {
      return _steer_mode;
    }
    int GetDrvShiftMode(void) {
      return _shift;
    }
    void SetAccelStroke(double x) {
      _accel_stroke = x;
    }
    void SetBrakeStroke(double x) {
      _brake_stroke = x;
    }
    void SetSteerTorque(double x) {
      _steer_torque = x;
    }
    void SetSteerAngle(double x) {
      _steer_angle = x;
    }
    void SetVelocity(double x) {
      _velocity = x;
    }
    void SetDriveMode(int x) {
      _drive_mode = x;
    }
    void SetSteerMode(int x) {
      _steer_mode = x;
    }
    void SetDrvShiftMode(int x) {
      _shift = x;
    }
    /*====================================*/

private:
    Ui::MainWindow *ui;
    pthread_t _logthread;

    /*======Autoware Socket=================*/ 
    pthread_t _cmdgetter;
    /*======Autoware variables============*/ 
    double _accel_stroke, _brake_stroke;
    double _steer_torque, _steer_angle;
    double _velocity;
    int _drive_mode, _steer_mode;
    int _shift;
    QTimer *_update_timer;
    struct tm *_s_time;
    timeval _getTime;
    struct {
      bool drvInf, strInf, brkInf;
    } _selectLog;
    /*====================================*/

private slots:
    void UpdateStateSim(void);
    void doDriveOverride(void);
    void doSteerOverride(void);
    void changeDriveMode(void);
    void changeSteerMode(void);
};

#endif // MAINWINDOW_H
