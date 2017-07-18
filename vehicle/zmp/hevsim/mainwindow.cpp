#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    pthread_create(&_logthread, NULL, LogThreadEntry, this);

    /////////////////////////////////////////////////////////
    // Autoware extension
    if (!ConfigSocket()) {
      printf("Error: failed to configure Autoware socket!\n");    
    }
    pthread_create(&_cmdgetter, NULL, CMDGetterEntry, this);
    pthread_detach(_cmdgetter);
    ////////////////////////////////////////////////////////

    _accel_stroke = 0.0;
    _brake_stroke = 0.0;
    _steer_torque = 0.0;
    _steer_angle = 0.0;
    _velocity = 0.0;
    _drive_mode = 0;
    _steer_mode = 0;
    _shift = 0;
    _update_timer = new QTimer(this);
    _selectLog.drvInf = true;
    _selectLog.strInf = true;
    _selectLog.brkInf = true;
    UpdateStateSim();

    connect(ui->drive_override_button, SIGNAL(clicked()), this, SLOT(doDriveOverride()));
    connect(ui->steer_override_button, SIGNAL(clicked()), this, SLOT(doSteerOverride()));
    connect(ui->drive_mode_button, SIGNAL(clicked()), this, SLOT(changeDriveMode()));
    connect(ui->steer_mode_button, SIGNAL(clicked()), this, SLOT(changeSteerMode()));
    connect(_update_timer, SIGNAL(timeout()), this, SLOT(UpdateStateSim()));
    _update_timer->start(100/*msec*/);
}

void MainWindow::UpdateStateSim(void)
{
  QString pstr = QString("Program");
  QString mstr = QString("Manual");
  time_t t;

  t = time(NULL);
  _s_time = gmtime(&t);
  gettimeofday(&_getTime, NULL);
  
  if (_drive_mode != 0) {
    ui->drive_mode_edit->setText(pstr);
    ui->drive_mode_button->setText(mstr);
  } else {
    ui->drive_mode_edit->setText(mstr);
    ui->drive_mode_button->setText(pstr);
  }
  if (_steer_mode != 0) {
    ui->steer_mode_edit->setText(pstr);
    ui->steer_mode_button->setText(mstr);
  } else {
    ui->steer_mode_edit->setText(mstr);
    ui->steer_mode_button->setText(pstr);
  }
  ui->velocity_edit->setText(QString::number(_velocity, 'f', 4));
  ui->accel_edit->setText(QString::number(_accel_stroke, 'f', 4));
  ui->brake_edit->setText(QString::number(_brake_stroke, 'f', 4));
  ui->angle_edit->setText(QString::number(_steer_angle, 'f', 4));
  ui->torque_edit->setText(QString::number(_steer_torque, 'f', 4));
}

void *MainWindow::LogThreadEntry(void *a)
{
  MainWindow *main = (MainWindow *)a;
  main->logThread();
  return NULL;
}

void MainWindow::logThread(void)
{
  while (1) {
    SendCAN();
  }
}

void MainWindow::doDriveOverride(void)
{
  _drive_mode = 0;
  UpdateStateSim();
}

void MainWindow::doSteerOverride(void)
{
  _steer_mode = 0;
  UpdateStateSim();
}

void MainWindow::changeDriveMode(void)
{
  _drive_mode = (_drive_mode != 0) ? 0:0x10;
  UpdateStateSim();
}

void MainWindow::changeSteerMode(void)
{
  _steer_mode = (_steer_mode != 0) ? 0:0x10;
  UpdateStateSim();
}

MainWindow::~MainWindow()
{
    delete _update_timer;
    delete ui;
}
