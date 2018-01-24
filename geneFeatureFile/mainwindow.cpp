#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{

    ui->setupUi(this);
    globalSettings = new QSettings("PKU", "featureSaver", this);
    ui->lineEdit_feature->setText(globalSettings->value("featurePath", "").toString());
    ui->lineEdit_velo->setText(globalSettings->value("veloDataPath", "").toString());
    ui->lineEdit_calib->setText(globalSettings->value("veloCalibPath", "").toString());
    ui->lineEdit_nav->setText(globalSettings->value("NavFilepath","").toString());

    connect(ui->pushButton_savePath, &QPushButton::clicked, [ = ]() {
        QString path = QFileDialog::getExistingDirectory(this, "Saving Position", ".");
        if (!path.isEmpty()) {
            ui->lineEdit_feature->setText(path);
            globalSettings->setValue("featurePath", path);
        }
    });

    connect(ui->pushButton_veloPath, &QPushButton::clicked, [ = ]() {
        QString path = QFileDialog::getOpenFileName(this, "Velodyne File Position", ".", "Velo (*.xyzi)");
        if (!path.isEmpty()) {
            ui->lineEdit_velo->setText(path);
            globalSettings->setValue("veloDataPath", path);
//            ui->lineEditName->setEnabled(true);
        }
    });

    connect(ui->pushButton_calib, &QPushButton::clicked, [ = ]() {
        QString path = QFileDialog::getOpenFileName(this, "Velodyne CalibFile Position", ".", "db (*.xml)");
        if (!path.isEmpty()) {
            ui->lineEdit_calib->setText(path);
            globalSettings->setValue("veloCalibPath", path);
        }
    });

    connect(ui->pushButton_navpath, &QPushButton::clicked, [ = ]() {
        QString path = QFileDialog::getOpenFileName(this, "Nav Position", ".", "nav (*.nav)");
        if (!path.isEmpty()) {
            ui->lineEdit_nav->setText(path);
            globalSettings->setValue("NavFilepath", path);
        }
    });

    stopProcess = 0;

    connect(ui->button_start, &QPushButton::clicked, this, &MainWindow::startExtracting);
    connect(ui->button_stop, &QPushButton::clicked, this, &MainWindow::stopProcessing);

    ui->progressBar->setRange(0,100);
    ui->progressBar->setValue(0);
}

MainWindow::~MainWindow()
{
    delete ui;
    delete this->featureSaver;
}

void MainWindow::stopProcessing()
{
    ui->checkBox_fixLaserTime->setEnabled(1);
    stopProcess = 1;
}

void MainWindow::startExtracting()
{
    ui->button_start->setEnabled(false);
    QString savePath = ui->lineEdit_feature->text();
    QString velofilePath = ui->lineEdit_velo->text();
    QString veloCalib = ui->lineEdit_calib->text();
    QString navfilepath = ui->lineEdit_nav->text();
    this->featureSaver = new mainProcess;
    this->featureSaver->frameloader.setPath(velofilePath.toStdString().c_str());
    this->featureSaver->savefeature.setCalibPath(veloCalib.toStdString().c_str());
    this->featureSaver->savefeature.setPath(savePath.toStdString().c_str());

    this->featureSaver->savefeature.setHighRangeMax(ui->lineEdit_highrangemax->text().toDouble());
    this->featureSaver->savefeature.setMidRangeMax(ui->lineEdit_midrangemax->text().toDouble());
    this->featureSaver->savefeature.setGndRangeMax(ui->lineEdit_gndrangemax->text().toDouble());

    this->featureSaver->loadNavFile(navfilepath);
    this->featureSaver->setDSSavePath(savePath, "");
    this->featureSaver->setIsFixVelodyneTimeDelay(ui->checkBox_fixLaserTime->isChecked());
    ui->checkBox_fixLaserTime->setEnabled(0);

    long long counter = 0;


    while(this->featureSaver->frameProcess() && (!stopProcess)){

        ui->progressBar->setValue(featureSaver->progress*100.0);

        if (counter++%20==0){
            cv::imshow("highVis",this->featureSaver->highVis);
            cv::imshow("midVis",this->featureSaver->midVis);
            cv::imshow("gndView", this->featureSaver->gndVis);
            cv::imshow("laneVis", this->featureSaver->laneVis);
            cv::waitKey(10);
        }
    }
    ui->button_start->setEnabled(true);

}

