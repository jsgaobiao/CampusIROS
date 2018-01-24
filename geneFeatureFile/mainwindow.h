/** @file
 *  @brief 离线特征提取
 *  @author 梅继林 (MEI Jilin)
 *  @date 2017.12
 *  @note 本项目使用MIT开源协议，请遵守该协议使用
 */

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

// QT
#include <QMainWindow>
#include <QFileDialog>
//#include <QMessageBox>
#include <QFile>
#include <QSettings>
//#include <QDebug>
//#include <QTimer>
#include "mainprocess.h"
namespace Ui {
class MainWindow;
}

/**
 * @brief The MainWindow class 离线特征提取主类
 */
class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

public:
    Ui::MainWindow *ui;
    QSettings *globalSettings; /**< 保存每次选择的路径 >*/
    mainProcess* featureSaver; /**< 特征提取 >*/

    bool stopProcess;
public slots:
    void startExtracting();
    void stopProcessing();
};

#endif // MAINWINDOW_H
