#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <ros/ros.h>
#include <QTreeWidgetItem>
#include <QTimer>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
private slots:
    void on_btnRefresh_clicked();
    void on_itemActivated(QTreeWidgetItem* item, int col);
    void on_process();

private:
    Ui::MainWindow *ui;
    ros::NodeHandle n;
    QTimer*         processTimer;
};

#endif // MAINWINDOW_H
