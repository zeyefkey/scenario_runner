#ifndef MAINWINDOW_H
#define MAINWINDOW_H


#include <iostream>

#include <QImage>
#include <QGraphicsScene>
#include <QMainWindow>
#include <QMouseEvent>

#include "carlastreamthread.h"

using namespace std::literals::chrono_literals;
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
    int updatePixmap(const QImage &image);

private:
    Ui::MainWindow *ui;

    CarlaStreamThread thread;

    QPixmap image_map;
    QImage  *imageObject;
    QGraphicsScene *scene;

protected:
    void mousePressEvent(QMouseEvent* event);
};

#endif // MAINWINDOW_H
