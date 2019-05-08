#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent): QMainWindow(parent), ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    thread.start();
    connect(&thread, SIGNAL(renderedImage(QImage)), this, SLOT(updatePixmap(QImage)));
}
MainWindow::~MainWindow() {
    delete ui;
    thread.quit();
}

int MainWindow::updatePixmap(const QImage &image) {

    image_map = QPixmap::fromImage(image);
    scene = new QGraphicsScene(this);
    scene->addPixmap(image_map);
    scene->setSceneRect(image_map.rect());
    ui->graphicsView->setScene(scene);
    ui->graphicsView->show();

    return 0;
}

void MainWindow::mousePressEvent(QMouseEvent* event) {
    try {
        thread.makeATesla(event->x(), event->y());
    } catch(...) {
        std::cout << "\nFailed to spawn actor !\n";
    }
}
