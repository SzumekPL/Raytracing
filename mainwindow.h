#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QImage>
#include <QVector>
#include <QDebug>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    const int H = 400;
    const int W = 400;

    QImage* paper = new QImage(W,H,QImage::Format_RGB32);
    Ui::MainWindow *ui;

private slots:
    void on_pushButton_clicked();

    void on_pushButton_3_clicked();

private:

    void raytracing();
};

#endif // MAINWINDOW_H
