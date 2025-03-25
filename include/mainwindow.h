#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include <QDebug>
#include <QLineEdit>
#include <iostream>
#include <stdio.h>
#include <fcntl.h>

#include "astar_test.hpp"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE


inline void SetLEColor(QLineEdit *le, QString color){
    le->setStyleSheet("QLineEdit{background-color:"+color+"}");
}


class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    void setQtNode(std::shared_ptr<astar_test::astar_test> node);


private:
    std::shared_ptr<astar_test::astar_test> astar_node_;

public slots:


private slots:

    void on_start_set_clicked();

    void on_end_set_clicked();

    void on_bt_next_clicked();
    void on_bx_end_clicked();

private:
    Ui::MainWindow *ui;
    QTimer timer;
};
#endif // MAINWINDOW_H
