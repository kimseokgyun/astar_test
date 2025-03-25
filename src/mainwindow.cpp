#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    connect(&timer, SIGNAL(timeout()), this, SLOT(onTimer()));
    
    timer.start(10); //10
    
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::setQtNode(std::shared_ptr<astar_test::astar_test> node) 
{
    astar_node_ = node;
}

void MainWindow::on_start_set_clicked()
{
    double x= ui->start_x->text().toDouble();
    double y= ui->start_y->text().toDouble();
    double theta = ui->start_th->text().toDouble();

    astar_node_->start.setIdentity();
    astar_node_->start(0,3) = x;
    astar_node_->start(1,3) = y;
    astar_node_->start(2,3) = 0.0;
    astar_node_ ->start(3,3) = 1.0;

    Eigen::Matrix3d rotation;
    rotation = Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ()); 
    astar_node_->start.block<3, 3>(0, 0) = rotation;  

}


void MainWindow::on_end_set_clicked()
{
    double x= ui->end_x->text().toDouble();
    double y= ui->end_y->text().toDouble();
    double theta = ui->end_th->text().toDouble();

    astar_node_->end.setIdentity();
    astar_node_->end(0,3) = x;
    astar_node_->end(1,3) = y;
    astar_node_->end(2,3) = 0.0;
    astar_node_ ->end(3,3) = 1.0;

    Eigen::Matrix3d rotation;
    rotation = Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ()); 
    astar_node_->end.block<3, 3>(0, 0) = rotation;  
}


void MainWindow::on_bt_next_clicked()
{
    astar_node_->astar_step_flag = true;
}

void MainWindow::on_bx_end_clicked()
{
    // astar_node_->astar_run = false;
}