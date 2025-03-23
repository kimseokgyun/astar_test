/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.15.3
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralwidget;
    QLabel *label;
    QLineEdit *start_x;
    QLineEdit *start_y;
    QLineEdit *start_th;
    QLabel *label_2;
    QLabel *label_3;
    QLabel *label_6;
    QLabel *label_8;
    QLineEdit *end_y;
    QLineEdit *end_x;
    QLineEdit *end_th;
    QPushButton *start_set;
    QPushButton *end_set;
    QMenuBar *menubar;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(800, 827);
        centralwidget = new QWidget(MainWindow);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        label = new QLabel(centralwidget);
        label->setObjectName(QString::fromUtf8("label"));
        label->setGeometry(QRect(30, 150, 121, 21));
        start_x = new QLineEdit(centralwidget);
        start_x->setObjectName(QString::fromUtf8("start_x"));
        start_x->setGeometry(QRect(160, 150, 61, 25));
        start_y = new QLineEdit(centralwidget);
        start_y->setObjectName(QString::fromUtf8("start_y"));
        start_y->setGeometry(QRect(230, 150, 61, 25));
        start_th = new QLineEdit(centralwidget);
        start_th->setObjectName(QString::fromUtf8("start_th"));
        start_th->setGeometry(QRect(300, 150, 61, 25));
        label_2 = new QLabel(centralwidget);
        label_2->setObjectName(QString::fromUtf8("label_2"));
        label_2->setGeometry(QRect(170, 130, 121, 21));
        label_3 = new QLabel(centralwidget);
        label_3->setObjectName(QString::fromUtf8("label_3"));
        label_3->setGeometry(QRect(250, 130, 121, 21));
        label_6 = new QLabel(centralwidget);
        label_6->setObjectName(QString::fromUtf8("label_6"));
        label_6->setGeometry(QRect(320, 130, 121, 21));
        label_8 = new QLabel(centralwidget);
        label_8->setObjectName(QString::fromUtf8("label_8"));
        label_8->setGeometry(QRect(30, 180, 121, 21));
        end_y = new QLineEdit(centralwidget);
        end_y->setObjectName(QString::fromUtf8("end_y"));
        end_y->setGeometry(QRect(230, 180, 61, 25));
        end_x = new QLineEdit(centralwidget);
        end_x->setObjectName(QString::fromUtf8("end_x"));
        end_x->setGeometry(QRect(160, 180, 61, 25));
        end_th = new QLineEdit(centralwidget);
        end_th->setObjectName(QString::fromUtf8("end_th"));
        end_th->setGeometry(QRect(300, 180, 61, 25));
        start_set = new QPushButton(centralwidget);
        start_set->setObjectName(QString::fromUtf8("start_set"));
        start_set->setGeometry(QRect(370, 150, 89, 25));
        end_set = new QPushButton(centralwidget);
        end_set->setObjectName(QString::fromUtf8("end_set"));
        end_set->setGeometry(QRect(370, 180, 89, 25));
        MainWindow->setCentralWidget(centralwidget);
        menubar = new QMenuBar(MainWindow);
        menubar->setObjectName(QString::fromUtf8("menubar"));
        menubar->setGeometry(QRect(0, 0, 800, 22));
        MainWindow->setMenuBar(menubar);
        statusbar = new QStatusBar(MainWindow);
        statusbar->setObjectName(QString::fromUtf8("statusbar"));
        MainWindow->setStatusBar(statusbar);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QCoreApplication::translate("MainWindow", "MainWindow", nullptr));
        label->setText(QCoreApplication::translate("MainWindow", "start (x,y  theta)", nullptr));
        label_2->setText(QCoreApplication::translate("MainWindow", "x", nullptr));
        label_3->setText(QCoreApplication::translate("MainWindow", "y", nullptr));
        label_6->setText(QCoreApplication::translate("MainWindow", "th", nullptr));
        label_8->setText(QCoreApplication::translate("MainWindow", "end (x,y,theta)", nullptr));
        start_set->setText(QCoreApplication::translate("MainWindow", "SET", nullptr));
        end_set->setText(QCoreApplication::translate("MainWindow", "SET", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
