/********************************************************************************
** Form generated from reading UI file 'mainUI.ui'
**
** Created by: Qt User Interface Compiler version 6.4.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINUI_H
#define UI_MAINUI_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QFrame>
#include <QtWidgets/QLabel>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_guiDlg
{
public:
    QFrame *frame;
    QLabel *label;

    void setupUi(QWidget *guiDlg)
    {
        if (guiDlg->objectName().isEmpty())
            guiDlg->setObjectName("guiDlg");
        guiDlg->resize(1142, 600);
        frame = new QFrame(guiDlg);
        frame->setObjectName("frame");
        frame->setGeometry(QRect(0, 10, 561, 591));
        frame->setFrameShape(QFrame::StyledPanel);
        frame->setFrameShadow(QFrame::Raised);
        frame->setMidLineWidth(15);
        label = new QLabel(guiDlg);
        label->setObjectName("label");
        label->setGeometry(QRect(570, 10, 571, 591));
        label->setMidLineWidth(2);
        label->setAlignment(Qt::AlignCenter);

        retranslateUi(guiDlg);

        QMetaObject::connectSlotsByName(guiDlg);
    } // setupUi

    void retranslateUi(QWidget *guiDlg)
    {
        guiDlg->setWindowTitle(QCoreApplication::translate("guiDlg", "chocachoca", nullptr));
        label->setText(QCoreApplication::translate("guiDlg", "TextLabel", nullptr));
    } // retranslateUi

};

namespace Ui {
    class guiDlg: public Ui_guiDlg {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINUI_H
