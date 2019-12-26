#ifndef RVIZQT_H
#define RVIZQT_H

#include <QWidget>
#include <rviz/panel.h>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include "ui_rvizqt.h"
#include "std_msgs/String.h"
#include <ros/ros.h>

#include <QString>

namespace Ui {
class RvizQT;
}

class RvizQT : public QWidget
{
    Q_OBJECT

public:
    explicit RvizQT(QWidget *parent = nullptr);
    ~RvizQT();
private Q_SLOTS:
    void click_change_status_button();
private:
    Ui::RvizQT *ui;
    //ros::NodeHandle nodeHnadle;
    //ros::Publisher chatterPub;
    int clickCounter;
    std::string controlModeStr = "on";
    std::string nonControlModeStr ="off";
};


namespace rvizQT {
class RvizPanel : public rviz::Panel{
    Q_OBJECT
public:
    RvizPanel();
private:
    RvizQT* ui;

};
}
#endif // RVIZQT_H

