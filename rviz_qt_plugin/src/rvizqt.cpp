#include "../include/rvizqt.h"


RvizQT::RvizQT(QWidget *parent):
    QWidget(parent),
    ui(new Ui::RvizQT)
{
    ui->setupUi(this);
    connect(ui->changeStatusButton, SIGNAL(clicked()), this, SLOT(click_change_status_button()));
    
    clickCounter =0;
    std::cout<<"finish initial!"<<std::endl;
}

void RvizQT::click_change_status_button(){
    if(clickCounter%2==0){
        ui->labelStatus->setText(QString::fromStdString(controlModeStr));
    }
    else{
        ui->labelStatus->setText(QString::fromStdString(nonControlModeStr));
    }
    clickCounter++;
    

}
RvizQT::~RvizQT()
{
    delete ui;
}

rvizQT::RvizPanel::RvizPanel(){
  ui = new RvizQT(this);
  QHBoxLayout* topic_layout = new QHBoxLayout;
  topic_layout->addWidget(ui);
  QVBoxLayout* layout = new QVBoxLayout;
  layout->addLayout(topic_layout);
  setLayout( layout );
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rvizQT::RvizPanel, rviz::Panel)

