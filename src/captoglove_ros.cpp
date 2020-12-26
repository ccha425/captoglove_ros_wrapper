#include <captoglove_ros/captoglove_ros.h>
#include <signal.h>
#include <iostream>

namespace captoglove_ros{

bool m_request_shutdown;

captoglove_ros::captoglove_ros(int argc, char** argv){
    m_init_argc = argc;
    m_init_argv = argv;
    m_request_shutdown = false;

    QCoreApplication::setApplicationName("CaptogloveROS");
    QCoreApplication::setApplicationVersion("0.8");

    QCommandLineParser parser;
    parser.addHelpOption();
    parser.addVersionOption();

    parser.addOptions({
            {{"c", "config path"},
                QCoreApplication::translate("main", "Location of the config .ini file"),
                QCoreApplication::translate("main", "path")}
        });

     parser.process(QCoreApplication::arguments());

    QString configFilePath = "";

    if (parser.isSet("c")){
        configFilePath = parser.values("c").at(0);
    }else{
        publishROSErrorToTerminal("No config path set. Using default values.");
    }


    m_captogloveAPI = new CaptoGloveAPI(this, configFilePath);

    sleep(5);

    if(!this->init()){
        publishROSErrorToTerminal("Master not found. Please restart node.");
        QCoreApplication::exit();
        return;
    }else{
        std::cout << "Succesfully initialized program!";
    }

    publishROSInfoToTerminal("Master found. Starting node");

    //this->start();


    m_captogloveAPI->run();

    std::cout << "Started API!";
}

captoglove_ros::~captoglove_ros(){

}


void captoglove_ros::shutdownHandler(int signum){
    m_request_shutdown = true;
}

bool captoglove_ros::init(){
    std::cout << "Entered initialization!";

    ros::init(m_init_argc, m_init_argv, "captoglove_ros", ros::init_options::NoSigintHandler);

    if (! ros::master::check()){
        return false;
    }

    ros::start();
    ros::NodeHandle nh("/");
    ros::NodeHandle pn("~");

    signal(SIGINT, captoglove_ros::shutdownHandler);

    if (m_captogloveAPI->getFingers()){
        connect(m_captogloveAPI, SIGNAL(updateFingerState(captoglove_v1::FingerFeedbackMsg)),
                SLOT(on_fingerStatesUpdated(captoglove_v1::FingerFeedbackMsg)));

        m_fingerFeedback_Publisher = nh.advertise<captoglove_ros_wrapper::FingerFeedbackMsg>("glove/fingers/state", 10);
    }

    return true;

}

void captoglove_ros::run(){
    int i = 0;
    ros::Rate r(100);

    ros::Duration(1).sleep();

    while(!m_request_shutdown){
        ros::spinOnce();

        i++;

        r.sleep();
    }

    // TODO: Add stop to CaptoGloveAPI method
    // m_captogloveAPI->stop(); --> commented it out to successfully build ros workspace

    delete m_captogloveAPI;
    ros::Duration(0.1).sleep();
    ros::shutdown();
    QCoreApplication::exit();

    return;
}

void captoglove_ros::on_fingerStatesUpdated(captoglove_v1::FingerFeedbackMsg msg){
    m_fingerFeedback_Publisher.publish(ros_translate::FingerFeedbackMsg_PB2ROS(msg));
}

void captoglove_ros::publishROSInfoToTerminal(QString info){
    QString Text = QString("%1 [INFO]" + info).arg(QDateTime::currentDateTime().toString("hh:mm:ss.zzz"));
    std::cout << "\033[34m" << Text.toStdString() << "\033[0m" << std::endl;

}

void captoglove_ros::publishROSErrorToTerminal(QString error){
    QString Text = QString("[%1] Error log: [ROS] " + error).arg(QDateTime::currentDateTime().toString("hh:mm:ss.zzz"));
    std::cout << "\033[31m" << Text.toStdString() << "\033[0m" << std::endl;
}


void captoglove_ros::publishInfoToTerminal(QString info){
    std::cout <<"\033[33m" << info.toStdString() << "\033[0m" << std::endl;
    //publishToROS(ros_translate::log_info, info);
}


void captoglove_ros::publishWarningToTerminal(QString warning){
    std::cout <<"\033[33m" << warning.toStdString() << "\033[0m" << std::endl;
    //publishToROS(ros_translate::log_warning, warning);
}

void captoglove_ros::publishErrorToTerminal(QString error){
    std::cout <<"\033[31m" << error.toStdString() << "\033[0m" << std::endl;
    //publishToROS(ros_translate::log_error, error);
}

void captoglove_ros::publishFatalToTerminal(QString fatal){
    std::cout <<"\033[1m\033[31m" << fatal.toStdString() << "\033[0m" << std::endl;
    //publishToROS(ros_translate::log_fatal, fatal);
}

//void captoglove_ros::publishToROS(ros_translate::LogType type, QString Text){
    //m_robotLogPublisher.publish()
//}

/*
    void captoglove_ros::publishAPIInfoToTerminal(QString info){
        QString Text = QString("[%1] Info log: [API] "+info).arg(QDateTime::currentDateTime().toString("hh:mm:ss dd/MM/yyyy"));
        std::cout <<"\033[34m"<< Text.toStdString() << "\033[0m" << std::endl;
        publishToROS(ros_translate::log_info, Text);
    }

    void captoglove_ros::publishAPIErrorToTerminal(QString error){
        QString Text = QString("[%1] Error log: [API] "+error).arg(QDateTime::currentDateTime().toString("hh:mm:ss dd/MM/yyyy"));
        std::cout <<"\033[31m"<< Text.toStdString()<< "\033[0m" << std::endl;
        publishToROS(ros_translate::log_error, Text);
    }


    void captoglove_ros::publishInfoToTerminal(QString info){
        std::cout <<"\033[34m" << info.toStdString() << "\033[0m" << std::endl;
        publishToROS(ros_translate::log_info, info);
    }

*/

}