#include <ros/ros.h>
#include <ros/package.h>
#include <signal.h>

#include <captogloveapi.h>
#include <logger.h>
#include <ros_translate.h>

#include <QtCore>
#include <QObject>
#include <QDebug>
#include <QString>
#include <time.h>
#include <math.h>
#include <string>

namespace captoglove_ros{

class captoglove_ros : public QThread
{
    Q_OBJECT
    public:
        explicit captoglove_ros(int argc, char** argv);
        ~captoglove_ros();
        bool init();
        void run();
        static void shutdownHandler(int signum);

    private slots:
        void publishROSInfoToTerminal                   (QString info);
        void publishROSErrorToTerminal                  (QString error);

        void publishAPIInfoToTerminal                   (QString info);
        void publishAPIErrorToTerminal                  (QString error);

        void publishInfoToTerminal                      (QString info);
        void publishWarningToTerminal                   (QString warning);
        void publishAPIErrorToTerminal                  (QString error);
        void publishFatalToTerminal                     (QString fatal);

        void on_fingerStatesUpdated                     (captoglove_v1::FingerFeedbackMsg);
        void on_batteryLevelUpdated                     (captoglove_v1::BatteryLevelMsg);
        void on_deviceInfoUpdated                       (captoglove_v1::DeviceInformationMsg);

    private:

        int m_init_argc;
        char** m_init_argv;

        CaptogloveAPI                                   *m_captogloveAPI

        // Publishers -> data from captoglove
        ros::Publisher m_FingerFeedbackMsg_Publisher;
        ros::Publisher m_BatteryLevelMsg_Publisher;
        ros::Publisher m_DeviceInfoMsg_Publisher;

        // Subscriber -> commands to captoglove
        ros::Subscriber m_FingerCommandMsg_Subscriber;

        }

}