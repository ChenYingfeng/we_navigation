#ifndef CMOVECONTROL_H
#define CMOVECONTROL_H
#include <ros/ros.h>
#include <vector>
#include <string>
#include "cnode.h"
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/GridCells.h>
#include <std_msgs/builtin_string.h>
#include <sensor_msgs/LaserScan.h>
#include "cpathplan.h"
#include "util.h"
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/recursive_mutex.hpp>
#include <math.h>
#include "xform.h"
#include "we_navigation/OPC.h"

using namespace std;
class CMoveControl
{
public:
    CMoveControl();
    ~CMoveControl();
protected:
    ros::NodeHandle nh;
    ros::NodeHandle privateNh;

    ros::Subscriber robotPoseSub;
    ros::Subscriber cmdSub;

    ros::Publisher candidatePointPub;
    ros::Publisher cornerPointPub;

    ros::Subscriber laserSub;
    boost::recursive_mutex laserMutex;
    sensor_msgs::LaserScan laserData;
    void LaserReceive(const sensor_msgs::LaserScanConstPtr laserData);

    ros::ServiceClient opcClient;
    bool OpcOperate(int type);
    ros::Publisher relayCmdPub;
    void RelayOperate(int type);
    bool afterCharge;
    bool isOnChargeTask;

    geometry_msgs::Pose2D robotPose,robotPoseNew;
    boost::recursive_mutex poseMutex;
    bool poseReady;
    void PoseReceived(const geometry_msgs::PointStampedConstPtr pose);

    boost::thread *controlThread;

    void Cmd(const std_msgs::StringConstPtr cmdStr);

    ros::Publisher cmdVelPub;
    void PublishCmdVel(float x,float y,float r);

    ros::Publisher posStatePub;

    CPathPlan pathPlanModel;
    vector <CNode> aimPathNodes;
    bool LoadMap();
    bool PathPlan(vector <unsigned int> aimPathIndex,int mode);

    string subPoseName;
    string adVelName;
    string nodeFilePath;
    string edgeFilePath;
    string laserTopicName;
    string opcServiceName;

    double minVx,maxVx;
    double minVy,maxVy;
    double minVr,maxVr;

    double dx_in,dy_in;
    double dx_out,dy_out;
    double distCornerPoint,tolerateDist;
    int backCount,forwardCount,adjustCount;
    int controlTimeUs;

    int curTask,taskStep,chargeStep;
    const static int TYPE_KEY_POINT=0;
    const static int TYPE_WORK_POINT=1;
    const static int TYPE_CHARGE_POINT=2;

    enum
    {
        TASK_NO,TASK_ON,TASK_NEW
    };
    enum
    {
        GO_INIT,GO_DIST,GO_TURN,GO_STOP,GO_ACTION,GO_WAIT,GO_CHARGE
    };
    enum
    {
        CHARGE_BEGIN,CHARGE_TURN,CHARGE_OPENDOOR,CHARGE_CLOSEDOOR,CHARGE_BACK_DOOR,CHARGE_ADJUST_ONE,
        CHARGE_ADJUST_FORWARD,CHARGE_RELAY1,CHARGE_OPC_START,CHARGE_ING,CHARGE_OPC_END,CHARGE_RELAY0,
        CHARGE_ADJUST_TWO,CHARGE_FORWARD_DOOR,CHARGE_FINISH
    };
    enum
    {
        TEST_MODE,WORK_MODE,CHARGE_MODE
    };

    int step;
    int workMode;
    CNode originalNode,targetNode;
    float targetSlope;

    bool goNext;
    bool isOnTask;
    bool pauseTask;

    void MoveThread();
    void Navi();
    float RobotGoLine();
    bool RobotTurn(float target);

    void RobotCharge();
    int obstacleCount;
    bool ChargeAdjustPose(float d_x,float d_y,int type=0);
    bool ChargeAdjust();

    double linefitError,linefitAngle;
    int linefitR,linefitFilterR;
    bool laserReady;
    int  countTime;

    void RobotStop()
    {
        PublishCmdVel(0.0,0.0,0.0);
    }

    vector <unsigned int> workNodesIndex;
    bool IsVisited(unsigned int index)
    {
        for(int i=0;i<workNodesIndex.size();i++)
        {
            if(index==workNodesIndex[i])
            {
                std::vector<unsigned int>::iterator it = workNodesIndex.begin()+i;
                workNodesIndex.erase(it);
                return false;
            }
        }
        return true;
    }
};

#endif // CMOVECONTROL_H
