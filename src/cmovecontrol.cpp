#include "cmovecontrol.h"

CMoveControl::CMoveControl():
    privateNh("~")
{
    privateNh.param("subPoseName", subPoseName, string("robot_pose"));
    privateNh.param("adVelName", adVelName, string("cmd_vel"));
    privateNh.param("nodeFilePath", nodeFilePath, string("/home/chenyingfeng/workspace/nodes.txt"));
    privateNh.param("edgeFilePath", edgeFilePath, string("/home/chenyingfeng/workspace/edges.txt"));
    privateNh.param("laserTopicName",laserTopicName,string("scan"));
    privateNh.param("opcServiceName",opcServiceName,string("opc_service"));

    privateNh.param("minVx", minVx, 0.02);
    privateNh.param("maxVx", maxVx, 0.8);

    privateNh.param("minVy", minVy, 0.01);
    privateNh.param("maxVy", maxVy, 0.1);

    privateNh.param("minVr", minVr, 0.05);
    privateNh.param("maxVr", maxVr, 0.5);

    privateNh.param("dx_in", dx_in, 2.5);
    privateNh.param("dy_in", dy_in, 2.0);

    privateNh.param("dx_out", dx_out, 1.5);
    privateNh.param("dy_out", dy_out, 2.0);

    privateNh.param("distCornerPoint", distCornerPoint, 2.75);
    privateNh.param("tolerateDist", tolerateDist, 0.07);

    privateNh.param("controlTimeUs",controlTimeUs,100000);
    privateNh.param("backCount", backCount, 200);
    privateNh.param("forwardCount",forwardCount,200);
    privateNh.param("adjustCount", adjustCount, 100);

    privateNh.param("linefitError", linefitError, 0.005);
    privateNh.param("linefitAngle", linefitAngle, xform::pi/180*75);
    privateNh.param("linefitR", linefitR, 15);
    privateNh.param("linefitFilterR", linefitFilterR, 30);

    controlThread=NULL;
    laserReady=false;
    poseReady=false;

    opcClient=nh.serviceClient<we_navigation::OPC>(opcServiceName);

    robotPoseSub=nh.subscribe(subPoseName,100,&CMoveControl::PoseReceived,this);
    laserSub=nh.subscribe(laserTopicName,1,&CMoveControl::LaserReceive,this);
    cmdSub=nh.subscribe("navigation_cmd",100,&CMoveControl::Cmd,this);

    cmdVelPub=nh.advertise<geometry_msgs::Twist>(adVelName, 20);
    posStatePub=nh.advertise<std_msgs::String>("navigation_state",20);
    relayCmdPub=nh.advertise<std_msgs::String>("base_cmd",20);

    candidatePointPub=nh.advertise<nav_msgs::GridCells>("candidate_points", 1);
    cornerPointPub=nh.advertise<nav_msgs::GridCells>("corner_points", 1);

    curTask=TASK_NO;
    taskStep=GO_STOP;
    chargeStep=CHARGE_BEGIN;

    afterCharge=false;
    isOnChargeTask=false;
    goNext=false;
    isOnTask=false;
    pauseTask=false;

    countTime=0;
    obstacleCount=0;

    workMode=WORK_MODE;

    robotPoseNew.x=1.0;
    robotPoseNew.y=1.0;
    robotPoseNew.theta=0.5;

    if(LoadMap())
        controlThread=new boost::thread(boost::bind(&CMoveControl::MoveThread,this));
    else
        ROS_ERROR("Init map failed,please check!");
}
CMoveControl::~CMoveControl()
{
    if(controlThread)
    {
        controlThread->join();
        delete controlThread;
        controlThread=NULL;
    }


}

void CMoveControl::PublishCmdVel(float x,float y,float r)
{
    geometry_msgs::Twist cmdVel;
    cmdVel.linear.x = x;
    cmdVel.linear.y = y;
    cmdVel.angular.z = r;
    cmdVelPub.publish(cmdVel);

}

void CMoveControl::Cmd(const std_msgs::StringConstPtr cmdStr)
{
    string cmd(cmdStr->data.c_str());
    ROS_INFO("Receive cmd:%s",cmd.c_str());
    if(cmd.find("pass ")==0)
    {
        if(isOnChargeTask)
        {
            ROS_ERROR("Now is on charge task!ignore this cmd!");
            return;
        }
        if(isOnTask)
        {
            ROS_WARN("Now is on a task,this task will be stop!");
            RobotStop();
            goNext=false;
            isOnTask=false;
            curTask=TASK_NO;
        }
        step=0;
        aimPathNodes.clear();
        workNodesIndex.clear();
        vector <unsigned int> aimPathIndex;
        stringstream ss(cmd.substr(5));
        unsigned int id,type;
        if(ss>>type)
        {
            if (type!=0&& type!=1)
            {
                ROS_ERROR("Navigation type is not right!");
                return;
            }
        }
        else
        {
            ROS_ERROR("Cmd is not right!");
            return;
        }
        while(ss>>id)
        {
            aimPathIndex.push_back(id);
            cout<<"get id:"<<id<<endl;
        }
        if(aimPathIndex.size()==0)
        {
            ROS_ERROR("Cmd is not right,No aim node index!");
            return;
        }
        for(int i=0;i<aimPathIndex.size();i++)
        {
            cout<<aimPathIndex[i];
            if(workMode==WORK_MODE)
            {
                if(pathPlanModel.GetNode(aimPathIndex[i]).type==TYPE_WORK_POINT)
                    workNodesIndex.push_back(aimPathIndex[i]);
            }
        }
        cout<<endl;

       PathPlan(aimPathIndex,type);
    }
    else if(cmd.find("gonext")==0)
    {
        if(!isOnTask)
        {
            ROS_WARN("Now is not on task,ignore this cmd!");
            return;
        }
        else
          goNext=true;

    }
    else if(cmd.find("workmode ")==0)
    {
        stringstream ss(cmd.substr(9));
        unsigned int type;
        if(ss>>type)
        {
            if(type==0)
                workMode=WORK_MODE;
            else if(type==1)
                workMode=CHARGE_MODE;
            else if(type==2)
                workMode=TEST_MODE;
            else
            {
                ROS_ERROR("Mode set error!");
                return;
            }

        }
        else
        {
            ROS_ERROR("Mode set error!");
            return;
        }

    }
    else if(cmd.find("stop")==0)
    {
        if(isOnChargeTask)
            ROS_WARN("Now is on charge! This operate may cause problem!");
        if(!isOnTask)
        {
            ROS_WARN("Now is not on task,ignore this cmd!");
            return;
        }
        else
        {
            ROS_INFO("Abandon this task!");
            RobotStop();

            curTask=TASK_NO;
            taskStep=GO_STOP;
            chargeStep=CHARGE_BEGIN;
            aimPathNodes.clear();
            workNodesIndex.clear();
            isOnTask=false;
            goNext=false;

        }

    }
    else if(cmd.find("charge on")==0)
    {
          curTask=TASK_ON;
          taskStep=GO_CHARGE;
          chargeStep=CHARGE_BEGIN;
          isOnTask=true;

    }
    else if(cmd.find("charge off")==0)
    {
        if(isOnChargeTask)
        {
          afterCharge=true;
        }
        else
        {
            ROS_ERROR("Not on charge task, so ignore this cmd!");
            return;
        }
    }
    else if(cmd.find("pause")==0)
    {
        if(isOnChargeTask)
            ROS_WARN("Now is on charge! This operate may cause problem!");
        if(!isOnTask)
        {
            ROS_WARN("Now is not on task,ignore this cmd!");
            return;
        }
        else
        {
            RobotStop();
            if(!pauseTask)
            {
                pauseTask=true;
                ROS_INFO("Pause this task!");
            }
            else
                ROS_WARN("Task now is already paused!");

        }

    }
    else if(cmd.find("resume")==0)
    {
        if(isOnChargeTask)
            ROS_WARN("Now is on charge! This operate may cause problem!");
        if(!isOnTask)
        {
            ROS_WARN("Now is not on task,ignore this cmd!");
            return;
        }
        else
        {
            if(pauseTask)
            {
                pauseTask=false;
                ROS_INFO("Resume this task!");
            }
            else
                ROS_WARN("Task now is already resumed!");

        }

    }
    else
    {
        ROS_WARN("Unknown Cmd!");
    }
//    stringstream cmdStr(cmd);

}
void CMoveControl::PoseReceived(const geometry_msgs::PointStampedConstPtr pose)
{
    poseMutex.lock();
    robotPoseNew.x=pose->point.x;
    robotPoseNew.y=pose->point.y;
    robotPoseNew.theta=pose->point.z;
    poseReady=true;
    poseMutex.unlock();
}
bool CMoveControl::PathPlan(vector <unsigned int> aimPathIndex,int mode)
{

    aimPathNodes.clear();
    uint edgeId,nodeId;
    if(pathPlanModel.ISOnNode(robotPose.x,robotPose.y,nodeId))
    {
        ROS_INFO("Robot now is on node %u",nodeId);
        vector<unsigned int> path;
        float pathLen;

        path=pathPlanModel.GetAimPath(nodeId,pathLen,aimPathIndex,mode);
        cout<<pathLen<<endl;
        cout<<path.size()<<endl;
        for(int i=0;i<path.size();i++)
            cout<<path[i]<<"-->";

        vector <unsigned int> aimDetailPathIndex;
        aimDetailPathIndex=pathPlanModel.GetDetailPath(path);

        cout<<"detail way:";
        for(int i=0;i<aimDetailPathIndex.size();i++)
        {
            aimPathNodes.push_back(pathPlanModel.GetNode(aimDetailPathIndex[i]));
            cout<<aimDetailPathIndex[i];
        }
        cout<<endl;

        cout<<endl;
        for(int i=0;i<aimPathNodes.size();i++)
            cout<<"("<<aimPathNodes[i].x<<" "<<aimPathNodes[i].y<<" "<<aimPathNodes[i].r<<")->";

        cout.flush();

        if(aimPathNodes.size()<=1)
        {
            ROS_WARN("The plan path node is less than two,so stop the task!");
            return false;
        }
        curTask=TASK_NEW;
        taskStep=GO_INIT;
        isOnTask=true;

    }
    else if(pathPlanModel.GetNeareatNode(robotPose.x,robotPose.y,nodeId,edgeId))
    {

        vector<unsigned int> path1,path2,path;
        float pathLen1,pathLen2;

        unsigned int a=pathPlanModel.GetEdge(edgeId).nodeA;
        unsigned int b=pathPlanModel.GetEdge(edgeId).nodeB;

        ROS_INFO("Robot now is on edge %u between node %u and %u",edgeId,a,b);

        path1=pathPlanModel.GetAimPath(pathPlanModel.GetEdge(edgeId).nodeA,pathLen1,aimPathIndex,mode);
        path2=pathPlanModel.GetAimPath(pathPlanModel.GetEdge(edgeId).nodeB,pathLen2,aimPathIndex,mode);

        if(mode==0)
        {
            pathLen1=pathLen1+sqrt((robotPose.x-pathPlanModel.GetNode(a).x)*(robotPose.x-pathPlanModel.GetNode(a).x)+(robotPose.y-pathPlanModel.GetNode(a).y)*(robotPose.y-pathPlanModel.GetNode(a).y));
            pathLen2=pathLen2+sqrt((robotPose.x-pathPlanModel.GetNode(b).x)*(robotPose.x-pathPlanModel.GetNode(b).x)+(robotPose.y-pathPlanModel.GetNode(b).y)*(robotPose.y-pathPlanModel.GetNode(b).y));
        }
        else
        {
            pathLen1=pathLen1+2*sqrt((robotPose.x-pathPlanModel.GetNode(a).x)*(robotPose.x-pathPlanModel.GetNode(a).x)+(robotPose.y-pathPlanModel.GetNode(a).y)*(robotPose.y-pathPlanModel.GetNode(a).y));
            pathLen2=pathLen2+2*sqrt((robotPose.x-pathPlanModel.GetNode(b).x)*(robotPose.x-pathPlanModel.GetNode(b).x)+(robotPose.y-pathPlanModel.GetNode(b).y)*(robotPose.y-pathPlanModel.GetNode(b).y));
        }
        cout<<pathLen1<<" "<<pathLen2<<endl;
        for(int i=0;i<path1.size();i++)
            cout<<path1[i]<<"->";
        cout<<endl;
        for(int i=0;i<path2.size();i++)
            cout<<path2[i]<<"->";
        cout<<endl;
        if(pathLen1>pathLen2)
        {
            path=path2;
        }
        else
        {
            path=path1;
        }
        cout<<path.size()<<endl;
        for(int i=0;i<path.size();i++)
            cout<<path[i]<<"-->";

        CNode temp;
        temp.x=robotPose.x;
        temp.y=robotPose.y;
        temp.r=robotPose.theta;
        temp.id=std::numeric_limits<unsigned int>::max();
        temp.type=TYPE_KEY_POINT;
        aimPathNodes.push_back(temp);

        vector <unsigned int> aimDetailPathIndex;
        aimDetailPathIndex=pathPlanModel.GetDetailPath(path);

        for(int i=0;i<aimDetailPathIndex.size();i++)
            aimPathNodes.push_back(pathPlanModel.GetNode(aimDetailPathIndex[i]));

        if(mode)
            aimPathNodes.push_back(temp);

        cout<<endl;
        for(int i=0;i<aimPathNodes.size();i++)
            cout<<"("<<aimPathNodes[i].x<<" "<<aimPathNodes[i].y<<" "<<aimPathNodes[i].r<<")->";

        cout.flush();

        if(aimPathNodes.size()<=1)
        {
            ROS_WARN("The plan path node is less than two,so stop the task!");
            return false;
        }
        curTask=TASK_NEW;
        taskStep=GO_INIT;
        isOnTask=true;
    }
    else
    {
        ROS_ERROR("Error,The robot now is not on any path!");
        return false;
    }
    return true;

}
void CMoveControl::LaserReceive(const sensor_msgs::LaserScanConstPtr laserData)
{
    laserMutex.lock();
    this->laserData=*laserData;
    laserReady=true;
    laserMutex.unlock();
}
bool CMoveControl::LoadMap()
{
    if(pathPlanModel.LoadNodeFile(nodeFilePath.c_str()))
    {
       ROS_INFO("Load node file from %s\n",nodeFilePath.c_str());
    }
    else
    {
        ROS_ERROR("Load node file from %s failed\n",nodeFilePath.c_str());
        return false;
    };
    if(pathPlanModel.LoadEdgeFile(edgeFilePath.c_str()))
    {
       ROS_INFO("Load edge file from %s\n",edgeFilePath.c_str());
    }
    else
    {
        ROS_ERROR("Load node file from %s failed\n",edgeFilePath.c_str());
        return false;
    }

    cout<<"********** Map Init **********"<<endl;
    cout<<"Nodes Number:"<<pathPlanModel.nodes.size()<<" Edges Number:"<<pathPlanModel.edges.size()<<endl;
    pathPlanModel.FloydWarshall();
    return true;
}
void CMoveControl::MoveThread()
{

    while(ros::ok())
    {
        if(!poseReady)
            ROS_WARN("Pose data is old!");
        poseMutex.lock();
        robotPose=robotPoseNew;
        poseReady=false;
        poseMutex.unlock();

        switch(curTask)
        {
         case TASK_NO:
//            curTask=TASK_NO;
 //           taskStep=GO_STOP;

            break;
         case TASK_NEW:
//            cout<<"New task!"<<endl;
            step=0;
            curTask=TASK_ON;
            taskStep=GO_INIT;

            break;

         case TASK_ON:
//            cout<<"On task!"<<endl;
            Navi();
            break;
        }
        usleep(controlTimeUs);
    }
}
void CMoveControl::Navi()
{

    bool ret=ChargeAdjustPose(0,0,1);
    if(ret)obstacleCount++;
    else obstacleCount=0;

    if(obstacleCount>5)
    {
        std_msgs::String str;
        str.data=string("obstacle detected!");
        posStatePub.publish(str);
        obstacleCount=0;
        RobotStop();
        ROS_WARN("Obstacle detected in front of robot!");
        return;
    }
    if(pauseTask)
    {
//        RobotStop();
        return;
    }

    switch(taskStep)
    {
     case GO_INIT:
//        cout<<"GO_INIT"<<endl;
        if(step==aimPathNodes.size()-1)
        {
            RobotStop();
            step=0;
            taskStep=TASK_NO;
            curTask=GO_STOP;
            isOnTask=false;
        }
        else
        {
            Point a,b;
            a.x=aimPathNodes[step].x;
            a.y=aimPathNodes[step].y;

            b.x=aimPathNodes[step+1].x;
            b.y=aimPathNodes[step+1].y;

            targetSlope=SlopeLine(a,b);
            originalNode=aimPathNodes[step];
            targetNode=aimPathNodes[step+1];
//            cout<<"targetSlope:"<<targetSlope<<endl;
            taskStep=GO_TURN;
            curTask=TASK_ON;
        }

        break;
     case GO_TURN:
//            cout<<"GO_TURN"<<endl;
            if(RobotTurn(targetSlope))
            {

                taskStep=GO_DIST;
                curTask=TASK_ON;
            }
        break;
     case GO_DIST:
 //       cout<<"GO_DIST"<<endl;

     {
        float slope=atan2(targetNode.y-robotPose.y,targetNode.x-robotPose.x);
        float dirDiff=xform::normalize(slope-robotPose.theta);

        if(fabs(dirDiff)>xform::pi/6)
        {
            taskStep=GO_TURN;
            curTask=TASK_ON;
            targetSlope=slope;
            RobotStop();
            break;
        }
        if(RobotGoLine()<0.05)
        {
            if(targetNode.type==TYPE_CHARGE_POINT)
            {
                if(workMode==CHARGE_MODE)//go turn to right direction
                {
                    taskStep=GO_CHARGE;
                    curTask=TASK_ON;
                    chargeStep=CHARGE_BEGIN;
                    step++;
                }
                else
                {
                    taskStep=GO_INIT;
                    curTask=TASK_ON;
                    step++;

                }

            }
            else if(targetNode.type==TYPE_WORK_POINT)
            {
                if(workMode==WORK_MODE)
                {
                    if(!IsVisited(targetNode.id))
                    {
                        taskStep=GO_ACTION;
                        curTask=TASK_ON;
                    }
                    else
                    {
                        taskStep=GO_INIT;
                        curTask=TASK_ON;
                        step++;

                    }
                }
                else
                {

                    taskStep=GO_INIT;
                    curTask=TASK_ON;
                    step++;

                }

            }
            else
            {
              taskStep=GO_INIT;
              curTask=TASK_ON;
              step++;
            }

        }
        break;
    }
     case GO_ACTION:
        if(RobotTurn(targetNode.r))
        {
            goNext=false;
            taskStep=GO_WAIT;
            curTask=TASK_ON;
        }
        break;
     case GO_WAIT:
        if(goNext)
        {
            goNext=false;
            taskStep=GO_INIT;
            curTask=TASK_ON;
            step++;
        }
        else
        {
            char str[128];
            sprintf(str,"OnNode:%u",targetNode.id);
            std_msgs::String state;
            state.data=string(str);
            posStatePub.publish(state);
        }

        break;

    case GO_CHARGE:
        RobotCharge();
        break;

    }
}
void CMoveControl::RobotCharge()
{
    switch(chargeStep)
    {
      case CHARGE_BEGIN:
        chargeStep=CHARGE_TURN;
        isOnChargeTask=true;
        break;
      case CHARGE_TURN:
        if(RobotTurn(targetNode.r))
        {
            ROS_INFO("Turn direction ok!");
            chargeStep=CHARGE_OPENDOOR;
        }
        break;
      case CHARGE_OPENDOOR:
        if(OpcOperate(0))
        {
            ROS_INFO("Opc open door ok!");
             countTime=0;
             if(!afterCharge)
              chargeStep=CHARGE_BACK_DOOR;
             else
              chargeStep=CHARGE_FORWARD_DOOR;
        }
        else
            ROS_ERROR("Opc open door failed!");
        break;
      case CHARGE_BACK_DOOR:
        countTime++;
        if(countTime<backCount) //todo list
            PublishCmdVel(-0.1,0,0);
        else
        {
            ROS_INFO("Back into room ok!");
            RobotStop();
            countTime=0;
            chargeStep=CHARGE_CLOSEDOOR;
        }
        break;
      case CHARGE_CLOSEDOOR:
        if(OpcOperate(1))
        {
            ROS_INFO("Opc close door ok!");
            if(!afterCharge)
                chargeStep=CHARGE_ADJUST_ONE;
            else
                chargeStep=CHARGE_FINISH;
        }
        else
            ROS_ERROR("Opc close door failed!");
        break;
      case CHARGE_ADJUST_ONE:
        if(ChargeAdjustPose(dx_in,dy_in))
        {
            RobotStop();
            ROS_INFO("Pose one adjust ok!");
            chargeStep=CHARGE_OPC_START;
        }
        break;
    case CHARGE_OPC_START:
         RelayOperate(1);
         sleep(3);
        if(OpcOperate(2))
        {
            ROS_INFO("Now is on charge!");
            chargeStep=CHARGE_ING;
        }
        else
        {
            ROS_INFO("Opc charge failed,try adjust again!");
            RelayOperate(0);
            chargeStep=CHARGE_ADJUST_FORWARD;
        }
      break;
     case CHARGE_ING:
        if(afterCharge)
        {
            ROS_INFO("Stop charge!");
            chargeStep=CHARGE_OPC_END;
        }
        break;
     case CHARGE_OPC_END:
        if(OpcOperate(3))
        {
            ROS_INFO("Opc stop charge ok!");
            RelayOperate(0);
            chargeStep=CHARGE_ADJUST_FORWARD;//tolist add delay
            countTime=0;
        }
        else
            ROS_INFO("Opc end charge failed!");
        break;
     case CHARGE_ADJUST_FORWARD:
        countTime++;
        if(countTime<adjustCount)
            PublishCmdVel(0.1,0.0,0.0);
        else
        {
            ROS_INFO("At the adjust pose!");
            RobotStop();
            if(!afterCharge)
                chargeStep=CHARGE_ADJUST_ONE;
            else
                chargeStep=CHARGE_ADJUST_TWO;
        }

        break;

     case CHARGE_ADJUST_TWO:
        if(ChargeAdjustPose(dx_out,dy_out))
        {
            RobotStop();
            ROS_INFO("At the pose in room for get out!");
            chargeStep=CHARGE_OPENDOOR;
        }
        break;
     case CHARGE_FORWARD_DOOR:
        countTime++;
        if(countTime<forwardCount)
            PublishCmdVel(0.1,0.0,0.0);
        else
        {
            ROS_INFO("At the pose out of room!");
            RobotStop();
            countTime=0;
            chargeStep=CHARGE_CLOSEDOOR;
        }
        break;
     case CHARGE_FINISH:
        ROS_INFO("Charge finish!");
        taskStep=GO_INIT;
        curTask=TASK_ON;
        isOnChargeTask=false;
        break;
    }
}

bool CMoveControl::ChargeAdjust()
{
   laserMutex.lock();
   sensor_msgs::LaserScan ldata=laserData;
   laserMutex.unlock();

   float angle_min=ldata.angle_min;
   float angle_max=ldata.angle_max;
   float angle_increment=ldata.angle_increment;
   int num=ldata.ranges.size();

   float *x=new float[num];
   float *y=new float[num];
   float angle=angle_min;

   int begin=0;
   int end=num-1;
   int windows=10;
   float wallLen=3.0;
   float wx=2.0;
   float wy=2.0;

   for(int i = 0; i < num; i++)
   {
       if(ldata.ranges[i] < 0.01)
       {
           x[i] = 0.0;
           y[i] = 0.0;
       }
       else
       {
           x[i] = cos(angle) * ldata.ranges[i];
           y[i] = sin(angle) * ldata.ranges[i];
       }
       angle=angle+angle_increment;
   }

   float *kDiff=new float[num];
   for(int i=0;i<num;i++)
       kDiff[i]=-0.01;

   for(int i=begin;i<=end-2*windows;i++)
   {
       if(ldata.ranges[i]>0.0 && ldata.ranges[i+2*windows]>0.0 && ldata.ranges[i+windows]>0.01)
       {
           float k1=atan2(y[i+windows]-y[i],x[i+windows]-x[i]);
           float k2=atan2(y[i+2*windows]-y[i+windows],x[i+2*windows]-x[i+windows]);

           kDiff[i+windows]=fabs(k1-k2);
       }
   }
   float max=-1.0;
   int left=-1;
   int right=-1;
   for(int i=begin+windows;i<=end-windows;i++)
   {
       if(kDiff[i]>max)
       {
           max=kDiff[i];
           left=i;
       }

   }

   if(max>xform::pi/6)
       ROS_INFO("One line node detected!");
   else
   {
       ROS_ERROR("first line node detected failed!");

       delete []x;
       delete []y;
       delete []kDiff;
       return false;
   }
   int j1=left-windows;
   if(j1<begin+windows)j1=begin+windows;
   int j2=left+windows;
   if(j2>end-windows)j2=end-windows;

   for(int i=j1;i<j2;i++)
       kDiff[i]=-0.01;

   max=-1.0;
   for(int i=begin+windows;i<=end-windows;i++)
   {
       if(kDiff[i]>max)
       {
           max=kDiff[i];
           right=i;
       }

   }
   if(max>xform::pi/6)
       ROS_INFO("two line node detected!");
   else
   {
       ROS_ERROR("second line node detected failed!");
       delete []x;
       delete []y;
       delete []kDiff;
       return false;
   }
   if(left>=right)
   {
       ROS_WARN("No line end points detected!");
       delete []x;
       delete []y;
       delete []kDiff;
       return false;
   }

   Point pointA,pointB;
   pointA.x=x[left];
   pointA.y=y[left];

   pointB.x=x[right];
   pointB.y=y[right];

   float dist=(pointA.x-pointB.x)*(pointA.x-pointB.x)+(pointA.y-pointB.y)*(pointA.y-pointB.y);
   dist=sqrt(dist);
   if(!(dist>wallLen-0.3 && dist<wallLen+0.3))
   {
       ROS_WARN("Wall length check error!");
       return false;
   }
//   int begin,end;

//   for(int i=1;i<num;i++)
//   {
//       if(fabs(ldata.ranges[i]-ldata.ranges[i-1])>1.0)
//       {
//           begin=i;
//       }

//   }

//   for(int i=num-1;i>0;i--)
//   {

//       if(fabs(ldata.ranges[i]-ldata.ranges[i-1])>1.0)
//       {vector <Point> ChargeAdjustPose(sensor_msgs::LaserScan &ldata)

//           end=i;
//       }
//   }

   float slope=SlopeLine(pointA,pointB);
   float dirDiff=xform::normalize(slope-xform::pi_2);
   float absDirDiff=fabs(dirDiff);

   float vx,vy,vr;
   vx=vr=vy=0.0;
   if(absDirDiff>0.05)
   {
       vr=dirDiff/absDirDiff*sqrt(absDirDiff/xform::pi)*0.2;
       PublishCmdVel(vx,vy,vr);

       delete []x;
       delete []y;
       delete []kDiff;
       return false;
   }

   float xx=pointB.x;
   float dx=xx-wx;
   if(fabs(dx)>0.05)
   {
       if(dx>0.0)
          vx=sqrt(dx)*0.2;
       else
          vx=-sqrt(dx)*0.2;
        PublishCmdVel(vx,vy,vr);

        delete []x;
        delete []y;
        delete []kDiff;
        return false;
   }

   float yy=pointA.y;
   float dy=yy-wy;
   if(fabs(dy)>0.05)
   {
       if(dy>0.0)
          vy=sqrt(dy)*0.2;
       else
          vy=-sqrt(dy)*0.2;
        PublishCmdVel(vx,vy,vr);

        delete []x;
        delete []y;
        delete []kDiff;
        return false;

   }

   delete []x;
   delete []y;
   delete []kDiff;
   return true;
}
float CMoveControl::RobotGoLine()
{
    float dist_x=sqrt((robotPose.x-targetNode.x)*(robotPose.x-targetNode.x)+(robotPose.y-targetNode.y)*(robotPose.y-targetNode.y));

    float slope=atan2(targetNode.y-robotPose.y,targetNode.x-robotPose.x);
    float dirDiff=xform::normalize(slope-robotPose.theta);
    float sgn=sign(cos(dirDiff));
    float absDirDiff=fabs(dirDiff);

    float vx=0.0;
    float vy=0.0;
    float vr=0.0;

    if(dist_x>2.0)
        vx=sgn*sqrt(dist_x)*0.8;
    else
        vx=sgn*sqrt(dist_x)*0.4;

    if(fabs(vx)>maxVx)
        vx=sgn*maxVx;
    if(fabs(vx)<minVx)
        vx=sgn*minVx;

    Point pose,pa,pb;
    pose.x=robotPose.x;
    pose.y=robotPose.y;

    pa.x=originalNode.x;
    pa.y=originalNode.y;

    pb.x=targetNode.x;
    pb.y=targetNode.y;

    float dline=PointToLineDist(pose,pa,pb);
    vy=sign(dline)*sqrt(dline)*0.08;
    if(fabs(vy)>maxVy)vy=sign(dline)*maxVy;
    if(fabs(vy)<minVy)vy=sign(dline)*minVy;

    if(absDirDiff>0.1)
       vr=sign(dirDiff)*sqrt(absDirDiff/xform::pi)*0.4;

    if(fabs(vr)>maxVr)vr=sign(dirDiff)*maxVr;
    if(fabs(vr)<minVr)vr=sign(dirDiff)*minVr;

    PublishCmdVel(vx,vy,vr);

    cout<<"***** RobotLine *****"<<endl;
    cout<<"dx:"<<dist_x<<endl;
    cout<<"dy:"<<dline<<endl;
    cout<<"dr:"<<dirDiff<<endl;
    cout<<"vx:"<<vx<<" vy:"<<vy<<" vr:"<<vr<<endl;
    return dist_x;

}
bool CMoveControl::RobotTurn(float target)
{
    float theta = xform::normalize(target - robotPose.theta);
    float angle_diff = fabs(theta);
    if (angle_diff < 0.05)
    {
      RobotStop();
      return true;
    }
    float speed=theta/angle_diff*sqrt(angle_diff/xform::pi)*0.5;
    PublishCmdVel(0,0,speed);

    cout<<"***** RobotTurn *****"<<endl;
    cout<<"dr:"<<theta<<endl;
    cout<<"vr:"<<speed<<endl;
    return false;

}

bool  CMoveControl::ChargeAdjustPose(float d_x,float d_y,int type/*=0*/)
{

    if(!laserReady)
        ROS_WARN("laser data is old!");
    sensor_msgs::LaserScan ldata;
    laserMutex.lock();
    ldata=laserData;
    laserReady=false;
    laserMutex.unlock();

    float angle_min=ldata.angle_min;
    float angle_max=ldata.angle_max;
    float angle_increment=ldata.angle_increment;
    int num=ldata.ranges.size();

//    cout<<"num:"<<num<<" angle_min:"<<angle_min<<" angle_max:"<<angle_max<<endl;
//    ROS_ERROR("num:%d angle_min:%f angle_max:%f",num,angle_min,angle_max);
    float *x=new float[num];
    float *y=new float[num];
    float angle=angle_min;
    char *valid=new char[num];
    float *kDiff=new float[num];
    char *flag=new char[num];


    for(int i = 0; i < num; i++)
    {
        valid[i]=1;
        kDiff[i]=0.0;
        flag[i]=0;
        if(ldata.ranges[i] < 0.01)
        {
            x[i] = 0.0;
            y[i] = 0.0;
            valid[i]=0;
        }
        else
        {
            x[i] = cos(angle) * ldata.ranges[i];
            y[i] = sin(angle) * ldata.ranges[i];
        }
        angle=angle+angle_increment;
    }

    if(type==1)
    {
        bool rr=false;
        int count=0;
        for(int i=0;i<num;i++)
        {
            if(valid[i])
            {
                float d_x=fabs(x[i]);
                float d_y=fabs(y[i]);
                if(d_x<=1.0 && d_y<=0.4)count++;
                if(count>=10)
                {
                    rr=true;
                    break;
                }
            }
        }

        delete []x;
        delete []y;
        delete []valid;
        delete []flag;
        delete []kDiff;
        return rr;
    }

    nav_msgs::GridCells points,cornerPoints;

    points.header.frame_id = string("laser_link");
    points.header.stamp = ros::Time::now();
    points.cell_height =0.02;
    points.cell_width = 0.02;

    cornerPoints.header.frame_id = string("laser_link");
    cornerPoints.header.stamp = ros::Time::now();
    cornerPoints.cell_height =0.05;
    cornerPoints.cell_width = 0.05;

 //   Smooth(x,y,30,num,valid);
    DetectCornerPoint(x,y,kDiff,flag,valid,num,linefitR,linefitError);
//    ROS_ERROR("finish deteect corn");
//    std::vector <Point> aa;
    vector <int> index;
    std::vector <Point> aa=FilterCornerPoint(index,x,y,kDiff,flag,num,linefitFilterR,linefitAngle);
//    ROS_ERROR("finish filt corn");

    points.cells.resize(aa.size());
    for(int i=0;i<aa.size();i++)
    {
        points.cells[i].x=aa[i].x;
        points.cells[i].y=aa[i].y;
        points.cells[i].z=0;
    }
    bool detect=false;
    Point A,B;
    for(int i=0;i<index.size();i++)
    {
        cout<<"index:"<<index[i]<<endl;
        for(int j=i+1;j<index.size();j++)
        {
            float dist=sqrt((x[index[i]]-x[index[j]])*(x[index[i]]-x[index[j]])+(y[index[i]]-y[index[j]])*(y[index[i]]-y[index[j]]));
//            ROS_ERROR("dist:%f i:%d j:%d",dist,i,j);
            if(fabs(dist-distCornerPoint)<tolerateDist)
            {
                ROS_ERROR("detectd->dist:%f i:%d j:%d",dist,i,j);
                cornerPoints.cells.resize(2);

                A.x=cornerPoints.cells[0].x=x[index[i]];
                A.y=cornerPoints.cells[0].y=y[index[i]];
                cornerPoints.cells[0].z=0.0;

                B.x=cornerPoints.cells[1].x=x[index[j]];
                B.y=cornerPoints.cells[1].y=y[index[j]];
                cornerPoints.cells[1].z=0.0;

                cornerPointPub.publish(cornerPoints);
                detect=true;
                break;


            }
            if(detect)break;

        }
    }


//    vector <int> index;
//    for(int i=0;i<num;i++)
//    {
//        if(flag[i])
//            index.push_back(i);
//    }
//    obstacles.cells.resize(index.size());
//    for(int i=0;i<index.size();i++)
//    {
//        obstacles.cells[i].x=x[index[i]];
//        obstacles.cells[i].y=y[index[i]];
//        obstacles.cells[i].z=0;
//    }
    candidatePointPub.publish(points);

    delete []x;
    delete []y;
    delete []valid;
    delete []flag;
    delete []kDiff;

    if(detect)
    {
        float vx=0.0,vy=0.0,vr=0.0;
        float theta = atan((A.x-B.x)/(A.y-B.y));
        float angle_diff = fabs(theta);
    //    cout<<"angle:"<<angle_diff<<endl;
        if (angle_diff>0.05)
        {
           vr=-theta/angle_diff*sqrt(angle_diff/xform::pi)*0.5;
           PublishCmdVel(vx,vy,vr);
           return false;
        }

//        float dy=d_y-fabs(A.y);
//        float absdy=fabs(dy);
//        if(fabs(absdy)>0.05)
//        {
//           vy=dy/absdy*sqrt(absdy)*0.08;
//           PublishCmdVel(vx,vy,vr);
//           return false;
//        }

        float dx=A.x-d_x;
        float absdx=fabs(dx);
        if(fabs(absdx)>0.05)
        {
           vx=dx/absdx*sqrt(absdx)*0.1;
           PublishCmdVel(vx,vy,vr);
           return false;
        }

        return true;
    }
    else
    {
        ROS_ERROR("No corner point detected!");
        return false;
    }

}
bool CMoveControl::OpcOperate(int type)
{
    we_navigation::OPC srv;
    if(type==0)
    {
        srv.request.operation.data=string("OpenDoor");
    }
    else if(type==1)
    {
        srv.request.operation.data=string("CloseDoor");
    }
    else if(type==2)
    {
        srv.request.operation.data=string("BeginCharge");
    }
    else if(type==3)
    {
        srv.request.operation.data=string("StopCharge");
    }
    else
    {
        ROS_ERROR("Wrong OPC operate type!");
        return false;
    }
    if(opcClient.call(srv))
    {
        ROS_INFO("Type: %d operate successfully",type);
        return srv.response.state;
    }
    else
    {
        ROS_ERROR("Failed to call service!");
        return false;
    }
    return true;

}
void CMoveControl::RelayOperate(int type)
{
    std_msgs::String cmdStr;
    if(type==0)
    {
        cmdStr.data=string("relay 0");
    }
    else if(type==1)
    {
        cmdStr.data=string("relay 1");
    }
    else
    {
         ROS_ERROR("Wrong Relay operate type!");
         return;
    }
    relayCmdPub.publish(cmdStr);
}
