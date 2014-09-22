#include "ros/ros.h"
#include "cmovecontrol.h"
#include "util.h"
int main(int argc, char** argv)
{
  ros::init(argc, argv, "robot_plan_and_move");
  ros::NodeHandle nh;

  float x[10],y[10],kDiff[10];
  char valid[10],flag[10];
  for(int i=0;i<10;i++)valid[i]=1;
  for(int i=0;i<10;i++)flag[i]=0;
  for(int i=0;i<10;i++)kDiff[i]=0.0;

  x[0]=0;
  y[0]=0;

  x[1]=1;
  y[1]=1;

  x[2]=2;
  y[2]=2;

  x[3]=3;
  y[3]=3;

  x[4]=4;
  y[4]=4;

  x[5]=5;
  y[5]=3;

  x[6]=6;
  y[6]=2;

  x[7]=7;
  y[7]=1;

  x[8]=8;
  y[8]=0;

  x[9]=9;
  y[9]=-1;

//  Smooth(x,y,4,10,valid);
//  DetectCornerPoint(x,y,kDiff,flag,valid,10,3,0.2);
//  std::vector <Point> aa=FilterCornerPoint(x,y,kDiff,flag,10,3,0.6);

//  for(int i=0;i<10;i++)
//  {
//      cout<<i<<" flag:"<<(int)flag[i]<<" kDiff:"<<kDiff[i]<<endl;
//  }
//  for(int i=0;i<aa.size();i++)
//  {
//      cout<<"x:"<<aa[i].x<<" y:"<<aa[i].y<<endl;
//  }




  CMoveControl ex;
//  ex.Cmd("pass 0 1 2 4");
//  ex.RobotTurn(xform::pi_2);

//  Point pp;
//  pp.x=2.0;
//  pp.y=1.0;

//  Point p1;
//  p1.x=0.0;
//  p1.y=0.0;

//  Point p2;
//  p2.x=0.0;
//  p2.y=10.0;

//  ROS_ERROR("line dist:%f",PointToLineDist(pp,p1,p2));



  ros::spin();
  return (0);
}
