#include "util.h"

/***************************************/
float SlopeLine(Point p1,Point p2)
{
    return atan2(p2.y-p1.y,p2.x-p1.x);
}
float PointToLineDist(Point pp,float k,float b)
{
    return fabs(k*pp.x-pp.y+b)/sqrt(k*k+1);
}
float PointToLineDist(Point pp,Point p1,Point p2)
{
    float cross=(p2.x - p1.x) * (pp.x - p1.x) + (p2.y - p1.y) * (pp.y - p1.y);
    if(cross<=0)
    {

        float dist=sqrt((pp.x - p1.x) * (pp.x - p1.x) + (pp.y - p1.y) * (pp.y - p1.y));
        if(dist>0.2)ROS_ERROR("Robot pose is abnormal!");
//        return sqrt((pp.x - p1.x) * (pp.x - p1.x) + (pp.y - p1.y) * (pp.y - p1.y));
        return 0.0;
    }

    float dd=(p2.x-p1.x)*(p2.x-p1.x)+(p2.y-p1.y)*(p2.y-p1.y);
    if(cross>=dd)
    {
        float dist=sqrt((pp.x-p2.x)*(pp.x-p2.x)+(pp.y-p2.y)*(pp.y-p2.y));
        if(dist>0.2)ROS_ERROR("Robot pose is abnormal!");
//        return sqrt((pp.x-p2.x)*(pp.x-p2.x)+(pp.y-p2.y)*(pp.y-p2.y));
        return 0.0;
    }

    float r=cross/dd;
    float px=p1.x+(p2.x-p1.x)*r;
    float py=p1.y+(p2.y-p1.y)*r;
    float angle1=SlopeLine(p1,p2);
    float angle2=SlopeLine(p1,pp);
    if(angle1>=angle2)
        return -sqrt((pp.x-px)*(pp.x-px)+(py-pp.y)*(py-pp.y));
    else
        return sqrt((pp.x-px)*(pp.x-px)+(py-pp.y)*(py-pp.y));

}
bool LineFit(float &kk,float &bb,float &ee,std::valarray <float> &data_x,std::valarray <float> &data_y)
{
    if(data_x.size()<=1)
    {
        ROS_WARN("Line fit point is less than one!");
        return false;

    }
    double A =0.0;
    double B =0.0;
    double C =0.0;
    double D =0.0;

    A = (data_x*data_x).sum();
    B = data_x.sum();
    C = (data_x*data_y).sum();
    D = data_y.sum();
    float k,b,tmp =0;
    if(tmp=(A*data_x.size()-B*B))
    {
        k = (C*data_x.size()-B*D)/tmp;
        b = (A*D-C*B)/tmp;
    }
    else
    {
        k=0;
        b=B/data_x.size();
        ROS_WARN("k is zero!");
    }
    kk=k;
    bb=b;
    std::valarray <float> error(data_x.size());
    error=data_x*k+b-data_y;
    ee=(error*error).sum()/error.size();
    return true;
}
void Smooth(float * const x,float * const y,uint windowSize,uint num,char * const valid)
{
    uint r=windowSize/2;
    for(int i=0;i<num;i++)
    {
        float sumX=0.0;
        float sumY=0.0;
        int n=0,count=0;
        for(int j=i;j>=0;j--)
        {
            if(valid[j])
            {
                sumX=sumX+x[j];
                sumY=sumY+y[j];
                n++;
                count++;
            }
            if(n>=r)
                break;

        }
        n=0;
        for(int j=i;j<num;j++)
        {
            if(valid[j])
            {
                sumX=sumX+x[j];
                sumY=sumY+y[j];
                n++;
                count++;
            }
            if(n>=r)
                break;

        }

        x[i]=sumX/count;
        y[i]=sumY/count;
    }


}

void DetectCornerPoint(float * const x,float *const y,float * const kDiff,char * const flag,char * const valid,uint num,uint r,float threshold)
{


    for(int i=r;i<num-r;i++)
    {
        int n=0;
        float xx[num],yy[num];

        for(int j=i;j>=0;j--)
        {
            if(valid[j])
            {
                xx[n]=x[j];
                yy[n]=y[j];
                n++;
            }
            if(n>=r)break;
        }

        std::valarray <float> left_x(xx,n),left_y(yy,n);
        float lk,lb,le;
        bool ret;
        ret=LineFit(lk,lb,le,left_x,left_y);

//        cout<<"i:"<<i<<" lk:"<<lk<<" lb:"<<lb<<" le:"<<le<<endl;
        if(!ret)continue;
        if(le>threshold)continue;

        n=0;
        for(int j=i;j<num;j++)
        {
            if(valid[j])
            {
                xx[n]=x[j];
                yy[n]=y[j];
                n++;
            }
            if(n>=r)break;

        }

        std::valarray <float> right_x(xx,n),right_y(yy,n);
        float rk,rb,re;
        ret=LineFit(rk,rb,re,right_x,right_y);
//        cout<<"i:"<<i<<" rk:"<<rk<<" rb:"<<rb<<" re:"<<re<<endl;
        if(!ret)continue;
        if(re>threshold)continue;

        flag[i]=1;
        kDiff[i]=fabs(atan(lk)-atan(rk));
        if(kDiff[i]> xform::pi_2)
            kDiff[i]=xform::pi-kDiff[i];
//        std::cout<<"i:"<<i<<" kDiff:"<<kDiff[i]<<std::endl;

    }


}
vector <Point> FilterCornerPoint(vector <int> &index,float * const x,float * const y,float * const kDiff,char * const flag,uint num,uint r,float threshold)
{
    for(int i=0;i<num;i++)
    {
//        cout<<"p:"<<i<<endl;
        if(flag[i] &&kDiff[i]>threshold)
        {
            int a=(i-r>=0?i-r:0);
            int b=(i+r>=num-1?num-1:i+r);
            bool ff=false;
            for(int j=a;j<=b;j++)
            {
                if(j!=i && kDiff[j]>threshold && flag[j])
                {
                    if(kDiff[j]>kDiff[i])
                        ff=true;
                }
            }
            if(ff==true)continue;
            else
            {
                for(int j=a;j<=b;j++)
                {
                    if(j!=i && kDiff[j]>threshold && flag[j])
                    {
                        kDiff[j]=-1.0;
                    }
                }

            }

        }

    }
    vector<Point> corPoint;
    index.clear();
    for(int i=0;i<num;i++)
    {
        if(flag[i] && kDiff[i]>threshold)
        {
            Point temp(x[i],y[i]);
            corPoint.push_back(temp);
            index.push_back(i);
        }
    }
    return corPoint;
}
float sign(float value)
{
    if(value>=0)
        return 1.0;
    else
        return -1.0;
}
