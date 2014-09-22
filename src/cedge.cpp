#include "cedge.h"

CEdge::CEdge(const CNode &A,const CNode &B,unsigned int id,string edgeName/*=string("unset")*/,unsigned int priority/*=0*/)
{
    this->nodeA=A.id;
    this->nodeB=B.id;
    this->id=id;
    this->edgeName=edgeName;
    this->length=sqrt((A.x-B.x)*(A.x-B.x)+(A.y-B.y)*(A.y-B.y));
    this->priority=priority;
}
CEdge::CEdge(const CNode &A,const CNode &B,unsigned int id,float l,string edgeName/*=string("unset")*/,unsigned int priority/*=0*/)
{
    this->nodeA=A.id;
    this->nodeB=B.id;
    this->id=id;
    this->edgeName=edgeName;
    this->length=l;
    this->priority=priority;
}
void CEdge::PrintEdgeInfo()
{
    cout<<"Edge ID:"<<this->id<<"  Edge Name:"<<this->edgeName<<endl;
    cout<<"Edge Length:"<<this->length<<"  priority:"<<this->priority<<"  "<<this->nodeA<<"<---->"<<this->nodeB<<endl;
}
