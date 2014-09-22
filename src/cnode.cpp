#include "cnode.h"

CNode::CNode()
{
    this->x=this->y=this->r=0.0;
    this->id=std::numeric_limits<unsigned int>::max();
    this->nodeName=string("unset");
    this->type=-1;

}

CNode::CNode(float x,float y,unsigned int id,std::string nodeName/*=unset*/)
{
    this->x=x;
    this->y=y;
    this->id=id;
    this->nodeName=nodeName;

    this->r=0.0;
    this->type=-1;

}
void CNode::PrintNodeInfo()
{
    cout<< "Node ID:"<<this->id<< "  Node Name:"<<this->nodeName<<"  Node Type:"<<this->type<<endl;
    cout<< "x:"<<this->x<< "  y:"<<this->y<<"  r:"<<this->r<<endl;
}
