#ifndef CNODE_H
#define CNODE_H

#include <string>
#include <iostream>
#include <limits>
using namespace std;

class CNode
{
public:
    CNode();
    CNode(float x,float y,unsigned int id,std::string nodeName=string("unset"));
    void SetR(float r)
    {
        this->r=r;
    }
    void SetType(int type)
    {
        this->type=type;
    }
    void PrintNodeInfo();


public:
    float x,y,r;
    unsigned int id;
    int type;
    std::string nodeName;

};

#endif // CNODE_H
