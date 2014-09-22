#ifndef CEDGE_H
#define CEDGE_H


#include <string>
#include <math.h>
#include <iostream>
#include <stdlib.h>
#include "cnode.h"
using namespace std;
class CEdge
{
public:
    CEdge(const CNode &A,const CNode &B,unsigned int id,string edgeName=string("unset"),unsigned int priority=100);
    CEdge(const CNode &A,const CNode &B,unsigned int id,float l,string edgeName=string("unset"),unsigned int priority=100);
    void PrintEdgeInfo();

public:
    float length;
    unsigned int id;
    std::string edgeName;

    unsigned int nodeA,nodeB,priority;
};

#endif // CEDGE_H
