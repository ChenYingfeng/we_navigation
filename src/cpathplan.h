#ifndef CPATHPLAN_H
#define CPATHPLAN_H

#include "cnode.h"
#include "cedge.h"
#include <vector>
#include <string>
#include <map>
#include <iostream>
#include <memory.h>
#include <limits>
#include <algorithm>
#include <fstream>
#include <sstream>
#include <ros/ros.h>
using namespace std;
template <class T>
void PrintMatrix(T **m,int l,int h);
class CPathPlan
{
public:
    CPathPlan();
    ~CPathPlan()
    {
        if (dist)
        {
            delete []dist[0];
            delete []dist;
        }
        if (prenode)
        {
            delete []prenode[0];
            delete []prenode;
        }

    }
public:
    vector<CNode> nodes;
    vector<CEdge> edges;
    map<unsigned int,unsigned int> nodeIdToIndex;
    map<unsigned int,unsigned int> edgeIdToIndex;

    unsigned int nodeMaxID;
    unsigned int edgeMaxID;

    void AddNode(float x,float y,float r,int type,string nodeName=string("unset"));
    void AddNode(float x, float y,string nodeName/*=string("unset")*/);
    void DeleteNode(unsigned int id);

    void AddEdge(unsigned int A,unsigned int B,unsigned int priority=100,string edgeName=string("unset"));
    void AddEdge(unsigned int A,unsigned int B,float l,unsigned int priority=100,string edgeName=string("unset"));
    void DeleteEdge(unsigned int id);
    CNode GetNode(unsigned id)
    {
        CNode temp;
        if(!nodeIdToIndex.count(id))
        {
            ROS_ERROR("Error,node is not exist!");
            return temp;
        }
        else
        return nodes[nodeIdToIndex[id]];
    }
    CEdge GetEdge(unsigned int id)
    {

        if(!edgeIdToIndex.count(id))
        {
            ROS_ERROR("Error,node is not exist!");
            return edges[edges.size()];
        }
        else
        return edges[edgeIdToIndex[id]];

    }
    void PrintNodes()
    {
        cout<<"********** Nodes: "<<nodes.size()<<"**********"<<endl;
        for(int i=0;i<nodes.size();i++)
            nodes[i].PrintNodeInfo();
    }
    void PrintEdges()
    {
        cout<<"********** Edges **********"<<endl;
        for(int i=0;i<edges.size();i++)
            edges[i].PrintEdgeInfo();
    }
    void ClearPathMap()
    {
        nodes.clear();
        edges.clear();
        nodeIdToIndex.clear();
        edgeIdToIndex.clear();
        nodeMaxID=edgeMaxID=0;
    }

public:
    bool mapChanged;
    float **dist;
    unsigned int **prenode;

    void InitMatrix();
    void FloydWarshall();
    bool GetShortestPath(unsigned int A,unsigned int B,float &minLen,vector <unsigned int>&path);

public:
    vector<unsigned int> GetAimPath(unsigned int startID,float &pathLen,vector<unsigned int> &aimNodes,int mode=0);
    vector<unsigned int> GetDetailPath(const vector<unsigned int>& aimPath);
    float GetPathLength(const vector<unsigned int> &path,int mode=0);
    bool  GetNeareatNode(float x,float y,unsigned int &nodeId,unsigned int &edgeId);
    bool  ISOnNode(float x,float y,unsigned int &nodeId);

public:
    bool LoadNodeFile(const char *nodeFilePath);
    bool LoadEdgeFile(const char *edgeFilePath);
    bool LoadMapFile();

};

#endif // CPATHPLAN_H
