#include "cpathplan.h"

CPathPlan::CPathPlan()
{
    nodeMaxID=edgeMaxID=0;
    mapChanged=false;
    dist=NULL;
    prenode=NULL;
}


void CPathPlan::AddNode(float x, float y,float r,int type, string nodeName/*=string("unset")*/)
{
    CNode temp(x,y,nodeMaxID,nodeName);
    temp.SetR(r);
    temp.SetType(type);
    nodes.push_back(temp);
    nodeIdToIndex.insert(pair<unsigned int,unsigned int>(nodeMaxID++,nodes.size()-1));
    mapChanged=true;
}

void CPathPlan::AddNode(float x, float y,string nodeName/*=string("unset")*/)
{
    CNode temp(x,y,nodeMaxID,nodeName);
    nodes.push_back(temp);
    nodeIdToIndex.insert(pair<unsigned int,unsigned int>(nodeMaxID++,nodes.size()-1));
    mapChanged=true;
}

void CPathPlan::AddEdge(unsigned int A, unsigned int B, unsigned int priority/*=0*/, string edgeName/*=string("unset")*/)
{
    if (!nodeIdToIndex.count(A) || !nodeIdToIndex.count(B))
    {
        ROS_ERROR("Error node id is given!");
        return;
    }
    CEdge temp(nodes[nodeIdToIndex[A]],nodes[nodeIdToIndex[B]],edgeMaxID,edgeName,priority);
    edges.push_back(temp);
    edgeIdToIndex.insert(pair<unsigned int,unsigned int>(edgeMaxID++,edges.size()-1));
    mapChanged=true;

}
void CPathPlan::AddEdge(unsigned int A, unsigned int B,float l, unsigned int priority/*=0*/, string edgeName/*=string("unset")*/)
{
    if (!nodeIdToIndex.count(A) || !nodeIdToIndex.count(B))
    {
        ROS_ERROR("Error node id is given!");
        return;
    }
    CEdge temp(nodes[nodeIdToIndex[A]],nodes[nodeIdToIndex[B]],edgeMaxID,l,edgeName,priority);
    edges.push_back(temp);
    edgeIdToIndex.insert(pair<unsigned int,unsigned int>(edgeMaxID++,edges.size()-1));
    mapChanged=true;

}
void CPathPlan::DeleteNode(unsigned int id)
{
    if (!nodeIdToIndex.count(id))
    {
        ROS_ERROR("Error Node ID");
        return;
    }
    unsigned int index=nodeIdToIndex[id];
    std::vector<CNode>::iterator it = nodes.begin()+index;
    nodes.erase(it);
    nodeIdToIndex.erase(id);

    for(int i=id+1;i<nodeMaxID;i++)
    {
        if (nodeIdToIndex.count(i))
        {
            nodeIdToIndex[i]=nodeIdToIndex[i]-1;
        }
    }


    for(int i=0;i<edgeMaxID;i++)
    {
        if (edgeIdToIndex.count(i))
        {
            CEdge &temp=edges[edgeIdToIndex[i]];
            if (temp.nodeA==id || temp.nodeB==id)
            {
                DeleteEdge(i);
            }
        }
    }
    mapChanged=true;

}
void CPathPlan::DeleteEdge(unsigned int id)
{
    if (!edgeIdToIndex.count(id))
    {
        ROS_ERROR("Error Node ID");
        return;
    }
    unsigned int index=edgeIdToIndex[id];
    std::vector<CEdge>::iterator it = edges.begin()+index;
    edges.erase(it);
    edgeIdToIndex.erase(id);

    for(int i=id+1;i<edgeMaxID;i++)
    {
        if (edgeIdToIndex.count(i))
        {
            edgeIdToIndex[i]=edgeIdToIndex[i]-1;
        }
    }
    mapChanged=true;

}


void CPathPlan::InitMatrix()
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

    int nodeNum=nodes.size();

    dist=new float *[nodeNum];
    dist[0]=new float[nodeNum*nodeNum];
    for(int i=1;i<nodeNum;i++)
        dist[i]=dist[i-1]+nodeNum;
    for(int i=0;i<nodeNum;i++)
        for(int j=0;j<nodeNum;j++)
        {
            if (i!=j)dist[i][j]=std::numeric_limits<int>::max();
            else dist[i][j]=0;
        }

    prenode=new unsigned int *[nodeNum];
    prenode[0]=new unsigned int[nodeNum*nodeNum];
    for(int i=1;i<nodeNum;i++)
        prenode[i]=prenode[i-1]+nodeNum;
    for(int i=0;i<nodeNum;i++)
        for(int j=0;j<nodeNum;j++)
            prenode[i][j]=std::numeric_limits<unsigned int>::max();


    for(int i=0;i<edges.size();i++)
    {
        CEdge temp=edges[i];
        int indexA=nodeIdToIndex[temp.nodeA];
        int indexB=nodeIdToIndex[temp.nodeB];
        dist[indexA][indexB]=dist[indexB][indexA]=temp.length;
        prenode[indexA][indexB]=indexA;
        prenode[indexB][indexA]=indexB;

    }

}

void CPathPlan::FloydWarshall()
{
    if(!mapChanged)
    {
        ROS_WARN("Map isn't changed,don't need plan again!");
        return;
    }
    InitMatrix();
    int nodeNum=nodes.size();
    for(int n=0;n<nodeNum;n++)
    {
        for(int i=0;i<nodeNum;i++)
        {
            for(int j=0;j<nodeNum;j++)
            {
                float a=dist[i][j];
                float b=dist[i][n]+dist[n][j];
                if (a>b)
                {
                    dist[i][j]=b;
                    prenode[i][j]=prenode[n][j];
                }

            }
        }
    }

    PrintMatrix(dist,nodeNum,nodeNum);
    //add check dist matrix
    PrintMatrix(prenode,nodeNum,nodeNum);
    mapChanged=false;
}
bool CPathPlan::GetShortestPath(unsigned int A,unsigned int B,float &minLen,vector<unsigned int> &path)
{
    if(A==B)
    {
        ROS_ERROR("Error,The nodes Id is same %d!",A);
        return false;
    }
//    int AA=A<B?A:B;
//    int BB=A<B?B:A;
    int indexA=nodeIdToIndex[A];
    int indexB=nodeIdToIndex[B];
    minLen=dist[indexA][indexB];
    if (int(minLen)==std::numeric_limits<int>::max())
    {
        ROS_ERROR("No path between the node %d and %d!",A,B);
        return false;
    }
    int last=indexB;
    while(last!=indexA)
    {
      path.push_back(nodes[last].id);
      last=prenode[indexA][last];
    }
    path.push_back(nodes[indexA].id);
//    if(A<B)
    reverse(path.begin(),path.end());
    return true;
}
vector<unsigned int> CPathPlan::GetAimPath(unsigned int startID,float &pathLen,vector<unsigned int> &aimNodes,int mode)
{
    vector <unsigned int> vb(aimNodes.begin(),aimNodes.end());
    vector <unsigned int> va;
    float min=0.0;


    va.push_back(startID);
    for(int i=0;i<vb.size();i++)
    {
        if (vb[i]==startID)
        {
            std::vector<unsigned int>::iterator it = vb.begin()+i;
            vb.erase(it);
            break;
        }
    }

    while(vb.size()>0)
    {
        min=std::numeric_limits<int>::max();
        int aa,bb;
        for(int a=0;a<va.size();a++)
        {
            for(int b=0;b<vb.size();b++)
            {
                if(dist[va[a]][vb[b]]<min)
                {
                    bb=b;
                }

            }
        }

        min=std::numeric_limits<int>::max();
        int insertPos=-1;
        for(int i=1;i<va.size();i++)
        {
            vector <unsigned int> tempVa(va.begin(),va.end());
            tempVa.insert(tempVa.begin()+i,vb[bb]);
            float tempLen=GetPathLength(tempVa,mode);
            if(tempLen<min)
            {
                insertPos=i;
                min=tempLen;
            }

        }
        vector <unsigned int> tempVa(va.begin(),va.end());
        tempVa.push_back(vb[bb]);
        float tempLen=GetPathLength(tempVa,mode);
        if(tempLen<min)
        {
            insertPos=-100;
            min=tempLen;
        }
        if(insertPos==-100)
            va.push_back(vb[bb]);
        else
            va.insert(va.begin()+insertPos,vb[bb]);

        std::vector<unsigned int>::iterator it = vb.begin()+bb;
        vb.erase(it);
    }
    pathLen=min;

//    cout<<"key way:";
//    for(int i=0;i<va.size();i++)
//        cout<<va[i];
//    cout<<endl;

    if(mode)va.push_back(startID);
    return va;
}
vector<unsigned int>  CPathPlan::GetDetailPath(const vector<unsigned int>& aimPath)
{
    if(aimPath.size()<2)
    {
        ROS_ERROR("Error,The plan path is too short!");

    }
    vector <unsigned int>result;
    for(int i=0;i<aimPath.size()-1;i++)
    {
        float minLen;
        vector <unsigned int> tempV;
        if(GetShortestPath(aimPath[i],aimPath[i+1],minLen,tempV))
        {
//            cout<<"detail way:"<<aimPath[i]<<"->"<<aimPath[i+1]<<endl;
//            for(int i=0;i<tempV.size();i++)
//            {
//                cout<<tempV[i];
//            }
//            cout<<endl;

            for(int i=0;i<tempV.size()-1;i++)
            {
                result.push_back(tempV[i]);
            }
        }
        else
        {
            ROS_ERROR("Error,Please make sure the map is right!");
        }
    }
    result.push_back(aimPath[aimPath.size()-1]);
    return result;
}
float CPathPlan::GetPathLength(const vector<unsigned int> &path,int mode)
{
  float sumLen=0.0;
  if (path.size()<=1)
  {
      ROS_ERROR("Path nodes is less than two!");
      return sumLen;
  }
  for(int i=0;i<path.size()-1;i++)
  {
      sumLen=sumLen+dist[path[i]][path[i+1]];
  }

  /****** This sum need to check********/
  ROS_ERROR("Sum is %f:",sumLen);
  if(!mode)
     return sumLen;
  else
     return sumLen+dist[path[path.size()-1]][path[0]];
}


bool CPathPlan::LoadNodeFile(const char *nodeFilePath)
{

    string line;

    ifstream fin(nodeFilePath,std::ios::in);
    if(!fin)
    {
        ROS_ERROR("Error, file read %s failed!",nodeFilePath);
        return false;
    }
    while(getline(fin,line))
    {
       float x,y,r;
       int type,id;

    //   cout<<line<<endl;
       int pos=line.find(' ');
       if (pos==-1)
           continue;

       pos=line.find('#');
       if (pos>=0)
           continue;

       istringstream word(line);
       word>>id;
       word>>type;
       word>>x;
       word>>y;
       word>>r;
       this->AddNode(x,y,r,type);
    }
    fin.close();
    return true;
}

bool CPathPlan::LoadEdgeFile(const char *edgeFilePath)
{

    string line;

    ifstream fin(edgeFilePath,std::ios::in);
    if(!fin)
    {
        ROS_ERROR("Error, file read %s failed!",edgeFilePath);
        return false;
    }
    while(getline(fin,line))
    {
       unsigned int a,b,priority=200;
       float l;

    //   cout<<line<<endl;
       int pos=line.find(' ');
       if (pos==-1)
           continue;

       pos=line.find('#');
       if (pos>=0)
           continue;

       istringstream word(line);
       word>>a;
       word>>b;
       word>>l;
       word>>priority;

       if (priority!=200)
         this->AddEdge(a,b,l,priority);
       else
         this->AddEdge(a,b);
    }
    fin.close();
    return true;
}
bool CPathPlan::ISOnNode(float x,float y,unsigned int &nodeId)
{
    for(int i=0;i<nodes.size();i++)
    {
        float dist=sqrt((x-nodes[i].x)*(x-nodes[i].x)+(y-nodes[i].y)*(y-nodes[i].y));
        if(dist<=0.10)
        {
            nodeId=i;
            return true;
        }
    }
    nodeId=std::numeric_limits<unsigned int>::max();
    return false;

}
bool CPathPlan::GetNeareatNode(float x,float y,unsigned int &nodeId,unsigned int &edgeId)
{

    bool getOne=false;
    float min=std::numeric_limits<float>::max();
    for(int i=0;i<edges.size();i++)
    {
        CNode a=GetNode(edges[i].nodeA);
        CNode b=GetNode(edges[i].nodeB);

        float aa=sqrt((a.x-x)*(a.x-x)+(a.y-y)*(a.y-y));
        float bb=sqrt((b.x-x)*(b.x-x)+(b.y-y)*(b.y-y));

        float ll=aa+bb;
      //  cout<<ll<<endl;
        float rate=ll/edges[i].length;
        if(rate<1.11)
        {
            if(min>rate)
            {
                getOne=true;
                min=rate;
                cout<<ll<<endl;
                cout<<edges[i].length<<endl;
                cout<<edges[i].nodeA<<edges[i].nodeB<<endl;

                nodeId=aa>bb?edges[i].nodeB:edges[i].nodeA;
                edgeId=edges[i].id;
            }

        }
    }
    if(getOne)
        return true;
    return false;

}
bool CPathPlan::LoadMapFile()
{

}

template <class T>
void PrintMatrix(T **m,int l,int h)
{
    cout<<"*********** Matrix **********"<<endl;
    for(int i=0;i<h;i++)
    {
        for(int j=0;j<l;j++)
        {
            cout<<m[i][j]<<" ";
        }
        cout<<endl;
    }
}
