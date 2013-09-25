#include <pcl/pcl_macros.h>    
//#include <pcl/apps/3d_rec_framework/pipeline/local_recognizer.h>
#include <string>
#include <sstream> 
#include <iostream>
#include <fstream> 
#include "I_SegmentedObjects.h"
      
using namespace std;

//initialize the transforms_ variable to transforms
void I_SegmentedObjects::setTransforms(boost::shared_ptr<vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > > & transforms)
{
    transforms_ = transforms;
}

//adds an object name to the list of objects
void I_SegmentedObjects::addObjectName(string &name)
{
    objectsNames.push_back(name);
}

//creates a file and opens it for future editing
void I_SegmentedObjects::createFile(const char* filename)
{
    stringstream fullFileName;
    fullFileName<<pathToFiles<<filename;
    string newFileName = fullFileName.str();

    testFile.open(newFileName.c_str());
} 

//saves to file the point clouds (code 0) or the segmented point clouds (code 1)
void I_SegmentedObjects::writePointCloudsToFile(int code)
{
    pcl::PCDWriter writer;
    vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> pointClouds_;   

    if (code==0)
        pointClouds_=pointClouds;
    else
        pointClouds_=segPointClouds;

    for (int j=0; j < pointClouds_.size(); ++j)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster = pointClouds_.at(j);

        stringstream ss;

        if (code==0)
            ss << pathToFiles << "cloud_" << j << ".pcd";
        else 
            ss << pathToFiles << "segCloud_" << j << ".pcd";

        writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); 
    }

    std::stringstream ss;
    ss << pathToFiles << "scene.pcd";
    writer.write<pcl::PointXYZ> (ss.str (), *scene, false); 
}

//writes to file for each detected object: the object name and the pose (rotation and translation), the list of objects based on height and the list of objects based on occlusion score
void I_SegmentedObjects::writeObjectsInfoToFile()
{
    testFile <<"Recognized models: \n";
    for(int t=0;t<objectsNames.size();++t)
    {
        testFile << objectsNames.at(t);

        Eigen::Matrix3f rotation = transforms_->at (t).block<3,3>(0, 0);
        Eigen::Vector3f translation = transforms_->at (t).block<3,1>(0, 3);

        testFile <<"\n";
        testFile <<"    | "<<rotation (0,0) << rotation (0,1) << rotation (0,2) << " | \n";
        testFile <<"    | "<<rotation (1,0) << rotation (1,1) << rotation (1,2) << " | \n";
        testFile <<"    | "<<rotation (2,0) << rotation (2,1) << rotation (2,2) << " | \n";
        testFile << "\n";
        testFile <<"t = <"<< translation (0)<< translation (1) << translation (2) <<" >\n";
    }
    //write the list of objects and the indices
    vector<int>listH = getIdsObjects(heightList);
    vector<int>listS = getIdsObjects(occlusionScoreList);

    testFile<<"Height values of recognized objects: "<<"\n";  
    for(int t=0;t<heightList.size();++t)      
        testFile<<heightList[t]<<" ";
    testFile<<endl;

    testFile<<"List of ordered objects id based on height: "<<"\n";  
    for(int t=0;t<listH.size();++t)      
        testFile<<listH[t]<<" ";
    testFile<<endl;

    testFile<<"Occlusion scores of recognized objects: "<<"\n";  
    for(int t=0;t<occlusionScoreList.size();++t)      
        testFile<<occlusionScoreList[t]<<" ";
    testFile<<endl;

    testFile<<"List of ordered objects id based on occlusionScore: "<<"\n";  
    for(int t=0;t<listS.size();++t)      
        testFile<<listS[t]<<" ";
    testFile<<endl;

    testFile.close();
}    

//returns the list in descending order 
vector<int> I_SegmentedObjects::getIdsObjects(vector<double> list)
{
    vector<int> orderedList;
    double value;
    vector<double> list_ = list;

    sort(list_.begin(), list_.end());

    for(int t=list_.size()-1;t>=0;--t)
    { 
        value = list_[t];
        for(int i=0;i<list.size();++i)
        {   
            if(value==list[i])
            {
                orderedList.push_back(i);
                i=list.size();
            }
        }
    }
    
    return orderedList;            
}  
