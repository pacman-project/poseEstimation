#ifndef __I_SEGMENTEDOBJECTS_H__
#define __I_SEGMENTEDOBJECTS_H__
#include <pcl/pcl_macros.h>  
#include <pcl/io/pcd_io.h>  
#include <boost/filesystem.hpp>
#include <string>
#include <sstream> 
#include <iostream>
#include <fstream>
       
using namespace std;

class I_SegmentedObjects
{
    protected:
        vector<string> objectsNames;
        //ordered list of objects based on height map- input (direction vector -dominat plane segmentation normal )
        boost::shared_ptr < vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > > transforms_; 
        ofstream testFile;

        vector<double> heightList, occlusionScoreList;

        vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> pointClouds;
        vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> segPointClouds;
        pcl::PointCloud<pcl::PointXYZ>::Ptr scene;

        const char* pathToConfigFile; 

    public:        

        I_SegmentedObjects(string pathToFile)
        {
           pathToConfigFile = parseConfigFile(pathToFile).c_str();
        }

        //writes to file information about the detected objects 
        void writeObjectsInfoToFile();        
   
        //parse configuration file 
        string parseConfigFile(string pathToFile);

        //creates a file to save information about objects
        void createFile(const char* filename);
        vector<int> getIdsObjects(vector<double> list);
        
        //returns the list of detected objects names 
        vector<string> getObjects()
        {
            return objectsNames;
        }

        // returns the name of the object at index position in the list
        string getObjectsNameAt(int &index) 
        {
            return objectsNames.at(index);
        }
        
       // adds a new object name to the list of objects
        void addObjectName(const string &name);
        
        //returns the vector of transformations (rotation and translation) of the detected objects
        boost::shared_ptr<vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > > getTransforms ()
        {
            return transforms_;
        }

        //sets the transformations variable to transforms
        void setTransforms(boost::shared_ptr<vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > > & transforms);

        //returns the vector of point clouds 
        vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> getPointClouds()
        {
            return pointClouds;
        }

        //returns the vector of segmented point clouds
        vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> getSegPointClouds()
        {
            return segPointClouds;
        }

        //returns the point cloud at index position in the list
        pcl::PointCloud<pcl::PointXYZ>::Ptr getPointCloudAt(int &index)
        {
            return pointClouds.at(index);
        }
        
        //adds a point cloud to the list 
        void addPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud)
        {
        pointClouds.push_back(pointCloud);
        }
   
        //adds a segmented point cloud to the list 
        void addSegPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud)
        {
            segPointClouds.push_back(pointCloud);
        }
  
        //adds the height value to the list of object heights
        void addHeight(double &value)
        {
            heightList.push_back(value);
        }

        //returns the list of object heights 
        vector<double> getHeightList()
        {
            return heightList;
        } 

        //adds an occlusion score to the list 
        void addOcclusionScore(double &value)
        {
            occlusionScoreList.push_back(value);
        }

        //returns the list containing occlusion scores for each object
        vector<double> getOcclusionScoreList()
        {
            return occlusionScoreList;
        } 
        
        //writes the point clouds to file (depending on variable code: '0' for point clouds and '1' for segmented point clouds)
        void writePointCloudsToFile(int code);        

        //sets the scene
        void addScene(pcl::PointCloud<pcl::PointXYZ>::Ptr scene_)
        {
            scene=scene_;
        }

        //returns the scene 
        pcl::PointCloud<pcl::PointXYZ>::Ptr getScene()
        {
            return scene;
        }
        
        ~I_SegmentedObjects()
        {
           pointClouds.clear();
           segPointClouds.clear();
        }
    };
#endif	 
        
        
         
