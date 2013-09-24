//#include <pcl/visualization/pcl_visualizer.h>
//#include <pcl/apps/3d_rec_framework/tools/openni_frame_source.h>
//#include <pcl/filters/passthrough.h>
#include "ParametersPoseEstimation.h"

using namespace std;

pcl::PointCloud<pcl::PointXYZ>::Ptr kinectGrabFrame()
{
        //get point cloud from the kinect 
        OpenNIFrameSource::OpenNIFrameSource camera;
        OpenNIFrameSource::PointCloudPtr frame;
     
        pcl::visualization::PCLVisualizer vis ("kinect");
       
        if (camera.isActive ())
        {
          pcl::ScopeTime frame_process ("Global frame processed ------------- ");
          frame = camera.snap ();
     
          pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_points (new pcl::PointCloud<pcl::PointXYZ>);
          pcl::copyPointCloud (*frame, *xyz_points);             
               
          return(xyz_points);
        }               
}

/** Based on the paper:
 * "A Global Hypotheses Verification Method for 3D Object Recognition",
 * A. Aldoma and F. Tombari and L. Di Stefano and Markus Vincze, ECCV 2012     
 */ 
   
int main (int argc, char ** argv)
{
    
      string filename = "../parametersFiles/config.txt";     
      //ParametersPoseEstimation params(filename);
      pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_points;        
      pcl::PCDWriter writer;  
      
      for (int i=0; i<3; i++)
      {
        ParametersPoseEstimation params(filename);
        I_SegmentedObjects objects;

        //change useKinect value in the configuration file depending wether the Kinect sensor is used or not  
        if(params.useKinect)
         {
            //grab a frame from Kinect and save it to a point cloud
        //    xyz_points = kinectGrabFrame();  
       
            //write the point cloud from Kinect to file     
            //writer.write<pcl::PointXYZ> ("scene.pcd", *xyz_points, false);          
         }       
           
        //perform pose estimation for the objects in the point cloud
        //params.recognizePose(objects,xyz_points);
        params.recognizePose(objects);
        
        sleep(1);
      }
 
       return(0); 
}
