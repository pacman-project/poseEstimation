#ifndef __PARAMETERSPOSEESTIMATION_H__
#define __PARAMETERSPOSEESTIMATION_H__
#include <string>
#include "I_SegmentedObjects.h"
#include <pcl/pcl_macros.h>
#include <numeric>
#include <pcl/apps/3d_rec_framework/utils/metrics.h>
#include <pcl/apps/dominant_plane_segmentation.h>
#include <pcl/recognition/hv/hv_papazov.h>
#include <pcl/common/angles.h>
#include <pcl/common/transforms.h>
#include <pcl/apps/3d_rec_framework/pipeline/local_recognizer.h>
#include <pcl/apps/3d_rec_framework/pc_source/mesh_source.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/apps/3d_rec_framework/feature_wrapper/local/shot_local_estimator.h>
#include <pcl/apps/3d_rec_framework/feature_wrapper/local/shot_local_estimator_omp.h>
#include <pcl/apps/3d_rec_framework/feature_wrapper/local/fpfh_local_estimator.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/recognition/cg/correspondence_grouping.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/hv/hv_papazov.h>
#include <pcl/recognition/hv/hv_go.h>
#include <pcl/recognition/hv/greedy_verification.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/octree/octree_search.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/apps/3d_rec_framework/tools/openni_frame_source.h>
#include <pcl/filters/passthrough.h>
       
using namespace std;

// class used to the parse configuration file and perform pose estimation
class ParametersPoseEstimation
{
    public:
        string pathPlyModels, desc_name, training_dir, test_file;
        int force_retrain, icp_iterations, use_cache, splits, scene, detect_clutter, hv_method, use_hv, CG_THRESHOLD_,useKinect;
        float thres_hyp, desc_radius, CG_SIZE_, sampling_density;
        float x, y, z, pitch, roll, yaw;

    //constructor which initilizes the main paramters 
    ParametersPoseEstimation(string filename)
    {
        x =0; y =0; z =0.5; pitch=0; roll = -0.7; yaw = 0;

        //check if the configuration file exists, otherwise use default configuration
        if (!boost::filesystem::exists(filename))
        {
            cout<<"Unable to find path to the configuration file, default parameters will be loaded\n";
            //string root_path = "/home/pacman/CODE/poseEstimation/data";
	    string root_path = "/home/pacman/CODE/pacman/poseEstimation/data";
            //pathPlyModels = "../data/PLY-MODELS/";
            pathPlyModels = root_path + "/PLY-MODELS/";
            desc_name = "shot_omp";//"fpfh"
            //training_dir = "../data/TRAINED-LOCAL-MODELS/";
            training_dir = root_path + "/TRAINED-LOCAL-MODELS/";
            force_retrain = 0;
            //icp_iterations = 5; //pacman objects
	    icp_iterations = 20; //intellact objects
            use_cache = 1;
            splits = 512;
            scene = -1;
            detect_clutter = 1;
            hv_method = 2;
            use_hv = 1;
            thres_hyp = 0.2f;
            desc_radius = 0.04f;
            CG_THRESHOLD_ = 1;
            CG_SIZE_ = 0.01f;
            //CG_SIZE_ = 0.005f;
            sampling_density = 0.01;
            useKinect = 0;
	    test_file = "";
        }
        else 
        {      
            //parse the configuration file
            parseConfigFile(filename);
        }
    };

    //parse configuration file and set parameters values
    void parseConfigFile(string filename);
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr kinectGrabFrame();
    pcl::PointCloud<pcl::PointXYZ>::Ptr loadPCDFile(string file_name);
    bool down_sample_check(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,double down_sample_size);
 
    //perform pose estimation for the 'xyz_points' scene
   // int recognizePose(I_SegmentedObjects &objects, pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_points);
    int recognizePose(I_SegmentedObjects &objects,pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_points);
};
#endif	 
