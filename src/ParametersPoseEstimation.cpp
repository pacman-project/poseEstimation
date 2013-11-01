#include <sstream> 
#include <iostream>
#include <fstream> 
#include "ParametersPoseEstimation.h"
      
using namespace std;

 void ParametersPoseEstimation::parseConfigFile(string filename)
 {
   ifstream inputFile;
   inputFile.open(filename.c_str());
   
   if (inputFile.good()) 
    while(!inputFile.eof())
    {
      string parameterName;
      string parameterValue;
 
      getline(inputFile,parameterName,':');

      if(parameterName == "descriptor_name")
       {
          getline(inputFile,parameterValue);
          desc_name = parameterValue;
       }  
      else if(parameterName == "models_dir")
      {
          getline(inputFile,parameterValue);
          pathPlyModels = parameterValue;
      } 
      else if(parameterName == "training_dir")
      {
          getline(inputFile,parameterValue);
          training_dir = parameterValue;
      } 
      else if(parameterName == "force_retrain")
      {
          getline(inputFile,parameterValue);
          force_retrain = atoi(parameterValue.c_str());
      } 
      else if(parameterName == "icp_iterations")
      {
          getline(inputFile,parameterValue);
          icp_iterations = atoi(parameterValue.c_str());
      } 
      else if(parameterName == "use_cache")
      {
          getline(inputFile,parameterValue);
          use_cache = atoi(parameterValue.c_str());
      } 
      else if(parameterName == "splits")
      {
          getline(inputFile,parameterValue);
          splits = atoi(parameterValue.c_str());
      } 
      else if(parameterName == "scene")
      {
          getline(inputFile,parameterValue);
          scene = atoi(parameterValue.c_str());
      } 
      else if(parameterName == "detect_clutter")
      {
          getline(inputFile,parameterValue);
          detect_clutter = atoi(parameterValue.c_str());
      } 
      else if(parameterName == "hv_method")
      {
          getline(inputFile,parameterValue);
          hv_method = atoi(parameterValue.c_str());
      } 
      else if(parameterName == "use_hv")
      {
          getline(inputFile,parameterValue);
          use_hv = atoi(parameterValue.c_str());
      } 
      else if(parameterName == "thres_hyp")
      {
          getline(inputFile,parameterValue);
          thres_hyp = atof(parameterValue.c_str());
      } 
      else if(parameterName == "desc_radius")
      {
          getline(inputFile,parameterValue);
          desc_radius = atof(parameterValue.c_str());
      } 
      else if(parameterName == "gc_threshold")
      {
          getline(inputFile,parameterValue);
          CG_THRESHOLD_ = atoi(parameterValue.c_str());
      } 
      else if(parameterName == "gc_size")
      {
          getline(inputFile,parameterValue);
          CG_SIZE_ = atof(parameterValue.c_str());
      } 
      else if(parameterName == "SAMPLING_DENSITY")
      {
          getline(inputFile,parameterValue);
          sampling_density = atof(parameterValue.c_str());
      } 
      else if(parameterName == "useKinect")
      {
          getline(inputFile,parameterValue);
          useKinect = atoi(parameterValue.c_str());
      } 
      else if(parameterName == "test_file")
      {
          getline(inputFile,parameterValue);
          test_file = parameterValue;
      }  
    }
   else
     cout<<"Unable to open file"<<filename;
   
   inputFile.close();
    
 }

//estimate occlusion score for a point cloud, given a scene and the resolution variable
template<typename PointT> 
float generateOcclusionScore(pcl::PointCloud<PointT> &cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr scene,float resolution)
    {     
      pcl::octree::OctreePointCloudSearch<PointT> scene_octree_(resolution);
      typename pcl::PointCloud<PointT>::Ptr scene_ (new pcl::PointCloud<PointT>);
      pcl::copyPointCloud(*scene, *scene_);      
     
      scene_octree_.setInputCloud(scene_);
      scene_octree_.addPointsFromInputCloud();        

      int free = 0, occupied = 0, occluded = 0;
      for (size_t j = 0; j < cloud.points.size(); j++)
      {
        PointT point = cloud.points[j];

        if (scene_octree_.isVoxelOccupiedAtPoint(point))
        {
          occupied++;

          continue;
        }

        Eigen::Vector3f sensor_orig = scene_->sensor_origin_.head(3);
        Eigen::Vector3f look_at = point.getVector3fMap() - sensor_orig;

        std::vector<int> indices;
        scene_octree_.getIntersectedVoxelIndices(sensor_orig, look_at, indices);
        
        bool is_occluded = false;
        if (indices.size() > 0)
        {
          for (size_t k = 0; k < indices.size(); k++)
          {
            Eigen::Vector3f ray = scene_->points[indices[k]].getVector3fMap() - sensor_orig;

            if (ray.norm() < look_at.norm())
            {
              is_occluded = true;
            }

          }
        }

        if (is_occluded)
        {
          occluded++;
          continue;
        }

        free++;

      }

      return ((float)occluded/cloud.points.size()); 
    } 

    //compute the threshold based on the k-nn distances from each point from a point cloud
    double computeThreshold(vector<float> dist)
    {

     double sum = std::accumulate(dist.begin(), dist.end(), 0.0);
     double mean =  sum / dist.size();
     double threshold = 0.0, accum = 0.0;
  
     for (size_t i = 0; i < dist.size (); ++i)
     {
       accum += (dist[i] - mean) * (dist[i] - mean);
     }
     
     double stdev = sqrt(accum / (dist.size()-1));

     threshold = mean + stdev;

     //return threshold;
      return mean;
      
    }

//estimate the segmented point cloud for a detected object model, using a kdtree to search for the nearests point in a scene of the model points
pcl::PointCloud<pcl::PointXYZ>::Ptr getSegmentedCloud(double &rate, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr model, pcl::PointCloud<pcl::PointXYZ>::Ptr &occluded_PointCloud)
    {

     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
     pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
     int nrCloudPoints, nrModelPoints, nrOccludedPoints, K = 1;
     double threshold;

     nrModelPoints = model->points.size();
   
     kdtree.setInputCloud (cloud);
     vector<int> pointIdxNKNSearch(K);
     vector<float> pointNKNSquaredDistance(K);
     vector<float> neighborsDistances;

     pcl::PointXYZ searchPoint;

     for (size_t i = 0; i < model->points.size (); ++i)
     {
       searchPoint=model->points[i];
   
      if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
  neighborsDistances.push_back(pointNKNSquaredDistance[0]);           
     }

     threshold = computeThreshold(neighborsDistances); 

     for (size_t i = 0; i < model->points.size (); ++i)
     {
       searchPoint=model->points[i];   
      if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 ) 
  if(pointNKNSquaredDistance[0]<threshold)
            cloud_cluster->points.push_back (cloud->points[pointIdxNKNSearch[0]]);        
        else
                  occluded_PointCloud->points.push_back (searchPoint);        
     }

     //compute the rate of matched points
     nrCloudPoints = cloud_cluster->points.size();  
     rate = (float)nrCloudPoints/nrModelPoints;     
          
     nrOccludedPoints =  occluded_PointCloud->points.size ();
     occluded_PointCloud->width = nrOccludedPoints;
     occluded_PointCloud->height = 1;
     occluded_PointCloud->is_dense = true;

     cloud_cluster->width = nrCloudPoints;
     cloud_cluster->height = 1;
     cloud_cluster->is_dense = true;

     return cloud_cluster; //only if it is above a treshold > 30% - to be determined
    } 
     

    double computeHighestPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input, Eigen::Vector4f table_plane_)
    {

     pcl::PointCloud<pcl::PointXYZ>::Ptr highestPointCloud (new pcl::PointCloud<pcl::PointXYZ>);
     pcl::SampleConsensusModelPerpendicularPlane<pcl::PointXYZ> perpendicularPlane (input); 
     vector<double> distances;
     double maxDist;

     perpendicularPlane.setAxis(Eigen::Vector3f (table_plane_[0], table_plane_[1], table_plane_[2]));
     perpendicularPlane.setEpsAngle (pcl::deg2rad (15.0));

     perpendicularPlane.getDistancesToModel(table_plane_,distances);     
     maxDist = *max_element(distances.begin(), distances.end());
         
     return maxDist;

    }

void getModelsInDirectory (bf::path & dir, std::string & rel_path_so_far, std::vector<std::string> & relative_paths, std::string & ext)
    {
    
      bf::directory_iterator end_itr;
      for (bf::directory_iterator itr (dir); itr != end_itr; ++itr)
      {
         //check if its a directory, then get models in it
         if (bf::is_directory (*itr))
         {
          #if BOOST_FILESYSTEM_VERSION == 3
          std::string so_far = rel_path_so_far + (itr->path ().filename ()).string() + "/";
          #else
          std::string so_far = rel_path_so_far + (itr->path ()).filename () + "/";
          #endif
     
         bf::path curr_path = itr->path ();
    	 getModelsInDirectory (curr_path, so_far, relative_paths, ext);
      }  
      else
      {
         //check that it is a ply file and then add, otherwise ignore..
         std::vector < std::string > strs;
         #if BOOST_FILESYSTEM_VERSION == 3
         std::string file = (itr->path ().filename ()).string();
         #else
         std::string file = (itr->path ()).filename ();
         #endif
     
         boost::split (strs, file, boost::is_any_of ("."));
         std::string extension = strs[strs.size () - 1];
     
         if (extension.compare (ext) == 0)
         {
           #if BOOST_FILESYSTEM_VERSION == 3
           std::string path = rel_path_so_far + (itr->path ().filename ()).string();
           #else
           std::string path = rel_path_so_far + (itr->path ()).filename ();
           #endif
     
           relative_paths.push_back (path);
         }
       }
     }
    }

//perform object recognition and pose estimation for a given scene
template<template<class > class DistT, typename PointT, typename FeatureT>
void recognizePoseObjects(typename pcl::rec_3d_framework::LocalRecognitionPipeline<DistT, PointT, FeatureT> & local, I_SegmentedObjects &objects, pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_points)
  
   {
      float Z_DIST_ = 1.25f, resolution = 0.0025f;
      double rate, highestDist,occlusionScore;       
      typename boost::shared_ptr<pcl::rec_3d_framework::Source<PointT> > model_source_ = local.getDataSource ();
      typedef typename pcl::PointCloud<PointT>::ConstPtr ConstPointInTPtr;
      typedef pcl::rec_3d_framework::Model<PointT> ModelT;
      pcl::PointCloud<pcl::PointXYZ>::Ptr scene (new pcl::PointCloud<pcl::PointXYZ>);;     
          
       //Step 1 -> Segment
       pcl::apps::DominantPlaneSegmentation<pcl::PointXYZ> dps;
       dps.setInputCloud (xyz_points);
       dps.setMaxZBounds (Z_DIST_);
       dps.setObjectMinHeight (0.05);
       dps.setMinClusterSize (1000);
       dps.setWSize (9);
       dps.setDistanceBetweenClusters (0.1f);     
      
       std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;
       std::vector<pcl::PointIndices> indices;
       dps.setDownsamplingSize (0.02f);
       dps.compute_fast (clusters);
       dps.getIndicesClusters (indices);
       Eigen::Vector4f table_plane_;
       Eigen::Vector3f normal_plane_ = Eigen::Vector3f (table_plane_[0], table_plane_[1], table_plane_[2]);
       dps.getTableCoefficients (table_plane_);
     
       if (clusters.size()==0)
       {
         cout<<"No clusters detected, adjust the values of the input parameters "<<endl;
       }
       else 
       {
        for (size_t i = 0; i < clusters.size (); i++)
        {
         *scene += *clusters[i];
        }         
      
      local.setVoxelSizeICP (0.005f);
      local.setInputCloud (scene);
    
      {
        pcl::ScopeTime ttt ("Recognition");
        local.recognize ();
      }     
    
      boost::shared_ptr < std::vector<ModelT> > models = local.getModels ();
      boost::shared_ptr < std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > > transforms = local.getTransforms ();       
      
      objects.createFile("testFile.txt");
      objects.setTransforms(transforms);
      objects.addScene(scene);

      for (size_t j = 0; j < models->size (); j++)
      {

         std::stringstream name;
         name << "cloud_" << j;
     
         ConstPointInTPtr model_cloud = models->at (j).getAssembled (resolution);
         typename pcl::PointCloud<PointT>::Ptr model_aligned (new pcl::PointCloud<PointT>);
         pcl::transformPointCloud (*model_cloud, *model_aligned, transforms->at (j));
    
         std::cout << models->at (j).id_ << std::endl;
        
         //get the occlusion score
         occlusionScore = generateOcclusionScore(*model_aligned, scene,resolution); 
         objects.addOcclusionScore(occlusionScore);          

         std::stringstream objectName_;
         objectName_ << models->at (j).id_ << endl;
         string objectName = objectName_.str();
         objects.addObjectName(objectName);

         pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud (new pcl::PointCloud<pcl::PointXYZ>),occluded_PointCloud(new pcl::PointCloud<pcl::PointXYZ>), model_seg;
         copyPointCloud(*model_aligned, *pointCloud);

         model_seg = getSegmentedCloud(rate, xyz_points, pointCloud, occluded_PointCloud);

         objects.addPointCloud(pointCloud);   
         objects.addSegPointCloud(model_seg);
         
         //compute the highest point in the segmented point cloud 
         highestDist = computeHighestPointCloud(model_seg,table_plane_);
         objects.addHeight(highestDist);  
       }
     
       objects.writeObjectsInfoToFile();
       objects.writePointCloudsToFile(0); 
       objects.writePointCloudsToFile(1);   
     } 
    } 

pcl::PointCloud<pcl::PointXYZ>::Ptr ParametersPoseEstimation::kinectGrabFrame()
{
        //get point cloud from the kinect 
        OpenNIFrameSource::OpenNIFrameSource camera;
        OpenNIFrameSource::PointCloudPtr frame;
     
        pcl::visualization::PCLVisualizer vis ("kinect");
        pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_points (new pcl::PointCloud<pcl::PointXYZ>);  
        if (camera.isActive ())
        {
          pcl::ScopeTime frame_process ("Global frame processed ------------- ");
          frame = camera.snap ();
          if( frame->points.size() < 10 )
                 cout << "point cloud is empty ...."<< endl;
          else
            pcl::copyPointCloud (*frame, *xyz_points);             
               
          
        }
 
        return(xyz_points);              
}

pcl::PointCloud<pcl::PointXYZ>::Ptr ParametersPoseEstimation::loadPCDFile(string file_name)
{
  pcl::PCDReader reader;
  pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_points(new pcl::PointCloud<pcl::PointXYZ>() );

  xyz_points->is_dense = true;
  xyz_points->width = 640; xyz_points->height = 480;
  xyz_points->points.resize (xyz_points->width *xyz_points->height);

  if (pcl::io::loadPCDFile<pcl::PointXYZ> (file_name, *xyz_points) == -1) //* load the file
  {
    cout << "could not read file:" << file_name << endl;
  }

  return xyz_points;
  
}

bool ParametersPoseEstimation::down_sample_check(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,double down_sample_size)
{
  bool is_valid = true;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::VoxelGrid<pcl::PointXYZ> vox;

  vox.setInputCloud(cloud);
  vox.setLeafSize(down_sample_size,down_sample_size,down_sample_size);
  vox.filter(*cloud_filtered);

  if( cloud_filtered->points.size() < 10 )
    is_valid = false;

  return is_valid;
}

//configure pose estimation based on parameters, regarding types of descriptors and hypothesis verification algorithms  
int ParametersPoseEstimation::recognizePose(I_SegmentedObjects &objects)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_points( new pcl::PointCloud<pcl::PointXYZ>() );

if(useKinect)
   {
      xyz_points = kinectGrabFrame();    
   }
   else
   {
      if(test_file=="")
        cout<<"Test file is empty, please provide a testing filename for testing the program..."<<endl;
      else
         xyz_points = loadPCDFile(test_file);    
   }
   
   if( xyz_points->points.size() < 10 )
      return -1;

   //check the down_sampling using a voxel grid
   bool is_valid = down_sample_check(xyz_points,0.02f);
   if( !is_valid )
   {
     cout << "cloud is empty after downsampling!!!, aborting...." << endl;
     return -1;
   }

   if (pathPlyModels.compare ("") == 0)
    {
         PCL_ERROR("Set the directory containing the models of main dataset in the config file\n");
         return -1;
    }

   if( xyz_points->points.size() < 10 )
      return -1;

   if (pathPlyModels.compare ("") == 0)
    {
   PCL_ERROR("Set the directory containing the models of main dataset in the config file\n");
         return -1;
    }
     
    bf::path models_dir_path = pathPlyModels;
    if (!bf::exists (models_dir_path))
    {
         PCL_ERROR("Models dir path %s does not exist, set it in the config file \n", pathPlyModels.c_str());
         return -1;
    }
    else
    {
      std::vector < std::string > files;
      std::string start = "";
      std::string ext = std::string ("ply");
      bf::path dir = models_dir_path;
      getModelsInDirectory (dir, start, files, ext);
    }
     
    //configure mesh source
    boost::shared_ptr<pcl::rec_3d_framework::MeshSource<pcl::PointXYZ> > mesh_source (new pcl::rec_3d_framework::MeshSource<pcl::PointXYZ>);
    
    mesh_source->setPath (pathPlyModels);
    mesh_source->setResolution (250);
    mesh_source->setTesselationLevel (1);
    mesh_source->setViewAngle (57.f);
    mesh_source->setRadiusSphere (1.5f);
    mesh_source->setModelScale (0.001f);
    mesh_source->generate (training_dir);
    
    boost::shared_ptr<pcl::rec_3d_framework::Source<pcl::PointXYZ> > cast_source;
    cast_source = boost::static_pointer_cast<pcl::rec_3d_framework::MeshSource<pcl::PointXYZ> > (mesh_source);
    
    //configure normal estimator
    boost::shared_ptr<pcl::rec_3d_framework::PreProcessorAndNormalEstimator<pcl::PointXYZ, pcl::Normal> > normal_estimator;
    normal_estimator.reset (new pcl::rec_3d_framework::PreProcessorAndNormalEstimator<pcl::PointXYZ, pcl::Normal>);
    normal_estimator->setCMR (false);
    normal_estimator->setDoVoxelGrid (true);
    normal_estimator->setRemoveOutliers (true);
    normal_estimator->setValuesForCMRFalse (0.003f, 0.012f);
    
    //configure keypoint extractor
    boost::shared_ptr<pcl::rec_3d_framework::UniformSamplingExtractor<pcl::PointXYZ> >
    uniform_keypoint_extractor (
    new pcl::rec_3d_framework::UniformSamplingExtractor<
    pcl::PointXYZ>);
    uniform_keypoint_extractor->setSamplingDensity (sampling_density);
    uniform_keypoint_extractor->setFilterPlanar (true);
    
    boost::shared_ptr<pcl::rec_3d_framework::KeypointExtractor<pcl::PointXYZ> > keypoint_extractor;
    keypoint_extractor = boost::static_pointer_cast<pcl::rec_3d_framework::KeypointExtractor<pcl::PointXYZ> > (uniform_keypoint_extractor);
    
    //configure cg algorithm (geometric consistency grouping)
    boost::shared_ptr<pcl::CorrespondenceGrouping<pcl::PointXYZ, pcl::PointXYZ> > cast_cg_alg;
    boost::shared_ptr<pcl::GeometricConsistencyGrouping<pcl::PointXYZ, pcl::PointXYZ> > gcg_alg (  new pcl::GeometricConsistencyGrouping<pcl::PointXYZ, pcl::PointXYZ>);
    gcg_alg->setGCThreshold (CG_THRESHOLD_);
    gcg_alg->setGCSize (CG_SIZE_);
    cast_cg_alg = boost::static_pointer_cast<pcl::CorrespondenceGrouping<pcl::PointXYZ, pcl::PointXYZ> > (gcg_alg);
     
    //configure hypothesis verificator
    boost::shared_ptr<pcl::PapazovHV<pcl::PointXYZ, pcl::PointXYZ> > papazov (new pcl::PapazovHV<pcl::PointXYZ, pcl::PointXYZ>);
    papazov->setResolution (0.005f);
    papazov->setInlierThreshold (0.005f);
    papazov->setSupportThreshold (0.08f);
    papazov->setPenaltyThreshold (0.05f);
    papazov->setConflictThreshold (0.02f);
    papazov->setOcclusionThreshold (0.01f);
       
    boost::shared_ptr<pcl::GlobalHypothesesVerification<pcl::PointXYZ, pcl::PointXYZ> > go (
    new pcl::GlobalHypothesesVerification<pcl::PointXYZ,
    pcl::PointXYZ>);
    go->setResolution (0.005f);
    go->setMaxIterations (7000);
    go->setInlierThreshold (0.05f);
    go->setRadiusClutter (0.04f);
    go->setRegularizer (3.f);
    go->setClutterRegularizer (7.5f);
    go->setDetectClutter (detect_clutter);
    go->setOcclusionThreshold (0.01f);
    
    boost::shared_ptr<pcl::GreedyVerification<pcl::PointXYZ, pcl::PointXYZ> > greedy (new pcl::GreedyVerification<pcl::PointXYZ, pcl::PointXYZ> (3.f));
    greedy->setResolution (0.005f);
    greedy->setInlierThreshold (0.005f);
    greedy->setOcclusionThreshold (0.01f);
    
    boost::shared_ptr<pcl::HypothesisVerification<pcl::PointXYZ, pcl::PointXYZ> > cast_hv_alg;
     
    switch (hv_method)
    {
     case 1:
       cast_hv_alg = boost::static_pointer_cast<pcl::HypothesisVerification<pcl::PointXYZ, pcl::PointXYZ> > (greedy);
       break;
     case 2:
       cast_hv_alg = boost::static_pointer_cast<pcl::HypothesisVerification<pcl::PointXYZ, pcl::PointXYZ> > (papazov);
       break;
     default:
       cast_hv_alg = boost::static_pointer_cast<pcl::HypothesisVerification<pcl::PointXYZ, pcl::PointXYZ> > (go);
    }
    
    if (desc_name.compare ("shot") == 0)
    {
      boost::shared_ptr<pcl::rec_3d_framework::SHOTLocalEstimation<pcl::PointXYZ, pcl::Histogram<352> > > estimator;
      estimator.reset (new pcl::rec_3d_framework::SHOTLocalEstimation<pcl::PointXYZ, pcl::Histogram<352> >);
      estimator->setNormalEstimator (normal_estimator);
      estimator->addKeypointExtractor (keypoint_extractor);
      estimator->setSupportRadius (desc_radius);
     
      boost::shared_ptr<pcl::rec_3d_framework::LocalEstimator<pcl::PointXYZ, pcl::Histogram<352> > > cast_estimator;
      cast_estimator = boost::dynamic_pointer_cast<pcl::rec_3d_framework::LocalEstimator<pcl::PointXYZ, pcl::Histogram<352> > > (estimator);
     
      pcl::rec_3d_framework::LocalRecognitionPipeline<flann::L1, pcl::PointXYZ, pcl::Histogram<352> > local;
      local.setDataSource (cast_source);
      local.setTrainingDir (training_dir);
      local.setDescriptorName (desc_name);
      local.setFeatureEstimator (cast_estimator);
      local.setCGAlgorithm (cast_cg_alg);
    
      if (use_hv)
          local.setHVAlgorithm (cast_hv_alg);

      local.setUseCache (static_cast<bool> (use_cache));
      local.initialize (static_cast<bool> (force_retrain));
     
      uniform_keypoint_extractor->setSamplingDensity (sampling_density);
      local.setICPIterations (icp_iterations);
      local.setKdtreeSplits (splits);
     
      recognizePoseObjects<flann::L1, pcl::PointXYZ, pcl::Histogram<352> > (local,objects,xyz_points);
     
    }
     
    if (desc_name.compare ("shot_omp") == 0)
    {
   
      boost::shared_ptr<pcl::rec_3d_framework::SHOTLocalEstimationOMP<pcl::PointXYZ, pcl::Histogram<352> > > estimator;
      estimator.reset (new pcl::rec_3d_framework::SHOTLocalEstimationOMP<pcl::PointXYZ, pcl::Histogram<352> >);
      estimator->setNormalEstimator (normal_estimator);
      estimator->addKeypointExtractor (keypoint_extractor);
      estimator->setSupportRadius (desc_radius);
     
      boost::shared_ptr<pcl::rec_3d_framework::LocalEstimator<pcl::PointXYZ, pcl::Histogram<352> > > cast_estimator;
      cast_estimator = boost::dynamic_pointer_cast<pcl::rec_3d_framework::LocalEstimator<pcl::PointXYZ, pcl::Histogram<352> > > (estimator);
     
      pcl::rec_3d_framework::LocalRecognitionPipeline<flann::L1, pcl::PointXYZ, pcl::Histogram<352> > local;
    
      local.setDataSource (cast_source);
      local.setTrainingDir (training_dir);
      local.setDescriptorName (desc_name);
      local.setFeatureEstimator (cast_estimator);
      local.setCGAlgorithm (cast_cg_alg);
      
      if (use_hv)
        local.setHVAlgorithm (cast_hv_alg);
    
      local.setUseCache (static_cast<bool> (use_cache));
      local.initialize (static_cast<bool> (force_retrain));    
    
      local.setThresholdAcceptHyp (thres_hyp);
     
      uniform_keypoint_extractor->setSamplingDensity (sampling_density);
      local.setICPIterations (icp_iterations);
      local.setKdtreeSplits (splits);
    
      recognizePoseObjects<flann::L1, pcl::PointXYZ, pcl::Histogram<352> > (local,objects,xyz_points);

    }
     
    if (desc_name.compare ("fpfh") == 0)
    {
       boost::shared_ptr<pcl::rec_3d_framework::FPFHLocalEstimation<pcl::PointXYZ, pcl::FPFHSignature33> > estimator;
       estimator.reset (new pcl::rec_3d_framework::FPFHLocalEstimation<pcl::PointXYZ, pcl::FPFHSignature33>);
       estimator->setNormalEstimator (normal_estimator);
       estimator->addKeypointExtractor (keypoint_extractor);
       estimator->setSupportRadius (desc_radius);
     
       boost::shared_ptr<pcl::rec_3d_framework::LocalEstimator<pcl::PointXYZ, pcl::FPFHSignature33> > cast_estimator;
       cast_estimator = boost::dynamic_pointer_cast<pcl::rec_3d_framework::LocalEstimator<pcl::PointXYZ, pcl::FPFHSignature33> > (estimator);
     
       pcl::rec_3d_framework::LocalRecognitionPipeline<flann::L1, pcl::PointXYZ, pcl::FPFHSignature33> local;
       local.setDataSource (cast_source);
       local.setTrainingDir (training_dir);
       local.setDescriptorName (desc_name);
       local.setFeatureEstimator (cast_estimator);
       local.setCGAlgorithm (cast_cg_alg);
    
       if (use_hv)
         local.setHVAlgorithm (cast_hv_alg);
     
       local.setUseCache (static_cast<bool> (use_cache));
       local.initialize (static_cast<bool> (force_retrain));
     
       uniform_keypoint_extractor->setSamplingDensity (sampling_density);
       local.setICPIterations (icp_iterations);
       local.setKdtreeSplits (splits);
     
       recognizePoseObjects<flann::L1, pcl::PointXYZ, pcl::FPFHSignature33> (local,objects,xyz_points);
    }
} 
