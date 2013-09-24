demo_task5.1
============

Software description
--------------------
Provided a set of object models (ply files), the software performs pose estimation of the detected objects in a scene. The scene can be captured from Kinect sensor or provided as a pcd file.
The object recognition and pose estimation are performed using algorithms available in PCL 1.7, for feature extraction, hypothesis verification, consistency grouping, etc.
The parameters used to configure the recognition pipeline are provided in /parametersFiles/config.txt.

The optput is saved in the folder /recognizedObjects and consists of a text file (with object ids, pose parameters, and the order of objects based on height), the segmented point clouds and the model point clouds.  

Installation
------------
The software module depends on PCL 1.7 (pcl-trunk). Additional information about complilation and required libraries can be found on:

http://pointclouds.org/downloads/source.html

Example files
-------------

There are two options:
a) record a scene using the Kinect sensor (set in the configuration file usekinect = 1)
Running the executable demo_Task5.1 will use as input the frame grabbed from the Kinect sensor.
b) use an already recorded scene as input (set in the configuration file usekinect = 0)
To visualize the scene you can use pcl_viewer

Note
----

Data files are available in a public repository: at https://github.com/mirelapopa/dataFiles.
Place the two folders (Ply-Models and Trained-Local-Models) in the demo_task5.1 folder.

Solution to some pcl1.7 problems:

- PCL 1.7 problem with metslib
(demo_task5.1/src/I_SegmentedObjects.h:5:65: fatal error:
pcl/apps/3d_rec_framework/pipeline/local_recognizer.h: No such file or
directory):

add PCL_ADD_INCLUDES(${SUBSYS_NAME} ${SUBSYS_NAME}/3rdparty/metslib
${metslib_incs})
at the and of pcl/recognition/CmakeLists.txt file + sudo make install
for pcl

- solution to vtk5.8 (error: invalid use of incomplete type ‘struct
vtkPolyLine’) - pcl error:

add vtkPolyLine.h header to pcl/segmentation/example_supervoxels.cpp
source:
https://github.com/henningpohl/pcl/commit/f66dc5df7b496b4a54f2638fdeac2142c1b6acb8
