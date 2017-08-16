# rail_agile
Modified version of [agile_grasp](http://wiki.ros.org/agile_grasp) to expose functionality with ROS action servers
using the message types defined in [rail_grasp_calculation](https://github.com/GT-RAIL/rail_grasp_calculation).  The
majority of the original agile_grasp package is included here, but this readme focuses on the changes made and how to
run the AGILE pipeline with the new action server.  For more information on the original package, see
[the AGILE readme](https://github.com/atenpas/agile_grasp/blob/master/README.md).

## find_grasps node
This node exposes action servers for executing either just antipodal grasp sampling (without classification), or the
full AGILE pipeline (with classification).  Relevant parameters, action servers, topics, and services are as follows:
 * **Action Servers**
   * `/rail_agile/sample_grasps`([rail_grasp_calculation_msgs/SampleGraspsAction](https://github.com/GT-RAIL/rail_grasp_calculation/blob/master/rail_grasp_calculation_msgs/action/SampleGrasps.action))  
 &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; Sample grasps over a point cloud within a given workspace.  Requires a point cloud
 and workspace as input.  Returns a list of antipodal grasps, unclustered and not passed through AGILE's SVM classifier.
 This will result in a large list of potential antipodal grasps.
   * `/rail_agile/sample_classify_grasps`([rail_grasp_calculation_msgs/SampleGraspsAction](https://github.com/GT-RAIL/rail_grasp_calculation/blob/master/rail_grasp_calculation_msgs/action/SampleGrasps.action))  
 &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; Sample grasps over a point cloud within a given workspace, using the full AGILE
 pipeline.  Requires a point cloud and workspace as input.  Returns a list of antipodal grasps, clustered and passed
 through AGILE's SVM classifier.  This will result in a small list of grasps that are more likely to be antipodal.
 * **Parameters**  
 &nbsp;&nbsp;&nbsp; See the AGILE documentation for a description of all parameters.  They are unchanged here, but are
 important to set appropriately for best performance.

## Installation
Clone the package into your catkin workspace and build it as follows:
```bash
cd (your catkin workspace)/src
git clone https://github.com/GT-RAIL/rail_agile.git
cd ..
catkin_make
```

## Usage
Run the find_grasps node with the following, optionally setting parameters with rosrun or in a launch file:
```bash
rosrun rail_agile find_grasps
```

Make sure that all parameters to describe your robot's hand are set correctly, and set the num_samples parameter
appropriately for the size of your point cloud.

To find grasps, create an action client that connects to either the `sample_grasps` action server or the
`sample_classify_grasps` action server depending on the number and quality of grasps you would like.  Pass in a
point cloud that you want to search for grasps from, and a workspace that constrains the area within the point cloud
to search for grasps in.  The result will contain an unordered list of all of the grasp poses found by AGILE.
