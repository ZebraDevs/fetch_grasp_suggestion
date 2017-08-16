# fetch_grasp_suggestion
Supervised and autonomous grasp suggestion for objects based on their presently observed point cloud geometry.

## Description
This package includes a framework that requests grasps from a grasp sampler that implements the SuggestGrasps action
found in [rail_grasp_calculation_msgs](https://github.com/GT-RAIL/rail_grasp_calculation), ranks them using the
RankGrasps action implemented in [rail_grasp_calculation], and refines the ranking by using a pairwise grasp ranking
formulation.  The package also includes functionality to collect new training examples for the pairwise ranking model,
scripts to retrain the model, and scripts to comprehensively evaluate different pairwise ranking models.  Additionally,
the package includes supporting nodes to run demos and perform physical execution of the suggested grasps on a Fetch
robot.

If you'd like to get the package up and running quickly, follow the [installation instructions](#Installation) and one
of the [examples](#Example Usage) below that fits your use case.  If you'd like more detailed information about
everything available in this package, we also include [detailed documentation of all functionality](#Detailed node and script documentation)
at the end of this page.

## Menu
* [Installation](#Installation)
* [Example Usage](#Example Usage)
* [Detailed node and script documentation](#Detailed node and script documentation)
  * [Primary ROS nodes](#Primary ROS nodes)
  * [Supporting ROS nodes](#Supporting ROS nodes)
  * [Demo ROS nodes](#Demo ROS nodes)
  * [Classifier training and evaluation scripts](#Classifier training and evaluation scripts)

## Installation
This package requires [scikit-learn](http://scikit-learn.org/stable/index.html).  Installation instructions can be found
[here](http://scikit-learn.org/stable/install.html).

There are a few other dependencies for generating plots to evaluate classifiers.  They can be installed with the
following:
```bash
sudo apt-get install python-matplotlib
pip install treeinterpreter
```

Install the package by cloning the `fetch_grasp_suggestion` repository into your catkin workspace and building it as follows:
```bash
cd (your catkin workspace)/src
git clone https://github.com/fetchrobotics/fetch_grasp_suggestion.git
cd ..
catkin_make
```

This package is designed to work with [rail_grasp_calculation](https://github.com/GT-RAIL/rail_grasp_calculation) and
[rail_agile](https://github.com/GT-RAIL/rail_agile).  These packages can be installed with instructions provided
[here](https://github.com/GT-RAIL/rail_grasp_calculation)

## Example Usage
The following sections will function as short tutorials to get the system up and running for specific use cases.  

### Grasping individual objects


### Clearing a cluttered scene without object segmentation


### Collecting new grasp preference training data


### Training a new classifier


### Evaluating alternative classifiers


## Detailed node and script documentation
The package contains a set of ROS nodes organized as shown in the diagram below.  Each section is documented separately
in the following subsections.  
![Node Diagram](readme/diagram.png)

### Primary ROS nodes
The following nodes handle the core functionality of this package, including grasp suggestion and pairwise
classification.

#### suggeter
This node interfaces with grasp sampling and ranking to get a set of grasp poses with heuristics, then uses this
information to re-rank the grasps by calling the pairwise ranking model on every grasp pair.  This node also receives
feedback on grasps selected by human operators to create new training examples for the pairwise ranking model.  Relevant
topics, services, action clients, and action servers, and parameters are as follows:
* **Subscribers**
  * `/head_camera/depth_registered/points`([pcl/PointCloud<pcl/PointXYZRGB>](http://wiki.ros.org/pcl_ros))  
  Point cloud stream of the entire environment, used for updating the scene context
  for grasp suggestion.  The point cloud subscribed to can be changed by setting the `cloud_topic` param.
  * `~/grasp_feedback`([rail_manipulation_msgs/GraspFeedback](https://github.com/GT-RAIL/rail_manipulation_msgs/blob/master/msg/GraspFeedback.msg))  
  Feedback for grasp selection.  Publish to this topic when the user selects a grasp
  to generate new training data for the pairwise ranking model.
  * `/rail_segmentation/segmented_objects`([rail_manipulation_msgs/SegmentedObjectList](https://github.com/GT-RAIL/rail_manipulation_msgs/blob/master/msg/SegmentedObjectList.msg))  
  (DEPRECATED) Incoming segmented objects, for use with the action server
  grasp suggestion pipeline.  The topic can be changed to any other source of segmented objects by setting the
  `segmentation_topic` param.  This has been deprecated in favor of the service grasp suggestion pipeline, which is
  recommended instead.
* **Publishers**
  * `~/grasps`([fetch_grasp_suggestion/RankedGraspList](https://github.com/fetchrobotics/fetch_grasp_suggestion/blob/master/fetch_grasp_suggestion/msg/RankedGraspList.msg))  
  (DEPRECATED) Topic for publishing grasps after performing grasp suggestion with
  the action server pipeline.  This has been deprecated in favor of the service grasp suggestion pipeline, which is 
  recommended instead.
* **Service Clients**
  * `/executor/clear_objects`([std_srvs/Empty](http://docs.ros.org/api/std_srvs/html/srv/Empty.html))  
  Clear collision objects from the scene.  Supports the (optional) grasp executor
  node included in this package.
  * `/executor/add_object`([fetch_grasp_suggestion/AddObject](https://github.com/fetchrobotics/fetch_grasp_suggestion/blob/master/fetch_grasp_suggestion/srv/AddObject.srv))  
  Add a collision object to the scene.  Supports the (optional) grasp executor node
  included in this package.
  * `/classify_all`([fetch_grasp_suggestion/ClassifyAll](https://github.com/fetchrobotics/fetch_grasp_suggestion/blob/master/fetch_grasp_suggestion/srv/ClassifyAll.srv))  
  Perform pairwise classification for all pairs of grasps.  Connects to the
  classifier node included in this package.
* **Service Servers**
  * `~/suggest_grasps`([rail_manipulation_msgs/SuggestGrasps](https://github.com/GT-RAIL/rail_manipulation_msgs/blob/master/srv/SuggestGrasps.srv))  
  Given an object point cloud, sample grasps and get an initial ranking for them by
  calculating grasp heuristics.
  * `~/suggest_grasps_scene`([rail_manipulation_msgs/SuggestGrasps](https://github.com/GT-RAIL/rail_manipulation_msgs/blob/master/srv/SuggestGrasps.srv))  
  Given a scene point cloud, sample grasps and get an initial ranking for them by
  calculating grasp heuristics.
  * `~/suggest_grasps_baseline`([rail_manipulation_msgs/SuggestGrasps](https://github.com/GT-RAIL/rail_manipulation_msgs/blob/master/srv/SuggestGrasps.srv))  
  Given a point cloud, sample grasps classified as antipodal using the full AGILE
  pipeline.  This is only included for evaluation purposes, as the suggest_grasps or suggest_grasps_scene servers
  produce better results.
  * `~/suggest_grasps_random`([rail_manipulation_msgs/SuggestGrasps](https://github.com/GT-RAIL/rail_manipulation_msgs/blob/master/srv/SuggestGrasps.srv))  
  Given an object point cloud, sample antipodal grasps with a random ordering.  This
  is included only for baseline testing, and should not be used for any real applications!
  * `~/pairwise_rank`([rail_manipulation_msgs/PairwiseRank](https://github.com/GT-RAIL/rail_manipulation_msgs/blob/master/srv/PairwiseRank.srv))  
  Re-rank the most recently computed grasp list for an object using the pairwise
  ranking model.
  * `~/pairwise_rank_scene`([rail_manipulation_msgs/PairwiseRank](https://github.com/GT-RAIL/rail_manipulation_msgs/blob/master/srv/PairwiseRank.srv))  
  Re-rank the most recently computed grasp list for a scene using the pairwise
  ranking model.
* **Action Clients**
  * `/rail_agile/sample_grasps`([rail_grasp_calculation_msgs/SampleGraspsAction](https://github.com/GT-RAIL/rail_grasp_calculation/blob/master/rail_grasp_calculation_msgs/action/SampleGrasps.action))  
  Antipodal grasp sampler using AGILE.  If you'd like to change the source for the
  initial sampled grasps, you can remap this topic to any other grasp sampling action server that implements the
  rail_grasp_calculation_msgs SampleGraspsAction.
  * `/rail_agile/sample_classify_grasps`([rail_grasp_calculation_msgs/SampleGraspsAction](https://github.com/GT-RAIL/rail_grasp_calculation/blob/master/rail_grasp_calculation_msgs/action/SampleGrasps.action))  
  Full AGILE grasp sampling pipeline.  Only required for the
  `~/suggest_grasps_baseline` service.
  * `/grasp_sampler/rank_grasps_object`([rail_grasp_calculation_msgs/RankGraspsAction](https://github.com/GT-RAIL/rail_grasp_calculation/blob/master/rail_grasp_calculation_msgs/action/RankGrasps.action))  
  Calculate heuristics and rank grasps for an object using rail_grasp_calculation.
  * `/grasp_sampler/rank_grasps_scene`([rail_grasp_calculation_msgs/RankGraspsAction](https://github.com/GT-RAIL/rail_grasp_calculation/blob/master/rail_grasp_calculation_msgs/action/RankGrasps.action))  
  Calculate heuristics and rank grasps for a scene using rail_grasp_calculation.
* **Action Servers**
  * `~/get_grasp_suggestions`([fetch_grasp_suggestion/SuggestGraspsAction](https://github.com/fetchrobotics/fetch_grasp_suggestion/blob/master/fetch_grasp_suggestion/action/SuggestGrasps.action))  
  (DEPRECATED) Sample grasps and calculate an initial ranking based on grasp
  heuristics by action server.  This is deprecated in favor of the service implementation `~/suggest_grasps`, which is
  recommended instead.
* **Parameters**
  * `segmentation_topic`(string, "rail_segmentation/segmented_objects")  
  Topic for incoming segmented object data.
  * `cloud_topic`(string, "head_camera/depth_registered/points")  
  Point cloud topic to update the scene where grasping is taking place.
  * `file_name`(string, "grasp_data")  
  Name of a csv text file to save new training data to.
  * `min_grasp_depth`(double, -0.03)  
  Minimum depth offset (in m) to adjust a suggested grasp by for execution.
  * `max_grasp_depth`(double, 0.03)  
  Maximum depth offset (in m) to adjust a suggested grasp by for execution.
 
#### classifier_node.py
This node implements the pairwise ranking model and exposes it as a service.
* **Service Servers**
  * `~/classify_grasp_pair`([fetch_grasp_suggestion/ClassifyGraspPair](https://github.com/fetchrobotics/fetch_grasp_suggestion/blob/master/fetch_grasp_suggestion/srv/ClassifyGraspPair.srv))  
  Perform a single classification for a pair of grasps.  If you're classifying more
  than one grasp, use the `~/classify_all` service instead, as it is very slow to re-instantiate the model for every
  classification instance.
  * `~/classify_all`([fetch_grasp_suggestion/ClassifyAll](https://github.com/fetchrobotics/fetch_grasp_suggestion/blob/master/fetch_grasp_suggestion/srv/ClassifyAll.srv))  
  Perform pairwise classification on all pairs of grasps, returning a re-ordered
  grasp list as a result.
* **Parameters**
  * `~/file_name`(string, "decision_tree.pkl")  
  Filename for an already-trained classifier model.  We include a pre-trained
  random forest model, and you can generate new models using the `train_classifier.py` script.  Filenames can be
  relative to the package path or absolute.
  * `~/n_jobs`(int, 1)  For classifiers that support this parameter, the number of jobs to use when performing
  classification.  Passing in -1 will use as much processing as is available.
  &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;
  * `~/interpret_trees`(bool, false)  
  Flag for generating a plot detailing the contributions of each feature over a set of classifications, only valid for
  decision tree and random forest classifiers.  NOTE: This is buggy, in that it tends to crash after generating plots
  repeatedly, so leave this off unless you specifically want plots to be generated.
  * `~/object_feature_size`(int, 6)  
  The length of the feature vector describing only the object (not the difference)
  of grasp heuristics).

### Supporting ROS nodes
The following nodes are optional nodes that can be used to execute the suggested grasps, or to select grasps using an
interactive marker server implementation for grasp execution and generation of new training examples.

#### executor
An optional standalone grasp executor for the Fetch robot.  Relevant services and actions are as follows:
* **Action Servers**
  * `~/execute_grasp`([fetch_grasp_suggestion/ExecuteGraspAction](https://github.com/fetchrobotics/fetch_grasp_suggestion/blob/master/fetch_grasp_suggestion/action/ExecuteGrasp.action))  
  Execute a grasp pose, which involves moving to the approach angle, opening the gripper, moving in a straight-line
  trajectory to the grasp pose, closing the gripper, and lifting the object.
  * `~/prepare_robot`([fetch_grasp_suggestion/PresetMoveAction](https://github.com/fetchrobotics/fetch_grasp_suggestion/blob/master/fetch_grasp_suggestion/action/PresetMove.action))  
  Move the arm to a "ready to grasp" pose that's out of the way of the camera.
  * `~/drop_position`([fetch_grasp_suggestion/PresetMoveAction](https://github.com/fetchrobotics/fetch_grasp_suggestion/blob/master/fetch_grasp_suggestion/action/PresetMove.action))  
  Move the arm to a preset position to drop an object.
* **Service Servers**
  * `~/add_object`([fetch_grasp_suggestion/AddObject](https://github.com/fetchrobotics/fetch_grasp_suggestion/blob/master/fetch_grasp_suggestion/srv/AddObject.srv))  
  Add an object to the MoveIt! collision scene.
  * `~/clear_objects`([std_srvs/Empty](http://docs.ros.org/api/std_srvs/html/srv/Empty.html))  
  Remove all collision objects from the MoveIt! collision scene.
  * `~/detach_objects`([std_srvs/Empty](http://docs.ros.org/api/std_srvs/html/srv/Empty.html))  
  Detach any collision objects currently attached to the gripper.
  * `~/drop_object`([std_srvs/Empty](http://docs.ros.org/api/std_srvs/html/srv/Empty.html))  
  Open the gripper and remove all collision objects.


#### selector
(DEPRECATED) An optional interactive marker server used for collecting new training examples.  This is deprecated in
favor of a better interface implementation in fetch_pbd, which is recommended instead.
* **Subscribers**
  * `/suggester/grasps`([fetch_grasp_suggestion/RankedGraspList](https://github.com/fetchrobotics/fetch_grasp_suggestion/blob/master/fetch_grasp_suggestion/msg/RankedGraspList.msg))  
  Subscriber for newly calculated and ranked grasps, coming from the `suggester`
  node by default.  The topic can be changed by setting the `grasps_topic` parameter.
  * `/rail_segmentation/segmented_objects`([rail_manipulation_msgs/SegmentedObjectList](https://github.com/GT-RAIL/rail_manipulation_msgs/blob/master/msg/SegmentedObjectList.msg))  
  Segmented objects subscriber.  Object information is used when generating new
  training examples.  This topic can be changed by setting the `segmentation_topic` parameter.
* **Service Servers**
  * `~/cycle_grasps`([fetch_grasp_suggestion/CycleGrasps](https://github.com/fetchrobotics/fetch_grasp_suggestion/blob/master/fetch_grasp_suggestion/srv/CycleGrasps.srv))  
  Advance the currently displayed grasp forward or backward.
* **Action Clients**
  * `/executor/execute_grasp`([fetch_grasp_suggestion/ExecuteGraspAction](https://github.com/fetchrobotics/fetch_grasp_suggestion/blob/master/fetch_grasp_suggestion/action/ExecuteGrasp.action))  
  Connection to the standalone grasp `executor` included in this package.
* **Action Servers**
  * `~/execute_selected_grasp`([fetch_grasp_suggestion/ExecuteSelectedGraspAction](https://github.com/fetchrobotics/fetch_grasp_suggestion/blob/master/fetch_grasp_suggestion/action/ExecuteSelectedGraspAction.action))  
  Execute the currently selected grasp and create new pairwise training examples
  based on the grasp executed and the grasps seen by the user.
* **Parameters**
  * `~/segmentation_topic`(string, "rail_sgementation/segmented_objects")  
  Topic for subscribing to segmented objects.
  * `~/grasps_topic`(string, "suggester/grasps")  
  Topic for subscribing to new calculated grasps
  * `~/file_name`(string, "grasp_data")  
  Filename of a .csv text file to save new training data to.

### Demo ROS nodes
The nodes in this section will run the full pipeline of processing a point cloud, sampling grasps for an object or
scene, calculate heuristics for those grasps, rank the grasp list with pairwise ranking, and execute the best grasp.
None of the nodes have a GUI (sorry!), but they can be called from the command line by simply publishing to the
appropriate ROS topic.  Further, before grasps are executed, the user has the option to view them first and must
provide keyboard input to actually execute the grasp for safety reasons.

#### test_grasp_suggestion
Test individual object grasping with a variety of different strategies, including our novel pairwise ranking method,
ranking based only on our heuristics, the full AGILE pipeline, and a baseline random grasp from antipodal candidates.
Each method is called by publishing a [std_msgs/Int32](http://docs.ros.org/api/std_msgs/html/msg/Int32.html) message
with the index of the object to be grasped to the appropriate test topic.  Relevant topics and parameters are as
follows:
* **Subscribers**
  * `~/grasp_object`([std_msgs/Int32](http://docs.ros.org/api/std_msgs/html/msg/Int32.html))  
  Grasp an object using the pairwise ranking method.
  * `~/grasp_object_heuristic`([std_msgs/Int32](http://docs.ros.org/api/std_msgs/html/msg/Int32.html))  
  Grasp an object using ranks derived directly from the heuristics.
  * `~/grasp_object_agile`([std_msgs/Int32](http://docs.ros.org/api/std_msgs/html/msg/Int32.html))  
  Grasp an object using the full AGILE pipeline.
  * `~/grasp_object_random`([std_msgs/Int32](http://docs.ros.org/api/std_msgs/html/msg/Int32.html))  
  Grasp an object using a random antipodal grasp.
* **Publishers**
  * `~/debug`([geometry_msgs/PoseStamped](http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html))  
  Grasp pose to be executed.  It's recommended to display this topic in rviz when
  monitoring grasp execution.
* **Parameters**
  * `~/segmentation_topic`(string, "rail_sgementation/segmented_objects")  
  Topic for subscribing to segmented objects.
  * `~/debug`(bool, true)  
  Publish the pose to be executed if true.

#### cluttered_scene_demo
Run a demo where the robot will continue to calculate and perform grasps until a surface in front of it is empty.  It
will loop through execution cycles that consist of moving the arm out of the way of the camera, taking a point cloud
snapshot, sampling and ranking grasps over the point cloud snapshot, executing the best grasp, moving to a drop
position off to the side of the robot, and dropping the object.  Starting the demo requires publishing an empty
message to a topic after the node is running.  Relevant topics and parameters are as follows:
* **Subscribers**
  * `~/run_demo`([std_msgs/Empty](http://docs.ros.org/indigo/api/std_msgs/html/msg/Empty.html))  
  Execute the demo.  As with `test_grasp_suggestion`, each grasp will still require keyboard input before execution,
  for safety.
* **Parameters**
  * `~/cloud_topic`(string, "head_camera/depth_registered/points")  
  Topic of the point cloud that includes the scene to be cleared.

### Classifier training and evaluation scripts
The following python scripts facilitate training and evaluating new classifiers.  Each script is implemented as a ROS
node, and can be run with rosrun to set parameters.

#### train_classifier.py
This is the main script for training a new classifier and saving the model.  Newly trained classifiers will be saved
in the current directory of the terminal in which the script was executed.
* **Parameters**
  * `~/classifier_types` (string[], ["decision_tree"])  
  The types of classifiers to train models for.  Supported classifiers include: `["decision_tree", "random_forest",
  "ada_boost", "knn", "svm", "logistic_regression", "nn1", "nn2"]`, where `"nn1"` is a neural network with a single
  hidden layer, and `"nn2"` is a neural network with two hidden layers.
  * `~/file_name` (string, "grasp_data.csv")  
  The filename containing the training data.  File paths can be relative or absolute.  If the path is relative, the
  script assumes the file is located in the directory (location of fetch_grasp_suggestion)/data/grasp_preferences/.

#### evaluate_classifier.py
Compare the performance of different types of classifiers with various tests and metrics, including k-folds cross
validation, detailed results on a train/test split, learning curve plots, ROC curves, and precision-recall curves.
* **Parameters**
  * `~/classifier_types` (string[], ["decision_tree"])  
  The types of classifiers to test.  Supported classifiers include: `["decision_tree", "random_forest",
  "ada_boost", "knn", "svm", "logistic_regression", "nn1", "nn2"]`, where `"nn1"` is a neural network with a single
  hidden layer, and `"nn2"` is a neural network with two hidden layers.
  * `~/file_name` (string, "grasp_data.csv")  
  The filename containing all of the data, which will be used to create training and testing sets.  File paths can be
  relative or absolute.  If the path is relative, the script assumes the file is located in the directory (location of
  fetch_grasp_suggestion)/data/grasp_preferences/.
  * `~/split` (double, 0.4)  
  Ratio used to split the data into a train and test set.  The value corresponds to the fraction of the total data to be
  used for the test set.
  * `~/generate_plots` (bool, false)  
  Flag for generating plots.  If true, the script will generate training curves, ROC curves, and precision-recall curves
  for each classifier specified in `classifier_types`.  Otherwise, the script will only provide the accuracy from
  cross validation and the detailed classification report over the train/test split in the terminal.

#### cross_validate_params.py
Perform cross validation to examine the effects of different model parameters for a given classifier type.
* **Parameters**
  * `~/classifier_types` (string[], ["decision_tree"])  
  The types of classifiers to use.  A separate cross-validation procedure will be carried out for each classifier type,
  with a set of parameters appropriate to that classifier type.  Supported classifiers include: `["decision_tree",
  "random_forest", "ada_boost", "knn", "svm", "logistic_regression", "nn1", "nn2"]`, where `"nn1"` is a neural network
  with a single hidden layer, and `"nn2"` is a neural network with two hidden layers.
  * `~/file_name` (string, "grasp_data.csv")  
  The filename containing all of the data, which will be used for training and testing during cross-validation.  File
  paths can be relative or absolute.  If the path is relative, the script assumes the file is located in the directory
  (location of fetch_grasp_suggestion)/data/grasp_preferences/.
  * `~/generate_plots` (bool, false)
  Flag for generating plots that show the effects of different parameter values on classification rate.  If true, one
  plot will be generated for each classifier type specified in `classifier_types`.  If false, the results will be
  reported only by text output in the terminal.  Generating plots is recommended for getting a better understanding of
  the parameter effects.
