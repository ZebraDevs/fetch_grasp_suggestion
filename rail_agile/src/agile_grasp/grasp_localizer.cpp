#include <agile_grasp/grasp_localizer.h>

GraspLocalizer::GraspLocalizer(ros::NodeHandle& node, const std::string& cloud_topic,
  const std::string& cloud_frame, int cloud_type, const std::string& svm_file_name, 
  const Parameters& params) 
: cloud_left_(new PointCloud()), cloud_right_(new PointCloud()),
cloud_frame_(cloud_frame), svm_file_name_(svm_file_name), num_clouds_(params.num_clouds_),
num_clouds_received_(0), size_left_(0),
sampleServer(node, "/rail_agile/sample_grasps", boost::bind(&GraspLocalizer::sampleGrasps, this, _1), false),
sampleClassifyServer(node, "/rail_agile/sample_classify_grasps", boost::bind(&GraspLocalizer::sampleClassifyGrasps, this, _1), false)
{
  // create localization object and initialize its parameters
  localization_ = new Localization(params.num_threads_, true, params.plotting_mode_);
  localization_->setCameraTransforms(params.cam_tf_left_, params.cam_tf_right_);
  localization_->setWorkspace(params.workspace_);
  localization_->setNumSamples(params.num_samples_);
  localization_->setFingerWidth(params.finger_width_);
  localization_->setHandOuterDiameter(params.hand_outer_diameter_);
  localization_->setHandDepth(params.hand_depth_);
  localization_->setInitBite(params.init_bite_);
  localization_->setHandHeight(params.hand_height_);

  min_inliers_ = params.min_inliers_;

  if (params.plotting_mode_ == 0)
  {
    plots_handles_ = false;
  }    
  else
  {
    plots_handles_ = false;    
    if (params.plotting_mode_ == 2)
      localization_->createVisualsPub(node, params.marker_lifetime_, cloud_left_->header.frame_id);
  }

  sampleServer.start();
  sampleClassifyServer.start();

  ROS_INFO("Grasp sampling servers are running.");
}

void GraspLocalizer::sampleGrasps(const rail_grasp_calculation_msgs::SampleGraspsGoalConstPtr &goal)
{
  boost::mutex::scoped_lock lock(sampleMutex);

  rail_grasp_calculation_msgs::SampleGraspsFeedback feedback;
  rail_grasp_calculation_msgs::SampleGraspsResult result;

  ROS_INFO("Received grasp sampling request");
  std::vector<int> indices(0);

  feedback.currentAction = "Setting workspace";
  sampleServer.publishFeedback(feedback);

  // set the new workspace based on cloud dimensions
  Eigen::VectorXd ws(6);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCLPointCloud2::Ptr tempCloud(new pcl::PCLPointCloud2);
  pcl_conversions::toPCL(goal->cloud, *tempCloud);
  pcl::fromPCLPointCloud2(*tempCloud, *pclCloud);

  if (goal->workspace.mode == rail_grasp_calculation_msgs::Workspace::WORKSPACE_VOLUME)
  {
    ws << goal->workspace.x_min, goal->workspace.x_max,
          goal->workspace.y_min, goal->workspace.y_max,
          goal->workspace.z_min, goal->workspace.z_max;
  }
  else
  {
    ws << goal->workspace.roiCenter.x - goal->workspace.roiDimensions.x/2.0,
          goal->workspace.roiCenter.x + goal->workspace.roiDimensions.x/2.0,
          goal->workspace.roiCenter.y - goal->workspace.roiDimensions.y/2.0,
          goal->workspace.roiCenter.y + goal->workspace.roiDimensions.y/2.0,
          goal->workspace.roiCenter.z - goal->workspace.roiDimensions.z/2.0,
          goal->workspace.roiCenter.z + goal->workspace.roiDimensions.z/2.0;
  }
  localization_->setWorkspace(ws);

  feedback.currentAction = "Localizing hands";
  sampleServer.publishFeedback(feedback);

  // localize grasps
  hands_ = localization_->localizeHands(pclCloud, pclCloud->size(), indices, false, false);

  result.graspList.header.frame_id = goal->cloud.header.frame_id;
  createPoseArray(hands_, result.graspList);

  sampleServer.setSucceeded(result);
}

void GraspLocalizer::sampleClassifyGrasps(const rail_grasp_calculation_msgs::SampleGraspsGoalConstPtr &goal)
{
  boost::mutex::scoped_lock lock(sampleMutex);

  rail_grasp_calculation_msgs::SampleGraspsFeedback feedback;
  rail_grasp_calculation_msgs::SampleGraspsResult result;

  ROS_INFO("Received grasp sampling request");
  std::vector<int> indices(0);

  feedback.currentAction = "Setting workspace";
  sampleClassifyServer.publishFeedback(feedback);

  // set the new workspace based on cloud dimensions
  Eigen::VectorXd ws(6);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCLPointCloud2::Ptr tempCloud(new pcl::PCLPointCloud2);
  pcl_conversions::toPCL(goal->cloud, *tempCloud);
  pcl::fromPCLPointCloud2(*tempCloud, *pclCloud);

  if (goal->workspace.mode == rail_grasp_calculation_msgs::Workspace::WORKSPACE_VOLUME)
  {
    ws << goal->workspace.x_min, goal->workspace.x_max,
        goal->workspace.y_min, goal->workspace.y_max,
        goal->workspace.z_min, goal->workspace.z_max;
  }
  else
  {
    ws << goal->workspace.roiCenter.x - goal->workspace.roiDimensions.x/2.0,
        goal->workspace.roiCenter.x + goal->workspace.roiDimensions.x/2.0,
        goal->workspace.roiCenter.y - goal->workspace.roiDimensions.y/2.0,
        goal->workspace.roiCenter.y + goal->workspace.roiDimensions.y/2.0,
        goal->workspace.roiCenter.z - goal->workspace.roiDimensions.z/2.0,
        goal->workspace.roiCenter.z + goal->workspace.roiDimensions.z/2.0;
  }
  localization_->setWorkspace(ws);

  feedback.currentAction = "Localizing hands";
  sampleClassifyServer.publishFeedback(feedback);

  // localize grasps
  hands_ = localization_->localizeHands(pclCloud, pclCloud->size(), indices, false, false);

  feedback.currentAction = "Classifying hands as antipodal";
  sampleClassifyServer.publishFeedback(feedback);
  antipodal_hands_ = localization_->predictAntipodalHands(hands_, svm_file_name_);

  feedback.currentAction = "Finding final handles";
  sampleClassifyServer.publishFeedback(feedback);
  handles_ = localization_->findHandles(antipodal_hands_, min_inliers_, 0.005);

  result.graspList.header.frame_id = goal->cloud.header.frame_id;
  createPoseArray(createGraspsMsg(handles_), result.graspList);

  sampleClassifyServer.setSucceeded(result);
}

void GraspLocalizer::createPoseArray(const std::vector<GraspHypothesis> &grasps, geometry_msgs::PoseArray &graspList)
{
  graspList.poses.clear();
  for (unsigned int i = 0; i < grasps.size(); i ++)
  {
    Eigen::Vector3d surfaceCenter = grasps[i].getGraspSurface();
    Eigen::Vector3d axis, approach, binormal;

    axis = grasps[i].getAxis();
    approach = grasps[i].getApproach();
    binormal = approach.cross(axis);

    Eigen::Matrix3d R = Eigen::MatrixXd::Zero(3,3);
    R.col(0) = approach;
    R.col(1) = axis;
    R.col(2) = binormal;

    tf::Matrix3x3 tfR;
    tf::matrixEigenToTF(R, tfR);
    tf::Quaternion q;
    tfR.getRotation(q);
    q.normalize();

    geometry_msgs::Pose graspPose;
    tf::pointEigenToMsg(surfaceCenter, graspPose.position);
    tf::quaternionTFToMsg(q, graspPose.orientation);

    graspList.poses.push_back(graspPose);
  }
}

void GraspLocalizer::createPoseArray(const rail_agile::Grasps &grasps, geometry_msgs::PoseArray &graspList)
{
  graspList.poses.clear();
  for (unsigned int i = 0; i < grasps.grasps.size(); i ++)
  {
    Eigen::Vector3d surfaceCenter;
    Eigen::Vector3d axis, approach, binormal;
    tf::vectorMsgToEigen(grasps.grasps[i].surface_center, surfaceCenter);
    tf::vectorMsgToEigen(grasps.grasps[i].axis, axis);
    tf::vectorMsgToEigen(grasps.grasps[i].approach, approach);

    binormal = approach.cross(axis);

    Eigen::Matrix3d R = Eigen::MatrixXd::Zero(3,3);
    R.col(0) = approach;
    R.col(1) = axis;
    R.col(2) = binormal;

    tf::Matrix3x3 tfR;
    tf::matrixEigenToTF(R, tfR);
    tf::Quaternion q;
    tfR.getRotation(q);
    q.normalize();

    geometry_msgs::Pose graspPose;
    tf::pointEigenToMsg(surfaceCenter, graspPose.position);
    tf::quaternionTFToMsg(q, graspPose.orientation);

    graspList.poses.push_back(graspPose);
  }
}

rail_agile::Grasps GraspLocalizer::createGraspsMsg(const std::vector<Handle>& handles)
{
  rail_agile::Grasps msg;
  for (int i = 0; i < handles.size(); i++)
    msg.grasps.push_back(createGraspMsg(handles[i]));
  msg.header.stamp = ros::Time::now();
  std::cout << "Created grasps msg containing " << msg.grasps.size() << " handles\n";
  return msg;
}

rail_agile::Grasp GraspLocalizer::createGraspMsg(const Handle& handle)
{
  rail_agile::Grasp msg;
  tf::vectorEigenToMsg(handle.getCenter(), msg.center);
  tf::vectorEigenToMsg(handle.getAxis(), msg.axis);
  tf::vectorEigenToMsg(handle.getApproach(), msg.approach);
  tf::vectorEigenToMsg(handle.getHandsCenter(), msg.surface_center);
  msg.width.data = handle.getWidth();
  return msg;
}
