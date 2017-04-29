#include <agile_grasp/grasp_localizer_server.h>


GraspLocalizerServer::GraspLocalizerServer(ros::NodeHandle& node, const std::string& cloud_topic, 
	const std::string& cloud_frame, int cloud_type, const std::string& svm_file_name, 
	const Parameters& params) 
  : cloud_left_(new PointCloud()), cloud_right_(new PointCloud()), 
  cloud_frame_(cloud_frame), svm_file_name_(svm_file_name), num_clouds_(params.num_clouds_), 
  num_clouds_received_(0), size_left_(0)
{
  // subscribe to input point cloud ROS topic
  if (cloud_type == CLOUD_SIZED)
		cloud_sub_ = node.subscribe(cloud_topic, 1, &GraspLocalizerServer::cloud_sized_callback, this);
	else if (cloud_type == POINT_CLOUD_2)
		cloud_sub_ = node.subscribe(cloud_topic, 1, &GraspLocalizerServer::cloud_callback, this);
  
  // create ROS publisher for grasps
  grasps_pub_ = node.advertise<agile_grasp::Grasps>("grasps", 10);
  grasps_handles_pub_ = node.advertise<agile_grasp::Grasps>("grasps_handles", 10);
  
  // service
  ROS_INFO("Starting service...");
  grasps_service = node.advertiseService("find_agile_grasps", &GraspLocalizerServer::service_callback,this);
  
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
			localization_->createVisualsPub(node, params.marker_lifetime_, cloud_frame_);
	}
}



Eigen::Matrix3d reorderHandAxes(const Eigen::Matrix3d& Q)
{
	std::vector<int> axis_order_;
	axis_order_.push_back(2);
	axis_order_.push_back(0);
	axis_order_.push_back(1);
	
	Eigen::Matrix3d R = Eigen::MatrixXd::Zero(3, 3);
	R.col(axis_order_[0]) = Q.col(0); // grasp approach vector
	R.col(axis_order_[1]) = Q.col(1); // hand axis
	R.col(axis_order_[2]) = Q.col(2); // hand binormal
	return R;
}

geometry_msgs::PoseStamped GraspLocalizerServer::graspToPose(agile_grasp::Grasp grasp, double hand_offset, std::string frame_id){
	
	Eigen::Vector3d center_; // grasp position
	Eigen::Vector3d surface_center_; //  grasp position projected back onto the surface of the object
	Eigen::Vector3d axis_; //  hand axis
	Eigen::Vector3d approach_; //  grasp approach vector
	Eigen::Vector3d binormal_; //  vector orthogonal to the hand axis and the grasp approach direction
	
	tf::vectorMsgToEigen(grasp.axis, axis_);
	tf::vectorMsgToEigen(grasp.approach, approach_);
	tf::vectorMsgToEigen(grasp.center, center_);
	tf::vectorMsgToEigen(grasp.surface_center, surface_center_);
	
	approach_ = -1.0 * approach_; // make approach vector point away from handle centroid
	binormal_ = axis_.cross(approach_); // binormal (used as rotation axis to generate additional approach vectors)
	
	//step 1: calculate hand orientation
	
	// rotate by 180deg around the grasp approach vector to get the "opposite" hand orientation
	Eigen::Transform<double, 3, Eigen::Affine> T_R(Eigen::AngleAxis<double>(M_PI/2, approach_));
	
	//to do: compute the second possible grasp by rotating -M_PI/2 instead
	
	// calculate first hand orientation
	Eigen::Matrix3d R = Eigen::MatrixXd::Zero(3, 3);
	R.col(0) = -1.0 * approach_;
	R.col(1) = T_R * axis_;
	R.col(2) << R.col(0).cross(R.col(1));
			
	Eigen::Matrix3d R1 = reorderHandAxes(R);
	tf::Matrix3x3 TF1;		
	tf::matrixEigenToTF(R1, TF1);
	tf::Quaternion quat1;
	TF1.getRotation(quat1);		
	quat1.normalize();
	
	// rotate by 180deg around the grasp approach vector to get the "opposite" hand orientation
	/*Eigen::Transform<double, 3, Eigen::Affine> T(Eigen::AngleAxis<double>(M_PI, approach_));
	
	// calculate second hand orientation
	Eigen::Matrix3d Q = Eigen::MatrixXd::Zero(3, 3);
	Q.col(0) = T * approach_;
	Q.col(1) = T * axis_;
	Q.col(2) << Q.col(0).cross(Q.col(1));
	
	// reorder rotation matrix columns according to axes ordering of the robot hand
	//Eigen::Matrix3d R1 = reorderHandAxes(R);
	Eigen::Matrix3d R2 = reorderHandAxes(Q);

	// convert Eigen rotation matrices to TF quaternions and normalize them
	tf::Matrix3x3 TF2;
	tf::matrixEigenToTF(R2, TF2);
	tf::Quaternion quat2;
	TF2.getRotation(quat2);
	quat2.normalize();
	
	std::vector<tf::Quaternion> quats;
	quats.push_back(quat1);
	quats.push_back(quat2);*/
	
	//use the first quaterneon for now
	tf::Quaternion quat = quat1;
	
	//angles to try
	double theta = 0.0;
	
	// calculate grasp position
	Eigen::Vector3d position;
	Eigen::Vector3d approach = -1.0 * approach_;
	if (theta != 0)
	{
		// project grasp bottom position onto the line defined by grasp surface position and approach vector
		Eigen::Vector3d s, b, a;
		position = (center_ - surface_center_).dot(approach) * approach;
		position += surface_center_;
	}
	else
		position = center_;
		
	// translate grasp position by <hand_offset_> along the grasp approach vector
	position = position + hand_offset * approach;
			
	geometry_msgs::PoseStamped pose_st;
	pose_st.header.stamp = ros::Time(0);
	pose_st.header.frame_id = frame_id;
	tf::pointEigenToMsg(position, pose_st.pose.position);
    tf::quaternionTFToMsg(quat, pose_st.pose.orientation);
	
	return pose_st;
}


bool GraspLocalizerServer::service_callback(agile_grasp::FindGrasps::Request &req, agile_grasp::FindGrasps::Response &res)
{
	//convert cloud to ROS
	sensor_msgs::PointCloud2 cloud = req.object_cloud;
	pcl::fromROSMsg(cloud, *cloud_left_);
	
	//hand offset approach
	//double hand_offset_approach = req.approach_offset;
	
	//compute grasp  points
	std::vector<int> indices(0);
	hands_ = localization_->localizeHands(cloud_left_, cloud_left_->size(), indices, false, false);
	antipodal_hands_ = localization_->predictAntipodalHands(hands_, svm_file_name_);
    handles_ = localization_->findHandles(antipodal_hands_, min_inliers_, 0.005);
    
    //create msg and fill in request
    agile_grasp::Grasps grasps = createGraspsMsg(handles_);
    res.grasps = grasps.grasps;
    
    for (unsigned int i = 0; i < grasps.grasps.size(); i++){
		geometry_msgs::PoseStamped p_grasp_i = graspToPose(grasps.grasps.at(i),HAND_OFFSET_GRASP,req.object_cloud.header.frame_id);
		geometry_msgs::PoseStamped p_approach_i = graspToPose(grasps.grasps.at(i),HAND_OFFSET_GRASP,req.object_cloud.header.frame_id);
		
		res.grasp_poses.push_back(p_grasp_i);
		res.approach_poses.push_back(p_approach_i);
		
	}
    
    
    //publish for visualuzation
    // publish handles
    grasps_handles_pub_.publish(grasps);
    ros::spinOnce();
      
    // publish hands contained in handles
    grasps_pub_.publish(createGraspsMsgFromHands(handles_));
    
    
    
    return true;
}


void GraspLocalizerServer::cloud_callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  if (num_clouds_received_ == num_clouds_)
    return;
  
  // get point cloud from topic
  if (cloud_frame_.compare(msg->header.frame_id) != 0 
			&& cloud_frame_.compare("/" + msg->header.frame_id) != 0)
  {
    std::cout << "Input cloud frame " << msg->header.frame_id << " is not equal to parameter " << cloud_frame_ 
			<< std::endl;
    std::exit(EXIT_FAILURE);
  }
  
  if (num_clouds_received_ == 0)
    pcl::fromROSMsg(*msg, *cloud_left_);
  else if (num_clouds_received_ == 1)
    pcl::fromROSMsg(*msg, *cloud_right_);
  std::cout << "Received cloud # " << num_clouds_received_ << " with " << msg->height * msg->width << " points\n";
  num_clouds_received_++;
}


void GraspLocalizerServer::cloud_sized_callback(const agile_grasp::CloudSized& msg)
{
  // get point cloud from topic
  if (cloud_frame_.compare(msg.cloud.header.frame_id) != 0)
  {
    std::cout << "Input cloud frame " << msg.cloud.header.frame_id << " is not equal to parameter "
      << cloud_frame_ << std::endl;
    std::exit(EXIT_FAILURE);
  }
  
  pcl::fromROSMsg(msg.cloud, *cloud_left_);
  size_left_ = msg.size_left.data;
  std::cout << "Received cloud with size_left: " << size_left_ << std::endl;
  num_clouds_received_ = 1;
}


void GraspLocalizerServer::localizeGrasps()
{
  ros::Rate rate(1);
  std::vector<int> indices(0);
  
  while (ros::ok())
  {
    // wait for point clouds to arrive
    if (num_clouds_received_ == num_clouds_)
    {
      // localize grasps
      if (num_clouds_ > 1)
      {
        PointCloud::Ptr cloud(new PointCloud());
        *cloud = *cloud_left_ + *cloud_right_;
        hands_ = localization_->localizeHands(cloud, cloud_left_->size(), indices, false, false);
      }
      else
      {
        hands_ = localization_->localizeHands(cloud_left_, cloud_left_->size(), indices, false, false);
			}
      
      antipodal_hands_ = localization_->predictAntipodalHands(hands_, svm_file_name_);
      handles_ = localization_->findHandles(antipodal_hands_, min_inliers_, 0.005);
      
      // publish handles
      grasps_handles_pub_.publish(createGraspsMsg(handles_));
      ros::Duration(1.0).sleep();
      
      // publish hands contained in handles
      grasps_pub_.publish(createGraspsMsgFromHands(handles_));
      ros::Duration(1.0).sleep();
      
      // reset
      num_clouds_received_ = 0;
    }
    
    ros::spinOnce();
    rate.sleep();
  }
}


agile_grasp::Grasps GraspLocalizerServer::createGraspsMsg(const std::vector<GraspHypothesis>& hands)
{
  agile_grasp::Grasps msg;
  
  for (int i = 0; i < hands.size(); i++)
	{
  	msg.grasps.push_back(createGraspMsg(hands[i]));
  }
  
  msg.header.stamp = ros::Time::now();  
  return msg;
}


agile_grasp::Grasp GraspLocalizerServer::createGraspMsg(const GraspHypothesis& hand)
{
  agile_grasp::Grasp msg;
  tf::vectorEigenToMsg(hand.getGraspBottom(), msg.center);
  tf::vectorEigenToMsg(hand.getAxis(), msg.axis);
  tf::vectorEigenToMsg(hand.getApproach(), msg.approach);
  tf::vectorEigenToMsg(hand.getGraspSurface(), msg.surface_center);
  msg.width.data = hand.getGraspWidth();
  return msg;
}


agile_grasp::Grasps GraspLocalizerServer::createGraspsMsgFromHands(const std::vector<Handle>& handles)
{
  agile_grasp::Grasps msg;  
  for (int i = 0; i < handles.size(); i++)
  {
    const std::vector<GraspHypothesis>& hands = handles[i].getHandList();
    const std::vector<int>& inliers = handles[i].getInliers();
    
    for (int j = 0; j < inliers.size(); j++)
    {
      msg.grasps.push_back(createGraspMsg(hands[inliers[j]]));
    }
  }
  msg.header.stamp = ros::Time::now();
  std::cout << "Created grasps msg containing " << msg.grasps.size() << " hands\n";
  return msg;
}


agile_grasp::Grasps GraspLocalizerServer::createGraspsMsg(const std::vector<Handle>& handles)
{
  agile_grasp::Grasps msg;  
  for (int i = 0; i < handles.size(); i++)
    msg.grasps.push_back(createGraspMsg(handles[i]));  
  msg.header.stamp = ros::Time::now();
  std::cout << "Created grasps msg containing " << msg.grasps.size() << " handles\n";
  return msg;
}


agile_grasp::Grasp GraspLocalizerServer::createGraspMsg(const Handle& handle)
{
  agile_grasp::Grasp msg;
  tf::vectorEigenToMsg(handle.getCenter(), msg.center);
  tf::vectorEigenToMsg(handle.getAxis(), msg.axis);
  tf::vectorEigenToMsg(handle.getApproach(), msg.approach);
  tf::vectorEigenToMsg(handle.getHandsCenter(), msg.surface_center);
  msg.width.data = handle.getWidth();
  return msg;
}
