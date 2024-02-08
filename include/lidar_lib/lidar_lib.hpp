#include <ros/ros.h>
#include <iostream>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <shape_msgs/SolidPrimitive.h>
#include "lidar_lib/KalmanTracker.h"
#include "lidar_lib/Hungarian.h"
#include "camera_lib/camera_lib.hpp" //include PointXYZV && OPENCV

using namespace std;

typedef pcl::PointXYZ PointType;

typedef struct 
{
	int id;
	Rect_<float> box;
}TrackingBox;

class Lidar_Func{
  private:
    ros::NodeHandle node_handle_;
    int cluster_size_min_, cluster_size_max_;
    double tolerance_;

    //SORT
    int box_x_offset_;
    int box_y_offset_;
    int max_age;
    int min_hits;
    double iouThreshold;
    vector<KalmanTracker> trackers;
  
  public:
    //CLUSTERING
    void clustering (const pcl::PointCloud<PointType>::Ptr cloud_in, vector<PointXYZV> &center_,double roi[5]);
    bool clustering_true(const pcl::PointCloud<PointType>::Ptr cloud_in,double roi[5]);
    void visualize_rviz(const vector<PointXYZV> &obj,const ros::Publisher &pub,const string &frame_id,const ros::Time &time);
    void rviz_clear(const ros::Publisher &pub, const string &frame_id,const ros::Time &time);
    //SORT
    double GetIOU(Rect_<float> bb_test, Rect_<float> bb_gt);
    void TestSORT(vector<PointXYZV> &point );
    void CameraSORT(vector<box_size> &point);
    
  
    Lidar_Func(){};
    Lidar_Func(ros::NodeHandle *nh) : node_handle_(*nh){
        ROS_INFO("Lidar_Func Start");
        node_handle_.param("/acc_controller/tolerance", tolerance_, 0.5);
        node_handle_.param("/acc_controller/cluster_size_max", cluster_size_max_, 2000);
        node_handle_.param("/acc_controller/cluster_size_min", cluster_size_min_, 3);
        node_handle_.param("/acc_controller/box_x_offset", box_x_offset_, 2);
        node_handle_.param("/acc_controller/box_y_offset", box_y_offset_, 2);
        node_handle_.param("/acc_controller/iouThreshold", iouThreshold, 0.3);
        ROS_INFO("tolerance: %f", tolerance_);
        ROS_INFO("cluster_size_max: %d", cluster_size_max_);
        ROS_INFO("cluster_size_min: %d", cluster_size_min_);
        ROS_INFO("box_x_offset: %d", box_x_offset_);
        ROS_INFO("box_y_offset: %d", box_y_offset_);
        ROS_INFO("iouThreshold: %f", iouThreshold);

        //SORT initialize
        max_age = 1;
        min_hits = 3;
    }
    ~Lidar_Func(){
        ROS_INFO("Lidar_Func Terminated");
    }
};

void Lidar_Func::clustering (const pcl::PointCloud<PointType>::Ptr cloud_in, vector<PointXYZV> &center_,double roi[5]){
//이부분
  pcl::VoxelGrid<PointType> vg;

  vg.setInputCloud (cloud_in);
  vg.setLeafSize (0.15f, 0.15f, 0.15f); // grid size: 5cm
  vg.filter (*cloud_in);

  pcl::PassThrough<PointType> pass; 

  pass.setInputCloud(cloud_in);
  pass.setFilterFieldName ("x");
  pass.setFilterLimits(roi[2], roi[1]);
  pass.filter(*cloud_in);

  pass.setInputCloud(cloud_in);
  pass.setFilterFieldName("y");
  pass.setFilterLimits(roi[4], roi[3]);
  pass.filter(*cloud_in);

  pass.setInputCloud(cloud_in);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits(-2.0, roi[0]);
  pass.filter(*cloud_in);


  pcl::search::KdTree<PointType>::Ptr kdtree(new pcl::search::KdTree<PointType>);
  kdtree -> setInputCloud(cloud_in);

  std::vector<pcl::PointIndices> clusterIndices;
  pcl::EuclideanClusterExtraction<PointType> ec;

  ec.setClusterTolerance(tolerance_);
  ec.setMinClusterSize(cluster_size_min_);
  ec.setMaxClusterSize(cluster_size_max_);
  ec.setSearchMethod(kdtree);
  ec.setInputCloud(cloud_in);
  ec.extract(clusterIndices);

  //ROS_INFO("cluster number %lu", clusterIndices.size());
  for (vector<pcl::PointIndices>::const_iterator it = clusterIndices.begin(); it != clusterIndices.end(); ++it) {
      pcl::PointCloud<PointType>::Ptr cluster_cloud (new pcl::PointCloud<PointType>);   
      for (vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit) {
        cluster_cloud->points.emplace_back(cloud_in->points[*pit]); 
      }            
      PointXYZV P;
      pcl::PointXYZ min_pt, max_pt;
      pcl::getMinMax3D (*cluster_cloud, min_pt, max_pt);
      P.width = max_pt.x - min_pt.x;
      P.height = max_pt.y - min_pt.y;
      P.z_height = max_pt.z - min_pt.z;
      P.x = min_pt.x;
      P.y = (max_pt.y + min_pt.y)/2;
      P.z = (max_pt.z + min_pt.z)/2;
      P.id = -1;
      P.vel = 0;
      if (P.width < 5 && P.height < 10) center_.emplace_back(P);
  }
}

bool Lidar_Func::clustering_true(const pcl::PointCloud<PointType>::Ptr cloud_in,double roi[5]){
  pcl::VoxelGrid<PointType> vg;

  vg.setInputCloud (cloud_in);
  vg.setLeafSize (0.1f, 0.1f, 0.1f); // grid size: 5cm
  vg.filter (*cloud_in);

  pcl::PassThrough<PointType> pass; 

  pass.setInputCloud(cloud_in);
  pass.setFilterFieldName ("x");
  pass.setFilterLimits(roi[2], roi[1]);
  pass.filter(*cloud_in);

  pass.setInputCloud(cloud_in);
  pass.setFilterFieldName("y");
  pass.setFilterLimits(roi[4], roi[3]);
  pass.filter(*cloud_in);

  pass.setInputCloud(cloud_in);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits(-2.0, roi[0]);
  pass.filter(*cloud_in);

  pcl::search::KdTree<PointType>::Ptr kdtree(new pcl::search::KdTree<PointType>);
  kdtree -> setInputCloud(cloud_in);

  std::vector<pcl::PointIndices> clusterIndices;
  pcl::EuclideanClusterExtraction<PointType> ec;

  ec.setClusterTolerance(tolerance_);
  ec.setMinClusterSize(cluster_size_min_);
  ec.setMaxClusterSize(cluster_size_max_);
  ec.setSearchMethod(kdtree);
  ec.setInputCloud(cloud_in);
  ec.extract(clusterIndices);
  
  if(clusterIndices.size()>0) return true;
  return false;
}

void Lidar_Func::visualize_rviz(const vector<PointXYZV> &obj, const ros::Publisher &pub, const string &frame_id,const ros::Time &time) {
  visualization_msgs::MarkerArray boxes;

  visualization_msgs::Marker obj_pt;
	geometry_msgs::Point point;
	obj_pt.header.frame_id = frame_id;
	obj_pt.header.stamp = time;
	obj_pt.ns ="center_points";
	obj_pt.action = visualization_msgs::Marker::ADD;
	obj_pt.pose.orientation.w = 1.0;
	obj_pt.id = 2;
	obj_pt.type = visualization_msgs::Marker::POINTS;
	obj_pt.scale.x = 0.3;
	obj_pt.scale.y = 0.3;
	obj_pt.color.a = 1.0;
	obj_pt.color.g = 1.0f;
  obj_pt.lifetime = ros::Duration(0.000000001);

  uint32_t shape = visualization_msgs::Marker::CUBE; //shape = CUBE (msg상 uint8 CUBE = 1) 큐브 모양 
	visualization_msgs::Marker marker; //vis msg marker 선언 
	marker.header.frame_id = frame_id;
	marker.header.stamp = time;
	marker.ns = "bounding_boxes";
	marker.type = shape; //shape = 1
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.orientation.w = 1.0; //Quaternion 회전 방향, w는 스칼라값(회전의 각도와 방향, -1 <= w <= 1), 나머지 x,y,z는 회전축
	marker.color.r = 1.0; 
  marker.color.g = 0.65; 
	marker.color.a = 0.3;
  marker.lifetime = ros::Duration(0.000000001);

  visualization_msgs::Marker node_name;
  node_name.header.frame_id = frame_id; // map frame 기준
  node_name.header.stamp = time;
  node_name.ns = "ID"; 
  node_name.color.a = 1.0;
  node_name.scale.z = 1.0;
  node_name.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  node_name.action = visualization_msgs::Marker::ADD;
  node_name.pose.orientation.w = 1.0;
  node_name.lifetime = ros::Duration(0.000000001);

  visualization_msgs::Marker node_name_2;
  node_name_2.header.frame_id = frame_id; // map frame 기준
  node_name_2.header.stamp = time;
  node_name_2.ns = "vel"; 
  node_name_2.color.a = 1.0;
  node_name_2.scale.z = 1.0;
  node_name_2.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  node_name_2.action = visualization_msgs::Marker::ADD;
  node_name_2.pose.orientation.w = 1.0;
  node_name_2.lifetime = ros::Duration(0.000000001);

	for (int i = 0 ; i<obj.size();i++){
		point.x = (obj)[i].x;
		point.y = (obj)[i].y;
		point.z = (obj)[i].z;
		obj_pt.points.emplace_back(point);

    if(obj[i].id!=-1){
    node_name.text = "id: " + std::to_string(obj[i].id);
    node_name.pose.position.x = obj[i].x;
    node_name.pose.position.y = obj[i].y;
    node_name.pose.position.z = 2;
    node_name.id = i;
    boxes.markers.emplace_back(node_name);
    }

    string str = std::to_string(obj[i].vel);
    str = str.substr(0, 5);
    node_name_2.text = "speed: " + str;
    node_name_2.pose.position.x = obj[i].x;
    node_name_2.pose.position.y = obj[i].y;
    node_name_2.pose.position.z = 4;
    node_name_2.id = i;
    boxes.markers.emplace_back(node_name_2);
    

    // create a 3D bounding box
    shape_msgs::SolidPrimitive bbox;
    bbox.type = shape_msgs::SolidPrimitive::BOX;
    bbox.dimensions.resize (3);
    bbox.dimensions[0] = obj[i].width;
    bbox.dimensions[1] = obj[i].height;
    bbox.dimensions[2] = obj[i].z_height;
    // publish the bounding box as a ROS message
    marker.pose.position.x = obj[i].x;
    marker.pose.position.y = obj[i].y;
    marker.pose.position.z = obj[i].z;
    marker.pose.orientation.w = 1.0;
    marker.id = i;
    marker.scale.x = bbox.dimensions[0];
    marker.scale.y = bbox.dimensions[1];
    marker.scale.z = bbox.dimensions[2];
    boxes.markers.emplace_back(marker);
	}
  boxes.markers.emplace_back(obj_pt);

	pub.publish(boxes);
}

void Lidar_Func::rviz_clear(const ros::Publisher &pub, const string &frame_id,const ros::Time &time) {
  visualization_msgs::MarkerArray boxes;
  visualization_msgs::Marker node_name;
  node_name.header.frame_id = frame_id; // map frame 기준
  node_name.header.stamp = time;
  node_name.ns = "DEL"; 
  node_name.color.a = 1.0;
  node_name.scale.z = 1.0;
  node_name.action = visualization_msgs::Marker::DELETEALL;
  boxes.markers.emplace_back(node_name);
  pub.publish(boxes);
}

double Lidar_Func::GetIOU(Rect_<float> bb_test, Rect_<float> bb_gt)
{
	float in = (bb_test & bb_gt).area();
	float un = bb_test.area() + bb_gt.area() - in;

	if (un < DBL_EPSILON)
		return 0;

	return (double)(in / un);
}

void Lidar_Func::TestSORT(vector<PointXYZV> &point)
{
	vector<TrackingBox> detData;
	vector<Rect_<float>> predictedBoxes;
	vector<vector<double>> iouMatrix;
	vector<int> assignment;
	set<int> unmatchedDetections;
	set<int> unmatchedTrajectories;
	set<int> allItems;
	set<int> matchedItems;
	vector<cv::Point> matchedPairs;
	vector<TrackingBox> frameTrackingResult;
	unsigned int trkNum = 0;
	unsigned int detNum = 0;

	for (int i = 0 ; i< point.size(); i++){
		// TrackingBox tb;
		// tb.id = -1;
		// float tpx = point[i].x;
		// float tpy = point[i].y-(point[i].height)/2;
		// float tpw = point[i].width;
    // if (tpw < 7.0) tpw = 7.0;
    // tpw = tpw*box_x_offset_;

		// float tph = point[i].height;
    // if (tpw < 4.0) tpw = 4.0;
    // tph = tph*box_y_offset_;

		// tb.box = Rect_<float>(tpx,tpy,tpw,tph);
		// detData.emplace_back(tb);

    //=========================MAKE CENTER BOX=========================
    TrackingBox tb;
		tb.id = -1;
		float tpx = point[i].x-(point[i].width/2)*(box_x_offset_-1);
		float tpy = point[i].y-(point[i].height/2)*(box_y_offset_-1);
		float tpw = point[i].width;
    if (tpw < 7.0) tpw = 7.0;
    tpw = tpw*box_x_offset_;

		float tph = point[i].height;
    if (tpw < 4.0) tpw = 4.0;

    tph = tph*box_y_offset_;
		tb.box = Rect_<float>(tpx,tpy,tpw,tph);
		detData.emplace_back(tb);
	}

	// ===========================main loop==============================
		if (trackers.size() == 0) // the first frame met
		{
			// initialize kalman trackers using first detections.
			for (unsigned int i = 0; i < detData.size(); i++)
			{
				KalmanTracker trk = KalmanTracker(detData[i].box);
				trackers.emplace_back(trk);
			}
			
			return;
		}

		// 3.1. get predicted locations from existing trackers.
		for (auto it = trackers.begin(); it != trackers.end();)
		{
			Rect_<float> pBox = (*it).predict();
			if (pBox.x>-50.0){
				predictedBoxes.emplace_back(pBox);
				it++;
			}
			else
			{
				it = trackers.erase(it);
			}
		}

		// 3.2. associate detections to tracked object (both represented as bounding boxes)
		trkNum = predictedBoxes.size();
		detNum = detData.size();
		iouMatrix.resize(trkNum, vector<double>(detNum, 0));

		for (unsigned int i = 0; i < trkNum; i++) // compute iou matrix as a distance matrix
		{
			for (unsigned int j = 0; j < detNum; j++)
			{
				// use 1-iou because the hungarian algorithm computes a minimum-cost assignment.
				iouMatrix[i][j] = 1 - GetIOU(predictedBoxes[i], detData[j].box);
			}
		}
    
    if(iouMatrix.size()>0){
  		// solve the assignment problem using hungarian algorithm.
	  	// the resulting assignment is [track(prediction) : detection], with len=preNum
      HungarianAlgorithm HungAlgo;
      HungAlgo.Solve(iouMatrix, assignment);
    }

		if (detNum > trkNum) //	there are unmatched detections
		{
			for (unsigned int n = 0; n < detNum; n++)
				allItems.insert(n);

			for (unsigned int i = 0; i < trkNum; ++i)
				matchedItems.insert(assignment[i]);

			set_difference(allItems.begin(), allItems.end(),
			matchedItems.begin(), matchedItems.end(),
			insert_iterator<set<int>>(unmatchedDetections, unmatchedDetections.begin()));
		}
		else if (detNum < trkNum) // there are unmatched trajectory/predictions
		{
			for (unsigned int i = 0; i < trkNum; ++i)
				if (assignment[i] == -1) // unassigned label will be set as -1 in the assignment algorithm
					unmatchedTrajectories.insert(i);
		}
		else
			;
		// filter out matched with low IOU
		matchedPairs.clear();
		for (unsigned int i = 0; i < trkNum; ++i)
		{
			if (assignment[i] == -1) // pass over invalid values
				continue;
			if (1 - iouMatrix[i][assignment[i]] < iouThreshold)
			{
				unmatchedTrajectories.insert(i);
				unmatchedDetections.insert(assignment[i]);
			}
			else
				matchedPairs.emplace_back(cv::Point(i, assignment[i]));
		}

		// 3.3. updating trackers
		// update matched trackers with assigned detections.
		// each prediction is corresponding to a tracker
		int detIdx, trkIdx;
		for (unsigned int i = 0; i < matchedPairs.size(); i++)
		{
			trkIdx = matchedPairs[i].x;
			detIdx = matchedPairs[i].y;
			trackers[trkIdx].update(detData[detIdx].box); // %%%%%%% maybe I can get velocity using this part
      point[detIdx].id = trackers[trkIdx].m_id;
		}

		// create and initialise new trackers for unmatched detections
		for (auto umd : unmatchedDetections)
		{
			KalmanTracker tracker = KalmanTracker(detData[umd].box);
			trackers.emplace_back(tracker);
		}

		// get trackers' output
		frameTrackingResult.clear();
		for (auto it = trackers.begin(); it != trackers.end();)
		{
			
			if (((*it).m_time_since_update < 1) && ((*it).m_hit_streak >= min_hits))
			{
				TrackingBox res;
				res.box = (*it).get_state();
				res.id = (*it).m_id + 1;
				frameTrackingResult.emplace_back(res);
				it++;
			}
			else
				it++;

			// remove dead tracklet
			if (it != trackers.end() && (*it).m_time_since_update > max_age)
				it = trackers.erase(it);
		}
		if (trackers.size() >= 500) trackers.clear();
}

void Lidar_Func::CameraSORT(vector<box_size> &point)
{
	vector<TrackingBox> detData;
	vector<Rect_<float>> predictedBoxes;
	vector<vector<double>> iouMatrix;
	vector<int> assignment;
	set<int> unmatchedDetections;
	set<int> unmatchedTrajectories;
	set<int> allItems;
	set<int> matchedItems;
	vector<cv::Point> matchedPairs;
	vector<TrackingBox> frameTrackingResult;
	unsigned int trkNum = 0;
	unsigned int detNum = 0;

	for (int i = 0 ; i< point.size(); i++){
		TrackingBox tb;
		tb.id = -1;
		float tpx = point[i].x_coord;
		float tpy = point[i].y_coord;
		float tpw = point[i].w_coord*box_x_offset_;
		float tph = point[i].h_coord*box_y_offset_;
		tb.box = Rect_<float>(tpx,tpy,tpw,tph);
		detData.emplace_back(tb);
	}

	// ===========================main loop==============================
		if (trackers.size() == 0) // the first frame met
		{
			// initialize kalman trackers using first detections.
			for (unsigned int i = 0; i < detData.size(); i++)
			{
				KalmanTracker trk = KalmanTracker(detData[i].box);
				trackers.emplace_back(trk);
			}
			
			return;
		}

		// 3.1. get predicted locations from existing trackers.
		for (auto it = trackers.begin(); it != trackers.end();)
		{
			Rect_<float> pBox = (*it).predict();
			if (pBox.x>0 && pBox.y>0){
				predictedBoxes.emplace_back(pBox);
				it++;
			}
			else
			{
				it = trackers.erase(it);
			}
		}

		// 3.2. associate detections to tracked object (both represented as bounding boxes)
		trkNum = predictedBoxes.size();
		detNum = detData.size();
		iouMatrix.resize(trkNum, vector<double>(detNum, 0));

		for (unsigned int i = 0; i < trkNum; i++) // compute iou matrix as a distance matrix
		{
			for (unsigned int j = 0; j < detNum; j++)
			{
				// use 1-iou because the hungarian algorithm computes a minimum-cost assignment.
				iouMatrix[i][j] = 1 - GetIOU(predictedBoxes[i], detData[j].box);
			}
		}
    
    if(iouMatrix.size()>0){
  		// solve the assignment problem using hungarian algorithm.
	  	// the resulting assignment is [track(prediction) : detection], with len=preNum
      HungarianAlgorithm HungAlgo;
      HungAlgo.Solve(iouMatrix, assignment);
    }

		if (detNum > trkNum) //	there are unmatched detections
		{
			for (unsigned int n = 0; n < detNum; n++)
				allItems.insert(n);

			for (unsigned int i = 0; i < trkNum; ++i)
				matchedItems.insert(assignment[i]);

			set_difference(allItems.begin(), allItems.end(),
			matchedItems.begin(), matchedItems.end(),
			insert_iterator<set<int>>(unmatchedDetections, unmatchedDetections.begin()));
		}
		else if (detNum < trkNum) // there are unmatched trajectory/predictions
		{
			for (unsigned int i = 0; i < trkNum; ++i)
				if (assignment[i] == -1) // unassigned label will be set as -1 in the assignment algorithm
					unmatchedTrajectories.insert(i);
		}
		else
			;
		// filter out matched with low IOU
		matchedPairs.clear();
		for (unsigned int i = 0; i < trkNum; ++i)
		{
			if (assignment[i] == -1) // pass over invalid values
				continue;
			if (1 - iouMatrix[i][assignment[i]] < iouThreshold)
			{
				unmatchedTrajectories.insert(i);
				unmatchedDetections.insert(assignment[i]);
			}
			else
				matchedPairs.emplace_back(cv::Point(i, assignment[i]));
		}

		// 3.3. updating trackers
		// update matched trackers with assigned detections.
		// each prediction is corresponding to a tracker
		int detIdx, trkIdx;
		for (unsigned int i = 0; i < matchedPairs.size(); i++)
		{
			trkIdx = matchedPairs[i].x;
			detIdx = matchedPairs[i].y;
			trackers[trkIdx].update(detData[detIdx].box); // %%%%%%% maybe I can get velocity using this part


		// create and initialise new trackers for unmatched detections
		for (auto umd : unmatchedDetections)
		{
			KalmanTracker tracker = KalmanTracker(detData[umd].box);
			trackers.emplace_back(tracker);
		}

   		// // get trackers' output
		frameTrackingResult.clear();
		for (auto it = trackers.begin(); it != trackers.end();)
		{
			
			if (((*it).m_time_since_update < 1) && ((*it).m_hit_streak >= min_hits))
			{
				TrackingBox res;
				res.box = (*it).get_state();
				res.id = (*it).m_id + 1;
				frameTrackingResult.emplace_back(res);
				it++;
			}
			else
				it++;

			// remove dead tracklet
			if (it != trackers.end() && (*it).m_time_since_update > max_age)
				it = trackers.erase(it);
		}
		}
}