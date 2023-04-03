#pragma once
#ifndef _DISTRIBUTED_MAPPING_H_
#define _DISTRIBUTED_MAPPING_H_
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <flann/flann.hpp>
#include <thread>
#include <deque>
// dlc_slam define
#include "descriptor.h"
#include "dlc_slam/loop_info.h"
#include "dlc_slam/global_descriptor.h"
#include "dlc_slam/geometric_verification.h"
#include "dlc_slam/save_distributed_trajectory.h"
// pcl
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
// mapping
#include "distributed_mapper/distributed_mapper.h"
#include "distributed_mapper/distributed_mapper_utils.h"
#include <gtsam/nonlinear/ISAM2.h>
// file iostream
#include <fstream>
#include <iostream>

using namespace gtsam;
using namespace std;

typedef pcl::PointXYZI PointPose3D;
struct PointPose6D
{
    float x;
	float y;
	float z;
	float intensity;
    float roll;
    float pitch;
    float yaw;
    double time;
};
POINT_CLOUD_REGISTER_POINT_STRUCT  (PointPose6D,
                                   (float, x, x) (float, y, y)
                                   (float, z, z) (float, intensity, intensity)
                                   (float, roll, roll) (float, pitch, pitch) (float, yaw, yaw)
                                   (double, time, time))

struct singleRobot {
	/*** robot information ***/
	int id; // robot id
	std::string name; // robot name, for example, 'a', 'b', etc.
	std::string odomFrame; // odom frame

	/*** ros subscriber and publisher ***/
	ros::Subscriber subCloudInfo; // pointcloud from real-time odometry
	ros::Subscriber subDescriptors; // descriptors from other robot
	ros::Publisher pubDescriptors; // publish this robot's descriptors
	ros::Publisher pubKeyFrame; // publish this robot's keyframe
	ros::Publisher pubGlobalMap;
	ros::Publisher pubPath; // publish this robot's path
	ros::Publisher pubtestPath;
	ros::ServiceClient clientGeometricVerification; // client for geometric verification service

	/*** message information ***/
	ros::Time timeCloudInputStamp; // recent keyframe timestamp
	double timeCloudInputCur; // and its double type
	std::vector<double> poseTimeStampVector; // and its array

	nav_msgs::Path localPath; // path in local frame
	nav_msgs::Path globalPath; // path in global frame

	pcl::PointCloud<PointPose3D>::Ptr cloudKeyPoses3D; // 3D poses
	pcl::PointCloud<PointPose6D>::Ptr cloudKeyPoses6D; // 6D poses

	geometry_msgs::Transform keyPose; // recent keyframe pose for independent robot
	pcl::PointCloud<PointPose3D>::Ptr keyFrame; // recent keyframe pointcloud
	std::vector<pcl::PointCloud<PointPose3D>> keyFrameArray; // and its array

	/*** optmazition ***/
	Pose3 pirorOdom; // piror factor
	NonlinearFactorGraph::shared_ptr gtSAMsubgraph; // subgraph
	NonlinearFactorGraph::shared_ptr connectSubgraph; // subgraph
	Values::shared_ptr initial; // initial guess
	Values::shared_ptr connectInitial; // initial guess

	boost::shared_ptr<distributed_mapper::DistributedMapper> distributedMapper;
};

enum class LiDARType { VELODYNE, LIVOX };
enum class DescriptorType { ScanContext, LidarIris, M2DP, GRSD, FPFH};

class paramsServer
{
public:
	paramsServer()
	{
		// Robot info
		std::string ns = nh.getNamespace(); // namespace of robot
		if(ns.length() != 2)
		{
			ROS_ERROR("Invalid robot prefix (should be either 'a-z' or 'A-Z'): %s", ns.c_str());
			ros::shutdown();
		}
		name = ns.substr(1, 1); // leave '/'
		id = name[0]-'a';

		nh.param<int>("/numberOfRobots", numberOfRobots, 1);
		if(numberOfRobots < 1)
		{
			ROS_ERROR("Invalid robot number (must be positive number): %d", numberOfRobots);
			ros::shutdown();
		}

		ns += "/dlc_slam";
		// Frames name
		nh.param<std::string>(ns + "/worldFrame", worldFrame, "world");
		nh.param<std::string>(ns + "/odomFrame", odomFrame, "map");

		// Lidar Sensor Configuration
		std::string sensorStr;
		nh.param<std::string>(ns + "/sensor", sensorStr, "velodyne");
		if(sensorStr == "velodyne")
		{
			sensor = LiDARType::VELODYNE;
		}
		else if(sensorStr == "livox")
		{
			sensor = LiDARType::LIVOX;
		}
		else
		{
			ROS_ERROR("Invalid sensor type (must be either 'velodyne' or 'ouster'): %s ", sensorStr.c_str());
			ros::shutdown();
		}
		nh.param<int>(ns + "/N_SCAN", N_SCAN, 16);

		// Mapping
		nh.param<bool>(ns + "/usePCM", usePCM, false);
		nh.param<int>(ns + "/maxIterationTime", maxIterationTime, 100);
		nh.param<bool>(ns + "/globalOptmizationEnableFlag", globalOptmizationEnableFlag, true);
		nh.param<float>(ns + "/mappingLeafSize", mappingLeafSize, 0.2);
		nh.param<float>(ns + "/mappingProcessInterval", mappingProcessInterval, 0.15);

		// Loop closure
		nh.param<bool>(ns + "/intraLoopClosureEnableFlag", intraLoopClosureEnableFlag, true);
		nh.param<bool>(ns + "/interLoopClosureEnableFlag", interLoopClosureEnableFlag, true);
		std::string descriptorType;
		nh.param<std::string>(ns + "/descriptorType", descriptorType, "");
		if(descriptorType == "ScanContext")
		{
			descriptor = DescriptorType::ScanContext;
		}
		else if(descriptorType == "LidarIris")
		{
			descriptor = DescriptorType::LidarIris;
		}
		else if(descriptorType == "M2DP")
		{
			descriptor = DescriptorType::M2DP;
		}
		else if(descriptorType == "GRSD")
		{
			descriptor = DescriptorType::GRSD;
		}
		else if(descriptorType == "FPFH")
		{
			descriptor = DescriptorType::FPFH;
		}
		else
		{
			interLoopClosureEnableFlag = false;
			ROS_WARN("Invalid descriptor type: %s, turn off interloop...", descriptorType.c_str());
		}
		nh.param<int>(ns + "/candidate", candidate, 10);
		nh.param<int>(ns + "/matchMode", matchMode, 2);
		nh.param<float>(ns + "/descriptLeafSize", descriptLeafSize, 0.4);
		nh.param<float>(ns + "/historyKeyframeSearchRadius", historyKeyframeSearchRadius, 5.0);
		nh.param<int>(ns + "/ransacMaxIter", ransacMaxIter, 1000);
		nh.param<float>(ns + "/ransacOutlierTreshold", ransacOutlierTreshold, 0.25);
		nh.param<float>(ns + "/inlierTreshold", inlierTreshold, 0.45);
		nh.param<float>(ns + "/historyKeyframeSearchTimeDiff", historyKeyframeSearchTimeDiff, 30.0);
		nh.param<int>(ns + "/historyKeyframeSearchNum", historyKeyframeSearchNum, 25);
		nh.param<float>(ns + "/historyKeyframeFitnessScore", historyKeyframeFitnessScore, 0.3);

		// Surrounding map
		nh.param<float>(ns + "/keyframeAddingDistThreshold", keyframeAddingDistThreshold, 1.0);
		nh.param<float>(ns + "/keyframeAddingAngleThreshold", keyframeAddingAngleThreshold, 0.2);

		// CPU Params
		nh.param<int>(ns + "/onboardCpuCores", onboardCpuCores, 4);
		nh.param<float>(ns + "/mappingProcessInterval", mappingProcessInterval, 0.15);
		nh.param<float>(ns + "/intraLoopClosureProcessInterval", intraLoopClosureProcessInterval, 0.1);
		nh.param<float>(ns + "/interLoopClosureProcessInterval", interLoopClosureProcessInterval, 0.5);
		nh.param<float>(ns + "/mapPublishInterval", mapPublishInterval, 2.5);
		nh.param<float>(ns + "/tfPublishInterval", tfPublishInterval, 0.1);
	}


	~paramsServer()
	{

	}


    gtsam::Pose3 pclPointTogtsamPose3(PointPose6D thisPoint)
    {
        return gtsam::Pose3(gtsam::Rot3::RzRyRx(double(thisPoint.roll), double(thisPoint.pitch), double(thisPoint.yaw)),
                                  gtsam::Point3(double(thisPoint.x),    double(thisPoint.y),     double(thisPoint.z)));
    }


    Eigen::Affine3f pclPointToAffine3f(PointPose6D thisPoint)
    { 
        return pcl::getTransformation(thisPoint.x, thisPoint.y, thisPoint.z, thisPoint.roll, thisPoint.pitch, thisPoint.yaw);
    }


	gtsam::Pose3 transformToGTSAMpose(const geometry_msgs::Transform& pose)
	{
		return gtsam::Pose3(gtsam::Rot3::Quaternion(pose.rotation.w, pose.rotation.x, pose.rotation.y, pose.rotation.z), 
							gtsam::Point3(pose.translation.x, pose.translation.y, pose.translation.z));
	}


	pcl::PointCloud<PointPose3D>::Ptr transformPointCloud(pcl::PointCloud<PointPose3D> cloudIn, PointPose6D* pose)
	{
		pcl::PointCloud<PointPose3D>::Ptr cloudOut(new pcl::PointCloud<PointPose3D>());

		int cloudSize = cloudIn.size();
		cloudOut->resize(cloudSize);

		Eigen::Affine3f transCur = pcl::getTransformation(pose->x, pose->y, pose->z, pose->roll, pose->pitch, pose->yaw);
		
		#pragma omp parallel for num_threads(onboardCpuCores)
		for(int i = 0; i < cloudSize; ++i)
		{
			const auto &pointFrom = cloudIn.points[i];
			cloudOut->points[i].x = transCur(0,0) * pointFrom.x + transCur(0,1) * pointFrom.y + transCur(0,2) * pointFrom.z + transCur(0,3);
			cloudOut->points[i].y = transCur(1,0) * pointFrom.x + transCur(1,1) * pointFrom.y + transCur(1,2) * pointFrom.z + transCur(1,3);
			cloudOut->points[i].z = transCur(2,0) * pointFrom.x + transCur(2,1) * pointFrom.y + transCur(2,2) * pointFrom.z + transCur(2,3);
			cloudOut->points[i].intensity = pointFrom.intensity;
		}
		return cloudOut;
	}


    ros::NodeHandle nh;

	// Robot info
    std::string name;
	int id;
	int numberOfRobots;

	// Frames name
	std::string worldFrame;
	std::string odomFrame;

	// Lidar Sensor Configuration
    LiDARType sensor;
    int N_SCAN;

	// voxel filter paprams
    float mappingLeafSize;
	float descriptLeafSize;

    // CPU Params
    int onboardCpuCores;
    float mappingProcessInterval;
	float mapPublishInterval;
	float tfPublishInterval;
	float intraLoopClosureProcessInterval;
	float interLoopClosureProcessInterval;

	// enable functions
    bool intraLoopClosureEnableFlag;
	bool interLoopClosureEnableFlag;
	bool globalOptmizationEnableFlag;

	// Surrounding map
    float keyframeAddingDistThreshold; 
    float keyframeAddingAngleThreshold; 

	// Loop closure
	DescriptorType descriptor;
	int candidate;
	int matchMode;
	bool usePCM;
	int maxIterationTime;
    float historyKeyframeSearchRadius;
	int ransacMaxIter;
	float ransacOutlierTreshold;
	float inlierTreshold;
    float historyKeyframeSearchTimeDiff;
    int historyKeyframeSearchNum;
    float historyKeyframeFitnessScore;

	// Save pcd
	std::string saveDistributedTrajectoryDirectory;
};


class distributed_mapping : public paramsServer
{
public:
	/*** robot team ***/
	std::vector<singleRobot> robots;

    /*** ros subscriber and publisher ***/
	ros::Publisher pubLoopClosureConstraint;
	// service
	ros::ServiceServer srvGeometricVerification;
	ros::ServiceServer srvSaveDistributedTrajectory;

	/*** message information ***/
	dlc_slam::global_descriptor globalDescriptorMsg;

	/*** mutex ***/
	std::vector<std::mutex> lockOnCallback; // lock on odometry infomation callback
	std::mutex mtxSC;

	/*** distributed loopclosure ***/
	int intraLoopPtr;
	int interLoopPtr;
	std::unique_ptr<scan_descriptor> scanDescriptor;

	std::deque<dlc_slam::loop_info> loopInfoQueue;
	std::deque<dlc_slam::loop_info> loopInfoCheck;
	bool** connected; // transform flag

	pcl::PointCloud<PointPose3D>::Ptr laserCloudDesDS;
	pcl::VoxelGrid<PointPose3D> downSizeFilterDes;

    pcl::VoxelGrid<PointPose3D> downSizeFilterICP;

	std::map<gtsam::Symbol, gtsam::Symbol> loopIndexContainer; // history loop closure index pair [fromIndex,toIndex]

	// RS loopclosure
	pcl::PointCloud<PointPose3D>::Ptr copyCloudKeyPoses3D;
	pcl::PointCloud<PointPose6D>::Ptr copyCloudKeyPoses6D;
	pcl::KdTreeFLANN<PointPose3D>::Ptr kdtreeHistoryKeyPoses;

	/*** pose graph optmazition ***/
	double rostimeLastOpt;
	bool loopCloseFlag; // optmization is needed

	noiseModel::Diagonal::shared_ptr odometryNoise; // odometry factor noise
	noiseModel::Diagonal::shared_ptr priorNoise; // prior factor noise
	
	// isam2
    ISAM2 *isam2;
    NonlinearFactorGraph iGraph;
    Values iInitial;
	Values isamEstimates;
	gtsam::Pose3 keyPoseEstimate;

	// distributed mapper
	bool DMdebug; // print debug message
	size_t maxIter;  // Maximum number of iterations of optimizer
	double rotationEstimateChangeThreshold;  // Difference between rotation estimate provides an early stopping condition
	double poseEstimateChangeThreshold; // Difference between pose estimate provides an early stopping condition
	double gamma; // Gamma value for over relaxation methods
	bool useFlaggedInit; // to use flagged initialization or not
	// update_type differetiates between Distributed Jacobi/Jacobi OverRelaxation (postUpdate) and Gauss-Seidel/Successive OverRelaxation (incUpdate)
	distributed_mapper::DistributedMapper::UpdateType updateType;
	bool useLandmarks; // use landmarks -- landmarks are given symbols as upper case of robot name
	double pcmThreshold; // confidence probability for the pairwise consistency computation.
	bool useCovariance; // use covariance in dataset file.
	bool useHeuristics; // Use heuristics-based algorithm for the max-clique solver.

	std::vector<Values> estimates; // estimations









	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
		class distributed_mapping: constructor and destructor
	* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
    distributed_mapping() : paramsServer()
    {
		/*** robot team ***/
		singleRobot robot; // each robot
		connected = new bool*[numberOfRobots]; // allocate memory
		for(int i = 0; i < numberOfRobots; i++)
        {
			connected[i] = new bool[numberOfRobots];
		}

		if(descriptor == DescriptorType::ScanContext)
		{
			scanDescriptor = std::unique_ptr<scan_descriptor>(new scan_context_descriptor());
		}
		else if(descriptor == DescriptorType::LidarIris)
		{
			scanDescriptor = std::unique_ptr<scan_descriptor>(new lidar_iris_descriptor(80,360,64,0.32,30,matchMode,candidate,4,18,1.6,0.75,numberOfRobots,id));
		}
		else if(descriptor == DescriptorType::M2DP)
		{
			scanDescriptor = std::unique_ptr<scan_descriptor>(new m2dp_descriptor());
		}
		else if(descriptor == DescriptorType::GRSD)
		{
			scanDescriptor = std::unique_ptr<scan_descriptor>(new grsd_descriptor());
		}
		else if(descriptor == DescriptorType::FPFH)
		{
			scanDescriptor = std::unique_ptr<scan_descriptor>(new fpfh_descriptor());
		}

        for(int it = 0; it < numberOfRobots; it++)
        {
			/*** robot information ***/
            robot.id = it; // robot ID and name
            robot.name = "/a";
			robot.name[1] += it;
			robot.odomFrame = robot.name + "/" + odomFrame; // odom frame

			/*** ros subscriber and publisher ***/
			// this robot
			if(it == id)
			{
				// publish global descriptor
				robot.pubDescriptors = nh.advertise<dlc_slam::global_descriptor>(robot.name+"/distributedMapping/globalDescriptor", 1);
				// cloud infomation
				// robot.subCloudInfo = nh.subscribe<dlc_slam::cloud_info>(robot.name+"/feature/cloud_info", 1, &distributed_mapping::laserCloudInfoHandler, this, ros::TransportHints().tcpNoDelay());
				connected[it][id] = true;
			}
			// other robot
			else
			{
				// client for geometric verification service
				robot.clientGeometricVerification = nh.serviceClient<dlc_slam::geometric_verification>(robot.name+"/distributedMapping/geometricVerification");
				// subscribe global descriptor
				robot.subDescriptors = nh.subscribe<dlc_slam::global_descriptor>(robot.name+"/distributedMapping/globalDescriptor", 50, boost::bind(&distributed_mapping::globalDescriptorHandler,this,_1,it));
				connected[it][id] = false;
			}

			// recent keyframe pointcloud added in global map
        	robot.pubKeyFrame = nh.advertise<sensor_msgs::PointCloud2>(robot.name+"/distributedMapping/keyframeCloud", 1);
			robot.pubGlobalMap = nh.advertise<sensor_msgs::PointCloud2>(robot.name+"/distributedMapping/globalMap", 1);

			// path for independent robot
        	robot.pubPath = nh.advertise<nav_msgs::Path>("distributedMapping/"+robot.name+"Path", 1);
			robot.pubtestPath = nh.advertise<nav_msgs::Path>("distributedMapping/"+robot.name+"testPath", 1);

			/*** message information ***/
			robot.timeCloudInputStamp.init();
			robot.poseTimeStampVector.clear();

			robot.globalPath.poses.clear();
			robot.localPath.poses.clear();

			robot.cloudKeyPoses3D.reset(new pcl::PointCloud<PointPose3D>());
			robot.cloudKeyPoses6D.reset(new pcl::PointCloud<PointPose6D>());

			robot.keyFrame.reset(new pcl::PointCloud<PointPose3D>());
			robot.keyFrameArray.clear();
			// if(it == id) robot.keyFrameArray.resize(2000);

			/*** optmazition ***/
			robot.gtSAMsubgraph.reset(new NonlinearFactorGraph);
			robot.connectSubgraph.reset(new NonlinearFactorGraph);
    		robot.connectInitial.reset(new Values);
    		robot.initial.reset(new Values);

			robot.distributedMapper = std::unique_ptr<distributed_mapper::DistributedMapper>(new distributed_mapper::DistributedMapper(robot.name[1]));
			robot.distributedMapper->setUseBetweenNoiseFlag(true);
			robot.distributedMapper->setUseLandmarksFlag(false);
			robot.distributedMapper->setVerbosity(distributed_mapper::DistributedMapper::ERROR);

			robots.push_back(robot);
		}

		/*** ros subscriber and publisher ***/
		pubLoopClosureConstraint = nh.advertise<visualization_msgs::MarkerArray>("distributedMapping/loop_closure_constraints", 1);
		// services
		srvGeometricVerification = nh.advertiseService("distributedMapping/geometricVerification", &distributed_mapping::geometricVerificationService, this);
		srvSaveDistributedTrajectory = nh.advertiseService("save_distributed_trajectory", &distributed_mapping::saveDistributedTrajectoryService, this);

		/*** mutex ***/
		lockOnCallback = std::vector<std::mutex>(numberOfRobots);

		/*** distributed loopclosure ***/
		interLoopPtr = 0;
		intraLoopPtr = 0;

		laserCloudDesDS.reset(new pcl::PointCloud<PointPose3D>()); 
		downSizeFilterDes.setLeafSize(descriptLeafSize, descriptLeafSize, descriptLeafSize);

		downSizeFilterICP.setLeafSize(mappingLeafSize, mappingLeafSize, mappingLeafSize);

		loopIndexContainer.clear();

		// RS loopclosure
		copyCloudKeyPoses3D.reset(new pcl::PointCloud<PointPose3D>());
		copyCloudKeyPoses6D.reset(new pcl::PointCloud<PointPose6D>());
		kdtreeHistoryKeyPoses.reset(new pcl::KdTreeFLANN<PointPose3D>());
    
		/*** pose graph optmazition ***/
		rostimeLastOpt = 0.0;
		loopCloseFlag = false;
		
		odometryNoise = noiseModel::Diagonal::Variances((Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());
		priorNoise = noiseModel::Diagonal::Variances((Vector(6) << 1e-2, 1e-2, M_PI*M_PI, 1e8, 1e8, 1e8).finished()); // rad*rad, meter*meter
	
		// isam2
		ISAM2Params parameters;
        parameters.relinearizeThreshold = 0.1;
        parameters.relinearizeSkip = 1;
        isam2 = new ISAM2(parameters);

		// distributed mapper
		DMdebug = false;
		rotationEstimateChangeThreshold = 1e-1;
		poseEstimateChangeThreshold = 1e-1;
		gamma = 1.0f;
		useFlaggedInit = true;
		updateType = distributed_mapper::DistributedMapper::incUpdate;
		useLandmarks = false;
		pcmThreshold = 0.75;
		useCovariance = false;
		useHeuristics = true;
	}


    ~distributed_mapping()
    {

    }










	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
		class distributed_mapping: handle message callback 
	* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
	void globalDescriptorHandler(const dlc_slam::global_descriptorConstPtr& msg, int& idIn)
	{
		// keyposes order (TODO: resend?)
		if(msg->index!=robots[idIn].cloudKeyPoses3D->points.size())
        {
			ROS_ERROR("\033[0;32m[globalDescriptorHandler(%d)] robot[%c] msgIndex[%d] but have odomIndex[%d].\033[0m", idIn, 'a'+idIn, msg->index, robots[idIn].cloudKeyPoses3D->points.size());
			return;
		}

		/*** handle messgae ***/
		// timestamp
        robots[idIn].timeCloudInputStamp = msg->header.stamp;
		// keypose
		robots[idIn].keyPose = msg->curPose;

		/*** factors ***/
		gtsam::Pose3 poseTo = transformToGTSAMpose(msg->curPose);
        if(robots[idIn].cloudKeyPoses3D->points.empty())
        {
			robots[idIn].pirorOdom = poseTo;
			robots[idIn].distributedMapper->insertValue(Symbol('a'+idIn, 0), poseTo);
        }
		else
		{
			gtsam::Pose3 poseFrom = transformToGTSAMpose(msg->prePose);
			NonlinearFactor::shared_ptr factor(new BetweenFactor<Pose3>(Symbol('a'+idIn, robots[idIn].cloudKeyPoses3D->size()-1),
				Symbol('a'+idIn, robots[idIn].cloudKeyPoses3D->size()), poseFrom.between(poseTo), odometryNoise));
			robots[idIn].distributedMapper->addFactor(factor);
			robots[idIn].distributedMapper->insertValue(Symbol('a'+idIn, robots[idIn].cloudKeyPoses3D->size()), poseTo);
			
			// compose();
        }

		// save pose in global key poses
		static PointPose3D thisPose3D;
		thisPose3D.x = poseTo.translation().x();
        thisPose3D.y = poseTo.translation().y();
        thisPose3D.z = poseTo.translation().z();
        thisPose3D.intensity = robots[idIn].cloudKeyPoses3D->size(); // keyframe index
        robots[idIn].cloudKeyPoses3D->push_back(thisPose3D);

		static PointPose6D thisPose6D;
		thisPose6D.x = thisPose3D.x;
        thisPose6D.y = thisPose3D.y;
        thisPose6D.z = thisPose3D.z;
        thisPose6D.intensity = thisPose3D.intensity;
        thisPose6D.roll  = poseTo.rotation().roll();
        thisPose6D.pitch = poseTo.rotation().pitch();
        thisPose6D.yaw = poseTo.rotation().yaw();
        thisPose6D.time = robots[idIn].timeCloudInputStamp.toSec(); // keyframe timestamp
		robots[idIn].cloudKeyPoses6D->push_back(thisPose6D);

		// local path for visualize
		updateLocalPath(msg->curPose, idIn);

		// publish path
		if(robots[idIn].pubPath.getNumSubscribers() != 0 && robots[idIn].globalPath.poses.empty())
		{
			robots[idIn].localPath.header.stamp = ros::Time::now();
			robots[idIn].localPath.header.frame_id = robots[idIn].odomFrame;
			robots[idIn].pubPath.publish(robots[idIn].localPath);
		}
		if(robots[idIn].pubtestPath.getNumSubscribers() != 0)
		{
			robots[idIn].localPath.header.stamp = ros::Time::now();
			robots[idIn].localPath.header.frame_id = robots[idIn].odomFrame;
			robots[idIn].pubtestPath.publish(robots[idIn].localPath);
		}

		mtxSC.lock();
		// save descriptors
		scanDescriptor->saveDescriptorAndKey(msg->values.data(), idIn, robots[idIn].cloudKeyPoses6D->size()-1);
		mtxSC.unlock();
	}


	void updateLocalPath(const geometry_msgs::Transform& poseIn, const int& idIn)
    {
		// save path
		static geometry_msgs::PoseStamped poseStamped;
        poseStamped.header.stamp = ros::Time::now();
        poseStamped.header.frame_id = robots[idIn].odomFrame;

        poseStamped.pose.position.x = poseIn.translation.x;
        poseStamped.pose.position.y = poseIn.translation.y;
        poseStamped.pose.position.z = poseIn.translation.z;
        poseStamped.pose.orientation.x = poseIn.rotation.x;
        poseStamped.pose.orientation.y = poseIn.rotation.y;
        poseStamped.pose.orientation.z = poseIn.rotation.z;
        poseStamped.pose.orientation.w = poseIn.rotation.w;

        robots[idIn].localPath.poses.push_back(poseStamped);
    }

	void loopInfoHandler(const dlc_slam::loop_infoConstPtr& msg, int& idIn)
	{
		
	}











	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
		class distributed_mapping: scan2map (keyframe)
	* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
	void performDistributedMapping(const gtsam::Pose3 poseTo, pcl::PointCloud<PointPose3D>::Ptr frameTo, ros::Time timeStamp)
    {
		// save keyframe cloud
		pcl::copyPointCloud(*frameTo, *robots[id].keyFrame);
		ROS_DEBUG("PointCloud SIZE:[%d]",robots[id].keyFrame->size());

		robots[id].keyFrameArray.push_back(*robots[id].keyFrame);
		robots[id].timeCloudInputStamp = timeStamp;
		robots[id].timeCloudInputCur = timeStamp.toSec();

		// add piror factor
        if(robots[id].cloudKeyPoses3D->points.empty())
        {
			robots[id].distributedMapper->addPrior(Symbol('a'+id, 0), poseTo, priorNoise);
			robots[id].distributedMapper->insertValue(Symbol('a'+id, 0), poseTo);
			iGraph.add(PriorFactor<Pose3>(Symbol('a'+id, 0), poseTo, priorNoise));
			iInitial.insert(Symbol('a'+id, 0), poseTo);
			ROS_DEBUG("createPrior:[%d]--[%.2f,%.2f,%.2f,%.2f,%.2f,%.2f]",id,
				poseTo.translation().x(),poseTo.translation().y(),poseTo.translation().z(),
				poseTo.rotation().roll(),poseTo.rotation().pitch(),poseTo.rotation().yaw());
        }
		// add odometry factor
		else
		{

			gtsam::Pose3 poseFrom = pclPointTogtsamPose3(robots[id].cloudKeyPoses6D->points.back());
			NonlinearFactor::shared_ptr factor(new BetweenFactor<Pose3>(Symbol('a'+id, robots[id].cloudKeyPoses3D->size()-1),
				Symbol('a'+id, robots[id].cloudKeyPoses3D->size()), poseFrom.between(poseTo), odometryNoise));
			robots[id].distributedMapper->addFactor(factor);
			robots[id].distributedMapper->insertValue(Symbol('a'+id, robots[id].cloudKeyPoses3D->size()), poseTo);
			iGraph.add(BetweenFactor<Pose3>(Symbol('a'+id, robots[id].cloudKeyPoses3D->size()-1), Symbol('a'+id, robots[id].cloudKeyPoses3D->size()), poseFrom.between(poseTo), odometryNoise));
			iInitial.insert(Symbol('a'+id, robots[id].cloudKeyPoses3D->size()), poseTo);
			ROS_DEBUG("createOdom[%d]:[%d-%d]--[%.2f,%.2f,%.2f,%.2f,%.2f,%.2f],[%.2f,%.2f,%.2f,%.2f,%.2f,%.2f]",id,
				robots[id].cloudKeyPoses3D->size()-1,robots[id].cloudKeyPoses3D->size(),
				poseFrom.translation().x(),poseFrom.translation().y(),poseFrom.translation().z(),
				poseFrom.rotation().roll(),poseFrom.rotation().pitch(),poseFrom.rotation().yaw(),
				poseTo.translation().x(),poseTo.translation().y(),poseTo.translation().z(),
				poseTo.rotation().roll(),poseTo.rotation().pitch(),poseTo.rotation().yaw());
        }

		// local path for visualize
		static geometry_msgs::PoseStamped poseStamped;
        poseStamped.header.stamp = ros::Time::now();
        poseStamped.header.frame_id = robots[id].odomFrame;
        poseStamped.pose.position.x = poseTo.translation().x();
        poseStamped.pose.position.y = poseTo.translation().y();
        poseStamped.pose.position.z = poseTo.translation().z();
        poseStamped.pose.orientation.x = poseTo.rotation().toQuaternion().x();
        poseStamped.pose.orientation.y = poseTo.rotation().toQuaternion().y();
        poseStamped.pose.orientation.z = poseTo.rotation().toQuaternion().z();
        poseStamped.pose.orientation.w = poseTo.rotation().toQuaternion().w();
        robots[id].localPath.poses.push_back(poseStamped);

		// optimizing
		gtsamOpt();

		// get latest keyframe estimation
		keyPoseEstimate = estimates[0].at<Pose3>(Symbol('a'+id, robots[id].cloudKeyPoses3D->size()));

		// save 3D keyframe pose
		PointPose3D thisPose3D;
		thisPose3D.x = keyPoseEstimate.translation().x();
		thisPose3D.y = keyPoseEstimate.translation().y();
		thisPose3D.z = keyPoseEstimate.translation().z();
		thisPose3D.intensity = robots[id].cloudKeyPoses3D->size(); // keyframe index
		robots[id].cloudKeyPoses3D->push_back(thisPose3D);

		// save 6D keyframe pose
		PointPose6D thisPose6D;
		thisPose6D.x = thisPose3D.x;
		thisPose6D.y = thisPose3D.y;
		thisPose6D.z = thisPose3D.z;
		thisPose6D.intensity = thisPose3D.intensity;
		thisPose6D.roll = keyPoseEstimate.rotation().roll();
		thisPose6D.pitch = keyPoseEstimate.rotation().pitch();
		thisPose6D.yaw = keyPoseEstimate.rotation().yaw();
		thisPose6D.time = robots[id].timeCloudInputCur; // keyframe timestamp
		robots[id].cloudKeyPoses6D->push_back(thisPose6D);
		ROS_DEBUG("save:[%d]--[%d]--[%.2f,%.2f,%.2f,%.2f,%.2f,%.2f]",id,(int)thisPose6D.intensity,
			thisPose6D.x,thisPose6D.y,thisPose6D.z,
			thisPose6D.roll,thisPose6D.pitch,thisPose6D.yaw);

		// save path for visualization
		updateGlobalPath(thisPose6D, id);
	}


	bool saveFrame(const gtsam::Pose3 poseTo)
    {
        if(robots[id].cloudKeyPoses3D->points.empty())
		{
			return true;
		}

        gtsam::Pose3 keyPoseLast = pclPointTogtsamPose3(robots[id].cloudKeyPoses6D->back());
        gtsam::Pose3 poseBetween = keyPoseLast.between(poseTo);

        float x = poseBetween.translation().x();
		float y = poseBetween.translation().y();
		float z = poseBetween.translation().z();
		float roll = poseBetween.rotation().roll();
		float pitch = poseBetween.rotation().pitch();
		float yaw = poseBetween.rotation().yaw();

        if(	abs(roll) < keyframeAddingAngleThreshold &&
        	abs(pitch) < keyframeAddingAngleThreshold && 
            abs(yaw) < keyframeAddingAngleThreshold &&
            sqrt(x*x + y*y + z*z) < keyframeAddingDistThreshold)
		{
			return false;
		}
        return true;
    }

	void gtsamOpt()
	{
		for(int i = 0; i < loopInfoQueue.size(); i++)
		{
			ROS_INFO("total loop: %d", loopInfoQueue.size());
			dlc_slam::loop_info thisLoop = loopInfoQueue.front();
			loopInfoQueue.pop_front();

			if(	thisLoop.index0 >= robots[thisLoop.robot0].cloudKeyPoses6D->size() || 
				thisLoop.index1 >= robots[thisLoop.robot1].cloudKeyPoses6D->size())
			{
				loopInfoQueue.push_back(thisLoop);
				continue;
			}

			ROS_INFO("add loop: %d-%d %d-%d", thisLoop.robot0,thisLoop.index0,thisLoop.robot1,thisLoop.index1);
			gtsam::Vector Vector6(6);
			Vector6 << thisLoop.noise, thisLoop.noise, thisLoop.noise, thisLoop.noise, thisLoop.noise, thisLoop.noise;
			noiseModel::Diagonal::shared_ptr constraintNoise = noiseModel::Diagonal::Variances(Vector6);
			NonlinearFactor::shared_ptr factor(new BetweenFactor<Pose3>(
				Symbol('a'+thisLoop.robot0, thisLoop.index0), Symbol('a'+thisLoop.robot1, thisLoop.index1),
				transformToGTSAMpose(thisLoop.betPose), constraintNoise));

			if(thisLoop.robot0 == thisLoop.robot1)
			{
				robots[thisLoop.robot0].distributedMapper->addFactor(factor);
				if(thisLoop.robot0 == id)
				{
					iGraph.add(factor);
				}
			}
			else
			{
				robots[thisLoop.robot0].distributedMapper->addFactor(factor);
				robots[thisLoop.robot1].distributedMapper->addFactor(factor);
			}

			// add loop constriant
			loopIndexContainer[Symbol('a'+thisLoop.robot0, thisLoop.index0)] = Symbol('a'+thisLoop.robot1, thisLoop.index1);

			loopCloseFlag = true;
		}

		std::vector <GraphAndValues> graphAndValuesVec; // vector of all graphs and initials
		bool disconnectedGraph = false;  // Flag to check whether graphs are connected or not
		std::vector<boost::shared_ptr<distributed_mapper::DistributedMapper>> distMappers; // Vector of distributed optimizers, one for each robot
		// Load subgraph and construct dist_mapper optimizers
		for(int i = 0; i < numberOfRobots; ++i)
		{
			int idCur = (i + id) % numberOfRobots;

			if(!connected[idCur][id])
			{
				continue;
			}

			// Continue if empty
			Values initial = robots[idCur].distributedMapper->currentEstimate();
			if(initial.empty())
			{
				disconnectedGraph = true;
				continue;
			}

			// Construct graph_and_values using cleaned up initial values
			GraphAndValues graph_and_values = std::make_pair(
				boost::make_shared<NonlinearFactorGraph>(robots[idCur].distributedMapper->currentGraph()),
				boost::make_shared<Values>(initial));
			graphAndValuesVec.push_back(graph_and_values);

			// Load subgraphs
			robots[idCur].distributedMapper->loadSubgraphAndCreateSubgraphEdge();

			// Add prior to the first robot
			if(idCur == id)
			{
				Key first_key = KeyVector(initial.keys()).at(0);
				robots[idCur].distributedMapper->addPrior(first_key, robots[idCur].pirorOdom, priorNoise);
			}

			// Check for graph connectivity
			std::set<char> neighboring_robots = robots[idCur].distributedMapper->getNeighboringChars();
			if(neighboring_robots.size() == 0) disconnectedGraph = true;

			// Push to the set of optimizers
			distMappers.push_back(robots[idCur].distributedMapper);
		}

		estimates.clear();
		// Distributed Estimate
		if(!disconnectedGraph && globalOptmizationEnableFlag)
		{
			ROS_DEBUG("[DM(%d)] Optimizing", id);

			// optimizing
			int maxCliqueSize = 0;
			estimates = distributed_mapper::distributedOptimizer(distMappers, maxIterationTime, maxCliqueSize, updateType, gamma,
				rotationEstimateChangeThreshold, poseEstimateChangeThreshold, useFlaggedInit, useLandmarks, DMdebug, true,
				pcmThreshold, useCovariance, usePCM, useHeuristics, graphAndValuesVec);

			ROS_DEBUG("[DM(%d)] Done", id);
		}
		// Single Estimate
		else
		{
			ROS_DEBUG("[ISAM(%d)] Optimizing", id);

			// iGraph.print("GTSAM Graph:\n");
			isam2->update(iGraph, iInitial);
			isam2->update();

			iGraph.resize(0);
			iInitial.clear();

			isamEstimates = isam2->calculateEstimate();
			estimates.push_back(isamEstimates);
			
			ROS_DEBUG("[ISAM(%d)] Done", id);
		}
	}


	void updateGlobalPath(const PointPose6D& poseIn, const int& idIn)
    {
		static geometry_msgs::PoseStamped poseStamped;
        poseStamped.header.stamp = ros::Time::now();
        poseStamped.header.frame_id = worldFrame;
        poseStamped.pose.position.x = poseIn.x;
        poseStamped.pose.position.y = poseIn.y;
        poseStamped.pose.position.z = poseIn.z;
        tf::Quaternion q = tf::createQuaternionFromRPY(poseIn.roll, poseIn.pitch, poseIn.yaw);
        poseStamped.pose.orientation.x = q.x();
        poseStamped.pose.orientation.y = q.y();
        poseStamped.pose.orientation.z = q.z();
        poseStamped.pose.orientation.w = q.w();

        robots[idIn].globalPath.poses.push_back(poseStamped);
    }


	bool updatePoses()
	{
		bool returnValue = false;

		if(robots[id].cloudKeyPoses3D->points.empty())
		{
			return returnValue;
		}

		if(loopCloseFlag == true)
        {
			// clear path
			for(int i = 0; i < numberOfRobots; i++)
			{
				robots[i].globalPath.poses.clear();
			}

			// Aggregate estimates from all the robots
			for(int i = 0; i < estimates.size(); i++)
			{
				int robotCur, index;
				// update key poses
				for(const Values::ConstKeyValuePair &key_value: estimates[i])
				{
					Symbol key = key_value.key;
					index = key.index();
					robotCur = key.chr()-'a';
					Pose3 pose = estimates[i].at<Pose3>(key);

					// update values
					// if(robots[robotCur].initial->exists(key))
					// {
					// 	robots[robotCur].initial->update(key, pose);
					// }

					robots[robotCur].cloudKeyPoses3D->points[index].x = pose.translation().x();
					robots[robotCur].cloudKeyPoses3D->points[index].y = pose.translation().y();
					robots[robotCur].cloudKeyPoses3D->points[index].z = pose.translation().z();

					robots[robotCur].cloudKeyPoses6D->points[index].x = robots[robotCur].cloudKeyPoses3D->points[index].x;
					robots[robotCur].cloudKeyPoses6D->points[index].y = robots[robotCur].cloudKeyPoses3D->points[index].y;
					robots[robotCur].cloudKeyPoses6D->points[index].z = robots[robotCur].cloudKeyPoses3D->points[index].z;
					robots[robotCur].cloudKeyPoses6D->points[index].roll = pose.rotation().roll();
					robots[robotCur].cloudKeyPoses6D->points[index].pitch = pose.rotation().pitch();
					robots[robotCur].cloudKeyPoses6D->points[index].yaw = pose.rotation().yaw();

					updateGlobalPath(robots[robotCur].cloudKeyPoses6D->points[index], robotCur);
				}
			}

            loopCloseFlag = false;
			returnValue = true;
        }
		
		// save updated keypose
		PointPose6D curTypePose = robots[id].cloudKeyPoses6D->points[robots[id].cloudKeyPoses6D->size()-1];
		robots[id].keyPose.translation.x = curTypePose.x;
		robots[id].keyPose.translation.y = curTypePose.y;
		robots[id].keyPose.translation.z = curTypePose.z;
		geometry_msgs::Quaternion q = tf::createQuaternionMsgFromRollPitchYaw(curTypePose.roll, curTypePose.pitch, curTypePose.yaw);
		robots[id].keyPose.rotation = q;

		return returnValue;
	}


	void makeDescriptors()
    {
		if(robots[id].cloudKeyPoses3D->points.empty())
		{
			return;
		}

		// downsample
		laserCloudDesDS->clear();
		downSizeFilterDes.setInputCloud(robots[id].keyFrame);
		downSizeFilterDes.filter(*laserCloudDesDS);

		// make and save global descriptors
		mtxSC.lock();
		std::vector<float> v = scanDescriptor->makeAndSaveDescriptorAndKey(*laserCloudDesDS, id, robots[id].cloudKeyPoses6D->size()-1);
		mtxSC.unlock();

		// begin to handle message
		// descriptors values
		globalDescriptorMsg.values.swap(v);
		// current pose
		globalDescriptorMsg.curPose = robots[id].keyPose;
		// previous pose
		if(robots[id].cloudKeyPoses6D->size() > 1)
		{
			PointPose6D preTypePose = robots[id].cloudKeyPoses6D->points[robots[id].cloudKeyPoses6D->size()-2];
			globalDescriptorMsg.prePose.translation.x = preTypePose.x;
			globalDescriptorMsg.prePose.translation.y = preTypePose.y;
			globalDescriptorMsg.prePose.translation.z = preTypePose.z;
			geometry_msgs::Quaternion q = tf::createQuaternionMsgFromRollPitchYaw(preTypePose.roll, preTypePose.pitch, preTypePose.yaw);
			globalDescriptorMsg.prePose.rotation = q;
		}
		// keyfame index
		globalDescriptorMsg.index = robots[id].cloudKeyPoses3D->size()-1;
		globalDescriptorMsg.header.stamp = robots[id].timeCloudInputStamp;
		// publish message
		robots[id].pubDescriptors.publish(globalDescriptorMsg);
	}


	void publishMessages()
    {
		publishAllPath();

		if(robots[id].pubPath.getNumSubscribers() != 0 && robots[id].globalPath.poses.empty())
		{
			robots[id].localPath.header.stamp = ros::Time::now();
			robots[id].localPath.header.frame_id = robots[id].odomFrame;
			robots[id].pubPath.publish(robots[id].localPath);
		}
		if(robots[id].pubtestPath.getNumSubscribers() != 0)
		{
			robots[id].localPath.header.stamp = ros::Time::now();
			robots[id].localPath.header.frame_id = robots[id].odomFrame;
			robots[id].pubtestPath.publish(robots[id].localPath);
		}
    }

	void publishAllPath()
	{
		// publish path
		for(int i = 0; i < numberOfRobots; i++)
		{
			if(robots[i].pubPath.getNumSubscribers() != 0 && !robots[i].globalPath.poses.empty())
			{
				robots[i].globalPath.header.stamp = ros::Time::now();
				robots[i].globalPath.header.frame_id = robots[i].odomFrame;
				robots[i].pubPath.publish(robots[i].globalPath);
			}
		}
	}








	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
		class distributed_mapping: loop closure
	* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
	void performIntraLoopClosure()
	{
		if(!loopInfoQueue.empty() || scanDescriptor->getSize(id) <= intraLoopPtr || !intraLoopClosureEnableFlag)
		{
			return;
		}

		/*** stage1 Place Recognition: find candidates with global descriptor ***/
		auto detectResult = scanDescriptor->detectIntraLoopClosureID(intraLoopPtr);
		int loopKeyCur = intraLoopPtr;
		int loopKeyPre = detectResult.first;
		intraLoopPtr++;

		if(detectResult.first < 0) // No loop found
		{
			return;
		}

		// check loop constraint added before
		auto it = loopIndexContainer.find(Symbol('a'+id, loopKeyCur));
        if(it != loopIndexContainer.end() || it->second == Symbol('a'+id, loopKeyPre))
		{
			return;
		}

		ROS_INFO("\033[1;34m[IntraLoop<%d>] found between [%d] and [%d].\033[0m",id,loopKeyCur,loopKeyPre);
		/*** stage2 Geometric Verification: icp matching ***/
		// extract cloud
		pcl::PointCloud<PointPose3D>::Ptr keyFrameCloudCur(new pcl::PointCloud<PointPose3D>());
		loopFindNearKeyframes(keyFrameCloudCur, loopKeyCur, 0);
		pcl::PointCloud<PointPose3D>::Ptr keyFrameCloudPre(new pcl::PointCloud<PointPose3D>());
        loopFindNearKeyframes(keyFrameCloudPre, loopKeyPre, historyKeyframeSearchNum);
		if(keyFrameCloudCur->size() < 300 || keyFrameCloudPre->size() < 1000)
		{
			return;
		}
        
		// ICP Settings
        static pcl::IterativeClosestPoint<PointPose3D, PointPose3D> icp;
        icp.setMaxCorrespondenceDistance(100);
		icp.setMaximumIterations(50);
        icp.setTransformationEpsilon(1e-6);
        icp.setEuclideanFitnessEpsilon(1e-6);
        icp.setRANSACIterations(0);

 		// Align clouds
		icp.setInputSource(keyFrameCloudCur);
		icp.setInputTarget(keyFrameCloudPre);
		pcl::PointCloud<PointPose3D>::Ptr unused_result(new pcl::PointCloud<PointPose3D>());
		icp.align(*unused_result);

		float icpFitnessScore = icp.getFitnessScore();
		if(icp.hasConverged() == false || icpFitnessScore > historyKeyframeFitnessScore)
		{
			ROS_INFO("\033[1;34m[IntraLoop<%d>] ICP fitness failed (%f > %f). Reject this intra loop.\033[0m", id, icpFitnessScore, historyKeyframeFitnessScore);
			return;
		}
		ROS_INFO("\033[1;34m[IntraLoop<%d>] ICP fitness passed (%f < %f). Add this intra loop.\033[0m", id, icpFitnessScore, historyKeyframeFitnessScore);

		// Get pose transformation
		float x, y, z, roll, pitch, yaw;
		Eigen::Affine3f tfICP;
		tfICP = icp.getFinalTransformation();
		pcl::getTranslationAndEulerAngles(tfICP, x, y, z, roll, pitch, yaw);
		ROS_INFO("\033[1;34m[IntraLoop<%d>] icp %.2f,%.2f,%.2f %.2f,%.2f,%.2f.\033[0m", id,
			x, y, z, roll, pitch, yaw);
		Eigen::Affine3f tfWrong = pclPointToAffine3f(robots[id].cloudKeyPoses6D->points[loopKeyCur]);
		Eigen::Affine3f tfCorrect = tfICP * tfWrong;
		pcl::getTranslationAndEulerAngles(tfCorrect, x, y, z, roll, pitch, yaw);
		Pose3 poseFrom = Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(x, y, z));
		Pose3 poseTo = pclPointTogtsamPose3(robots[id].cloudKeyPoses6D->points[loopKeyPre]);
		Pose3 poseBet = poseFrom.between(poseTo);
		ROS_INFO("\033[1;34m[IntraLoop<%d>] betPose %.2f,%.2f,%.2f.\033[0m", id,
			poseBet.translation().x(), poseBet.translation().y(), poseBet.translation().z());
		
		/*** stage3 Optimazation: add factor to gtsam ***/
		dlc_slam::loop_info thisLoop;
		thisLoop.robot0 = id;
		thisLoop.robot1 = id;
		thisLoop.index0 = loopKeyCur;
		thisLoop.index1 = loopKeyPre;
		thisLoop.noise = icpFitnessScore;
		thisLoop.betPose.translation.x = poseBet.translation().x();
		thisLoop.betPose.translation.y = poseBet.translation().y();
		thisLoop.betPose.translation.z = poseBet.translation().z();
		thisLoop.betPose.rotation.w = poseBet.rotation().toQuaternion().w();
		thisLoop.betPose.rotation.x = poseBet.rotation().toQuaternion().x();
		thisLoop.betPose.rotation.y = poseBet.rotation().toQuaternion().y();
		thisLoop.betPose.rotation.z = poseBet.rotation().toQuaternion().z();
		loopInfoQueue.push_back(thisLoop);
	}


	void loopFindNearKeyframes(pcl::PointCloud<PointPose3D>::Ptr& nearKeyframes, const int& key, const int& searchNum)
    {
        // extract near keyframes
        nearKeyframes->clear();
        int cloudSize = robots[id].keyFrameArray.size();
        for(int i = -searchNum; i <= searchNum; ++i)
        {
            int keyNear = key + i;
            if(keyNear < 0 || keyNear >= cloudSize)
			{
				continue;
			}
			*nearKeyframes += *transformPointCloud(robots[id].keyFrameArray[keyNear], &robots[id].cloudKeyPoses6D->points[keyNear]);
        }

        if (nearKeyframes->empty())
            return;

        // downsample near keyframes
        pcl::PointCloud<PointPose3D>::Ptr cloud_temp(new pcl::PointCloud<PointPose3D>());
        downSizeFilterICP.setInputCloud(nearKeyframes);
        downSizeFilterICP.filter(*cloud_temp);
        *nearKeyframes = *cloud_temp;
    }


	bool geometricVerificationService(dlc_slam::geometric_verificationRequest& req, dlc_slam::geometric_verificationResponse& res)
    {
		ROS_INFO("\033[1;35m[geometricVerificationService<%d>] is call.\033[0m", id);
		if(req.robotPre == id) // ransac
		{
			/*** calculate transform using icp ***/
			// extract cloud
			pcl::PointCloud<PointPose3D>::Ptr keyFrameCloudCur(new pcl::PointCloud<PointPose3D>());
			pcl::PointCloud<PointPose3D>::Ptr keyFrameCloudPre(new pcl::PointCloud<PointPose3D>());
			// downsample near keyframes
			pcl::fromROSMsg(req.featureCloud, *keyFrameCloudCur);
			downSizeFilterICP.setInputCloud(keyFrameCloudCur);
			downSizeFilterICP.filter(*keyFrameCloudCur);
			loopFindNearKeyframes(keyFrameCloudPre, req.keyPre, historyKeyframeSearchNum);

			if(keyFrameCloudCur->size() < 300 || keyFrameCloudPre->size() < 1000)
			{
				res.success = false;
				return true;
			}

			// initial matching
			boost::shared_ptr<pcl::Correspondences> correspondences(new pcl::Correspondences);
        	pcl::registration::CorrespondenceEstimation<PointPose3D, PointPose3D> corrEst;
			corrEst.setInputCloud(keyFrameCloudCur);
			corrEst.setInputTarget(keyFrameCloudPre);
			corrEst.determineCorrespondences(*correspondences);

			// RANSAC matching to find inlier
			pcl::Correspondences newCorrespondences;
			pcl::registration::CorrespondenceRejectorSampleConsensus<PointPose3D> corrsRansac;
			corrsRansac.setInputCloud(keyFrameCloudCur);
			corrsRansac.setTargetCloud(keyFrameCloudPre);
			corrsRansac.setMaxIterations(ransacMaxIter);
			corrsRansac.setInlierThreshold(ransacOutlierTreshold);
			corrsRansac.setInputCorrespondences(correspondences);
			corrsRansac.getCorrespondences(newCorrespondences);

			// calculate tf with SVD
			Eigen::Matrix<float, 4, 4> transform;
			pcl::registration::TransformationEstimationSVD<PointPose3D, PointPose3D> svd_trans;
			svd_trans.estimateRigidTransformation(*keyFrameCloudCur, *keyFrameCloudPre, newCorrespondences, transform);

			float x, y, z, roll, pitch, yaw;
			Eigen::Affine3f tfSVD;
			tfSVD = transform;
			pcl::getTranslationAndEulerAngles(tfSVD, x, y, z, roll, pitch, yaw);
			ROS_INFO("\033[1;35m[InterLoop<%d>] SVD Transform: %.2f,%.2f,%.2f,%.2f,%.2f,%.2f", id, x, y, z, roll, pitch, yaw);

			if(newCorrespondences.size() < inlierTreshold*correspondences->size())
			{
				res.success = false;
				ROS_INFO("\033[1;35m[InterLoop<%d>] RANSAC inlier failed (%.2f < %.2f). Reject this loop.\033[0m", id, newCorrespondences.size()*1.0/correspondences->size()*1.0, inlierTreshold);
				return true;
			}

			// Get pose transformation
			// float x, y, z, roll, pitch, yaw;
			// Eigen::Affine3f correctionLidarFrame;
			// correctionLidarFrame = icp.getFinalTransformation();
			Eigen::Affine3f tWrong = pclPointToAffine3f(robots[req.robotCur].cloudKeyPoses6D->points[req.keyCur]);
			Eigen::Affine3f tCorrect = tfSVD * tWrong;
			pcl::getTranslationAndEulerAngles(tCorrect, x, y, z, roll, pitch, yaw);
			Pose3 poseFrom = Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(x, y, z));
			Pose3 poseTo = pclPointTogtsamPose3(robots[req.robotPre].cloudKeyPoses6D->points[req.keyPre]);
			Pose3 poseBet = poseFrom.between(poseTo);
			res.poseBetween.translation.x = poseBet.translation().x();
			res.poseBetween.translation.y = poseBet.translation().y();
			res.poseBetween.translation.z = poseBet.translation().z();
			geometry_msgs::Quaternion q = tf::createQuaternionMsgFromRollPitchYaw(poseBet.rotation().roll(), poseBet.rotation().pitch(), poseBet.rotation().yaw());
			res.poseBetween.rotation = q;
			res.success = true;
			ROS_INFO("\033[1;35m[InterLoop<%d>] fitness (%f < %f) inlier (%.2f > %.2f). Add this loop.\033[0m", id, newCorrespondences.size()*1.0/correspondences->size()*1.0, inlierTreshold);
			return true;
		}
		else
		{
			return false;
		}
	}


	// Inter Loop Closure
	void performInterLoopClosure()
    {
		if(!loopInfoQueue.empty() || scanDescriptor->getSize() <= interLoopPtr || !interLoopClosureEnableFlag)
		{
			return;
		}

		/*** stage1 Place Recognition: find candidates with global descriptor ***/
		auto detectResult = scanDescriptor->detectInterLoopClosureID(interLoopPtr);
		int8_t loopRobotCur = scanDescriptor->getIndex(interLoopPtr).first;
		int8_t loopRobotPre = scanDescriptor->getIndex(detectResult.first).first;
		int loopKeyCur = scanDescriptor->getIndex(interLoopPtr).second;
		int loopKeyPre = scanDescriptor->getIndex(detectResult.first).second;
		float bias = detectResult.second;
		interLoopPtr++;

		if(detectResult.first < 0) // No loop found
		{
			return;
		}

		// check loop constraint added before
		auto it = loopIndexContainer.find(Symbol('a'+id, loopKeyCur));
        if(it != loopIndexContainer.end() || it->second == Symbol('a'+id, loopKeyPre))
		{
			return;
		}

		ROS_INFO("\033[1;36m[InterLoop<%d>] found between [%d]-[%d][%d] and [%d]-[%d][%d].\033[0m",id,loopRobotCur,loopKeyCur,interLoopPtr-1,loopRobotPre,loopKeyPre,detectResult.first);

		/*** stage2 Geometric Verification: send to other robot for matching ***/
		bool callFlag;
		dlc_slam::geometric_verification srv;
		if(loopRobotCur != id)
		{
			dlc_slam::loop_info thisLoop;
			thisLoop.robot0 = loopRobotCur;
			thisLoop.robot1 = loopRobotPre;
			thisLoop.index0 = loopKeyCur;
			thisLoop.index1 = loopKeyPre;
			thisLoop.noise = 999.0;
			

			// ask for other robot[loopRobotPre]
			// srv.request.keyPre = loopKeyPre;
			// srv.request.keyCur = loopKeyCur;
			// srv.request.robotPre = loopRobotPre;
			// srv.request.robotCur = loopRobotCur;
			// pcl::PointCloud<PointPose3D>::Ptr emptyCloud(new pcl::PointCloud<PointPose3D>());
			// pcl::toROSMsg(*emptyCloud, srv.request.featureCloud);
			// callFlag = robots[loopRobotCur].clientGeometricVerification.call(srv);
			
		}
		else
		{
			// ask for robot[loopRobotPre]
			srv.request.keyPre = loopKeyPre;
			srv.request.keyCur = loopKeyCur;
			srv.request.robotPre = loopRobotPre;
			srv.request.robotCur = loopRobotCur;
			pcl::PointCloud<PointPose3D>::Ptr tfCloud(new pcl::PointCloud<PointPose3D>());
			pcl::toROSMsg(*transformPointCloud(robots[loopRobotCur].keyFrameArray[loopKeyCur], &robots[loopRobotCur].cloudKeyPoses6D->points[loopKeyCur]), srv.request.featureCloud);
			// pcl::toROSMsg(*transformPointCloud(robots[loopRobotCur].keyFrameArray[loopKeyCur], &robots[loopRobotPre].cloudKeyPoses6D->points[loopKeyPre]), srv.request.featureCloud);
			// pcl::toROSMsg(robots[loopRobotCur].keyFrameArray[loopKeyCur], srv.request.featureCloud);
			callFlag = robots[loopRobotPre].clientGeometricVerification.call(srv);
			// connected[loopRobotPre] = connected[loopRobotPre] || callFlag;
		}

		if(callFlag)
		{
			if(srv.response.success)
			{
				ROS_DEBUG("\033[1;35m[InterLoop] success result.\033[0m");
			}
			else
			{
				ROS_DEBUG("\033[1;35m[InterLoop] failed result.\033[0m");
				return;
			}
		}
		else
		{
			ROS_ERROR("[InterLoop] Failed to call geometric verification service!");
			return;
		}

		/*** stage3 Optimazation: add factor to gtsam ***/
		Pose3 poseBetween = transformToGTSAMpose(srv.response.poseBetween);
		// float robustNoiseScore = 0.5; // constant is ok...
		float robustNoiseScore = 0.2; // change to max fitness score...
		Vector robustNoiseVector6(6);
		robustNoiseVector6 << robustNoiseScore, robustNoiseScore, robustNoiseScore, robustNoiseScore, robustNoiseScore, robustNoiseScore;
		noiseModel::Diagonal::shared_ptr constraintNoise = noiseModel::Diagonal::Variances(robustNoiseVector6);
		if(loopRobotCur > loopRobotPre)
		{
			swap(loopRobotCur,loopRobotPre);
			swap(loopKeyCur,loopKeyPre);
			poseBetween = poseBetween.inverse();
		}
		NonlinearFactor::shared_ptr factor(new BetweenFactor<Pose3>(Symbol('a'+loopRobotCur, loopKeyCur), Symbol('a'+loopRobotPre, loopKeyPre), poseBetween, constraintNoise));

		robots[int(loopRobotCur)].distributedMapper->addFactor(factor);
		robots[int(loopRobotPre)].distributedMapper->addFactor(factor);
		if(loopRobotCur == id && loopRobotPre == id)
		{
			iGraph.add(factor);
		}
		loopCloseFlag = true;

		// add loop constriant
		loopIndexContainer[Symbol('a'+loopRobotCur, loopKeyCur)] = Symbol('a'+loopRobotPre, loopKeyPre);

		ROS_INFO("\033[1;35m[InterLoop] loop factor[%lld][%lld]\033[0m",Symbol('a'+loopRobotCur, loopKeyCur),Symbol('a'+loopRobotPre, loopKeyPre));
	}


	void publishLoopClosureConstraint()
	{
		if(loopIndexContainer.empty())
		{
			return;
		}

		// loop nodes
		visualization_msgs::Marker nodes;
		nodes.header.frame_id = robots[id].odomFrame;
		nodes.header.stamp = ros::Time::now();
		nodes.action = visualization_msgs::Marker::ADD;
		nodes.type = visualization_msgs::Marker::SPHERE_LIST;
		nodes.ns = "loop_nodes";
		nodes.id = 0;
		nodes.pose.orientation.w = 1;
        nodes.scale.x = 0.3; nodes.scale.y = 0.3; nodes.scale.z = 0.3; 
        nodes.color.r = 0; nodes.color.g = 0.8; nodes.color.b = 1;
        nodes.color.a = 1;
		
		// loop edges
		visualization_msgs::Marker constraints;
        constraints.header.frame_id = robots[id].odomFrame;
        constraints.header.stamp = ros::Time::now();
        constraints.action = visualization_msgs::Marker::ADD;
        constraints.type = visualization_msgs::Marker::LINE_LIST;
        constraints.ns = "loop_constraints";
        constraints.id = 1;
        constraints.pose.orientation.w = 1;
        constraints.scale.x = 0.1;
        constraints.color.r = 0.9; constraints.color.g = 0.9; constraints.color.b = 0;
        constraints.color.a = 1;

		for(auto it = loopIndexContainer.begin(); it != loopIndexContainer.end(); ++it)
		{
	        gtsam::Symbol key0 = it->first;
            gtsam::Symbol key1 = it->second;
			int8_t robot0 = symbolChr(key0) - 'a';
			int8_t robot1 = symbolChr(key1) - 'a';
			int index0 = symbolIndex(key0);
			int index1 = symbolIndex(key1);

            geometry_msgs::Point p;
			p.x = robots[robot0].cloudKeyPoses3D->points[index0].x;
			p.y = robots[robot0].cloudKeyPoses3D->points[index0].y;
			p.z = robots[robot0].cloudKeyPoses3D->points[index0].z;
            nodes.points.push_back(p);
            constraints.points.push_back(p);
            p.x = robots[robot1].cloudKeyPoses3D->points[index1].x;
			p.y = robots[robot1].cloudKeyPoses3D->points[index1].y;
			p.z = robots[robot1].cloudKeyPoses3D->points[index1].z;
            nodes.points.push_back(p);
            constraints.points.push_back(p);
		}

		visualization_msgs::MarkerArray markerArray;
		markerArray.markers.push_back(nodes);
        markerArray.markers.push_back(constraints);
        pubLoopClosureConstraint.publish(markerArray);
	}


	void loopClosureThread()
    {
		// Terminate the thread if loop closure are not needed.
		if(!intraLoopClosureEnableFlag && !interLoopClosureEnableFlag)
		{
			return;
		}

        while(true)
        {
            performIntraLoopClosure();
            performInterLoopClosure();
            publishLoopClosureConstraint();

			double timeNow = ros::Time::now().toSec();
			if(!loopInfoQueue.empty() && (timeNow - rostimeLastOpt >= mappingProcessInterval))
			{
				ROS_INFO("\033[1;35m[InterLoop] LastOpt:%.4f, timeNow:%.4f, diff:%.4f, opt here..  \033[0m", rostimeLastOpt, timeNow, timeNow-rostimeLastOpt);
				gtsamOpt();
				updatePoses();
				publishAllPath();
			}
        }
    }







	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
		class distributed_mapping: publish transform
	* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
	void publishTfThread()
	{
		ros::Rate rate(10); // update global map every 0.1s
		float x, y, z, roll, pitch, yaw;

        while(ros::ok())
		{
			rate.sleep();
			
			static tf::TransformBroadcaster tfworld2map;
			tf::Transform world2Odom = tf::Transform(tf::createQuaternionFromRPY(0, 0, 0), tf::Vector3(0, 0, 0));
			tfworld2map.sendTransform(tf::StampedTransform(world2Odom, ros::Time::now(), worldFrame, robots[id].odomFrame));
        }
	}


	bool saveDistributedTrajectoryService(dlc_slam::save_distributed_trajectoryRequest& req, dlc_slam::save_distributed_trajectoryResponse& res)
    {
		ROS_INFO("Saving distributed trajectory to files...");
		string directory;
		if(req.destination.empty())
		{
			directory = std::getenv("HOME") + saveDistributedTrajectoryDirectory;
		}
		else
		{
			directory = std::getenv("HOME") + req.destination;
		}
      	cout << "Save directory: " << directory << endl;

		// create directory and remove old files;
		int unused = system((std::string("exec rm -r ") + directory).c_str());
      	unused = system((std::string("mkdir -p ") + directory).c_str());

		vector<pcl::PointCloud<PointPose3D>> distributedKeyPoses3D; // distributed keypose(3Dof)
		vector<pcl::PointCloud<PointPose6D>> distributedKeyPoses6D; // distributed keypose(6Dof)
		for(int i = 0; i < numberOfRobots; i++)
		{
			pcl::PointCloud<PointPose3D> tmpPointType;
			distributedKeyPoses3D.push_back(tmpPointType);
			pcl::PointCloud<PointPose6D> tmpPointTypePose;
			distributedKeyPoses6D.push_back(tmpPointTypePose);
		}

		// extract data
		size_t curId = id;
		for(size_t i = 0; i < estimates.size(); i++)
		{
			ofstream TUMfile, KITTIfile;
			TUMfile.open(directory + "/" + robots[curId].name + "_tum_distributed_trajectory.txt");
			TUMfile.setf(ios::fixed);
			TUMfile.precision(10);
			KITTIfile.open(directory + "/" + robots[curId].name + "_kitti_distributed_trajectory.txt");
			KITTIfile.setf(ios::fixed);
			KITTIfile.precision(10);

			Key key;
			Pose3 pose;
			PointPose3D thisPose3D;
			PointPose6D thisPose6D;
			Eigen::Matrix4f posem;
			int index = 0;
			for(const Values::ConstKeyValuePair &key_value: estimates[i])
			{
				key = key_value.key;
				pose = estimates[i].at<Pose3>(key);

				thisPose3D.x = pose.translation().x();
				thisPose3D.y = pose.translation().y();
				thisPose3D.z = pose.translation().z();

				thisPose6D.x = thisPose3D.x;
				thisPose6D.y = thisPose3D.y;
				thisPose6D.z = thisPose3D.z;
				thisPose6D.roll = pose.rotation().roll();
				thisPose6D.pitch = pose.rotation().pitch();
				thisPose6D.yaw = pose.rotation().yaw();

				TUMfile << robots[curId].poseTimeStampVector[index] << " " << thisPose3D.x << " " << thisPose3D.y << " " << thisPose3D.z << " ";
				TUMfile << pose.rotation().toQuaternion().x() << " " << pose.rotation().toQuaternion().y() << " ";
				TUMfile << pose.rotation().toQuaternion().z() << " " << pose.rotation().toQuaternion().w() << endl;
				index++;

				posem = pclPointToAffine3f(thisPose6D).matrix();
				for(int m = 0; m < 3; m++)
				{
					KITTIfile << posem(m,0) << " " << posem(m,1) << " " << posem(m,2) << " " << posem(m,3);
					if(m != 2) KITTIfile << " ";
					else KITTIfile << endl;
				}

				distributedKeyPoses3D[curId].push_back(thisPose3D);
				distributedKeyPoses6D[curId].push_back(thisPose6D);
			}
			ROS_INFO("trajectory size:[%d,%d]", robots[curId].poseTimeStampVector.size(), index);

			curId = (curId + 1) % numberOfRobots;

			TUMfile.close();
			KITTIfile.close();
		}

		// save key frame transformations
		int ret;
		ROS_INFO("Save distributed key frame transformations");
		for(int i = 0; i < numberOfRobots; i++)
		{
			pcl::io::savePCDFileBinary(directory + "/" + robots[i].name + "_distributed_trajectory.pcd", distributedKeyPoses3D[i]);
			ret = pcl::io::savePCDFileBinary(directory + "/" + robots[i].name + "_distributed_transformations.pcd", distributedKeyPoses6D[i]);
		}
		res.success = ret == 0;

	  	ROS_INFO("Saving distributed trajectory to files completed.");
		
		// ofstream runTimefile;
		// runTimefile.open(directory + "/distributed_run_time.txt");
		// runTimefile.setf(ios::fixed);
		// runTimefile.precision(10);
		// for(int i = 0; i < distributed_process_time.size(); i++)
		// {
		// 	runTimefile << distributed_process_time[i] << endl;
		// }
		// runTimefile.close();

      	return true;
    }

	void globalMapThread()
	{
		ros::Rate rate(1.0/mapPublishInterval);
        while(ros::ok())
		{
            rate.sleep();
			publishGlobalMap();
        }
	}

	void publishGlobalMap()
	{
		if(robots[id].pubGlobalMap.getNumSubscribers() == 0 || robots[id].cloudKeyPoses3D->points.empty() == true)
		{
			return;
		}

        pcl::PointCloud<PointPose3D>::Ptr globalMapKeyFrames(new pcl::PointCloud<PointPose3D>());
        pcl::PointCloud<PointPose3D>::Ptr globalMapKeyFramesDS(new pcl::PointCloud<PointPose3D>());

		pcl::PointCloud<PointPose3D>::Ptr copyKeyPoses3D(new pcl::PointCloud<PointPose3D>());
		pcl::PointCloud<PointPose6D>::Ptr copyKeyPoses6D(new pcl::PointCloud<PointPose6D>());

        *copyKeyPoses3D = *robots[id].cloudKeyPoses3D;
        *copyKeyPoses6D = *robots[id].cloudKeyPoses6D;

        // extract visualized and downsampled key frames
        for(int i = 0; i < copyKeyPoses3D->size(); i++)
		{
            int thisKeyInd = (int)copyKeyPoses3D->points[i].intensity;
            *globalMapKeyFrames += *transformPointCloud(robots[id].keyFrameArray[thisKeyInd], &copyKeyPoses6D->points[thisKeyInd]);
        }

        // downsample visualized points
        pcl::VoxelGrid<PointPose3D> downSizeFilterGlobalMapKeyFrames; // for global map visualization
        downSizeFilterGlobalMapKeyFrames.setLeafSize(0.4, 0.4, 0.4); // for global map visualization
        downSizeFilterGlobalMapKeyFrames.setInputCloud(globalMapKeyFrames);
        downSizeFilterGlobalMapKeyFrames.filter(*globalMapKeyFramesDS);

		sensor_msgs::PointCloud2 tempCloud;
		pcl::toROSMsg(*globalMapKeyFramesDS, tempCloud);
		tempCloud.header.stamp = ros::Time::now();
		tempCloud.header.frame_id = worldFrame;
		robots[id].pubGlobalMap.publish(tempCloud);
    }
};

#endif