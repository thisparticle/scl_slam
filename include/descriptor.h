#pragma once
#ifndef _DESCRIPTOR_H_
#define _DESCRIPTOR_H_

// descriptor
#include "KDTreeVectorOfVectorsAdaptor.h"
#include <nabo/nabo.h>
//
#include <ros/ros.h>
#include <vector>
#include <opencv2/opencv.hpp>

#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/grsd.h>
#include <pcl/common/pca.h>
#include <fenv.h>

using namespace std;

class scan_descriptor
{
public:

	virtual std::vector<float> makeAndSaveDescriptorAndKey(const pcl::PointCloud<pcl::PointXYZI>& scan, const int8_t robot, const int index) = 0;
	
	virtual void saveDescriptorAndKey(const float* descriptorMat, const int8_t robot, const int index) = 0;
	
	virtual std::pair<int, float> detectIntraLoopClosureID(const int currentPtr) = 0;

	virtual std::pair<int, float> detectInterLoopClosureID(const int currentPtr) = 0;
	
	virtual std::pair<int8_t, int> getIndex(const int key) = 0;
	
	virtual int getSize(const int idIn = -1) = 0;
};

class grsd_descriptor : public scan_descriptor
{
public:
	grsd_descriptor()
	{
		inputScan.reset(new pcl::PointCloud<pcl::PointXYZI>);
		tree.reset(new pcl::search::KdTree<pcl::PointXYZI>());
		scanNormals.reset(new pcl::PointCloud<pcl::Normal>);
		descriptors.reset(new pcl::PointCloud<pcl::GRSDSignature21>());
	}

	void save(const std::vector<float> grsdVec, const int8_t robot, const int index)
	{
		Eigen::VectorXf grsdMat = Eigen::Map<const Eigen::Matrix<float, -1, -1, Eigen::RowMajor>>(grsdVec.data(), 21, 1);
		grsdDict.push_back(grsdMat);
		grsdVecDict.push_back(grsdVec);
		grsdIndexs.push_back(std::make_pair(robot,index));
	}

	std::vector<float> makeAndSaveDescriptorAndKey(const pcl::PointCloud<pcl::PointXYZI>& scan, const int8_t robot, const int index)
	{
		// std::cout << "GRSD START " << std::endl;
		// clock_t startTime, endTime;
		// startTime = ros::Time::now().toNSec();
		// std::cout << "scan size : " << scan.points.size() << std::endl;

		// Extract point cloud
		*inputScan = scan;
		// std::cout << "inputScan size : " << inputScan->points.size() << std::endl;

		// Extract surface normals for point cloud
		pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> ne;
		ne.setInputCloud(inputScan);
		ne.setSearchMethod(tree);
		// ne.setKSearch(10);
		ne.setRadiusSearch(neRadius);
		ne.compute(*scanNormals);
		// std::cout << "scanNormals size : " << scanNormals->points.size() << std::endl;

		// Get rid off NaNs (Grsd doesn't filter them and will break).
		// TODO
		// std::vector<int> indices_good_cloud;
		// std::vector<int> indices_good_cloud_normals;
		// pcl::removeNaNFromPointCloud(*inputScan, *inputScan, indices_good_cloud);
		// pcl::removeNaNNormalsFromPointCloud(*scanNormals, *scanNormals, indices_good_cloud_normals);

		// Create GrsdE class and pass data+normals to it
		pcl::GRSDEstimation<pcl::PointXYZI, pcl::Normal, pcl::GRSDSignature21> grsd;
		grsd.setInputCloud(inputScan);
  		grsd.setInputNormals(scanNormals);
		grsd.setSearchMethod(tree);
		grsd.setRadiusSearch(2.0);

		// Extract descriptors
		grsd.compute(*descriptors);
  		auto hist = descriptors->points[0].histogram;
		std::vector<float> grsdVec{hist, hist + 21};
		save(grsdVec, robot, index);

		// endTime = ros::Time::now().toNSec();
		// std::cout << "GRSD END " << (double)(endTime - startTime) / 10e6 << " ms" << std::endl;
		return grsdVec;
	}
	
	void saveDescriptorAndKey(const float* descriptorMat, const int8_t robot, const int index)
	{
		// decode grsd
		std::vector<float> grsdVec;
		grsdVec.insert(grsdVec.end(), descriptorMat, descriptorMat+21);

		save(grsdVec, robot, index);
	}

	std::pair<int, float> detectIntraLoopClosureID(const int curPtr)
	{

	}
	
	std::pair<int, float> detectInterLoopClosureID(const int currentPtr)
	{
		int loopId { -1 }; // init with -1, -1 means no loop

		auto curKey = grsdVecDict[currentPtr]; // current observation (query)

		if((int)grsdVecDict.size() < numExcludeRecent + 1)
		{
			std::pair<int,float> result {loopId, 0.0};
			return result; // Early return 
		}

		// tree_ reconstruction (not mandatory to make everytime)
		if(treeMakingPeriodConter % treeMakingPeriod == 0) // to save computation cost
		{
			grsdVecSearch.clear();
			grsdVecSearch.assign(grsdVecDict.begin(), grsdVecDict.end() - numExcludeRecent);

			grsdTree.reset(); 
			grsdTree = std::make_unique<KDTreeVectorOfVectorsAdaptor<std::vector<std::vector<float>>,float>>(21, grsdVecSearch, 10);
		}
		treeMakingPeriodConter += 1;

		// candidates from key tree
		// knn search
		std::vector<size_t> candidateIndexes(1); 
		std::vector<float> outDistsSqr(1);

		nanoflann::KNNResultSet<float> knnsearchResult(1);
		knnsearchResult.init(&candidateIndexes[0], &outDistsSqr[0]);
		grsdTree->index->findNeighbors(knnsearchResult, &curKey[0], nanoflann::SearchParams(10)); 

		// distance 
		auto cur = grsdDict[currentPtr]; // current observation (query)
		auto candidate = grsdDict[candidateIndexes[0]];
		float min_dist = (cur - candidate).norm();
		int nn_idx = candidateIndexes[0];

		// loop threshold check
		if(min_dist < 160)
		{
			loopId = nn_idx; 
			ROS_INFO("\033[1;33m[GRSD Loop found] btn %d and %d. Dis: %.2f.\033[0m", currentPtr, nn_idx, min_dist);
		}
		else
		{
			ROS_DEBUG("\033[1;33m[GRSD Not loop] btn %d and %d. Dis: %.2f.\033[0m", currentPtr, nn_idx, min_dist);
		}

		std::pair<int, float> result {loopId, min_dist};
		return result;
	}
	
	std::pair<int8_t, int> getIndex(const int key)
	{
		return grsdIndexs[key];
	}
	
	int getSize(const int idIn = -1)
	{
		return grsdIndexs.size();
	}

private:
    std::vector<Eigen::VectorXf> grsdDict;
    std::vector<std::vector<float>> grsdVecDict;
	std::vector<std::vector<float>> grsdVecSearch;
	std::vector<std::pair<int8_t,int>> grsdIndexs;
    std::unique_ptr<KDTreeVectorOfVectorsAdaptor<std::vector<std::vector<float>>,float>> grsdTree;

	float neRadius = 0.5;
	int numExcludeRecent = 30;
	int treeMakingPeriodConter = 0;
	int treeMakingPeriod = 10;
	int numCandidates = 10;

	pcl::PointCloud<pcl::PointXYZI>::Ptr inputScan;
	pcl::search::KdTree<pcl::PointXYZI>::Ptr tree;
	pcl::PointCloud<pcl::Normal>::Ptr scanNormals;
	pcl::PointCloud<pcl::GRSDSignature21>::Ptr descriptors;
};

class shot_descriptor : public scan_descriptor
{
public:
	std::vector<float> makeAndSaveDescriptorAndKey(const pcl::PointCloud<pcl::PointXYZI>& scan, const int8_t robot, const int index)
	{
		std::cout<<"SHOT START "<<std::endl;

		// // Extract point cloud
		// pcl::PointCloud<pcl::PointXYZI>::Ptr inputScan(new pcl::PointCloud<pcl::PointXYZI>);
		// *inputScan = scan;

		// // Extract surface normals for point cloud
		// pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> ne;
		// pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>());
		// pcl::PointCloud<pcl::Normal>::Ptr scanNormals(new pcl::PointCloud<pcl::Normal>);
		// ne.setInputCloud(inputScan);
		// ne.setSearchMethod(tree);
		// ne.setRadiusSearch(neRadius);
		// ne.compute(*scanNormals);

		// // Get rid off NaNs (Grsd doesn't filter them and will break).
		// std::vector<int> indices_good_cloud;
		// std::vector<int> indices_good_cloud_normals;
		// pcl::removeNaNFromPointCloud(*inputScan, *inputScan, indices_good_cloud);
		// pcl::removeNaNNormalsFromPointCloud(*scanNormals, *scanNormals, indices_good_cloud_normals);

		std::cout<<"SHOT END "<<std::endl;
	}
	
	void saveDescriptorAndKey(const float* descriptorMat, const int8_t robot, const int index)
	{

	}

	std::pair<int, float> detectIntraLoopClosureID(const int curPtr)
	{

	}
	
	std::pair<int, float> detectInterLoopClosureID(const int currentPtr)
	{

	}
	
	std::pair<int8_t, int> getIndex(const int key)
	{

	}
	
	int getSize(const int idIn = -1)
	{

	}
};

class fpfh_descriptor : public scan_descriptor
{
public:
	fpfh_descriptor()
	{
		inputScan.reset(new pcl::PointCloud<pcl::PointXYZI>);
		tree.reset(new pcl::search::KdTree<pcl::PointXYZI>());
		scanNormals.reset(new pcl::PointCloud<pcl::Normal>);
	}

	// static pcl::PointXYZI calculateCentroid(const pcl::PointCloud<pcl::PointXYZI>& point_cloud)
	// {
	// 	pcl::PointXYZI centroid;
	// 	centroid.x = 0.0;
	// 	centroid.y = 0.0;
	// 	centroid.z = 0.0;
	// 	centroid.intensity = 0.0;

	// 	const size_t n_points = point_cloud.points.size();
	// 	for(const auto& point : point_cloud.points)
	// 	{
	// 		centroid.x += point.x / n_points;
	// 		centroid.y += point.y / n_points;
	// 		centroid.z += point.z / n_points;
	// 		centroid.intensity += point.intensity / n_points;
	// 	}

	// 	// Check that there were no overflows, underflows, or invalid float operations.
	// 	if(std::fetestexcept(FE_OVERFLOW))
	// 	{
	// 		ROS_WARN("Overflow error in centroid computation.");
	// 	}
	// 	else if(std::fetestexcept(FE_UNDERFLOW))
	// 	{
	// 		ROS_WARN("Underflow error in centroid computation.");
	// 	}
	// 	else if(std::fetestexcept(FE_INVALID))
	// 	{
	// 		ROS_WARN("Invalid Flag error in centroid computation.");
	// 	}
	// 	else if(std::fetestexcept(FE_DIVBYZERO))
	// 	{
	// 		ROS_WARN("Divide by zero error in centroid computation.");
	// 	}
	// 	return centroid;
	// }

	void save(const std::vector<float> fpfhVec, const int8_t robot, const int index)
	{
		Eigen::VectorXf fpfhMat = Eigen::Map<const Eigen::Matrix<float, -1, -1, Eigen::RowMajor>>(fpfhVec.data(), 21, 1);
		fpfhDict.push_back(fpfhMat);
		fpfhVecDict.push_back(fpfhVec);
		fpfhIndexs.push_back(std::make_pair(robot,index));
	}

	std::vector<float> makeAndSaveDescriptorAndKey(const pcl::PointCloud<pcl::PointXYZI>& scan, const int8_t robot, const int index)
	{
		std::cout << "FPFH START " << std::endl;
		clock_t startTime, endTime;
		startTime = ros::Time::now().toNSec();

		// Extract point cloud
		*inputScan = scan;

		// Extract surface normals for point cloud
		pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> ne;
		ne.setInputCloud(inputScan);
		ne.setSearchMethod(tree);
		ne.setKSearch(10);
		// ne.setRadiusSearch(neRadius);
		ne.compute(*scanNormals);

		// Get rid off NaNs (FPFH doesn't filter them and will break).
		// std::vector<int> indices_good_cloud;
		// std::vector<int> indices_good_cloud_normals;
		// pcl::removeNaNFromPointCloud(*inputScan, *inputScan, indices_good_cloud);
		// pcl::removeNaNNormalsFromPointCloud(*scanNormals, *scanNormals, indices_good_cloud_normals);

		// Get centroid
		// pcl::PointXYZI centroid = calculateCentroid(*inputScan);
		// inputScan->push_back(centroid);
		// fake normal for centroid makes descriptor invariant to centroid normal
  		// pcl::Normal centroidNormal(0.0,0.0,1.0);
		// scanNormals->push_back(centroidNormal);

		// Create FPFHE class and pass data+normals to it.
		pcl::FPFHEstimation<pcl::PointXYZI, pcl::Normal, pcl::FPFHSignature33> fpfh;
		fpfh.setInputCloud(inputScan);
		fpfh.setInputNormals(scanNormals);
		fpfh.setSearchMethod(tree);
		// Only compute SPFH for centroid.
		std::vector<int> indices(inputScan->size()-1);  // We don't want to include the last point, which is the centroid.
		std::iota(std::begin(indices), std::end(indices), 0);
		int nr_subdiv = 11; // Make param.
		Eigen::MatrixXf histF1(1, nr_subdiv), histF2(1, nr_subdiv), histF3(1, nr_subdiv); 
		histF1.setZero(); 
		histF2.setZero(); 
		histF3.setZero();
		fpfh.computePointSPFHSignature(*inputScan, *scanNormals, (inputScan->size())-1, 0, indices, histF1, histF2, histF3);

		// Extract descriptors
		Eigen::MatrixXf histTot(1, 3*nr_subdiv);
		histTot.setZero();
		histTot << histF1, histF2, histF3;
		Eigen::VectorXf fpfhMat(3*nr_subdiv);
		fpfhMat = histTot.row(0);
		std::vector<float> fpfhVec(&fpfhMat[0], fpfhMat.data()+fpfhMat.cols()*fpfhMat.rows());
		save(fpfhVec, robot, index);

		endTime = ros::Time::now().toNSec();
		std::cout << "FPFH END " << (double)(endTime - startTime) / 10e6 << " ms" << std::endl;
		return fpfhVec;
	}

	void saveDescriptorAndKey(const float* descriptorMat, const int8_t robot, const int index)
	{
		// decode fpfh
		std::vector<float> fpfhVec;
		fpfhVec.insert(fpfhVec.end(), descriptorMat, descriptorMat+33);

		save(fpfhVec, robot, index);
	}

	std::pair<int, float> detectIntraLoopClosureID(const int curPtr)
	{
		
	}

	std::pair<int, float> detectInterLoopClosureID(const int currentPtr)
	{
		int loopId { -1 }; // init with -1, -1 means no loop

		auto curKey = fpfhVecDict[currentPtr]; // current observation (query)

		if((int)fpfhVecDict.size() < numExcludeRecent + 1)
		{
			std::pair<int,float> result {loopId, 0.0};
			return result; // Early return 
		}

		// tree_ reconstruction (not mandatory to make everytime)
		if(treeMakingPeriodConter % treeMakingPeriod == 0) // to save computation cost
		{
			fpfhVecSearch.clear();
			fpfhVecSearch.assign(fpfhVecDict.begin(), fpfhVecDict.end() - numExcludeRecent);

			fpfhTree.reset(); 
			fpfhTree = std::make_unique<KDTreeVectorOfVectorsAdaptor<std::vector<std::vector<float>>,float>>(33, fpfhVecSearch, 10);
		}
		treeMakingPeriodConter += 1;

		// candidates from key tree
		// knn search
		std::vector<size_t> candidateIndexes(1); 
		std::vector<float> outDistsSqr(1);

		nanoflann::KNNResultSet<float> knnsearchResult(1);
		knnsearchResult.init(&candidateIndexes[0], &outDistsSqr[0]);
		fpfhTree->index->findNeighbors(knnsearchResult, &curKey[0], nanoflann::SearchParams(10)); 

		// distance 
		auto cur = fpfhDict[currentPtr]; // current observation (query)
		auto candidate = fpfhDict[candidateIndexes[0]];
		float min_dist = (cur - candidate).norm();
		int nn_idx = candidateIndexes[0];

		// loop threshold check
		if(min_dist < 100)
		{
			loopId = nn_idx; 
			ROS_INFO("\033[1;33m[FPFH Loop found] btn %d and %d. Dis: %.2f.\033[0m", currentPtr, nn_idx, min_dist);
		}
		else
		{
			ROS_INFO("\033[1;33m[FPFH Not loop] btn %d and %d. Dis: %.2f.\033[0m", currentPtr, nn_idx, min_dist);
		}

		std::pair<int, float> result {loopId, min_dist};
		return result;
	}

	std::pair<int8_t, int> getIndex(const int key)
	{
		return fpfhIndexs[key];
	}
	
	int getSize(const int idIn = -1)
	{
		return fpfhIndexs.size();
	}

private:
    std::vector<Eigen::VectorXf> fpfhDict;
    std::vector<std::vector<float>> fpfhVecDict;
	std::vector<std::vector<float>> fpfhVecSearch;
	std::vector<std::pair<int8_t,int>> fpfhIndexs;
    std::unique_ptr<KDTreeVectorOfVectorsAdaptor<std::vector<std::vector<float>>,float>> fpfhTree;

	float neRadius = 0.5;
	int numExcludeRecent = 30;
	int treeMakingPeriodConter = 0;
	int treeMakingPeriod = 10;
	int numCandidates = 10;

	pcl::PointCloud<pcl::PointXYZI>::Ptr inputScan;
	pcl::search::KdTree<pcl::PointXYZI>::Ptr tree;
	pcl::PointCloud<pcl::Normal>::Ptr scanNormals;
};

class lidar_iris_descriptor : public scan_descriptor
{
public:
	struct featureDesc
    {
        cv::Mat1b img;
        cv::Mat1b T;
        cv::Mat1b M;
    };

	lidar_iris_descriptor(
		int rows 				= 80, //80
		int cols 				= 360, //360
		int nscan 				= 64,
		double distThres 		= 0.32,
    	int numExcludeRecent 	= 30,
		int matchNum			= 2,
		int numCandidates 		= 10, 
		int nscale				= 4,
		int minWaveLength 		= 18,
		float mult 				= 1.6,
		float sigmaOnf 			= 0.75,
		int robotNum 			= 1,
		int thisID 				= 0) :
    	_rows(rows),
    	_cols(cols),
		_nscan(nscan),
		_distThres(distThres),
    	_numExcludeRecent(numExcludeRecent),
		_matchNum(matchNum),
		_numCandidates(numCandidates),
		_nscale(nscale),
		_minWaveLength(minWaveLength),
		_mult(mult),
		_sigmaOnf(sigmaOnf),
		_robotNum(robotNum),
		_thisID(thisID)
	{
		for(int i = 0; i < _robotNum; i++)
		{
			std::vector<featureDesc> baseA;
			Eigen::MatrixXf baseB;
			std::vector<int> baseC;
			irisFeatures.push_back(baseA);
			irisFeatureRowKey.push_back(baseB);
			local2Global.push_back(baseC);
		}
	}

	~lidar_iris_descriptor()
	{
	}

	std::pair<Eigen::VectorXf, cv::Mat1b> getIris(const pcl::PointCloud<pcl::PointXYZI> &cloud)
	{
		cv::Mat1b irisMap = cv::Mat1b::zeros(_rows, _cols);
		Eigen::MatrixXf irisRowKeyMat = Eigen::MatrixXf::Zero(_rows, _cols);

		// 16-line
		if(_nscan == 16)
		{
			for(auto p : cloud.points)
			{
				float dis = sqrt(p.data[0] * p.data[0] + p.data[1] * p.data[1]);
				float arc = (atan2(p.data[2], dis) * 180.0f / M_PI) + 15; //[-15, 15]度 + 15； [0, 30]度
				float yaw = (atan2(p.data[1], p.data[0]) * 180.0f / M_PI) + 180; //atan2: (-pi, pi]
				int Q_dis = std::min(std::max((int)floor(dis), 0), (_rows-1));
				int Q_arc = std::min(std::max((int)floor(arc / 4.0f), 0), 7); //paper中是按照yl, yh分了8个bins。这里是按照pitch角度分了8个bins
				int Q_yaw = std::min(std::max((int)floor(yaw + 0.5), 0), (_cols-1));
				irisMap.at<uint8_t>(Q_dis, Q_yaw) |= (1 << Q_arc); //将1左移Q_arc位， 相当于乘以2的n次方; 按位或
				if(irisRowKeyMat(Q_dis, Q_yaw) < p.data[2])
				{
					irisRowKeyMat(Q_dis, Q_yaw) = p.data[2]; // update for taking maximum value at that bin
				}
			}
		}
		// 64-line
		else if(_nscan == 64)
		{
			for(auto p : cloud.points)
			{
				float dis = sqrt(p.data[0] * p.data[0] + p.data[1] * p.data[1]); // xy-plane distance
				float arc = (atan2(p.data[2], dis) * 180.0f / M_PI) + 24.9; // [0, 26.9] deg.
				float yaw = (atan2(p.data[1], p.data[0]) * 180.0f / M_PI) + 180; // (-pi, pi] rad.
				int Q_dis = std::min(std::max((int)floor(dis), 0), (_rows-1));
				int Q_arc = std::min(std::max((int)floor(arc / 4.0f), 0), 7);
				// int Q_arc = std::min(std::max((int)ceil(p.data[2] + 5), 0), 7);
				int Q_yaw = std::min(std::max((int)floor(yaw + 0.5), 0), (_cols-1));
				irisMap.at<uint8_t>(Q_dis, Q_yaw) |= (1 << Q_arc);
				if(irisRowKeyMat(Q_dis, Q_yaw) < p.data[2])
				{
					irisRowKeyMat(Q_dis, Q_yaw) = p.data[2]; // update for taking maximum value at that bin
				}
			}
		}

		// for(int i = 0; i < _rows; i++)
		// {
		// 	for(int j = 0; j < _cols; j++)
		// 	{
		// 		irisRowKeyMat(i,j) = irisMap.at<uint8_t>(i, j);
		// 	}
		// }

		Eigen::VectorXf rowKey = Eigen::VectorXf::Zero(_rows);
		// for(int i = 0; i < _rows; i++)
		// {
			rowKey = irisRowKeyMat.rowwise().mean();
			// rowKey(i) = currRow.mean();
		// }

		std::pair<Eigen::VectorXf, cv::Mat1b> result {rowKey, irisMap};
		return result;
	}

	inline cv::Mat circRowShift(const cv::Mat &src, int shift_m_rows)
	{
		if(shift_m_rows == 0)
		{
			return src.clone();
		}
		shift_m_rows %= src.rows;
		int m = shift_m_rows > 0 ? shift_m_rows : src.rows + shift_m_rows;
		cv::Mat dst(src.size(), src.type());
		src(cv::Range(src.rows - m, src.rows), cv::Range::all()).copyTo(dst(cv::Range(0, m), cv::Range::all()));
		src(cv::Range(0, src.rows - m), cv::Range::all()).copyTo(dst(cv::Range(m, src.rows), cv::Range::all()));
		return dst;
	}

	inline cv::Mat circColShift(const cv::Mat &src, int shift_n_cols)
	{
		if(shift_n_cols == 0)
		{
			return src.clone();
		}
		shift_n_cols %= src.cols;
		int n = shift_n_cols > 0 ? shift_n_cols : src.cols + shift_n_cols;
		cv::Mat dst(src.size(), src.type());
		src(cv::Range::all(), cv::Range(src.cols - n, src.cols)).copyTo(dst(cv::Range::all(), cv::Range(0, n)));
		src(cv::Range::all(), cv::Range(0, src.cols - n)).copyTo(dst(cv::Range::all(), cv::Range(n, src.cols)));
		return dst;
	}

	cv::Mat circShift(const cv::Mat &src, int shift_m_rows, int shift_n_cols)
	{
		return circColShift(circRowShift(src, shift_m_rows), shift_n_cols);
	}

	std::vector<cv::Mat2f> logGaborFilter(const cv::Mat1f &src, unsigned int nscale, int minWaveLength, double mult, double sigmaOnf)
	{
		int rows = src.rows;
		int cols = src.cols;
		cv::Mat2f filtersum = cv::Mat2f::zeros(1, cols);
		std::vector<cv::Mat2f> EO(nscale);
		int ndata = cols;
		if(ndata % 2 == 1)
		{
			ndata--;
		}
		cv::Mat1f logGabor = cv::Mat1f::zeros(1, ndata);
		cv::Mat2f result = cv::Mat2f::zeros(rows, ndata);
		cv::Mat1f radius = cv::Mat1f::zeros(1, ndata / 2 + 1);
		radius.at<float>(0, 0) = 1;
		for(int i = 1; i < ndata / 2 + 1; i++)
		{
			radius.at<float>(0, i) = i / (float)ndata;
		}
		double wavelength = minWaveLength;
		for(int s = 0; s < nscale; s++)
		{
			double fo = 1.0 / wavelength;
			double rfo = fo / 0.5;
			
			cv::Mat1f temp; //(radius.size());
			cv::log(radius / fo, temp);
			cv::pow(temp, 2, temp);
			cv::exp((-temp) / (2 * log(sigmaOnf) * log(sigmaOnf)), temp);
			temp.copyTo(logGabor.colRange(0, ndata / 2 + 1));
			
			logGabor.at<float>(0, 0) = 0;
			cv::Mat2f filter;
			cv::Mat1f filterArr[2] = {logGabor, cv::Mat1f::zeros(logGabor.size())};
			cv::merge(filterArr, 2, filter);
			filtersum = filtersum + filter;
			for(int r = 0; r < rows; r++)
			{
				cv::Mat2f src2f;
				cv::Mat1f srcArr[2] = {src.row(r).clone(), cv::Mat1f::zeros(1, src.cols)};
				cv::merge(srcArr, 2, src2f);
				cv::dft(src2f, src2f);
				cv::mulSpectrums(src2f, filter, src2f, 0);
				cv::idft(src2f, src2f);
				src2f.copyTo(result.row(r));
			}
			EO[s] = result.clone();
			wavelength *= mult;
		}
		filtersum = circShift(filtersum, 0, cols / 2);
		return EO;
	}

	void logFeatureEncode(const cv::Mat1b &src, unsigned int nscale, int minWaveLength, double mult, double sigmaOnf, cv::Mat1b &T, cv::Mat1b &M)
	{
		cv::Mat1f srcFloat;
		src.convertTo(srcFloat, CV_32FC1);
		auto list = logGaborFilter(srcFloat, nscale, minWaveLength, mult, sigmaOnf);
		std::vector<cv::Mat1b> Tlist(nscale * 2), Mlist(nscale * 2);
		for (int i = 0; i < list.size(); i++)
		{
			cv::Mat1f arr[2];
			cv::split(list[i], arr);
			Tlist[i] = arr[0] > 0;
			Tlist[i + nscale] = arr[1] > 0;

			cv::Mat1f m;
			cv::magnitude(arr[0], arr[1], m);
			Mlist[i] = m < 0.0001;
			Mlist[i + nscale] = m < 0.0001;
		}
		cv::vconcat(Tlist, T);
		cv::vconcat(Mlist, M);
	}

	featureDesc getFeature(const cv::Mat1b &src)
	{
		featureDesc desc;
		desc.img = src;
		logFeatureEncode(src, _nscale, _minWaveLength, _mult, _sigmaOnf, desc.T, desc.M);
		return desc;
	}

	featureDesc getFeature(const cv::Mat1b &src, std::vector<float> &vec)
	{
		cv::Mat1f temp;
		src.convertTo(temp, CV_32FC1);
		cv::reduce((temp != 0) / 255, temp, 1, CV_REDUCE_AVG);
		vec = temp.isContinuous() ? temp : temp.clone();
		return getFeature(src);
	}

	void recomb(cv::Mat &src, cv::Mat &dst) // Recombinate image quaters
	{
		int cx = src.cols >> 1;
		int cy = src.rows >> 1;
		cv::Mat tmp;
		tmp.create(src.size(), src.type());
		src(cv::Rect(0, 0, cx, cy)).copyTo(tmp(cv::Rect(cx, cy, cx, cy)));
		src(cv::Rect(cx, cy, cx, cy)).copyTo(tmp(cv::Rect(0, 0, cx, cy)));
		src(cv::Rect(cx, 0, cx, cy)).copyTo(tmp(cv::Rect(0, cy, cx, cy)));
		src(cv::Rect(0, cy, cx, cy)).copyTo(tmp(cv::Rect(cx, 0, cx, cy)));
		dst = tmp;
	}

	void forwardFFT(cv::Mat &Src, cv::Mat *FImg, bool do_recomb = true) // 2D Forward FFT
	{
		int M = cv::getOptimalDFTSize(Src.rows);
		int N = cv::getOptimalDFTSize(Src.cols);
		cv::Mat padded;
		copyMakeBorder(Src, padded, 0, M - Src.rows, 0, N - Src.cols, cv::BORDER_CONSTANT, cv::Scalar::all(0));
		cv::Mat planes[] = { cv::Mat_<float>(padded), cv::Mat::zeros(padded.size(), CV_32F) };
		cv::Mat complexImg;
		merge(planes, 2, complexImg);
		dft(complexImg, complexImg);
		split(complexImg, planes);
		planes[0] = planes[0](cv::Rect(0, 0, planes[0].cols & -2, planes[0].rows & -2));
		planes[1] = planes[1](cv::Rect(0, 0, planes[1].cols & -2, planes[1].rows & -2));
		if(do_recomb)
		{
			recomb(planes[0], planes[0]);
			recomb(planes[1], planes[1]);
		}
		planes[0] /= float(M*N);
		planes[1] /= float(M*N);
		FImg[0] = planes[0].clone();
		FImg[1] = planes[1].clone();
	}

	void highpass(cv::Size sz, cv::Mat& dst)
	{
		cv::Mat a = cv::Mat(sz.height, 1, CV_32FC1);
		cv::Mat b = cv::Mat(1, sz.width, CV_32FC1);

		float step_y = CV_PI / sz.height;
		float val = -CV_PI*0.5;

		for(int i = 0; i < sz.height; ++i)
		{
			a.at<float>(i) = cos(val);
			val += step_y;
		}

		val = -CV_PI*0.5;
		float step_x = CV_PI / sz.width;
		for(int i = 0; i < sz.width; ++i)
		{
			b.at<float>(i) = cos(val);
			val += step_x;
		}

		cv::Mat tmp = a*b;
		dst = (1.0 - tmp).mul(2.0 - tmp);
	}

	float logpolar(cv::Mat& src, cv::Mat& dst)
	{
		float radii = src.cols;
		float angles = src.rows;
		cv::Point2f center(src.cols / 2, src.rows / 2);
		float d = cv::norm(cv::Vec2f(src.cols - center.x, src.rows - center.y));
		float log_base = std::pow(10.0, log10(d) / radii);
		float d_theta = CV_PI / (float)angles;
		float theta = CV_PI / 2.0;
		float radius = 0;
		cv::Mat map_x(src.size(), CV_32FC1);
		cv::Mat map_y(src.size(), CV_32FC1);
		for(int i = 0; i < angles; ++i)
		{
			for(int j = 0; j < radii; ++j)
			{
				radius = std::pow(log_base, float(j));
				float x = radius * sin(theta) + center.x;
				float y = radius * cos(theta) + center.y;
				map_x.at<float>(i, j) = x;
				map_y.at<float>(i, j) = y;
			}
			theta += d_theta;
		}
		cv::remap(src, dst, map_x, map_y, CV_INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
		return log_base;
	}

	cv::RotatedRect logPolarFFTTemplateMatch(cv::Mat& im0, cv::Mat& im1/*, double canny_threshold1, double canny_threshold2*/)
	{
		// Accept 1 or 3 channel CV_8U, CV_32F or CV_64F images.
		CV_Assert((im0.type() == CV_8UC1) || (im0.type() == CV_8UC3) ||
			(im0.type() == CV_32FC1) || (im0.type() == CV_32FC3) ||
			(im0.type() == CV_64FC1) || (im0.type() == CV_64FC3));

		CV_Assert(im0.rows == im1.rows && im0.cols == im1.cols);

		CV_Assert(im0.channels() == 1 || im0.channels() == 3 || im0.channels() == 4);

		CV_Assert(im1.channels() == 1 || im1.channels() == 3 || im1.channels() == 4);

		//cv::Mat im0_tmp = im0.clone();
		//cv::Mat im1_tmp = im1.clone();
		if(im0.channels() == 3)
		{
			cv::cvtColor(im0, im0, cv::ColorConversionCodes::COLOR_BGR2GRAY);
		}

		if(im0.channels() == 4)
		{
			cv::cvtColor(im0, im0, cv::ColorConversionCodes::COLOR_BGRA2GRAY);
		}

		if(im1.channels() == 3)
		{
			cv::cvtColor(im1, im1, cv::ColorConversionCodes::COLOR_BGR2GRAY);
		}

		if(im1.channels() == 4)
		{
			cv::cvtColor(im1, im1, cv::ColorConversionCodes::COLOR_BGRA2GRAY);
		}

		if(im0.type() == CV_32FC1)
		{
			im0.convertTo(im0, CV_8UC1, 255.0);
		}

		if(im1.type() == CV_32FC1)
		{
			im1.convertTo(im1, CV_8UC1, 255.0);
		}

		if(im0.type() == CV_64FC1)
		{
			im0.convertTo(im0, CV_8UC1, 255.0);
		}

		if(im1.type() == CV_64FC1)
		{
			im1.convertTo(im1, CV_8UC1, 255.0);
		}

		// Canny(im0, im0, canny_threshold1, canny_threshold2); // you can change this
		// Canny(im1, im1, canny_threshold1, canny_threshold2);
		
		// Ensure both images are of CV_32FC1 type
		im0.convertTo(im0, CV_32FC1, 1.0 / 255.0);
		im1.convertTo(im1, CV_32FC1, 1.0 / 255.0);

		cv::Mat F0[2], F1[2];
		cv::Mat f0, f1;
		forwardFFT(im0, F0);
		forwardFFT(im1, F1);
		cv::magnitude(F0[0], F0[1], f0);
		cv::magnitude(F1[0], F1[1], f1);

		// Create filter 
		cv::Mat h;
		highpass(f0.size(), h);

		// Apply it in freq domain
		f0 = f0.mul(h);
		f1 = f1.mul(h);

		float log_base;
		cv::Mat f0lp, f1lp;

		log_base = logpolar(f0, f0lp);
		log_base = logpolar(f1, f1lp);

		// Find rotation and scale
		cv::Point2d rotation_and_scale = cv::phaseCorrelate(f1lp, f0lp);

		float angle = 180.0 * rotation_and_scale.y / f0lp.rows;
		float scale = pow(log_base, rotation_and_scale.x);
		// --------------
		if(scale > 1.8)
		{
			rotation_and_scale = cv::phaseCorrelate(f1lp, f0lp);
			angle = -180.0 * rotation_and_scale.y / f0lp.rows;
			scale = 1.0 / pow(log_base, rotation_and_scale.x);
			if (scale > 1.8)
			{
				std::cout << "Images are not compatible. Scale change > 1.8" << std::endl;
				return cv::RotatedRect();
			}
		}
		// --------------
		if(angle < -90.0)
		{
			angle += 180.0;
		}
		else if(angle > 90.0)
		{
			angle -= 180.0;
		}

		// Now rotate and scale fragment back, then find translation
		cv::Mat rot_mat = cv::getRotationMatrix2D(cv::Point(im1.cols / 2, im1.rows / 2), angle, 1.0 / scale);

		// rotate and scale
		cv::Mat im1_rs;
		cv::warpAffine(im1, im1_rs, rot_mat, im1.size());

		// find translation
		cv::Point2d tr = cv::phaseCorrelate(im1_rs, im0);

		// compute rotated rectangle parameters
		cv::RotatedRect rr;
		rr.center = tr + cv::Point2d(im0.cols / 2, im0.rows / 2);
		rr.angle = -angle;
		rr.size.width = im1.cols / scale;
		rr.size.height = im1.rows / scale;

		//im0 = im0_tmp.clone();
		//im1 = im1_tmp.clone();

		return rr;
	}

	cv::RotatedRect fftMatch(const cv::Mat& im0, const cv::Mat& im1)
	{
		cv::Mat im0_tmp = im0.clone();
		cv::Mat im1_tmp = im1.clone();
		return logPolarFFTTemplateMatch(im0_tmp, im1_tmp);
	}

	void getHammingDistance(const cv::Mat1b &T1, const cv::Mat1b &M1, const cv::Mat1b &T2, const cv::Mat1b &M2, int scale, float &dis, int &bias)
	{
		dis = NAN;
		bias = -1;
		for(int shift = scale - 2; shift <= scale + 2; shift++)
		{
			cv::Mat1b T1s = circShift(T1, 0, shift);
			cv::Mat1b M1s = circShift(M1, 0, shift);
			cv::Mat1b mask = M1s | M2;
			int MaskBitsNum = cv::sum(mask / 255)[0];
			int totalBits = T1s.rows * T1s.cols - MaskBitsNum;
			cv::Mat1b C = T1s ^ T2;
			C = C & ~mask;
			int bitsDiff = cv::sum(C / 255)[0];
			if(totalBits == 0)
			{
				dis = NAN;
			}
			else
			{
				float currentDis = bitsDiff / (float)totalBits;
				if(currentDis < dis || isnan(dis))
				{
					dis = currentDis;
					bias = shift;
				}
			}
		}
		return;
	}

	float compare(const featureDesc &img1, const featureDesc &img2, int *bias)
	{
		if(_matchNum==2)
		{
			auto firstRect = fftMatch(img2.img, img1.img);
			int firstShift = firstRect.center.x - img1.img.cols / 2;
			float dis1;
			int bias1;
			getHammingDistance(img1.T, img1.M, img2.T, img2.M, firstShift, dis1, bias1);
			
			auto T2x = circShift(img2.T, 0, 180);
			auto M2x = circShift(img2.M, 0, 180);
			auto img2x = circShift(img2.img, 0, 180);
			
			auto secondRect = fftMatch(img2x, img1.img);
			int secondShift = secondRect.center.x - img1.img.cols / 2;
			float dis2 = 0;
			int bias2 = 0;
			getHammingDistance(img1.T, img1.M, T2x, M2x, secondShift, dis2, bias2);
			
			if (dis1 < dis2)
			{
				if (bias)
					*bias = bias1;
				return dis1;
			}
			else
			{
				if (bias)
					*bias = (bias2 + 180) % 360;
				return dis2;
			}
		}
		if(_matchNum==1)
		{
			auto T2x = circShift(img2.T, 0, 180);
			auto M2x = circShift(img2.M, 0, 180);
			auto img2x = circShift(img2.img, 0, 180);

			auto secondRect = fftMatch(img2x, img1.img);
			int secondShift = secondRect.center.x - img1.img.cols / 2;
			float dis2 = 0;
			int bias2 = 0;
			getHammingDistance(img1.T, img1.M, T2x, M2x, secondShift, dis2, bias2);
			if (bias)
				*bias = (bias2 + 180) % 360;
			return dis2;
		}
		if(_matchNum==0)
		{
			auto firstRect = fftMatch(img2.img, img1.img);
			int firstShift = firstRect.center.x - img1.img.cols / 2;
			float dis1;
			int bias1;
			getHammingDistance(img1.T, img1.M, img2.T, img2.M, firstShift, dis1, bias1);
			if (bias)
				*bias = bias1;
			return dis1;
		}
	}

	// User-side API
	void saveDescriptorAndKey(const float* iris, const int8_t robot, const int index)
	{
		cv::Mat1b irisMap = cv::Mat1b::zeros(_rows, _cols);
		Eigen::VectorXf rowKey = Eigen::VectorXf::Zero(_rows);
		for(int row_idx = 0; row_idx < irisMap.rows; row_idx++)
		{
			for(int col_idx = 0; col_idx < irisMap.cols; col_idx++)
			{
				irisMap(row_idx, col_idx) = iris[row_idx*(irisMap.cols+1)+col_idx+1];
			}
		}

		for(int row_idx = 0; row_idx < irisMap.rows; row_idx++)
		{
			rowKey(row_idx) = iris[row_idx+irisMap.rows*irisMap.cols];
		}
		
		save(irisMap, rowKey, robot, index);
	}

	void save(const cv::Mat1b iris, Eigen::MatrixXf rowKey, const int8_t robot, const int index)
	{
		std::vector<float> rowKeyVec;
		auto irisFeature = getFeature(iris);

		// iris freature (descriptor) for single robot
		irisFeatures[robot].push_back(irisFeature);
		// row key for knn search
		irisFeatureRowKey[robot].conservativeResize(_rows, irisFeatures[robot].size());
		irisFeatureRowKey[robot].block(0, irisFeatures[robot].size()-1, _rows, 1) = rowKey.block(0, 0, _rows, 1);
		// trasform local index to global
		local2Global[robot].push_back(irisFeatureIndexs.size());
		// descriptor global index
		irisFeatureIndexs.push_back(std::make_pair(robot,index)); //index

		// ROS_INFO("save descriptor<%d> r:%d l:%d, g:%d~ irisFeatures:%d irisFeatureRowKey:%d local2Global:%d", _thisID, robot, index, irisFeatureIndexs.size()-1, 
		// 	irisFeatures[robot].size(), irisFeatureRowKey[robot].cols(), local2Global[robot].size());
	}

	std::vector<float> makeAndSaveDescriptorAndKey(const pcl::PointCloud<pcl::PointXYZI>& scan, const int8_t robot, const int index)
	{
		auto scanIris = getIris(scan);
		save(scanIris.second, scanIris.first, robot, index);

		std::vector<float> vT;
		for(int row_idx = 0; row_idx < scanIris.second.rows; row_idx++)
		{
			for(int col_idx = 0; col_idx < scanIris.second.cols; col_idx++)
			{
				vT.push_back(scanIris.second(row_idx, col_idx));
			}
		}

		for(int row_idx = 0; row_idx < scanIris.second.rows; row_idx++)
		{
			vT.push_back(scanIris.first(row_idx, 0));
		}
		
		return vT;
	}

	std::pair<int, float> detectIntraLoopClosureID(const int curPtr)
	{
		std::pair<int, float> result {-1, 0.0};
		Eigen::VectorXf curKey = irisFeatureRowKey[_thisID].col(curPtr);; // current query row key
		featureDesc curFeature = irisFeatures[_thisID][curPtr]; // current feature

		// step 1: candidates from rowkey tree_
		if(curPtr < _numExcludeRecent + _numCandidates + 1)
		{
			return result; // Early return 
		}

		// tree reconstruction
		Eigen::MatrixXf newIrisFeatureRowKey;
		int historyIndex = curPtr - _numExcludeRecent;
		newIrisFeatureRowKey.conservativeResize(_rows, historyIndex);
		newIrisFeatureRowKey.block(0, 0, _rows, historyIndex) = irisFeatureRowKey[_thisID].block(0, 0, _rows, historyIndex);
		kdTree = Nabo::NNSearchF::createKDTreeTreeHeap(newIrisFeatureRowKey, _rows);

		// search n nearest neighbors
        Eigen::VectorXi indice(_numCandidates);
        Eigen::VectorXf distance(_numCandidates);
		float minDis = 10000000.0;
		int minIndex = -1;
		int minBias = 0;

		// knn search
		kdTree->knn(curKey, indice, distance, _numCandidates);

		// step 2: pairwise distance
		for(int i = 0; i < std::min(_numCandidates, int(indice.size())); i++)
		{
			if(indice[i] >= local2Global[_thisID].size())
			{
				continue;
			}

			int bias;
			featureDesc candidateFeature = irisFeatures[_thisID][indice[i]];
			float candidateDis = compare(curFeature, candidateFeature, &bias);

			if(candidateDis < minDis)
			{
				minDis = candidateDis;
				minIndex = indice[i];
				minBias = bias;
			}
		}

		// loop threshold check
		if(minDis < _distThres)
		{
			result.first = minIndex;
			result.second = minBias;
			ROS_INFO("\033[1;33m[Iris Intra Loop<%d>] btn %d and %d. Dis: %.2f.\033[0m", _thisID, curPtr, minIndex, minDis);
		}
		else
		{
			ROS_INFO("\033[1;33m[Iris Intra Not loop<%d>] btn %d and %d. Dis: %.2f.\033[0m", _thisID, curPtr, minIndex, minDis);
		}
		return result;
	}

	std::pair<int, float> detectInterLoopClosureID(const int curPtr)
	{
		std::pair<int, float> result {-1, 0.0};
		int curRobot = irisFeatureIndexs[curPtr].first;
		int curIndex = irisFeatureIndexs[curPtr].second;
		Eigen::VectorXf curKey = irisFeatureRowKey[curRobot].col(curIndex);; // current query row key
		featureDesc curFeature = irisFeatures[curRobot][curIndex]; // current feature
		ROS_INFO("\033[1;33m[Iris Inter Loop<%d>] curPtr %d--->%d %d.\033[0m", _thisID, curPtr, curRobot, curIndex);

		// step 1: candidates from rowkey tree_
		// tree reconstruction
		Eigen::MatrixXf newIrisFeatureRowKey;
		std::vector<int> newLocal2Global;
		std::vector<featureDesc> newIrisFeatures;
		if(curRobot == _thisID)
		{
			for(int i = 0; i < _robotNum; i++)
			{
				if(i != _thisID && local2Global[i].size() > 0)
				{
					int curRow = newIrisFeatureRowKey.cols();
					int addRow = local2Global[i].size();
					// ROS_INFO("\033[1;33m[Iris Inter Loop<%d>] add %d size %d.\033[0m", _thisID, i, addRow);
					newIrisFeatureRowKey.conservativeResize(_rows, curRow + addRow);
					newIrisFeatureRowKey.block(0, curRow, _rows, addRow) = irisFeatureRowKey[i].block(0, 0, _rows, addRow);
					newLocal2Global.insert(newLocal2Global.end(), local2Global[i].begin(), local2Global[i].end());
					newIrisFeatures.insert(newIrisFeatures.end(), irisFeatures[i].begin(), irisFeatures[i].end());
				}
			}
		}
		else
		{
			if(local2Global[_thisID].size() > 0)
			{
				int thisSize = local2Global[_thisID].size();
				// ROS_INFO("\033[1;33m[Iris Inter Loop<%d>] add row %d.\033[0m", _thisID, thisSize);
				newIrisFeatureRowKey.conservativeResize(_rows, thisSize);
				newIrisFeatureRowKey.block(0, 0, _rows, thisSize) = irisFeatureRowKey[_thisID].block(0, 0, _rows, thisSize);
				newLocal2Global.insert(newLocal2Global.end(), local2Global[_thisID].begin(), local2Global[_thisID].end());
				newIrisFeatures.insert(newIrisFeatures.end(), irisFeatures[_thisID].begin(), irisFeatures[_thisID].end());
			}
		}
		// ROS_INFO("\033[1;33m[Iris Inter Loop<%d>] Size: %d. %d. %d.\033[0m", _thisID, newIrisFeatureRowKey.cols(), newLocal2Global.size(), newIrisFeatures.size());

		if(newLocal2Global.size() < _numCandidates + 1)
		{
			return result;
		}

		kdTree = Nabo::NNSearchF::createKDTreeTreeHeap(newIrisFeatureRowKey);

		// search n nearest neighbors
        Eigen::VectorXi indice(_numCandidates);
        Eigen::VectorXf distance(_numCandidates);
		float minDis = 10000000.0;
		int minIndex = -1;
		int minBias = 0;

		// knn search
		kdTree->knn(curKey, indice, distance, _numCandidates);

		// step 2: pairwise distance
		for(int i = 0; i < std::min(_numCandidates, int(indice.size())); i++)
		{
			if(indice[i] >= newLocal2Global.size())
			{
				ROS_INFO("\033[1;33m[Iris Inter Loop<%d>] candiate %d continue.\033[0m", _thisID, indice[i]);
				continue;
			}

			int bias;
			featureDesc candidateFeature = newIrisFeatures[indice[i]];
			float candidateDis = compare(curFeature, candidateFeature, &bias);

			ROS_INFO("\033[1;33m[Iris Inter Loop<%d>] candiate %d dis%.2f.\033[0m", _thisID, indice[i], candidateDis);

			if(candidateDis < minDis)
			{
				minDis = candidateDis;
				minIndex = newLocal2Global[indice[i]];
				minBias = bias;
			}
		}

		// loop threshold check
		if(minDis < _distThres)
		{
			result.first = minIndex;
			result.second = minBias;
			ROS_INFO("\033[1;33m[Iris Inter Loop<%d>] btn %d-%d and %d-%d. Dis: %.2f. Bias:%d\033[0m", _thisID,
				irisFeatureIndexs[curPtr].first, irisFeatureIndexs[curPtr].second,
				irisFeatureIndexs[minIndex].first, irisFeatureIndexs[minIndex].second, minDis, minBias);
		}
		else
		{
			ROS_INFO("\033[1;33m[Iris Inter Not loop<%d>] btn %d-%d and %d-%d. Dis: %.2f. Bias:%d\033[0m", _thisID,
				irisFeatureIndexs[curPtr].first, irisFeatureIndexs[curPtr].second,
				irisFeatureIndexs[minIndex].first, irisFeatureIndexs[minIndex].second, minDis, minBias);
		}
		return result;
	}

	std::pair<int8_t, int> getIndex(const int key)
	{
		return irisFeatureIndexs[key];
	}

	int getSize(const int idIn = -1)
	{
		if(idIn == -1)
		{
			return irisFeatureIndexs.size();
		}
		else
		{
			return local2Global[idIn].size();
		}
	}

private:
    int _rows;
    int _cols;
	int _nscan;
	double _distThres;
	int _nscale;
    int _minWaveLength;
    float _mult;
    float _sigmaOnf;
    int _matchNum;
	int _robotNum;
	int _thisID;

	// flann::Index<flann::L2<float>> vecList;
    // std::vector<featureDesc> featureList;
	// flann::Matrix<int> indices;
    // flann::Matrix<float> dists;
    // std::vector<int> indicesBuffer;
    // std::vector<float> distsBuffer;

    // tree
    int _numExcludeRecent;
	int _numCandidates;

	// config 
    // int _treeMakingPeriod;
    // int tree_making_period_conter;
	
	std::vector<std::vector<featureDesc>> irisFeatures;
	std::vector<Eigen::MatrixXf> irisFeatureRowKey;
	std::vector<std::vector<int>> local2Global;
	std::vector<std::pair<int8_t,int>> irisFeatureIndexs;
	Nabo::NNSearchF* kdTree = NULL;
};

class scan_context_descriptor : public scan_descriptor
{
public:
    scan_context_descriptor(
		int numRing 			= 20,
		int numSector 			= 60,
		int numCandidates 		= 3,
		double distThres 		= 0.14,
		double lidarHeight 		= 1.65,
		double maxRadius 		= 80.0,
		int numExcludeRecent 	= 100,
		int treeMakingPeriod 	= 10,
		double searchRatio 		= 0.1) :
	PC_NUM_RING(numRing), 		// 20 in the original paper (IROS 18)
	PC_NUM_SECTOR(numSector), 	// 60 in the original paper (IROS 18)
	NUM_CANDIDATES_FROM_TREE(numCandidates), // 10 is enough. (refer the IROS 18 paper)
	SC_DIST_THRES(distThres), 	// empirically 0.1-0.2 is fine (rare false-alarms) for 20x60 polar context (but for 0.15 <,
								// DCS or ICP fit score check (e.g., in LeGO-LOAM) should be required for robustness).
								// 0.4-0.6 is good choice for using with robust kernel (e.g., Cauchy, DCS) + icp fitness threshold / if not, recommend 0.1-0.15
	LIDAR_HEIGHT(lidarHeight),  // lidar height : add this for simply directly using lidar scan in the lidar local coord (not robot base coord)
								// if you use robot-coord-transformed lidar scans, just set this as 0.
	PC_MAX_RADIUS(maxRadius),   // 80 meter max in the original paper (IROS 18)
	NUM_EXCLUDE_RECENT(numExcludeRecent), // simply just keyframe gap (related with loopClosureFrequency in yaml), but node position distance-based exclusion is ok. 
	TREE_MAKING_PERIOD_(treeMakingPeriod), 	// i.e., remaking tree frequency, to avoid non-mandatory every remaking, to save time cost / in the LeGO-LOAM integration,
											// it is synchronized with the loop detection callback (which is 1Hz) so it means the tree is updated evrey 10 sec.
											// But you can use the smaller value because it is enough fast ~ 5-50ms wrt N.
	SEARCH_RATIO(searchRatio) // for fast comparison, no Brute-force, but search 10 % is okay. // not was in the original conf paper, but improved ver.		
	{
		double PC_UNIT_SECTORANGLE = 360.0 / double(PC_NUM_SECTOR);
		double PC_UNIT_RINGGAP = PC_MAX_RADIUS / double(PC_NUM_RING);
		int tree_making_period_conter = 0;

		// reserving data space (of std::vector) could be considered. but the descriptor is lightweight so don't care.
		// polarcontexts_timestamp_.clear(); // optional.
		polarcontexts_.clear();
		// polarcontext_invkeys_.clear();
		// polarcontext_vkeys_.clear();
		polarcontext_indexs_.clear();
		polarcontext_invkeys_mat_.clear();
		polarcontext_invkeys_to_search_.clear();
	}

	~scan_context_descriptor()
	{

	}

	/*** scan context param-independent helper functions ***/
	float xy2theta(float & _x, float & _y)
	{
		// first quadrant
		if((_x >= 0) & (_y >= 0))
		{
			return (180/M_PI) * atan(_y / _x);
		}
		// second quadrant
		if((_x < 0) & (_y >= 0))
		{
			return 180 - ((180/M_PI) * atan(_y / (-_x)));
		}
		// third quadrant
		if((_x < 0) & (_y < 0))
		{
			return 180 + ((180/M_PI) * atan(_y / _x));
		}
		// forth quadrant
		if((_x >= 0) & (_y < 0))
		{
			return 360 - ((180/M_PI) * atan((-_y) / _x));
		}
	}

	Eigen::MatrixXd circshift(const Eigen::MatrixXd &_mat, int _num_shift)
	{
		// shift columns to right direction 
		assert(_num_shift >= 0);

		if(_num_shift == 0)
		{
			Eigen::MatrixXd shifted_mat( _mat );
			return shifted_mat; // Early return 
		}

		Eigen::MatrixXd shifted_mat = Eigen::MatrixXd::Zero(_mat.rows(), _mat.cols());
		for(int col_idx = 0; col_idx < _mat.cols(); col_idx++)
		{
			int new_location = (col_idx + _num_shift) % _mat.cols();
			shifted_mat.col(new_location) = _mat.col(col_idx);
		}

		return shifted_mat;
	}

	std::vector<float> eig2stdvec(Eigen::MatrixXd _eigmat)
	{
		std::vector<float> vec(_eigmat.data(), _eigmat.data() + _eigmat.size());
		return vec;
	}

	/*** scan context functions ***/
    Eigen::MatrixXd makeScancontext(const pcl::PointCloud<pcl::PointXYZI>& _scan_down, std::vector<float>* vT)
	{
		// TicToc t_making_desc;

		int num_pts_scan_down = _scan_down.points.size();

		// main
		const int NO_POINT = -1000;
		Eigen::MatrixXd desc = NO_POINT * Eigen::MatrixXd::Ones(PC_NUM_RING, PC_NUM_SECTOR);

		pcl::PointXYZI pt;
		float azim_angle, azim_range; // wihtin 2d plane
		int ring_idx, sctor_idx;
		// #pragma omp parallel for num_threads(parallelCores)
		for(int pt_idx = 0; pt_idx < num_pts_scan_down; pt_idx++)
		{
			pt.x = _scan_down.points[pt_idx].x; 
			pt.y = _scan_down.points[pt_idx].y;
			pt.z = _scan_down.points[pt_idx].z + LIDAR_HEIGHT; // naive adding is ok (all points should be > 0).

			// xyz to ring, sector
			azim_range = sqrt(pt.x * pt.x + pt.y * pt.y);
			azim_angle = xy2theta(pt.x, pt.y);

			// if range is out of roi, pass
			if(azim_range > PC_MAX_RADIUS)
			{
				continue;
			}

			ring_idx = std::max(std::min(PC_NUM_RING, int(ceil((azim_range / PC_MAX_RADIUS) * PC_NUM_RING))), 1);
			sctor_idx = std::max(std::min(PC_NUM_SECTOR, int(ceil((azim_angle / 360.0) * PC_NUM_SECTOR))), 1);

			// taking maximum z 
			if(desc(ring_idx-1, sctor_idx-1) < pt.z) // -1 means cpp starts from 0
			{
				desc(ring_idx-1, sctor_idx-1) = pt.z; // update for taking maximum value at that bin
			}
		}

		// reset no points to zero (for cosine dist later)
		// #pragma omp parallel for num_threads(parallelCores)
		for(int row_idx = 0; row_idx < desc.rows(); row_idx++)
		{
			for(int col_idx = 0; col_idx < desc.cols(); col_idx++)
			{
				if(desc(row_idx, col_idx) == NO_POINT)
				{
					desc(row_idx, col_idx) = 0;
				}
				vT->push_back(desc(row_idx, col_idx));
			}
		}

		// t_making_desc.toc("PolarContext making");

		return desc;
	}

    Eigen::MatrixXf makeRingkeyFromScancontext(const Eigen::MatrixXd& _desc)
	{
		// summary: rowwise mean vector
		Eigen::MatrixXf invariant_key(_desc.rows(), 1);
		// #pragma omp parallel for num_threads(parallelCores)
		for(int row_idx = 0; row_idx < _desc.rows(); row_idx++)
		{
			Eigen::MatrixXd curr_row = _desc.row(row_idx);
			invariant_key(row_idx, 0) = curr_row.mean();
		}

		return invariant_key;
	}

    Eigen::MatrixXd makeSectorkeyFromScancontext(const Eigen::MatrixXd& _desc)
	{
		// summary: columnwise mean vector
		Eigen::MatrixXd variant_key(1, _desc.cols());
		// #pragma omp parallel for num_threads(parallelCores)
		for(int col_idx = 0; col_idx < _desc.cols(); col_idx++)
		{
			Eigen::MatrixXd curr_col = _desc.col(col_idx);
			variant_key(0, col_idx) = curr_col.mean();
		}

		return variant_key;
	}

    int fastAlignUsingVkey(const Eigen::MatrixXd& _vkey1, const Eigen::MatrixXd& _vkey2)
	{
		int argmin_vkey_shift = 0;
		double min_veky_diff_norm = 10000000;
		// #pragma omp parallel for num_threads(parallelCores)
		for(int shift_idx = 0; shift_idx < _vkey1.cols(); shift_idx++)
		{
			Eigen::MatrixXd vkey2_shifted = circshift(_vkey2, shift_idx);

			Eigen::MatrixXd vkey_diff = _vkey1 - vkey2_shifted;

			double cur_diff_norm = vkey_diff.norm();
			if(cur_diff_norm < min_veky_diff_norm)
			{
				argmin_vkey_shift = shift_idx;
				min_veky_diff_norm = cur_diff_norm;
			}
		}

		return argmin_vkey_shift;
	}

    double distDirectSC(const Eigen::MatrixXd&_sc1, const Eigen::MatrixXd&_sc2 ) // "d" (eq 5) in the original paper (IROS 18)
	{
		int num_eff_cols = 0; // i.e., to exclude all-nonzero sector
		double sum_sector_similarity = 0;
		// #pragma omp parallel for num_threads(parallelCores)
		for(int col_idx = 0; col_idx < _sc1.cols(); col_idx++)
		{
			Eigen::VectorXd col_sc1 = _sc1.col(col_idx);
			Eigen::VectorXd col_sc2 = _sc2.col(col_idx);
			
			if((col_sc1.norm() == 0) | (col_sc2.norm() == 0))
			{
				continue; // don't count this sector pair.
			}

			double sector_similarity = col_sc1.dot(col_sc2) / (col_sc1.norm() * col_sc2.norm());

			sum_sector_similarity = sum_sector_similarity + sector_similarity;
			num_eff_cols = num_eff_cols + 1;
		}
		
		double sc_sim = sum_sector_similarity / num_eff_cols;
		return 1.0 - sc_sim;
	}

    std::pair<double, int> distanceBtnScanContext(const Eigen::MatrixXd&_sc1, const Eigen::MatrixXd&_sc2) // "D" (eq 6) in the original paper (IROS 18)
	{
		// 1. fast align using variant key (not in original IROS18)
		Eigen::MatrixXd vkey_sc1 = makeSectorkeyFromScancontext(_sc1);
		Eigen::MatrixXd vkey_sc2 = makeSectorkeyFromScancontext(_sc2);
		int argmin_vkey_shift = fastAlignUsingVkey(vkey_sc1, vkey_sc2);

		int SEARCH_RADIUS = round(0.5 * SEARCH_RATIO * _sc1.cols()); // a half of search range 
		std::vector<int> shift_idx_search_space { argmin_vkey_shift };
		for(int ii = 1; ii < SEARCH_RADIUS + 1; ii++)
		{
			shift_idx_search_space.push_back((argmin_vkey_shift + ii + _sc1.cols()) % _sc1.cols());
			shift_idx_search_space.push_back((argmin_vkey_shift - ii + _sc1.cols()) % _sc1.cols());
		}
		std::sort(shift_idx_search_space.begin(), shift_idx_search_space.end());

		// 2. fast columnwise diff 
		int argmin_shift = 0;
		double min_sc_dist = 10000000;
		for(int num_shift: shift_idx_search_space)
		{
			Eigen::MatrixXd sc2_shifted = circshift(_sc2, num_shift);
			double cur_sc_dist = distDirectSC(_sc1, sc2_shifted);
			if(cur_sc_dist < min_sc_dist)
			{
				argmin_shift = num_shift;
				min_sc_dist = cur_sc_dist;
			}
		}

		return std::make_pair(min_sc_dist, argmin_shift);
	}

    // User-side API
	void saveDescriptorAndKey(const float* descriptorMat, const int8_t robot, const int index)
	{
		// decode scan context
		Eigen::MatrixXd sc = Eigen::MatrixXd::Ones(PC_NUM_RING, PC_NUM_SECTOR);
		for(int row_idx = 0; row_idx < sc.rows(); row_idx++)
		{
			for(int col_idx = 0; col_idx < sc.cols(); col_idx++)
			{
				sc(row_idx, col_idx) = descriptorMat[row_idx*sc.cols()+col_idx];
			}
		}

		save(sc, robot, index);
	}

	void save(const Eigen::MatrixXd sc, const int8_t robot, const int index)
	{
		Eigen::MatrixXf ringkey = makeRingkeyFromScancontext(sc);
		Eigen::MatrixXd sectorkey = makeSectorkeyFromScancontext(sc);
		// std::vector<float> polarcontext_invkey_vec = eig2stdvec(ringkey);

		polarcontexts_.push_back(sc);  // desc
		// polarcontext_invkeys_.push_back(ringkey); // ring key
		// polarcontext_vkeys_.push_back(sectorkey); // sector key
		// polarcontext_invkeys_mat_.push_back(polarcontext_invkey_vec); // key
		polarcontextRowKey.conservativeResize(PC_NUM_RING, polarcontexts_.size());
		polarcontextRowKey.block(0, polarcontexts_.size()-1, PC_NUM_RING, 1) = ringkey.block(0, 0, PC_NUM_RING, 1);
		polarcontext_indexs_.push_back(std::make_pair(robot,index)); // indexs

		// ROS_DEBUG("\033[1;33m[Save SC] robot %c %d in descs %d.\033[0m", char(robot+'a'), index, polarcontext_invkeys_mat_.size());
	}

    std::vector<float> makeAndSaveDescriptorAndKey(const pcl::PointCloud<pcl::PointXYZI>& scan, const int8_t robot, const int index)
	{
		std::vector<float> vT;
		Eigen::MatrixXd sc = makeScancontext(scan, &vT); // v1 size:(PC_NUM_RING, PC_NUM_SECTOR)
		save(sc, robot, index);

		return vT;
	}

	std::pair<int, float> detectIntraLoopClosureID(const int curPtr)
	{
		std::pair<int, float> result {-1, 0.0};
		Eigen::VectorXf curKey = polarcontextRowKey.col(curPtr);; // current query row key
		auto curFeature = polarcontexts_[curPtr]; // current feature

		// step 1: candidates from rowkey tree_
		if(curPtr < NUM_EXCLUDE_RECENT + NUM_CANDIDATES_FROM_TREE + 1)
		{
			return result; // Early return 
		}

		// tree reconstruction
		Eigen::MatrixXf newIrisFeatureRowKey;
		int historyIndex = curPtr - NUM_EXCLUDE_RECENT;
		newIrisFeatureRowKey.conservativeResize(PC_NUM_RING, historyIndex);
		newIrisFeatureRowKey.block(0, 0, PC_NUM_RING, historyIndex) = polarcontextRowKey.block(0, 0, PC_NUM_RING, historyIndex);
		// kdTree = Nabo::NNSearchF::createKDTreeTreeHeap(newIrisFeatureRowKey, PC_NUM_RING);
		kdTree = Nabo::NNSearchF::createKDTreeLinearHeap(newIrisFeatureRowKey, PC_NUM_RING);
		

		// search n nearest neighbors
        Eigen::VectorXi indice(NUM_CANDIDATES_FROM_TREE);
        Eigen::VectorXf distance(NUM_CANDIDATES_FROM_TREE);
		float minDis = 10000000.0;
		int minIndex = -1;
		int minBias = 0;

		// knn search
		kdTree->knn(curKey, indice, distance, NUM_CANDIDATES_FROM_TREE);

		// step 2: pairwise distance
		for(int i = 0; i < std::min(NUM_CANDIDATES_FROM_TREE, int(indice.size())); i++)
		{
			auto polarcontext_candidate = polarcontexts_[indice[i]];
			std::pair<double, int> sc_dist_result = distanceBtnScanContext(curFeature, polarcontext_candidate); 

			double candidateDis = sc_dist_result.first;
			int candidate_align = sc_dist_result.second;

			if(candidateDis < minDis)
			{
				minDis = candidateDis;
				minIndex = indice[i];
				minBias = candidate_align;
			}
		}

		// loop threshold check
		if(minDis < SC_DIST_THRES)
		{
			result.first = minIndex;
			result.second = minBias;
			ROS_INFO("\033[1;33m[SC Loop] btn %d and %d. Dis: %.2f.\033[0m", curPtr, minIndex, minDis);
		}
		else
		{
			// ROS_DEBUG("\033[1;33m[SC Not loop] btn %d and %d. Dis: %.2f.\033[0m", curPtr, minIndex, minDis);
			ROS_INFO("\033[1;33m[SC Not loop] btn %d and %d. Dis: %.2f.\033[0m", curPtr, minIndex, minDis);
		}
		return result;
	}

    std::pair<int, float> detectInterLoopClosureID(const int currentPtr) // int: nearest node index, float: relative yaw  
	{
		int loop_id { -1 }; // init with -1, -1 means no loop (== LeGO-LOAM's variable "closestHistoryFrameID")

		auto curr_key = polarcontext_invkeys_mat_[currentPtr]; // current observation (query)
		auto curr_desc = polarcontexts_[currentPtr]; // current observation (query)

		// step 1: candidates from ringkey tree_
		if((int)polarcontext_invkeys_mat_.size() < NUM_EXCLUDE_RECENT + 1)
		{
			std::pair<int,float> result {loop_id, 0.0};
			return result; // Early return 
		}

		// tree_ reconstruction (not mandatory to make everytime)
		if(tree_making_period_conter % TREE_MAKING_PERIOD_ == 0) // to save computation cost
		{
			// TicToc t_tree_construction;

			polarcontext_invkeys_to_search_.clear();
			polarcontext_invkeys_to_search_.assign(polarcontext_invkeys_mat_.begin(), polarcontext_invkeys_mat_.end() - NUM_EXCLUDE_RECENT ) ;

			polarcontext_tree_.reset(); 
			polarcontext_tree_ = std::make_unique<KDTreeVectorOfVectorsAdaptor<std::vector<std::vector<float>>,float>>(PC_NUM_RING /* dim */, polarcontext_invkeys_to_search_, 10 /* max leaf */);
			// tree_ptr_->index->buildIndex(); // inernally called in the constructor of KDTreeVectorOfVectorsAdaptor<std::vector<std::vector<float>>,float> (for detail, refer the nanoflann and KDtreeVectorOfVectorsAdaptor)
			// t_tree_construction.toc("Tree construction");
		}
		tree_making_period_conter = tree_making_period_conter + 1;

		double min_dist = 10000000; // init with somthing large
		int nn_align = 0;
		int nn_idx = -1;

		// knn search
		std::vector<size_t> candidate_indexes( NUM_CANDIDATES_FROM_TREE ); 
		std::vector<float> out_dists_sqr( NUM_CANDIDATES_FROM_TREE );

		// TicToc t_tree_search;
		nanoflann::KNNResultSet<float> knnsearch_result( NUM_CANDIDATES_FROM_TREE );
		knnsearch_result.init( &candidate_indexes[0], &out_dists_sqr[0] );
		polarcontext_tree_->index->findNeighbors(knnsearch_result, &curr_key[0] /* query */, nanoflann::SearchParams(10)); 
		// t_tree_search.toc("Tree search");

		// step 2: pairwise distance (find optimal columnwise best-fit using cosine distance)
		// TicToc t_calc_dist;   
		for(int candidate_iter_idx = 0; candidate_iter_idx < NUM_CANDIDATES_FROM_TREE; candidate_iter_idx++)
		{
			auto polarcontext_candidate = polarcontexts_[candidate_indexes[candidate_iter_idx]];
			std::pair<double, int> sc_dist_result = distanceBtnScanContext(curr_desc, polarcontext_candidate); 

			double candidate_dist = sc_dist_result.first;
			int candidate_align = sc_dist_result.second;

			if(candidate_dist < min_dist)
			{
				if(candidate_indexes[candidate_iter_idx] == currentPtr) continue;
				min_dist = candidate_dist;
				nn_align = candidate_align;

				nn_idx = candidate_indexes[candidate_iter_idx];
			}
		}
		// t_calc_dist.toc("Distance calc");

		// loop threshold check
		if(min_dist < SC_DIST_THRES)
		{
			loop_id = nn_idx; 
			ROS_DEBUG("\033[1;33m[SC Loop found] btn %d and %d. Dis: %.2f, Yaw diff: %.2f deg.\033[0m", min_dist, currentPtr, nn_idx, nn_align * PC_UNIT_SECTORANGLE);
		}
		else
		{
			ROS_DEBUG("\033[1;33m[SC Not loop] btn %d and %d. Dis: %.2f, Yaw diff: %.2f deg.\033[0m", min_dist, currentPtr, nn_idx, nn_align * PC_UNIT_SECTORANGLE);
		}

		// To do: return also nn_align (i.e., yaw diff)
		float yaw_diff_rad = nn_align * PC_UNIT_SECTORANGLE * M_PI / 180.0;
		std::pair<int, float> result {loop_id, yaw_diff_rad};

		return result;
	}

	std::pair<int8_t, int> getIndex(const int key)
	{
		return polarcontext_indexs_[key];
	}

	int getSize(const int idIn = -1)
	{
		return polarcontext_indexs_.size();
	}

public:
    double LIDAR_HEIGHT;

    int PC_NUM_RING;
    int PC_NUM_SECTOR;
    double PC_MAX_RADIUS;
    double PC_UNIT_SECTORANGLE;
    double PC_UNIT_RINGGAP;

    // tree
    int NUM_EXCLUDE_RECENT;
    int NUM_CANDIDATES_FROM_TREE;

    // loop thres
    double SEARCH_RATIO;
    double SC_DIST_THRES; 

    // config 
    int TREE_MAKING_PERIOD_; 
    int tree_making_period_conter;

    // data 
    // std::vector<double> polarcontexts_timestamp_; // optional.
    std::vector<Eigen::MatrixXd> polarcontexts_;
    // std::vector<Eigen::MatrixXd> polarcontext_invkeys_;
    // std::vector<Eigen::MatrixXd> polarcontext_vkeys_;
	std::vector<std::pair<int8_t,int>> polarcontext_indexs_;

    std::vector<std::vector<float>> polarcontext_invkeys_mat_;
	Eigen::MatrixXf polarcontextRowKey;
    std::vector<std::vector<float>> polarcontext_invkeys_to_search_;
    std::unique_ptr<KDTreeVectorOfVectorsAdaptor<std::vector<std::vector<float>>,float>> polarcontext_tree_;
	Nabo::NNSearchF* kdTree = NULL;
};

class m2dp_descriptor : public scan_descriptor
{
public:
	m2dp_descriptor()
	{
		azimuthList = *new vector<double>(numP);
		for(int i = 0; i < numP; i++)
		{
			azimuthList[i] = -M_PI_2 + i * M_PI / (numP - 1);
		}

		elevationList = *new vector<double>(numQ);
		for(int i = 0; i < numQ; i++)
		{
			elevationList[i] = i * M_PI_2 / (numQ - 1);
		}
	}

	std::vector<float> makeAndSaveDescriptorAndKey(const pcl::PointCloud<pcl::PointXYZI>& scan, const int8_t robot, const int index)
	{
		cloudFiltered.reset(new pcl::PointCloud<pcl::PointXYZI>());
		pcl::PCA<pcl::PointXYZI> pca;
		pca.project(scan, *cloudFiltered);

		maxRho = 0;
		cloudPca.resize(cloudFiltered->points.size(), 3);
		for(int i = 0; i < cloudFiltered->points.size(); i++)
		{
			cloudPca(i, 0) = cloudFiltered->points[i].x;
			cloudPca(i, 1) = cloudFiltered->points[i].y;
			cloudPca(i, 2) = -cloudFiltered->points[i].z;

			// get the farthest point distance
			double temp_rho = sqrt(
				  cloudFiltered->points[i].x * cloudFiltered->points[i].x
				+ cloudFiltered->points[i].x * cloudFiltered->points[i].x
				+ cloudFiltered->points[i].z * cloudFiltered->points[i].z);

			if(temp_rho > maxRho)
			{
				maxRho = temp_rho;
			}
		}

		// get the signature matrix A
		A = GetSignatureMatrix();

		Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
		Eigen::MatrixXd u = svd.matrixU();
		Eigen::MatrixXd v = svd.matrixV();
		Eigen::Matrix<double, 1, 64> u_temp;
		Eigen::Matrix<double, 1, 128> v_temp;
		u_temp = u.col(0);
		v_temp = v.col(0);
		m2dp << u_temp, v_temp;

		std::vector<float> m2dpVec(&m2dp[0], m2dp.data()+m2dp.cols()*m2dp.rows());
		save(m2dpVec, robot, index);

		return m2dpVec;
	}

	Eigen::Matrix<double, 64, 128> GetSignatureMatrix()
	{
		vector<double> thetaList(numT + 1);
		for(int i = 0; i <= numT; i++)
		{
			thetaList[i] = -M_PI + i * 2 * M_PI / (numT);
		}

		vector<double> rhoList(numR + 1);
		for(int i = 0; i <= numR; i++)
		{
			rhoList[i] = i * sqrt(maxRho) / numR;
			rhoList[i] = rhoList[i] * rhoList[i];
		}
		// make sure all points in bins
    	rhoList[rhoList.size() - 1] = rhoList[rhoList.size() - 1] + 0.001;

		Eigen::Matrix<double, 64, 128> result_A;
		int index_A = 0;
		// loop on azimuth
		for(int i = 0; i < azimuthList.size();i++)
		{
			auto azm = azimuthList[i];
			// loop on elevation
			for(int j = 0; j < elevationList.size();j++)
			{
				auto elv = elevationList[j];
				// normal vector vecN of the selected 2D plane
				Eigen::Matrix<double, 1, 3> vecN;
				sph2cart(azm, elv, 1, vecN);
				// distance of vector [1,0,0] to the surface with normal vector vecN
				Eigen::Matrix<double, 1, 3> op(1, 0, 0);
				Eigen::MatrixXd h = op * vecN.transpose();
				// a new vector, c = h*vecN, so that vector [1,0,0]-c is the projection of x-axis onto the plane with normal vector vecN
				Eigen::Matrix<double, 1, 3> c = h(0) * vecN;
				// x-axis - c, the projection
				Eigen::Matrix<double, 1, 3> px = op - c;
				// given the normal vector vecN and the projected x-axis px, the y- axis is cross(vecN,px)
				Eigen::Matrix<double, 1, 3> py = vecN.cross(px);
				// projection of data onto space span{px,py}
				Eigen::MatrixXd pcx = cloudPca * px.transpose();
				Eigen::MatrixXd pcy = cloudPca * py.transpose();
				// pdata = np.array([pcx,pcy])
				// represent data in polar coordinates
				// vector<double> rho;
				// vector<double> theta;
				std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> points; // x: rho  y: theta
				for(int i = 0; i < pcx.rows(); i++)
				{
					Eigen::Vector2d temp;
					cart2pol(pcx(i), pcy(i), temp);
					points.push_back(temp);
				}
				// main function, count points in bins
				Eigen::MatrixXd hist; //16*8    thetaList 17   rhoList 9
				histogram2d(points, thetaList, rhoList, hist);
				hist = hist / cloudFiltered->points.size();
				int hist_size = hist.cols() * hist.rows();
				for (int i = 0; i < hist_size; i++)
				{
					result_A(index_A, i) = hist(i);
				}
				index_A++;
			}
		}
		return result_A;
	}

	void sph2cart(double azm, double elv, double r, Eigen::Matrix<double, 1, 3> &vecN)
	{
		double x, y, z;
		x = r * cos(elv) * cos(azm);
		y = r * cos(elv) * sin(azm);
		z = r * sin(elv);
		vecN << x, y, z;
	}

	void cart2pol(double x, double y, Eigen::Vector2d &vecN)
	{
		vecN.x() = sqrt(x * x + y * y); // rho
		vecN.y() = atan2(y, x);         // phi
	}

	void histogram2d(std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> points, vector<double> thetaList, vector<double> rhoList, Eigen::MatrixXd &hist)
	{
		int row, col;
		row = thetaList.size() - 1;
		col = rhoList.size() - 1;
		hist = Eigen::MatrixXd::Zero(row, col);
		// Points x: rho  y: theta
		for(auto pt : points)
		{
			int row_index = -1, col_index = -1;
			for(int i = 0; i <= row; i++)
			{
				if(pt.y() < thetaList[i])
				{
					row_index = i - 1;
					break;
				}
			}
			for(int j = 0; j <= col; j++)
			{
				if(pt.x() < rhoList[j])
				{
					col_index = j - 1;
					break;
				}
			}
			if(row_index >= 0 && row_index < row && col_index >= 0 && col_index < col)
			{
				hist(row_index, col_index)++;
			}
		}
	}

	void save(const std::vector<float> m2dpVec, const int8_t robot, const int index)
	{
		Eigen::VectorXf m2dpMat = Eigen::Map<const Eigen::Matrix<float, -1, -1, Eigen::RowMajor>>(m2dpVec.data(), 128, 1);
		m2dpDict.push_back(m2dpMat);
		m2dpVecDict.push_back(m2dpVec);
		m2dpIndexs.push_back(std::make_pair(robot,index));
	}
	
	void saveDescriptorAndKey(const float* descriptorMat, const int8_t robot, const int index)
	{
		// decode fpfh
		std::vector<float> m2dpVec;
		m2dpVec.insert(m2dpVec.end(), descriptorMat, descriptorMat+128);

		save(m2dpVec, robot, index);
	}

	std::pair<int, float> detectIntraLoopClosureID(const int curPtr)
	{

	}
	
	std::pair<int, float> detectInterLoopClosureID(const int currentPtr)
	{

	}
	
	std::pair<int8_t, int> getIndex(const int key)
	{
		return m2dpIndexs[key];
	}
	
	int getSize(const int idIn = -1)
	{
		return m2dpIndexs.size();
	}
	
private:
    int numT = 16; // number of bins in theta, the 't' in paper
    int numR = 8; // number of bins in rho, the 'l' in paper
    int numP = 4; // number of azimuth angles, the 'p' in paper
    int numQ = 16; // number of elevation angles, the 'q' in paper

	Eigen::MatrixXd cloudPca;

	vector<double> azimuthList;
    vector<double> elevationList;
    
	double maxRho;

    Eigen::Matrix<double, 64, 128> A;
    Eigen::Matrix<double,1,192> m2dp;

	pcl::PointCloud<pcl::PointXYZI>::Ptr cloudFiltered;


    std::vector<Eigen::VectorXf> m2dpDict;
    std::vector<std::vector<float>> m2dpVecDict;
	std::vector<std::vector<float>> m2dpVecSearch;
	std::vector<std::pair<int8_t,int>> m2dpIndexs;
/*
	M2DP m2dp(raw_cloud);
    Eigen::Matrix<double, 1, 192> desM2dp;
    desM2dp = m2dp.get_m2dp_result();
    Eigen::MatrixXd A_m2dp;
    A_m2dp = m2dp.get_m2dp_A();
    cout << desM2dp << endl;
*/
};

class M2DP {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    M2DP(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    ~M2DP() = default;
    Eigen::Matrix<double, 1, 192> get_m2dp_result()
    {
        return m2dp_result;
    }

    Eigen::Matrix<double, 64, 128> get_m2dp_A()
    {
        return A;
    }

private :
    // key parameter
    // number of bins in theta, the 't' in paper
    int numT = 16;
    // number of bins in rho, the 'l' in paper
    int numR = 8;
    // number of azimuth angles, the 'p' in paper
    int numP = 4;
    // number of elevation angles, the 'q' in paper
    int numQ = 16;
    // input pointcloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud{new pcl::PointCloud<pcl::PointXYZ>()};
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered{new pcl::PointCloud<pcl::PointXYZ>()};
    Eigen::MatrixXd cloud_pca;
    // output m2dp result
    Eigen::Matrix<double,1,192> m2dp_result;

    vector<double> azimuthList;
    vector<double> elevationList;
    double maxRho=0;
    Eigen::Matrix<double, 64, 128> A;

    // main function, get the signature matrix A
    Eigen::Matrix<double, 64, 128> GetSignatureMatrix();

    void cart2sph(double &azm, double &elv, double &r, Eigen::Vector3d vecN);
    void sph2cart(double azm, double elv, double r, Eigen::Matrix<double, 1, 3> &vecN);
    void cart2pol(double x, double y, Eigen::Vector2d &vecN);
    void pol2cart(double rho, double phi, Eigen::Vector2d &vecN);

    void histogram2d(std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> points, vector<double> thetaList, vector<double> rhoList, Eigen::MatrixXd &hist);
};

#endif
