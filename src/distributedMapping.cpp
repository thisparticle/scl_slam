#include "distributedMapping.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "homr");

	distributed_mapping dm;

    ROS_INFO("\033[1;32m----> Distributed loop closure[%s] started.\033[0m", dm.name.c_str());

	std::thread loopClosureThread_(&distributed_mapping::loopClosureThread, &dm);
	std::thread publishTfThread_(&distributed_mapping::publishTfThread, &dm);
	
	ros::spin();

	loopClosureThread_.join();
	publishTfThread_.join();

    return 0;
}