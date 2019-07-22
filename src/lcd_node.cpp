#include "ros/ros.h"
#include "visualization_msgs/MarkerArray.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/Bool.h"
#include <iostream>

using namespace std;

class point
{
public:
float x;
float y;
float z;
};

void trajCb(const visualization_msgs::MarkerArray&);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");

  ros::NodeHandle n;

  ros::Publisher loopClosurePub = n.advertise<std_msgs::Bool>("isLoopClosed", 100); 
  ros::Subscriber markerArraySub = n.subscribe("trajectory_node_list", 100, trajCb);
       
  while (ros::ok())
  {
  ros::spinOnce();
  }

  return 0;
}

void trajCb(const visualization_msgs::MarkerArray& msg)
{
	const int validSize = 40; // Number of array elements to compare
	const int disSimilarityThreshold = 5; // Number of points that have to be different for loop closure detection
	
	static int arraySize = msg.markers[2].points.size(); // Input array size
	
	if(msg.markers[2].points.size() < validSize)
	{
	//cout << "Less than " << validSize << " points received" << endl;
	return;
	}
	
	if(arraySize == msg.markers[2].points.size())
	{
	//cout << "Same array with " << arraySize << " points received" << endl;
	return;
	}
	
	static geometry_msgs::Point traj[validSize]; // Last validSize elements of the input array
	
	arraySize = msg.markers[2].points.size();
	
	int similarityCount = 0;
	int zeroCount = 0;
	
	//cout << "Here" << endl;
	
	int count = 0;
	for (int i = arraySize-validSize-1; i < arraySize-1; i++)
	{	
	//cout << "Here a" <<endl;
	//cout << count << endl;
	
		//cout << traj[count].x << " " << msg.markers[2].points[i].x << endl;
		if(traj[count].x == msg.markers[2].points[i].x && traj[count].y == msg.markers[2].points[i].y && traj[count].y == msg.markers[2].points[i].y)
		{
		similarityCount++;
		}
		
		if(traj[count].x == 0 && traj[count].y == 0 && traj[count].z == 0)
		{
		zeroCount++;
		}
		
	//cout << "Here b" <<endl;
		
	traj[count] = msg.markers[2].points[i+1];
	count++;
	}
	
	//cout << "Here 1" <<endl;
	cout << "Similarity Count : " << similarityCount << endl;
	if(similarityCount < validSize - disSimilarityThreshold && zeroCount != validSize)
	{
	cout << "Loop Closure Detected" << endl;
	}
}




