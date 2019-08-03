#include "ros/ros.h"
#include "ros/package.h"
#include "cartographer_ros_msgs/TrajectoryQuery.h"
#include "geometry_msgs/Point.h"
#include "lcd_pkg/PoseGraph.h"
#include "std_msgs/UInt32.h"
#include "math.h"
#include <iostream>
#include <fstream>
#include <sys/stat.h> 

using namespace std;

ros::Publisher poseGraphPub;
int arraySize_param = -1;
double mseThreshold_param = -1;
double publish_rate = -1;
ofstream lout;

void trajCb(const vector<geometry_msgs::PoseStamped>&);
float mse(vector<geometry_msgs::Point>&, vector<geometry_msgs::Point>&);
int sc(vector<geometry_msgs::Point>&, vector<geometry_msgs::Point>&);
int zc(vector<geometry_msgs::Point>&);
void initializeLogs(string, string);


int main(int argc, char **argv)
{
  ros::init(argc, argv, "lcd_node");

  ros::NodeHandle nh;

	poseGraphPub = nh.advertise<lcd_pkg::PoseGraph>("pose_graph", 100);
  ros::ServiceClient client = nh.serviceClient<cartographer_ros_msgs::TrajectoryQuery>("trajectory_query");
  
  while(arraySize_param == -1 || mseThreshold_param == -1 || publish_rate == -1)
  {
  ros::param::get("lcd_node/arraySize_param", arraySize_param);
  ros::param::get("lcd_node/mseThreshold_param", mseThreshold_param);
  ros::param::get("lcd_node/publish_rate", publish_rate);
  
  ROS_INFO("Waiting for Parameters");
  }
  
  initializeLogs("lcd_pkg", "lcd_node_logs.txt");
  lout << "arraySize_param = " << arraySize_param << endl;
  lout << "mseThreshold_param = " << mseThreshold_param << endl;
  lout << "publish_rate = " << publish_rate << endl << endl;
  
  cartographer_ros_msgs::TrajectoryQuery srv;
  srv.request.trajectory_id = 0;

	ros::Rate loop_rate(publish_rate);
	
	while(ros::ok())
	{
		if (client.call(srv))
		{
			cout << "Trajectory status = " << srv.response.status.message << endl; 
		  cout << "Trajectory size = " << srv.response.trajectory.size() << endl;
		  
		  if(srv.response.trajectory.size() > 0)
		  cout << "Current Odometry: position (" << srv.response.trajectory.back().pose.position.x << ", "
		  																			 << srv.response.trajectory.back().pose.position.y << ", "
		  																			 << srv.response.trajectory.back().pose.position.z << ") "
		  									 << " orientation (" << srv.response.trajectory.back().pose.orientation.x << ", "
		  									  									 << srv.response.trajectory.back().pose.orientation.y << ", "
		  									  									 << srv.response.trajectory.back().pose.orientation.z << ", "
		  									  									 << srv.response.trajectory.back().pose.orientation.w << ")" << endl;
 		  trajCb(srv.response.trajectory);
		  cout << "-----------------------------------" <<endl;
		}
		else
		{
		}
		//ROS_ERROR("Failed to call service 'trajectory_query'");
		
		ros::spinOnce();
		
		loop_rate.sleep();
  }

  return 0;
}

void trajCb(const vector<geometry_msgs::PoseStamped>& msg)
{	
	static uint32_t nLoopClosures = 0;
	static bool isPopulated_prevTruncatedArray = false;
	
	const int truncatedArraySize = arraySize_param; // Number of array elements to compare
	const float mseThreshold = mseThreshold_param; // Mean square error threshold between two arrays to declare dissimilarilty
	
	static uint32_t prevInputArraySize = 0; // Previous input array size
	
	static vector<geometry_msgs::Point> prevTruncatedArray(truncatedArraySize); // truncatedArraySize number of elements of the previous input array
	
	uint32_t newInputArraySize = msg.size(); // New input array size
	
	int nNewPoses = newInputArraySize - prevInputArraySize; // Difference in arrays lengths
	
	vector<geometry_msgs::Point> newTruncatedArray(truncatedArraySize); // truncatedArraySize number of elements of the new input array
	
	lout << "prevInputArraySize: " << prevInputArraySize << ", newInputArraySize: " << newInputArraySize << ", truncatedArraySize: " << truncatedArraySize << endl;
	
	if(newInputArraySize < (truncatedArraySize+nNewPoses) || newInputArraySize <= prevInputArraySize)
	{
	prevInputArraySize = newInputArraySize;
	//ROS_INFO("Waiting to accumulate enough points");
	return;
	}
	
	int count = 0;
	
	if(prevInputArraySize > 0 && isPopulated_prevTruncatedArray)
	{
		count = 0;
		for (int i = newInputArraySize-truncatedArraySize-nNewPoses; i < newInputArraySize-nNewPoses; i++)
		{
			newTruncatedArray[count].x = msg[i].pose.position.x;
			newTruncatedArray[count].y = msg[i].pose.position.y;
			newTruncatedArray[count].z = msg[i].pose.position.z;
			count ++;
		}
		
		float meanSquareError = mse(prevTruncatedArray, newTruncatedArray);
		int zeroCount = zc(prevTruncatedArray);
		int similarityCount = sc(prevTruncatedArray, newTruncatedArray);	

		lout << "---------------------------------------" << endl;
		lout << "Mean Square Error = " << meanSquareError;
		lout << " - Zero Count = " << zeroCount;
		lout << " - Similarity Count = " << similarityCount << endl;
		lout << "---------------------------------------" << endl;
		
		cout << "---------------------------------------" << endl;
		cout << "Mean Square Error = " << meanSquareError;
		cout << " - Zero Count = " << zeroCount;
		cout << " - Similarity Count = " << similarityCount << endl;
		cout << "---------------------------------------" << endl;
		
		if(meanSquareError > mseThreshold)
		{
			lout << "Loop Closure Detected" << endl;
			cout << "Loop Closure Detected" << endl;
			nLoopClosures = nLoopClosures + 1;
			//getchar();
		}
	}
	
	lout << "Updating static variables for the next callback" << endl;
	count = 0;
	for (int i = newInputArraySize-truncatedArraySize; i < newInputArraySize; i++)
	{
		
		prevTruncatedArray[count].x = msg[i].pose.position.x;
		prevTruncatedArray[count].y = msg[i].pose.position.y;
		prevTruncatedArray[count].z = msg[i].pose.position.z;
		
		count ++;
	}
	isPopulated_prevTruncatedArray = true;
	lout << "prevTruncatedArray populated" << endl;
	prevInputArraySize = newInputArraySize;
	lout << "prevInputArraySize reset to newInputArraySize" << endl;
	
	lout << "Publishing number of loop closures and the recent pose array" << endl;
	lcd_pkg::PoseGraph poseGraph;
	poseGraph.nLoopClosures = nLoopClosures;
	poseGraph.poseArray = msg;
	poseGraphPub.publish(poseGraph);
	lout << "<====================================>" << endl << endl;
	
}

float mse(vector<geometry_msgs::Point>& array1, vector<geometry_msgs::Point>& array2)
{
const int size = array1.size();

float se = 0;
	for (int i = 0; i < size; i++)
	{
	se += pow((array1[i].x - array2[i].x),2) + pow((array1[i].y - array2[i].y),2) + pow((array1[i].z - array2[i].z),2);
	lout << "(" << array1[i].x << ", " << array1[i].y << ", " << array1[i].z << ")" << " - " << "(" << array2[i].x << ", " << array2[i].y << ", " << array2[i].z << ")" << endl;
	//cout << "Square Error:" << se << endl;  
	}

return se/size;
}

int sc(vector<geometry_msgs::Point>& array1, vector<geometry_msgs::Point>& array2)
{
const int size = array1.size();

int similarityCount = 0;
	for(int i = 0; i < size; i++)
	{
		if(array1[i].x == array2[i].x && array1[i].y == array2[i].y && array1[i].z == array2[i].z)
		similarityCount++;
	}
return similarityCount;
}

int zc(vector<geometry_msgs::Point>& array)
{
const int size = array.size();

int zeroCount;
	for(int i = 0; i < size; i++)
	{
		if(array[i].x == 0 && array[i].y == 0 && array[i].z == 0)
		zeroCount ++;
	}
return zeroCount;
}

void initializeLogs(string packageName, string fileName)
{
  string logFilePath = "";
	logFilePath = ros::package::getPath(packageName);
	logFilePath.append("/logs/");
	
	mkdir(logFilePath.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
	
	logFilePath.append(fileName);
	
	cout << "Log File Path : " << logFilePath << endl;
	lout.open (logFilePath, ios::trunc);
	
	while(!lout.is_open())
	{
	lout.open (logFilePath, ios::trunc);
	cout << "Waiting for the log file to open" << endl;
	}
	
	cout << "Writing logs to file ..." << endl;
	cout << "Use 'tail -f " << logFilePath << "' to view" << endl;
	
	const time_t ctt = time(0);
  lout << asctime(localtime(&ctt)) << endl;
  
}


