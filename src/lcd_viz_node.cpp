#include "ros/ros.h"
#include "ros/package.h"
#include "visualization_msgs/MarkerArray.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/Bool.h"
#include "math.h"
#include <iostream>
#include <fstream>
#include <sys/stat.h> 

using namespace std;

class point
{
public:
float x;
float y;
float z;
};

int arraySize_param = -1;
double mseThreshold_param = -1;
ofstream lout;

void trajCb(const visualization_msgs::MarkerArray&);
float mse(vector<geometry_msgs::Point>&, vector<geometry_msgs::Point>&);
int sc(vector<geometry_msgs::Point>&, vector<geometry_msgs::Point>&);
int zc(vector<geometry_msgs::Point>&);
void initializeLogs(string, string);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "lcd_viz_node");

  ros::NodeHandle n;

  ros::Publisher loopClosurePub = n.advertise<std_msgs::Bool>("isLoopClosed", 100); 
  ros::Subscriber markerArraySub = n.subscribe("trajectory_node_list", 100, trajCb);
  
  while(arraySize_param == -1 || mseThreshold_param == -1)
  {
  ros::param::get("lcd_viz_node/arraySize_param", arraySize_param);
  ros::param::get("lcd_viz_node/mseThreshold_param", mseThreshold_param);
  
  ROS_INFO("Waiting for Parameters");
  }
  
  initializeLogs("lcd_pkg", "manuever_logs.txt");
  lout << "arraySize_param = " << arraySize_param << endl;
  lout << "mseThreshold_param = " << mseThreshold_param << endl << endl;
  
  ros::spin();

  return 0;
}

void trajCb(const visualization_msgs::MarkerArray& msg)
{

	const int truncatedArraySize = arraySize_param; // Number of array elements to compare
	const float mseThreshold = mseThreshold_param; // Mean square error threshold between two arrays to declare dissimilarilty
	
	static int prevInputArraySize = 0; // Previous input array size
	
	static vector<geometry_msgs::Point> prevTruncatedArray(truncatedArraySize); // truncatedArraySize number of elements of the previous input array
	
	int newInputArraySize = msg.markers[2].points.size(); // New input array size
	
	vector<geometry_msgs::Point> newTruncatedArray(truncatedArraySize); // truncatedArraySize number of elements of the new input array
	
	lout << "newInputArraySize: " << newInputArraySize << ", truncatedArraySize: " << truncatedArraySize << ", prevInputArraySize: " << prevInputArraySize << endl;
	
	if(newInputArraySize < truncatedArraySize || newInputArraySize <= prevInputArraySize)
	{
	prevInputArraySize = newInputArraySize;
	//ROS_INFO("Waiting to accumulate enough points");
	return;
	}
	prevInputArraySize = newInputArraySize;
	
	int count = 0;
	for (int i = newInputArraySize-truncatedArraySize-1; i < newInputArraySize-1; i++)
	{
	newTruncatedArray[count] = msg.markers[2].points[i];
	count ++;
	}
	
	float meanSquareError = mse(prevTruncatedArray, newTruncatedArray);
	int zeroCount = zc(prevTruncatedArray);
	int similarityCount = sc(prevTruncatedArray, newTruncatedArray);
	
	count = 0;
	for (int i = newInputArraySize-truncatedArraySize; i < newInputArraySize; i++)
	{
	prevTruncatedArray[count] = msg.markers[2].points[i];
	count ++;
	}
	
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
	
	if(meanSquareError > mseThreshold && zeroCount < truncatedArraySize)
	{
	lout << "Loop Closure Detected" << endl;
	cout << "Loop Closure Detected" << endl;
	//getchar();
	}
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


