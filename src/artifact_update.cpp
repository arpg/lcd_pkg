#include "ros/ros.h"
#include "ros/package.h"
#include "cartographer_ros_msgs/TrajectoryQuery.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/TransformStamped.h"
#include "Transform.h"
#include "marble_artifact_detection_msgs/ArtifactArray.h"
#include "lcd_pkg/ArtifactStamped.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int32.h"
#include "math.h"
#include <iostream>
#include <fstream>
#include <sys/stat.h> 

using namespace std;

int arraySize_param = -1;
double mseThreshold_param = -1;
bool loopClosureFlag = false;
ofstream lout;

void artifactCb(const marble_artifact_detection_msgs::ArtifactArray&);
void trajCb(const vector<geometry_msgs::PoseStamped>&);
float mse(vector<geometry_msgs::Point>&, vector<geometry_msgs::Point>&);
int sc(vector<geometry_msgs::Point>&, vector<geometry_msgs::Point>&);
int zc(vector<geometry_msgs::Point>&);
void initializeLogs(string, string);

class ArtifactArray
{
	vector <uint> poseGraphIndex;
	vector <tf::Transform> world2robot;
	vector <tf::Transform> robot2artifact;
	vector <tf::Transform> world2artifact;
	vector <string> objectClass;
	
	public:
	
	int size() // Returns artifact array size
	{
		return poseGraphIndex.size();
	}
	
	void clear()
	{
		poseGraphIndex.clear();
		world2robot.clear();
		robot2artifact.clear();
		world2artifact.clear();
		objectClass.clear();
	}
	
	void populate(vector <marble_artifact_detection_msgs:ArtifactArray>& artifactArray, vector<geometry_msgs::Pose>& poseGraph, ArtifactArray&); // Populates the array with the incoming information
	{
		clear();
		
		for (int i = 0; i < artifactArray.base_poses.size().size(); i++)
		{
			poseGraphIndex.push_back(artifactArray.base_pose_indices[i]);
			
			tf::Transform transform;
			
			tf::Vector3 translation_world2robot(poseGraph[artifactArray.base_pose_indices[i]].pose.position.x,
																					poseGraph[artifactArray.base_pose_indices[i]].pose.position.y,
																					poseGraph[artifactArray.base_pose_indices[i]].pose.position.z);
			tf::Quaternion rotation_world2robot(poseGraph[artifactArray.base_pose_indices[i]].pose.orientation.x,
																					poseGraph[artifactArray.base_pose_indices[i]].pose.orientation.y,
																					poseGraph[artifactArray.base_pose_indices[i]].pose.orientation.z,
																					poseGraph[artifactArray.base_pose_indices[i]].pose.orientation.w);
			
			transform.setOrigin(translation_world2robot);
			transform.setRotation(rotation_world2robot);
			world2robot.push_back(transform);
			
			
			tf::Vector3 translation_robot2artifact(artifactArray.artifacts[i].position.x - poseGraph(artifactArray.base_pose_indices[i]).pose.position.x,
																						 artifactArray.artifacts[i].position.y - poseGraph(artifactArray.base_pose_indices[i]).pose.position.y,
																						 artifactArray.artifacts[i].position.z - poseGraph(artifactArray.base_pose_indices[i]).pose.position.z);
			tf::Quaternion rotation_robot2artifact(0, 0, 0, 1);	
			
			transform.setOrigin(translation_robot2artifact);
			transform.setRotation(rotation_robot2artifact);
			robot2artifact.push_back(transform);																	 
			
			transform.mult(world2robot.back(), robot2artifact.back());
			world2artifact.push_back(transform);
			
			objectClass.push_back(artifacts[i].obj_class);
			
			prune(size()-1, aRadius);
		}
	}
	
	void prune(int artifactArrayIndex, float aRadius); // Keeps one if multiple artifacts are found at the same location
	{
		for (int i = 0; i < size(); i++)
		{
			if(i != artifactArrayIndex && objectClass[i] == objectClass[artifactArrayIndex] && distance(world2artifact[i], world2artifact[artifactArrayIndex]) < aRadius )
			{
				poseGraphIndex.erase(poseGraphIndex.begin()+artifactArrayIndex);
				world2robot.erase(world2robot.begin()+artifactArrayIndex);
				robot2artifact.erase(robot2artifact.begin()+artifactArrayIndex);
				world2artifact.erase(world2artifact.begin()+artifactArrayIndex);
				objectClass.erase(objectClass.begin()+artifactArrayIndex);
			}
		}
	}
	
	float distance(tf::Transform transform1, tf::Transform transform2)
	{
		d = pow(transform1.getOrigin().getX()-transform2.getOrigin().getX(),2) + 
				pow(transform1.getOrigin().getY()-transform2.getOrigin().getY(),2) + 
				pow(transform1.getOrigin().getZ()-transform2.getOrigin().getZ(),2);
		
		return sqrt(d);
	}
	
	void publish()
	{
		lcd_pkg::ArtifactStamped artifactStamped;
		lcd_pkg::Artifact artifact;
		
		ros::Publisher artifactArrayPub = nh.advertise<lcd_pkg::ArtifactStamped>("artifact_array", 100);
		
		for (int i = 0; i < size(); i++)
		{
			artifact.poseGraphIndex = poseGraphIndex[i];
			
			artifact.objectClass(objectClass[i]);
			
			artifact.transform.translation.x = world2artifact[i].translation.x;
			artifact.transform.translation.y = world2artifact[i].translation.y;
			artifact.transform.translation.z = world2artifact[i].translation.z;
			
			artifact.transform.rotation.x = world2artifact[i].rotation.x;
			artifact.transform.rotation.y = world2artifact[i].rotation.y;
			artifact.transform.rotation.z = world2artifact[i].rotation.z;
			artifact.transform.rotation.w = world2artifact[i].rotation.w;
			
			ArtifactStamped.artifact.push_back(artifact);
		}
		
		artifactArrayPub.publish(ArtifactStamped);
	}
	
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "lcd_node");

  ros::NodeHandle nh;

  ros::Publisher loopClosurePub = nh.advertise<std_msgs::Int32>("number_of_loop_closures", 100);
  ros::Subscriber artifactArraySub = nh.subscribe("/H01/artifact_array", 10, artifactCb); 
  ros::ServiceClient client = nh.serviceClient<cartographer_ros_msgs::TrajectoryQuery>("trajectory_query");
  
  while(arraySize_param == -1 || mseThreshold_param == -1)
  {
  ros::param::get("lcd_node/arraySize_param", arraySize_param);
  ros::param::get("lcd_node/mseThreshold_param", mseThreshold_param);
  
  ROS_INFO("Waiting for Parameters");
  }
  
  initializeLogs("lcd_pkg", "manuever_logs.txt");
  lout << "arraySize_param = " << arraySize_param << endl;
  lout << "mseThreshold_param = " << mseThreshold_param << endl << endl;
  
  cartographer_ros_msgs::TrajectoryQuery srv;
  srv.request.trajectory_id = 0;

	ros::Rate loop_rate(200);
	
	while(ros::ok())
	{
		if (client.call(srv))
		{
			cout << "Trajectory status = " << srv.response.status.message << endl; 
		  cout << "Trajectory size = " << srv.response.trajectory.size() << endl;
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

void artifactCb(const marble_artifact_detection_msgs::ArtifactArray& msg)
{
	static ArtifactArray artifactArray;
	
	if(msg.num_artifacts == 0)
	return;
	// Check for new values
	// Split transforms and copy the new content
	// Update the world2robot transforms for the artifactArray from the pose graph
	// Recalculate the world2artifact transforms for the artifactArray
	// Compare new content with the old content for similarity
	
	for(int i = 0; i < msg.artifacts.size(); i++)
	{
		artifactArray.populate(msg.artifacts, artifactArray);
	}
}

void trajCb(const vector<geometry_msgs::PoseStamped>& msg)
{
	static bool isPopulated_prevTruncatedArray = false;
	
	const int truncatedArraySize = arraySize_param; // Number of array elements to compare
	const float mseThreshold = mseThreshold_param; // Mean square error threshold between two arrays to declare dissimilarilty
	
	static int prevInputArraySize = 0; // Previous input array size
	
	static vector<geometry_msgs::Point> prevTruncatedArray(truncatedArraySize); // truncatedArraySize number of elements of the previous input array
	
	int newInputArraySize = msg.size(); // New input array size
	
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
			loopClosureFlag = true;
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

