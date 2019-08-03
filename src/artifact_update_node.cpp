#include "ros/ros.h"
#include "ros/package.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/LinearMath/Transform.h"
#include "marble_artifact_detection_msgs/ArtifactArray.h"
#include "lcd_pkg/ArtifactStamped.h"
#include "lcd_pkg/PoseGraph.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int32.h"
#include "math.h"
#include <iostream>
#include <fstream>
#include <sys/stat.h> 

using namespace std;

bool loopClosureFlag = false;
vector<geometry_msgs::PoseStamped> currentPoseArray;
ofstream lout;

void artifactCb(const marble_artifact_detection_msgs::ArtifactArray&);
void poseGraphCb(const lcd_pkg::PoseGraph&);
void initializeLogs(string, string);

class ArtifactArray
{
	vector <uint32_t> poseArrayIndex;
	vector <tf::Transform> world2robot;
	vector <tf::Transform> robot2artifact;
	vector <tf::Transform> world2artifact;
	vector <string> objectClass;
	vector <float> objectProbability;
	vector <bool> isReported;
	vector <int> imageId;
	vector <string> vehicleReporter;
	string owner;
	float simRadius = -1;
	
	public:
	
	int size() // Returns artifact array size
	{
		return poseArrayIndex.size();
	}
	
	float getSimRadius()
	{
		return simRadius;
	}
	
	void setSimRadius(float radius)
	{
		simRadius = radius;
	}
	
	void clear()
	{
		poseArrayIndex.clear();
		world2robot.clear();
		robot2artifact.clear();
		world2artifact.clear();
		objectClass.clear();
		objectProbability.clear();
		isReported.clear();
		imageId.clear();
		vehicleReporter.clear();
		owner = "";
	}
	
	void populate(const marble_artifact_detection_msgs::ArtifactArray& artifactArray, const vector<geometry_msgs::PoseStamped>& poseArray) // Populates the array with the incoming information
	{
		clear();
		owner = artifactArray.owner;
		
		
		for (int i = 0; i < artifactArray.num_artifacts; i++)
		{
			poseArrayIndex.push_back(artifactArray.base_pose_indices[i]);
			
			tf::Transform transform;
			
			tf::Vector3 translation_world2robot(poseArray[artifactArray.base_pose_indices[i]].pose.position.x,
																					poseArray[artifactArray.base_pose_indices[i]].pose.position.y,
																					poseArray[artifactArray.base_pose_indices[i]].pose.position.z);
			tf::Quaternion rotation_world2robot(poseArray[artifactArray.base_pose_indices[i]].pose.orientation.x,
																					poseArray[artifactArray.base_pose_indices[i]].pose.orientation.y,
																					poseArray[artifactArray.base_pose_indices[i]].pose.orientation.z,
																					poseArray[artifactArray.base_pose_indices[i]].pose.orientation.w);
			
			transform.setOrigin(translation_world2robot);
			transform.setRotation(rotation_world2robot);
			world2robot.push_back(transform);
			
			
			tf::Vector3 translation_robot2artifact(artifactArray.artifacts[i].position.x - poseArray[artifactArray.base_pose_indices[i]].pose.position.x,
																						 artifactArray.artifacts[i].position.y - poseArray[artifactArray.base_pose_indices[i]].pose.position.y,
																						 artifactArray.artifacts[i].position.z - poseArray[artifactArray.base_pose_indices[i]].pose.position.z);
			tf::Quaternion rotation_robot2artifact(0, 0, 0, 1);	
			
			transform.setOrigin(translation_robot2artifact);
			transform.setRotation(rotation_robot2artifact);
			robot2artifact.push_back(transform);																	 
			
			transform.mult(world2robot.back(), robot2artifact.back());
			world2artifact.push_back(transform);
			
			objectClass.push_back(artifactArray.artifacts[i].obj_class);
			objectProbability.push_back(artifactArray.artifacts[i].obj_prob);
			isReported.push_back(artifactArray.artifacts[i].has_been_reported);
			imageId.push_back(artifactArray.artifacts[i].image_id);
			vehicleReporter.push_back(artifactArray.artifacts[i].vehicle_reporter);
			
			prune(size()-1, simRadius);
		}
	}
	
	void prune(int artifactArrayIndex, float radius) // Keeps one if multiple artifacts are found at the same location
	{
		for (int i = 0; i < size(); i++)
		{
			if(i != artifactArrayIndex && objectClass[i] == objectClass[artifactArrayIndex] && distance(world2artifact[i], world2artifact[artifactArrayIndex]) < radius )
			{
				poseArrayIndex.erase(poseArrayIndex.begin()+artifactArrayIndex);
				world2robot.erase(world2robot.begin()+artifactArrayIndex);
				robot2artifact.erase(robot2artifact.begin()+artifactArrayIndex);
				world2artifact.erase(world2artifact.begin()+artifactArrayIndex);
				objectClass.erase(objectClass.begin()+artifactArrayIndex);
			}
		}
	}
	
	float distance(tf::Transform transform1, tf::Transform transform2)
	{
		float d = pow(transform1.getOrigin().getX()-transform2.getOrigin().getX(),2) + 
							pow(transform1.getOrigin().getY()-transform2.getOrigin().getY(),2) + 
							pow(transform1.getOrigin().getZ()-transform2.getOrigin().getZ(),2);
		
		return sqrt(d);
	}
	
	void publish()
	{
		lcd_pkg::ArtifactStamped artifactStamped;
		lcd_pkg::Artifact artifact;
		
		ros::NodeHandle nh;
		
		ros::Publisher artifactArrayPub = nh.advertise<lcd_pkg::ArtifactStamped>("updated_artifact_array", 100);
		
		artifactStamped.header.stamp = ros::Time::now();
		artifactStamped.nArtifacts = size();
		artifactStamped.owner = owner;
		
		
		for (int i = 0; i < size(); i++)
		{
			artifact.poseArrayIndex = poseArrayIndex[i];
			
			artifact.objClass = objectClass[i];
			artifact.objProb = objectProbability[i];
			
			artifact.isReported = isReported[i];
			artifact.imageId = imageId[i];
			artifact.vehicleReporter = vehicleReporter[i];
			
			artifact.pose.position.x = world2artifact[i].getOrigin().getX();
			artifact.pose.position.y = world2artifact[i].getOrigin().getY();
			artifact.pose.position.z = world2artifact[i].getOrigin().getZ();
			
			artifact.pose.orientation.x = world2artifact[i].getRotation().x();
			artifact.pose.orientation.y = world2artifact[i].getRotation().y();
			artifact.pose.orientation.z = world2artifact[i].getRotation().z();
			artifact.pose.orientation.w = world2artifact[i].getRotation().w();
			
			artifactStamped.artifact.push_back(artifact);
		}
		
		artifactArrayPub.publish(artifactStamped);
	}
	
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "artifact_update_node");

  ros::NodeHandle nh;

  ros::Subscriber artifactArraySub = nh.subscribe("artifact_array", 10, artifactCb); 
  ros::Subscriber poseGraphSub = nh.subscribe("pose_graph", 10, poseGraphCb);
  
  //while(simRadius_param == -1)
  //{
  //ros::param::get("artifact_update_node/simRadius_param", simRadius_param);
  //ros::param::get("lcd_node/mseThreshold_param", mseThreshold_param);
  
 // ROS_INFO("Waiting for Parameters");
  //}
  
  initializeLogs("lcd_pkg", "artifact_update_node_logs.txt");
  //lout << "arraySize_param = " << arraySize_param << endl;
  //lout << "mseThreshold_param = " << mseThreshold_param << endl << endl;
  
	while(ros::ok())
	{
		ros::spinOnce();
  }

  return 0;
}

void artifactCb(const marble_artifact_detection_msgs::ArtifactArray& msg)
{
	ArtifactArray artifactArray;
	
	static bool isInitialized = false;
	if(!isInitialized)
	{
		float simRadius_param = -1;
		while(simRadius_param == -1)
		{
			ros::param::get("artifact_update_node/simRadius_param", simRadius_param);
			
			ROS_INFO("Waiting for Parameters");
		}
		artifactArray.setSimRadius(simRadius_param);
		isInitialized = true;
  }
	
	if(msg.num_artifacts == 0)
	return;
	// Check for new values
	// Split transforms and copy the new content
	// Update the world2robot transforms for the artifactArray from the pose graph
	// Recalculate the world2artifact transforms for the artifactArray
	// Compare new content with the old content for similarity
	
	artifactArray.populate(msg, currentPoseArray);
	artifactArray.publish();
}

void poseGraphCb(const lcd_pkg::PoseGraph& msg)
{
	currentPoseArray = msg.poseArray;
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


