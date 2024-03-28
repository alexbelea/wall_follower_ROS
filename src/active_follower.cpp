#include <math.h>
#include <ros/ros.h>
//you need to include the header for any ROS message type you use
#include "std_msgs/Int64.h"
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

//Add your own definitions here
#define WHEEL_RADIUS 0.07

class NodeClass
{
  public:
    NodeClass(); //CONSTRUCTOR. It will initialize the node
  private: 	//CLASS METHODS (FUNCTIONS) AND MEMBERS (VARIABLES)
  //1) Subscriptions to topics
	//CALLBACKS to attach to each topic. You need one callback per subscribed topic
	//Replace "std_msgs::Int64" to MATCH the corresponding topic message type 
    void myCallback1(const std_msgs::Int64::ConstPtr& msg);

  //2) Advertisements to topics to publish to, and associted messages
    ros::Publisher my_pub1; //one Publisher object pet topic 
    std_msgs::Int64 my_msg1; //one message object per topic, change "std_msgs::Int64" to the desired type
	
  //3) Any other function you may need
    //You can usually do most stuff into the callback functions
	
  //4) Class members
    ros::NodeHandle nh; //ROS main node handler

    //Any other variable you may need to do your task
    long int int_data;  

};

// One callback function. You need one per subscribed topic
// Part (or all) of the work can be done here
//Replace "std_msgs::Int64" by the corresponding topic's message type
void NodeClass::myCallback1(const std_msgs::Int64::ConstPtr& msg)
{
  int_data = msg->data;

  //If the node needs to calculate and publish data, it can be done here
  my_msg1.data = int_data * 2; //Dumb example, just publish twice the input value  
  my_pub1.publish(my_msg1); 
}

// MAIN function.
// UPDATE THE NODE NAME (don't use spaces, use underscores)
// MATCH THIS NAME IN THE CMakeLists.txt compilation configuration file
int main(int argc, char **argv)
{
  ros::init(argc, argv, "YOUR_NODE_NAME");
  NodeClass nodeclass; //starts everything by calling constructor
}

//CLASS CONSTRUCTOR.
//Usually all initialization goes here
NodeClass::NodeClass()
{
//1) Attach subscription callbacks. Subscriber objects like my_sub1 are needed but not mentioned again
  //Update the TOPIC_NAME and change "std_msgs::Int64" to its corresponding message type
  ros::Subscriber my_sub1 = nh.subscribe<std_msgs::Int64>("/TOPIC_NAME", 10, &NodeClass::myCallback1, this);

//2) Advertise to published topics
  //Choose the topic name you wish (no spaces!) and change "std_msgs::Int64" to your selected message type
  my_pub1 = nh.advertise<std_msgs::Int64>("/MY_TOPIC_NAME", 50);

//3) initialize working variables
  int_data = 0;

// Node ready to rock. If ACTIVE operation, we call loop()
// If REACTIVE ONLY operation (all done in callbacks), we just leave the node idle with ros::spin()
   ros::spin();  //Go to idle, the callback functions will do everything
}

