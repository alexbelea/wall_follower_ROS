#include <math.h>
#include <ros/ros.h>
//you need to include the header for any ROS message type you use
#include "std_msgs/Int64.h"
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

//Add your own definitions here
#define WHEEL_RADIUS 0.07

class ActiveFollower
{
  public:
    ActiveFollower(); //CONSTRUCTOR. It will initialize the node
  private: 	//CLASS METHODS (FUNCTIONS) AND MEMBERS (VARIABLES)
  //1) Subscriptions to topics
	//CALLBACKS to attach to each topic. You need one callback per subscribed topic
	//Replace "std_msgs::Int64" to MATCH the corresponding topic message type 
    void leftEncoderCallback(const std_msgs::Int64::ConstPtr& msg);
    void rightEncoderCallback(const std_msgs::Int64::ConstPtr& msg);
    void sonarCallback(const std_msgs::Int16::ConstPtr& msg);

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

// MAIN function.
// UPDATE THE NODE NAME (don't use spaces, use underscores)
// MATCH THIS NAME IN THE CMakeLists.txt compilation configuration file
int main(int argc, char **argv)
{
  ros::init(argc, argv, "active_follower");
  ActiveFollower active_follower; //starts everything by calling constructor
}

//CLASS CONSTRUCTOR.
//Usually all initialization goes here
ActiveFollower::ActiveFollower()
{
//1) Attach subscription callbacks. Subscriber objects like my_sub1 are needed but not mentioned again
  //Update the TOPIC_NAME and change "std_msgs::Int64" to its corresponding message type
  ros::Subscriber my_sub1 = nh.subscribe<std_msgs::Int64>("/TOPIC_NAME", 10, &ActiveFollower::myCallback1, this);

//2) Advertise to published topics
  //Choose the topic name you wish (no spaces!) and change "std_msgs::Int64" to your selected message type
  my_pub1 = nh.advertise<std_msgs::Int64>("/MY_TOPIC_NAME", 50);

//3) initialize working variables
  int_data = 0;

// Node ready to rock. If ACTIVE operation, we call loop()
// If REACTIVE ONLY operation (all done in callbacks), we just leave the node idle with ros::spin()
   ros::spin();  //Go to idle, the callback functions will do everything
}

// One callback function per subscribed topic
// Part (or all) of the work can be done here
//Replace "std_msgs::Int64" by the corresponding topic's message type
//Callback function attached to the left encoder topic
void SenseAndAvoid::leftEncoderCallback(const std_msgs::Int64::ConstPtr& msg)
{
  left_count = msg->data; //update left encoder pulse count
}
//Callback function attached to the right encoder topic
void SenseAndAvoid::rightEncoderCallback(const std_msgs::Int64::ConstPtr& msg)
{
  right_count = msg->data; //update right encoder pulse count
}

//The sonar callback function is used to update the Finite State Machine (FSM)
void SenseAndAvoid::sonarCallback(const std_msgs::Int16::ConstPtr& msg)
{
  int d = msg->data; //update sonar distance
  //Evaluate FSM
  switch(state){
   case FORWARD: //currently moving forward
     if((d>0)&&(d<OBJECT_DIST_NEAR)) //obstacle close, go reverse!
     {
      ROS_INFO("REVERSE");
      state = REVERSE; //change state to reverse
      vel_msg.linear.x = 0.0;
      vel_msg.angular.z = 0.0;
      vel_pub.publish(vel_msg); //stop
     }
     else //keep moving forward
     {
      ROS_INFO("Dist %d cm",d);
      vel_msg.linear.x = LINEAR_SPEED;
      vel_msg.angular.z = 0.0;
      vel_pub.publish(vel_msg);
     }
     break;
   case REVERSE: //currently reversing
     if(d == 0) break; //no valid detection
     else if(d>OBJECT_DIST_SAFE) //obstacle away, now turn
     {
      ROS_INFO("TURN");
      state = TURN;
      old_left_count = left_count;
	  old_right_count = right_count;
	  //generate random angle to rotate in MIN to MAX range
	  alpha = MIN_ANGLE_DEG + rand()%(MAX_ANGLE_DEG - MIN_ANGLE_DEG); //deg
	  alpha = alpha * PI / 180.0; //to rad
     }
     else //keep moving backward
     {
      //YOUR CODE HERE
      ROS_INFO("Dist %d cm",d);
      vel_msg.linear.x = LINEAR_SPEED*-1;
      vel_msg.angular.z = 0.0;
      vel_pub.publish(vel_msg);
     }
     break;
   case TURN:
     float theta = Rotation_rad();
     if(theta > alpha) //turn complete
     {
      state = FORWARD;
      ROS_INFO("FORWARD");
      vel_msg.linear.x = 0.0;
      vel_msg.angular.z = 0.0;
      vel_pub.publish(vel_msg); //stop
     }
     else //keep rotating
     {
      ROS_INFO("Rotated %.1f, target %.1f",theta,alpha);
      //YOUR CODE HERE
      vel_msg.linear.x = 0.0;     //set linear speed to zero
      vel_msg.angular.z = alpha;  //random angular speed calculated in case reverse  
      vel_pub.publish(vel_msg);   //publish the message with new parameters
     }
	 break;
  }//end switch
}//end sonarCallback

//calculates rotated angle from encoder readings
//using the differential drive model
float SenseAndAvoid::Rotation_rad()
{
	float angle;
	int increment_l = left_count - old_left_count;
	int increment_r = right_count - old_right_count;
	angle = WHEEL_R*(increment_r - increment_l)/(TRACK_L * ENC_TICKS_RAD);
	return angle;
}
