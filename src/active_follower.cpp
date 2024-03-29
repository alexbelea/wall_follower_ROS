#include <math.h>
#include <ros/ros.h>
//you need to include the header for any ROS message type you use
#include "std_msgs/Int16.h"
#include "std_msgs/Int64.h"
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

//Add your own definitions here
#define LINEAR_SPEED 0.3 // m/s
#define ANGULAR_SPEED 0.7 // rad/s
#define OBJECT_DIST_DETECTED 50 // cm
#define OBJECT_DIST_FOLLOW 30 // cm
#define PI 3.1416
#define MIN_ANGLE_DEG 
#define DEBUG 1  //switch this to 1 to see debug info in command line

enum STATE { STRAIGHT, TURN_OPP_SIDE, FOLLOW }; //possible FSM states
/*
 * STRAIGHT - no    wall detected, keep going forward
 * TURN_OPP_SIDE - FRONT wall detected, pick a random side
 * FOLLOW - SIDE  wall detected, follow wall PID 
 */

class ActiveFollower
{
  public:
    ActiveFollower(); //CONSTRUCTOR. It will initialize the node
  private: 	//CLASS METHODS (FUNCTIONS) AND MEMBERS (VARIABLES)
  //1) Subscriptions to topics
	//CALLBACKS to attach to each topic. You need one callback per subscribed topic
	//Replace "std_msgs::Int64" to MATCH the corresponding topic message type 
  //void myCallback1(const std_msgs::Int64::ConstPtr& msg);

  // CALLBACK FUNCTIONS FOR ALL 3 SONARS
    void CentralSonarCallback(const std_msgs::Int16::ConstPtr& msg);
    void LeftSonarCallback(const std_msgs::Int16::ConstPtr& msg);
    void RightSonarCallback(const std_msgs::Int16::ConstPtr& msg);
    void UpdateFSM(); // Function where FSM is updated
	//node variables
	STATE fsm_state; //robot's FSM state
  //2) Advertisements to topics to publish to, and associted messages
    ros::Publisher vel_pub;   //publisher for /cmd_vel
	
  //3) Any other function you may need
    //You can usually do most stuff into the callback functions
	
  //4) Class members
    ros::NodeHandle nh;       //ROS main node handler


    //Any other variable you may need to do your task
      
    int center_dist, left_dist, right_dist;
    geometry_msgs::Twist vel_msg; //Twist message to publish

  // Subscribers to sonars
    ros::Subscriber center_sonar_sub;     // subscriber for central sonar  topic /arduino/sonar_2
    ros::Subscriber left_sonar_sub;       // subscriber for left sonar     topic /arduino/sonar_3
    ros::Subscriber right_sonar_sub;      // subscriber for right sonar,   topic /arduino/sonar_1

};
//--------^END OF CLASS^--------

// MAIN function.
// UPDATE THE NODE NAME (don't use spaces, use underscores)
// MATCH THIS NAME IN THE CMakeLists.txt compilation configuration file
int main(int argc, char **argv)
{
  ros::init(argc, argv, "active_follower"); //name should reflect what is in CMakeLists.txt target_link_libraries
  ActiveFollower activefollower; //starts everything by calling constructor
}
//---^^END OF MAIN^^--------

//CLASS CONSTRUCTOR.
//Usually all initialization goes here
ActiveFollower::ActiveFollower()
{
//1) Attach subscription callbacks. Subscriber objects like my_sub1 are needed but not mentioned again
  //Update the TOPIC_NAME and change "std_msgs::Int64" to its corresponding message type
  //                                              ("topic"           , Hz, address of callback function, pointer to the current object instance that the method belongs to )
  center_sonar_sub = nh.subscribe<std_msgs::Int16>("/arduino/sonar_2", 10, &ActiveFollower::CentralSonarCallback, this);
  left_sonar_sub = nh.subscribe<std_msgs::Int16>("/arduino/sonar_3", 10, &ActiveFollower::LeftSonarCallback, this);
  right_sonar_sub = nh.subscribe<std_msgs::Int16>("/arduino/sonar_1", 10, &ActiveFollower::RightSonarCallback, this);

//2) Advertise to published topics
  //Choose the topic name you wish (no spaces!) and change "std_msgs::Int64" to your selected message type

  vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10); // for publishing the speeds

//3) initialize working variables
  
  fsm_state = STRAIGHT; // initially the robot goes straight until any wall is found;
  center_dist = 0;
  left_dist   = 0;
  right_dist  = 0;

// Node ready to rock. If ACTIVE operation, we call loop()
// If REACTIVE ONLY operation (all done in callbacks), we just leave the node idle with ros::spin()
   ros::spin();  //Go to idle, the callback functions will do everything
}
//---------^^END OF CONSTRUCTOR^^--------



// One callback function. You need one per subscribed topic
// Part (or all) of the work can be done here
//Replace "std_msgs::Int64" by the corresponding topic's message type

// Callback function attached to CENTER sonar topic
void ActiveFollower::CentralSonarCallback(const std_msgs::Int16::ConstPtr& msg)
{
  center_dist = msg->data;
  UpdateFSM();
}

// Callback function attached to LEFT sonar topic
void ActiveFollower::LeftSonarCallback(const std_msgs::Int16::ConstPtr& msg)
{
  left_dist = msg->data;
}

// Callback function attached to RIGHT sonar topic
void ActiveFollower::RightSonarCallback(const std_msgs::Int16::ConstPtr& msg)
{
  right_dist = msg->data;
}

void ActiveFollower::UpdateFSM()  // HERE all the magic happens
{ if(DEBUG){ //print info for debug
  ROS_INFO("\nfsm_state: %d\ncenter_dist: %d\nleft_dist: %d\nright_dist: %d", fsm_state, center_dist, left_dist, right_dist);
  
}
  switch(fsm_state){
    case STRAIGHT:
    if(   // only SIDE detected, go to FOLLOW case
        (center_dist == 0) &&      
            ( (left_dist >= 1) || (right_dist >= 1) ) )
      {
           ROS_INFO("Follow");
           fsm_state = FOLLOW;  
      }
    else if( // Only FRONT or SIDE detected - go to TURN_OPP_SIDE
        (center_dist >= 1) && (left_dist == 0) && (right_dist == 0) )
      {
        ROS_INFO("TURN_OPP_SIDE");
        fsm_state = TURN_OPP_SIDE;
      }

    else    // No walls detected- go straight
      {
        ROS_INFO("STRAIGHT");
        vel_msg.linear.x = LINEAR_SPEED;
        vel_msg.angular.z = 0.0;
        vel_pub.publish(vel_msg);
      }
    break;

   case FOLLOW:
    ROS_INFO("INSIDE FOLLOW CASE");

   break;

   case TURN_OPP_SIDE: 
   if(  // SIDE detected, not FRONT - go to FOLLOW
       (center_dist == 0) && ( (left_dist >=1) || (right_dist >= 1) )  )
      {
       ROS_INFO("FOLLOW");
       fsm_state = FOLLOW;
      }
   else if( // no wall - go to STRAIGHT 
            (center_dist == 0) && (left_dist ==0) && (right_dist ==0) )
            {
              ROS_INFO("STRAIGHT");
              fsm_state = STRAIGHT;
            }
   else // FRONT & SIDE - keep turning away from wall until only side wall is detected
    {
      ROS_INFO("TURN_OPP_SIDE");
      vel_msg.linear.x = LINEAR_SPEED;     //set linear speed to zero
      vel_msg.angular.z = ANGULAR_SPEED - LINEAR_SPEED;  //random angular speed calculated in case reverse  
      vel_pub.publish(vel_msg);   //publish the message with new parameters
    }

   break;
  }
}