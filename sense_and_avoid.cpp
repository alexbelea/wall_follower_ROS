#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int64.h"
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */

#define LINEAR_SPEED 0.3 // m/s
#define ANGULAR_SPEED 0.7 // rad/s
#define OBJECT_DIST_NEAR 30 // cm
#define OBJECT_DIST_SAFE 50 // cm
#define PI 3.1416
#define MIN_ANGLE_DEG 30  //degrees
#define MAX_ANGLE_DEG 120 //degrees
#define WHEEL_R 0.07 //r, metres
#define TRACK_L 0.36 //L, metres
#define ENC_TICKS_RAD 644/(2*PI) //encoder ticks per wheel rotation rad

enum STATE { FORWARD, REVERSE, TURN }; //possible FSM states

class SenseAndAvoid
{
  public:
    SenseAndAvoid(); //constructor method
  private:
    //callback functions to attach to subscribed topics
    void leftEncoderCallback(const std_msgs::Int64::ConstPtr& msg);
    void rightEncoderCallback(const std_msgs::Int64::ConstPtr& msg);
    void sonarCallback(const std_msgs::Int16::ConstPtr& msg);
	//node variables
	STATE state; //robot's FSM state
    ros::NodeHandle nh; //ROS node handler
    ros::Publisher vel_pub; //publisher for /cmd_vel
	geometry_msgs::Twist vel_msg; //Twist message to publish
    ros::Subscriber sonar_sub; //subscriber for central sonar
    ros::Subscriber left_encoder_sub; //subscriber for left encoder
    ros::Subscriber right_encoder_sub;//subscriber for right encoder
	//encoder count handling
    long int left_count, right_count, old_left_count, old_right_count;
	float Rotation_rad(); //calculates rotated angle
	float alpha; //random angle to rotate
};

// Main function
int main(int argc, char **argv)
{
  //initialize ROS node
  ros::init(argc, argv, "sense_and_avoid");
  //Create the object that contains node behaviour
  SenseAndAvoid sense_and_avoid; //this calls the constructor method
  //keep ROS updating the callback functions until node is killed
  ros::spin();
}

//Constructor. ROS initializations are done here.
SenseAndAvoid::SenseAndAvoid()
{
  //subscribe to topics
  left_encoder_sub = nh.subscribe<std_msgs::Int64>("/arduino/encoder_left_value", 10, &SenseAndAvoid::leftEncoderCallback, this);
  right_encoder_sub = nh.subscribe<std_msgs::Int64>("/arduino/encoder_right_value", 10, &SenseAndAvoid::rightEncoderCallback, this);
  sonar_sub = nh.subscribe<std_msgs::Int16>("/arduino/sonar_2", 10, &SenseAndAvoid::sonarCallback, this);
  //advertise topics to publish
  vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
  //init auxiliary variables
  state = FORWARD;  //initial robot state
  left_count = 0; 
  right_count = 0;
  srand(time(NULL)); //init random number generator for angle
}

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
     else //keep rptating
     {
      ROS_INFO("Rotated %.1f, target %.1f",theta,alpha);
      //YOUR CODE HERE
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
