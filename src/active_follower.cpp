#include <math.h>
#include <ros/ros.h>
//you need to include the header for any ROS message type you use
#include "std_msgs/Int16.h"
#include "std_msgs/Int64.h"
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

#define DEBUG 1  //switch this to 1 to see debug info in command line

//Add your own definitions here
#define LINEAR_SPEED 0.3 // m/s
#define ANGULAR_SPEED 0.5 // rad/s
#define OBJECT_DIST_DETECTED 90 // cm
#define OBJECT_DIST_FOLLOW 40 // cm
#define PI 3.1416
#define MIN_ANGLE_DEG
#define POLL_RATE 20.0


// PID control
#define KP 0.1
#define KI 0.01 //start at zero to fine-tune
#define KD 0.02
#define ERROR_MAX 10

enum STATE { STRAIGHT, TURN_OPP_SIDE, FOLLOW }; //possible FSM states

/*
 * STRAIGHT - no    wall detected, keep going forward
 * TURN_OPP_SIDE - FRONT wall detected, pick a random side
 * FOLLOW - SIDE  wall detected, follow wall PID 
 */

//enum DIR { CW, CC}; // keep track of follow direction clockwise/counterclockwise //not sure if needed

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
    void WallDetect();
    void PIDcontrol();
	//node variables
	STATE fsm_state; //robot's FSM state
  //2) Advertisements to topics to publish to, and associted messages
    ros::Publisher vel_pub;   //publisher for /cmd_vel
	
  //3) Any other function you may need
    //You can usually do most stuff into the callback functions
	
  //4) Class members
    ros::NodeHandle nh;       //ROS main node handler


    //Any other variable you may need to do your task

    
    int center_dist, left_dist, right_dist;// variables holding distance values
    
    geometry_msgs::Twist vel_msg; //Twist message to publish

    bool center, left, right;// bools to keep track of wall detection

    double error,
       previous_error,
         derivative,
           integral,
             old_left_dist, old_right_dist; // PID controller vars



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
  center_sonar_sub = nh.subscribe<std_msgs::Int16>("/arduino/sonar_2", POLL_RATE, &ActiveFollower::CentralSonarCallback, this);
  left_sonar_sub = nh.subscribe<std_msgs::Int16>("/arduino/sonar_3", POLL_RATE, &ActiveFollower::LeftSonarCallback, this);
  right_sonar_sub = nh.subscribe<std_msgs::Int16>("/arduino/sonar_1", POLL_RATE, &ActiveFollower::RightSonarCallback, this);

//2) Advertise to published topics
  //Choose the topic name you wish (no spaces!) and change "std_msgs::Int64" to your selected message type

  vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10); // for publishing the speeds

//3) initialize working variables
  
  fsm_state = STRAIGHT; // initially the robot goes straight until any wall is found;
  center_dist = 0;
  left_dist   = 0;
  right_dist  = 0;

  center = false;
  left = false;
  right = false;

  error            = 0;
  previous_error   = 0;
  derivative       = 0;
  integral         = 0;

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
  // call the functions doing the work here
  WallDetect();
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
  ROS_INFO("\nCenter: %d\nLeft: %d\nRight: %d", center, left, right);
}
  switch(fsm_state){
    case STRAIGHT:
    if( !center && (left || right) ) // only SIDE detected, go to FOLLOW case
      {
           ROS_INFO("Follow");
           fsm_state = FOLLOW;  
      }
    else if( center && (left || right) )//  CENTER and at least a SIDE detected - go to TURN_OPP_SIDE
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
    if( center && (left || right) ){ //if CENTER and either left or right, go to turn opp side to turn corner 
    ROS_INFO("TURN_OPP_SIDE");
    fsm_state = TURN_OPP_SIDE;
   }
   else{  //do follow PID logic 
    integral = 0; //reset integral before calling PIDcontrol   to avoid PID going bananas
    derivative = 0;
    PIDcontrol();
   }
   break;

   case TURN_OPP_SIDE: 
   if( !center && ( left || right ) )  // SIDE detected, not CENTER - go to FOLLOW
      {
       ROS_INFO("FOLLOW");
       fsm_state = FOLLOW;
      }
   else if( !center && !left && !right ) // no wall - go to STRAIGHT 
            {
              ROS_INFO("STRAIGHT");
              fsm_state = STRAIGHT;
            }
   else // FRONT & SIDE - keep turning away from wall until only side wall is detected
    {
      ROS_INFO("TURN_OPP_SIDE");
      vel_msg.linear.x = 0.0;     
      if(left && !right){
        vel_msg.angular.z = -ANGULAR_SPEED; // detected left wall so TURN RIGHT
      }
      else if(right && !left)
      {
        vel_msg.angular.z = ANGULAR_SPEED; // detected right wall so TURN LEFT
      } 
      else if(right && left){       // detected both LEFT and RIGHT - turn based on comparing distance

        if(left_dist > right_dist){ // turned more to RIGHT, so keep turning RIGHT
        vel_msg.angular.z = ANGULAR_SPEED/2;
        }
        else if(left_dist < right_dist){ // turned more to LEFT, so keep turning LEFT
        vel_msg.angular.z = ANGULAR_SPEED/2;
        }


      }
      vel_pub.publish(vel_msg);   //publish the message with new parameters
    }

   break;
  }
}

// detects walls within required distance and updates related bools
// needed to simpliy if statements
void ActiveFollower::WallDetect(){
  // detect center wall
  if( (center_dist > 0) && (center_dist <= OBJECT_DIST_DETECTED) ) center = true; else center = false;

  // detect left wall
  if( (left_dist > 0) && (left_dist <= OBJECT_DIST_DETECTED) ) left = true; else left = false;

  // detect right wall
  if( (right_dist > 0) && (right_dist <= OBJECT_DIST_DETECTED) ) right = true; else right = false;
}


void ActiveFollower::PIDcontrol() {
  
    double PID_output;
    double theta;
    // Calculate error for left or right, and only if distance not zero (wrong reading)
    if     (left && (left_dist != 0)  ) error = OBJECT_DIST_FOLLOW - left_dist;
    else if(right && (right_dist !=0) ) error = OBJECT_DIST_FOLLOW - right_dist;
    else{ //no walls detected, return to state 0
      error = 0;
      fsm_state = STRAIGHT;
      ROS_INFO("NO SIDE WALL DETECTED");
      }

    // Update integral and avoid windup
    if(abs(error) > ERROR_MAX) integral += error* (1/POLL_RATE); // only add error if below threshold

    // Calculate derivative
    //derivative = error - previous_error;
    if(left) { derivative = (left_dist - old_left_dist) / (1/POLL_RATE); } // for left side
    if(right){ derivative = (right_dist - old_right_dist) / (1/POLL_RATE); } // for right side

    // Calculate PID_output <-- distance to OBJECT_DIST_DETECTED
    PID_output = KP * (error + KI * integral + KD * derivative);

    // Update previous 
    //previous_error = error;
    old_left_dist = left_dist;
    old_right_dist = right_dist;
    
    // convert PID_output to angle theta to output
    theta = tan(PID_output/2);

    if( theta > PI ){  // check if theta greater than +-180 degrees, limit
      theta = PI;
    }
    else if(theta < -PI ){ 
      theta = -PI;
    }
    if(DEBUG){
      ROS_INFO("~~~~~~~~~~PID loop~~~~~~~~~~");
      ROS_INFO("PID output: %.3f\nTheta: %.3f\nIntegral: %.3f\nDerivative: %.1f\nError: %.1f\nOld Left Dist: %.1f\nOld Right Dist: %.1f", PID_output, theta, integral, derivative, error, old_left_dist, old_right_dist);
      }
    //Apply speeds 
    vel_msg.linear.x = LINEAR_SPEED;
    vel_msg.angular.z = theta;
    vel_pub.publish(vel_msg);
}
