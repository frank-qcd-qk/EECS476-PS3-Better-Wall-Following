#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>  // boolean message
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h>  //Including the Float32 class from std_msgs
#include <nav_msgs/Odometry.h>

nav_msgs::Odometry
    g_odom;  // odom is not actually used in this code, but could be
// callback extracts the robot heading w/rt world

//! Flag used for making sure code runs on turtlebot:
const bool turtleFlag = false;

//! Tunable variables:
const float ACCEPTABLE_SIDE_CLEARANCE =
    0.25;  //* Set the acceptable side clearance
const float ACCEPTABLE_forward_CLEARANCE =
    1.0;  //* Set the acceptable side clearance
const float fovAngle =
    atan2f(ACCEPTABLE_SIDE_CLEARANCE, ACCEPTABLE_forward_CLEARANCE);
// some timing constants
const double LINEAR_SPEED = 0.1;  // m/s LINEAR_speed command
const double YAW_RATE = 0.1;      // 0.1; // rad/sec yaw rate command
const double SAMPLE_DT = 0.01;    // specify a sample period of 10ms

//* these values to be set within the laser callback parameters of the lidar:
float angle_min_ = 0.0;
float angle_max_ = 0.0;
float angle_increment_ = 0.0;
float range_min_ = 0.0;
float range_max_ = 0.0;
//* These values are the index for laserCallback
int rightSideIndex = -1;
int centerSideIndex = -1;
int leftSideIndex = -1;
int totalLaserPoints = -1;
int topRightIndex = -1;
int topLeftIndex = -1;

//! Wall following states:
enum wallFollowingStates {
  dead_reckoning = 1,
  measuring = 2,
  finding = 3,
  corner = 4,
  correcting = 5
};

wallFollowingStates state = measuring;

std::vector<float> forwardClearanceList;
std::vector<float> wallClearanceList;

float wall_steer_factor = 10.0;
float wall_distance_factor = 0.0;
//! Wall following timer
double dead_reckoning_timer = 0.0;
float maxSawAngle = 0.0;
float minSawAngle = 0.0;

//! Odometry Global variables
double g_phi = -100;  // robot heading--absolute,  w/rt world (per odom); gets
                      // updated by odom

//! Odometry callback
void odomCallback(const nav_msgs::Odometry& odom_msg) {
  g_odom = odom_msg;
  // here is a means to convert quaternion to a scalar heading angle
  double quat_z = odom_msg.pose.pose.orientation.z;
  double quat_w = odom_msg.pose.pose.orientation.w;
  g_phi = 2.0 * atan2(quat_z, quat_w);  // cheap conversion from quaternion to
                                        // heading for planar motion
  // ROS_INFO("g_phi = %f", g_phi);
}

//! Used for quickly transition the state.
void transitionState(wallFollowingStates newState)  // Everytime this is called,
                                                    // the robot will change
                                                    // state; state timer will
                                                    // reset
{
  state = newState;
}

float calculatStandardDev(std::vector<float> v) {
  double sum = std::accumulate(v.begin(), v.end(), 0.0);
  double mean = sum / v.size();

  std::vector<double> diff(v.size());
  std::transform(v.begin(), v.end(), diff.begin(),
                 std::bind2nd(std::minus<double>(), mean));
  double sq_sum =
      std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
  float stdev = std::sqrt(sq_sum / v.size());
  return stdev;
}

float calculateAverage(std::vector<float> v) {
  double sum = std::accumulate(v.begin(), v.end(), 0.0);
  double average = sum / v.size();
  return average;
}

void laserAwareness(const sensor_msgs::LaserScan& laser_scan) {
  forwardClearanceList.clear();
  wallClearanceList.clear();
  for (int i = 0; i <= topLeftIndex; i++) {
    //* Data Aquisition
    float currentReadout = laser_scan.ranges[i];
    if (std::isinf(currentReadout) || std::isnan(currentReadout)) {
      continue;
    }
    // ROS_INFO("-------------------------");
    // ROS_INFO("Current Raw Readout is %i:%f", i, currentReadout);
    //*Right_Back section
    if (i < rightSideIndex) {
      // ROS_INFO("Right_Back");
      float currentAngle =
          angle_max_ - (float)(3.1415 / 2) -
          i * angle_increment_;  // gives us angle for conversion
      currentAngle = fabs(currentAngle);
      float calculatedSideAcctual = cos(currentAngle) * currentReadout;
      wallClearanceList.push_back(calculatedSideAcctual);
      // ROS_INFO("DEBUG: currentAngle is: %f, sideAcctual is: %f",
      // currentAngle, calculatedSideAcctual);
    }

    //*Right_front_bottom section
    if (i < topRightIndex && i >= rightSideIndex) {
      // ROS_INFO("Right_front_bottom");
      float currentAngle =
          i * angle_increment_ -
          (angle_max_ - (3.1415 / 2));  // gives us angle for conversion
      currentAngle = fabs(currentAngle);
      float calculatedSideAcctual = cos(currentAngle) * currentReadout;
      wallClearanceList.push_back(calculatedSideAcctual);
      // ROS_INFO("DEBUG: currentAngle is: %f, sideAcctual is: %f",
      // currentAngle, calculatedSideAcctual);
    }

    //*Right_front_top section
    if (i >= topRightIndex && i <= centerSideIndex) {
      // ROS_INFO("Right_front_top");
      float currentAngle =
          angle_max_ - i * angle_increment_;  // gives us angle for conversion
      currentAngle = fabs(currentAngle);
      float calculatedSideAcctual = cos(currentAngle) * currentReadout;
      forwardClearanceList.push_back(calculatedSideAcctual);
      // ROS_INFO("DEBUG: currentAngle is: %f, sideAcctual is: %f",
      // currentAngle, calculatedSideAcctual);
    }
    //*Left_front_top section
    if (i >= centerSideIndex && i <= topLeftIndex) {
      // ROS_INFO("Left_front_top");
      float currentAngle =
          i * angle_increment_ - angle_max_;  // gives us angle for conversion
      currentAngle = fabs(currentAngle);
      float calculatedSideAcctual = cos(currentAngle) * currentReadout;
      forwardClearanceList.push_back(calculatedSideAcctual);
      // ROS_INFO("DEBUG: currentAngle is: %f, sideAcctual is: %f",
      // currentAngle, calculatedSideAcctual);
    }
  }
}

// callback function to interpret lidar pings;
// look for blockage ahead, clearance to right, and clearance slightly ahead on
// right
// use computed values for wall following
void laserCallback(const sensor_msgs::LaserScan& laser_scan) {
  //! First Call
  if (centerSideIndex < 0) {
    // for first message received, set up the desired index of LIDAR range to
    // eval
    angle_min_ = laser_scan.angle_min;
    angle_max_ = laser_scan.angle_max;
    angle_increment_ = laser_scan.angle_increment;
    range_min_ = laser_scan.range_min;
    range_max_ = laser_scan.range_max;

    //! Index calculation
    //*Total Laser Points
    totalLaserPoints = (int)((angle_max_ - angle_min_) / angle_increment_);
    ROS_INFO("TotalLaserPoints is: %i", totalLaserPoints);
    //*Center Point:
    centerSideIndex = (int)((0.0 - angle_min_) / angle_increment_);
    ROS_INFO("Calculated Center Index is: %i", centerSideIndex);
    //*Sharp Right and Sharp Left
    float rightAngle = angle_max_ - (3.1415 / 2);
    float leftAngle = angle_min_ + (3.1415 / 2);
    rightSideIndex = (int)(rightAngle / angle_increment_);
    leftSideIndex = totalLaserPoints + (int)(leftAngle / angle_increment_);
    ROS_INFO("Calculated: Right Index is: %i Left Index is: %i", rightSideIndex,
             leftSideIndex);
    //*Top Right and Top Left Calculation
    topRightIndex = centerSideIndex - ((int)(fovAngle / angle_increment_));
    topLeftIndex = centerSideIndex + ((int)(fovAngle / angle_increment_));
    ROS_INFO("Calculated: TOP Right Index is: %i TOP Left Index is: %i",
             topRightIndex, topLeftIndex);
  }
  laserAwareness(laser_scan);
  /*
  ROS_INFO("forwardClearanceList");
  for(int i=0; i<forwardClearanceList.size(); ++i)
      ROS_INFO_STREAM("i: "<<i << " Value: "<< forwardClearanceList[i] << ' ');
  ROS_INFO("wallClearanceList");
  for(int i=0; i<wallClearanceList.size(); ++i)
      ROS_INFO_STREAM("i: "<<i << " Value: "<< wallClearanceList[i] << ' ');
  */
  wall_steer_factor = calculatStandardDev(wallClearanceList);
  ROS_INFO("Current STD for wall is: %f", wall_steer_factor);
  wall_distance_factor = calculateAverage(wallClearanceList);
  ROS_INFO("Current average for wall is: %f", wall_steer_factor);

}

int wallLengthCalculation(float maxSweepAngle, float minSweepAngle) {
  return 0;
}

// node to send Twist commands to the Simple 2-Dimensional Robot Simulator via
// cmd_vel
int main(int argc, char** argv) {
  ros::init(argc, argv, "commander");
  ros::NodeHandle
      n;  // two lines to create a publisher object that can talk to ROS
  ros::Publisher twist_commander =
      n.advertise<geometry_msgs::Twist>("/robot0/cmd_vel", 1);
  ros::Subscriber lidar_subscriber =
      n.subscribe("robot0/laser_0", 1, laserCallback);
  ros::Subscriber odom_subscriber = n.subscribe("robot0/odom", 1, odomCallback);

  geometry_msgs::Twist twist_cmd;  // this is the message type required to send
                                   // twist commands to STDR
  // start with all zeros in the command message; should be the case by default,
  // but just to be safe..
  twist_cmd.linear.x = 0.0;
  twist_cmd.linear.y = 0.0;
  twist_cmd.linear.z = 0.0;
  twist_cmd.angular.x = 0.0;
  twist_cmd.angular.y = 0.0;
  twist_cmd.angular.z = 0.0;
  double phi_cmd = 0.0;

  ros::Rate loop_timer(1 / SAMPLE_DT);  // create a ros object from the ros
                                        // “Rate” class; set 100Hz rate
  double timer = 0.0;
  // start sending some zero-velocity commands, just to warm up communications
  // with STDR
  for (int i = 0; i < 10; i++) {
    twist_commander.publish(twist_cmd);
    ros::spinOnce();
    loop_timer.sleep();
  }
  while (centerSideIndex < 0) {
    ros::spinOnce();
    loop_timer.sleep();
  }
  // make sure have feedback from odom
  while (g_phi < -50) {
    ros::spinOnce();
    loop_timer.sleep();
  }
  phi_cmd = g_phi;

  //! WALL-FOLLOWING ALGORITHM:
  ROS_INFO("starting wall following algorithm");
  while (ros::ok()) {
    switch (state) {
      case dead_reckoning: {
        //* Move!
        ROS_INFO("CASE: dead_reckoning START, robot moving forward %d seconds.",
                 dead_reckoning_timer);
        twist_cmd.linear.x = LINEAR_SPEED;
        twist_cmd.angular.z = 0.0;
        twist_commander.publish(twist_cmd);
        ros::Duration(dead_reckoning_timer).sleep();
        twist_cmd.linear.x = 0.0;
        twist_cmd.angular.z = 0.0;
        twist_commander.publish(twist_cmd);
        ROS_INFO("CASE: dead_reckoning END, handover to measuring!");
        transitionState(measuring);
        break;
      }
      case measuring: {
        int result = -1;
        ROS_INFO("CASE: measuring, Initial Checkup");

        if (wall_steer_factor > 0.05) {
          result = 4;
        } else {
          result = 1;
          dead_reckoning_timer = tan(maxSawAngle)*wall_distance_factor/LINEAR_SPEED;
        }


        ROS_INFO("Measuring Result code: %i", result);
        //* Internal switching
        if (result == -1) {
          // ROS_ERROR("CASE: measuring result come unexpected!");
        } else if (result == 1) {
          ROS_INFO("CASE: measuring, Switching to dead_reckoning");
          transitionState(dead_reckoning);
        } else if (result == 2) {
          ROS_INFO(
              "CASE: measuring, Front Wall limitation experienced, Switching "
              "to corner");
          transitionState(corner);
        } else if (result == 3) {
          ROS_INFO(
              "CASE: measuring, No More Wall Found, Switching to finding wall");
          transitionState(finding);
        } else if (result == 4) {
          ROS_INFO("CASE: measuring, Need fine tuning trajectory");
          transitionState(correcting);
        }
        break;
      }
      case correcting: {
        while (wall_steer_factor > 0.05) {
          ROS_INFO("CASE: correcting...");
          twist_cmd.linear.x = 0.0;
          twist_cmd.angular.z = 0.5;
          twist_commander.publish(twist_cmd);
          for (int i = 0; i < 10; i++) {
            twist_commander.publish(twist_cmd);
            ros::spinOnce();
            loop_timer.sleep();
          }
        }
        twist_cmd.linear.x = 0.0;
        twist_cmd.angular.z = 0.0;
        twist_commander.publish(twist_cmd);
        ROS_INFO("CASE: adjusting END, handover to measuring!");
        transitionState(measuring);
        break;
      }
    }

    ros::spinOnce();
  }  // loop forever
}