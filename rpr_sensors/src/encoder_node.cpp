//encoder motor control node
#include <Encoder.hpp>
#include <ros/ros.h>
#include <string.h>

int main(int argc, char **argv)
{

  //send notification that node is launching
  ROS_INFO("[NODE LAUNCH]: starting encoder_node");

  //initialize node and create node handler
  ros::init(argc, argv, "encoder_node");
  ros::NodeHandle node_private("~");

  //define motor_number via parameter
  /*int motor_number;
  if (!node_private.getParam("motor_number", motor_number))
  {
    ROS_ERROR("line sensor refresh rate not defined in config file: sd_sensors/config/sensors.yaml");
    ROS_BREAK();
  }*/

  //define motor_number via passed argument
  char *encoder_number;
  if (argc == 2)
  {
    encoder_number = argv[1];
  }
  else
  {
    ROS_ERROR("incorrect number of arguments");
    ROS_BREAK();
  }

  //ROS_INFO("%d", motor_number);
  std::string encoder_path = "/encoder/encoder_";
  encoder_path = encoder_path + boost::lexical_cast<std::string>(encoder_number);
  ROS_INFO("encoder path: %s", encoder_path.c_str());


  int ssPin;
  if (!node_private.getParam(encoder_path + "/ssPin", ssPin))
  {
    ROS_ERROR("slave select pin gone bad");
    ROS_BREAK();
  }
  ROS_INFO("slave select pin: %d", ssPin);
/*
  //get output pin from parameter server
  int output_pin;
  if (!node_private.getParam("ball_sensor/output_pin", output_pin))
  {
    ROS_ERROR("ball sensor output pin not defined in config file: sd_sensors/config/sensors.yaml");
    ROS_BREAK();
  }

  //create BallSensor type object using defined outpin pin from parameter server
  BallSensor sensor(output_pin);

  //create sd_msgs/Ball type message to publish ball (proximity) sensor data
  sd_msgs::Ball ball_msg;

  //set ball sensor frame id
  ball_msg.header.frame_id = "ball_sensor_link";

  //----------------------------------------------------------

  //create publisher to publish ball sensor message with buffer size 10, and latch set to false
  ros::Publisher ball_sensor_pub = node_private.advertise<sd_msgs::Ball>("ball_sensor", 10, false);

  //get refresh rate of sensor in hertz from parameter server
  float refresh_rate;
  if (!node_private.getParam("ball_sensor/refresh_rate", refresh_rate))
  {
    ROS_ERROR("ball sensor refresh rate not defined in config file: sd_sensors/config/sensors.yaml");
    ROS_BREAK();
  }
*/
  //set refresh rate of ROS loop to defined refresh rate from parameter server
  float refresh_rate = 1;
  ros::Rate loop_rate(refresh_rate);

  while (ros::ok())
  {
/*
    //set time of current ball sensor reading
    ball_msg.header.stamp = ros::Time::now();

    //check whether a ball is currently detected by sensor and set message data
    ball_msg.ball_detected = sensor.ballDetected();

    //publish ball sensor message
    ball_sensor_pub.publish(ball_msg);

    //spin once because ROS
    ros::spinOnce();

    //sleep until next sensor reading
    loop_rate.sleep();*/

  }

  return 0;
}
