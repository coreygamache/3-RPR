//encoder motor control node
#include <Encoder.hpp>
#include <ros/ros.h>
#include <rpr_msgs/Encoder.h>
#include <string.h>

int main(int argc, char **argv)
{

  //send notification that node is launching
  ROS_INFO("[NODE LAUNCH]: starting encoder_node");

  //initialize node and create node handler
  ros::init(argc, argv, "encoder_node");
  ros::NodeHandle node_private("~");

  //define encoder_number via passed argument
  char *encoder_number;
  if (argc == 2)
  {
    encoder_number = argv[1];
  }
  else
  {
    ROS_ERROR("[ERROR] encoder_node received incorrect number of arguments");
    ROS_BREAK();
  }

  //set encoder_path for retrieving parameters from parameter server for this node
  std::string encoder_name = "encoder_";
  encoder_name = encoder_name + boost::lexical_cast<std::string>(encoder_number);
  std::string encoder_path = "/encoder/" + encoder_name;
  //ROS_INFO("encoder path: %s", encoder_path.c_str()); //display path (for testing)

  //retrieve encoder slave select pin from parameter server
  int ss_pin;
  if (!node_private.getParam(encoder_path + "/ss_pin", ss_pin))
  {
    ROS_ERROR("[ERROR] slave select pin not defined in config file: rpr_sensors/config/sensors.yaml, path: %s", encoder_path.c_str());
    ROS_BREAK();
  }
  ROS_INFO("slave select pin: %d", ss_pin); //display slave select pin (for testing)

  //retrieve encoder counts per revolution from parameter server
  float counts_per_rev;
  if (!node_private.getParam(encoder_path + "/counts_per_rev", counts_per_rev))
  {
    ROS_ERROR("[ERROR] counts per revolution not defined in config file: rpr_sensors/config/sensors.yaml, path: %s", encoder_path.c_str());
    ROS_BREAK();
  }

  //retrieve encoder refresh rate from parameter server
  float refresh_rate;
  if (!node_private.getParam(encoder_path + "/refresh_rate", refresh_rate))
  {
    ROS_ERROR("[ERROR] refresh rate not defined in config file: rpr_sensors/config/sensors.yaml, path: %s", encoder_path.c_str());
    ROS_BREAK();
  }

  //create Encoder type object using defined slave select pin and counts per revolution from parameter server
  Encoder encoder(counts_per_rev, ss_pin);

  //create rpr_msgs/Encoder type message to publish encoder data
  rpr_msgs::Encoder encoder_msg;

  //set encoder frame id and number
  encoder_msg.header.frame_id = "0";
  encoder_msg.encoder_number = int(encoder_number[0]) - int('0');

  //create publisher to publish encoder message with buffer size 10, and latch set to false
  ros::Publisher encoder_pub = node_private.advertise<rpr_msgs::Encoder>(encoder_name, 10, false);

  //set refresh rate of ROS loop to defined refresh rate from parameter server
  ros::Rate loop_rate(refresh_rate);

  while (ros::ok())
  {

    //set message timestamp to time of encoder reading
    encoder_msg.header.stamp = ros::Time::now();

    //get current encoder position set message data [deg]
    encoder_msg.position = encoder.readPosition() / counts_per_rev * 360;

    //publish encoder message
    encoder_pub.publish(encoder_msg);

    //process callback function calls
    ros::spinOnce();

    //sleep until next sensor reading
    loop_rate.sleep();

  }

  return 0;
}
