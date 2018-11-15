//encoder motor control node
#include <Motor.hpp>
#include <ros/ros.h>
#include <rpr_msgs/Encoder.h>
#include <rpr_msgs/MotorActuation.h>
#include <string.h>

//callback function called to process messages on encoder_(motor_number) topic
void encoderCallback(const rpr_msgs::Encoder::ConstPtr& msg)
{

}

//callback function to process MotorActuation service requests to this node
bool motorActuationCallback(rpr_msgs::MotorActuation::Request &req, rpr_msgs::MotorActuation::Response &res)
{

}

int main(int argc, char **argv)
{

  //send notification that node is launching
  ROS_INFO("[NODE LAUNCH]: starting encoder_motor_node");

  //initialize node and create node handler
  ros::init(argc, argv, "encoder_motor_node");
  ros::NodeHandle node_private("~");

  //define motor_number via parameter
  /*int motor_number;
  if (!node_private.getParam("motor_number", motor_number))
  {
    ROS_ERROR("line sensor refresh rate not defined in config file: sd_sensors/config/sensors.yaml");
    ROS_BREAK();
  }*/

  //define motor_number via passed argument
  char *motor_number;
  if (argc == 2)
  {
    motor_number = argv[1];
  }
  else
  {
    ROS_ERROR("incorrect number of arguments");
    ROS_BREAK();
  }

  //set motor path for retrieving parameters from parameter server for this node
  std::string motor_path = "/motor/motor_";
  motor_path = motor_path + boost::lexical_cast<std::string>(motor_number);
  ROS_INFO("motor path: %s", motor_path.c_str());

  //set encoder path for subscribing to this motor's encoder topic
  std::string encoder_topic = "/encoder/encoder_" + boost::lexical_cast<std::string>(motor_number);
  ROS_INFO("encoder topic: %s", encoder_topic.c_str());

  //retrieve motor alarm pin from parameter server
  int alarm_pin;
  if (!node_private.getParam(motor_path + "/alarm_pin", alarm_pin))
  {
    ROS_ERROR("[ERROR] alarm pin not defined in config file: rpr_hardware_interface/config/sd_hardware_interface.yaml, path: %s", motor_path.c_str()");
    ROS_BREAK();
  }
  //ROS_INFO("alarm pin: %d", alarm_pin);

  //retrieve motor direction pin from parameter server
  int direction_pin;
  if (!node_private.getParam(motor_path + "/direction_pin", direction_pin))
  {
    ROS_ERROR("[ERROR] direction pin not defined in config file: rpr_hardware_interface/config/sd_hardware_interface.yaml, path: %s", motor_path.c_str()");
    ROS_BREAK();
  }
  //ROS_INFO("direction pin: %d", direction_pin);

  //retrieve motor enable pin from parameter server
  int enable_pin;
  if (!node_private.getParam(motor_path + "/enable_pin", enable_pin))
  {
    ROS_ERROR("[ERROR] enable pin not defined in config file: rpr_hardware_interface/config/sd_hardware_interface.yaml, path: %s", motor_path.c_str()");
    ROS_BREAK();
  }
  ROS_INFO("enable pin: %d", enable_pin);

  //retrieve motor max rpm from parameter server
  int max_rpm;
  if (!node_private.getParam(motor_path + "/max_rpm", max_rpm))
  {
    ROS_ERROR("[ERROR] max rpm not defined in config file: rpr_hardware_interface/config/sd_hardware_interface.yaml, path: %s", motor_path.c_str()");
    ROS_BREAK();
  }
  //ROS_INFO("max rpm: %d", max_rpm);

  //retrieve motor minimum high pulse width from parameter server
  int min_high_pulse_width;
  if (!node_private.getParam(motor_path + "/min_high_pulse_width", min_high_pulse_width))
  {
    ROS_ERROR("[ERROR] min high pulse width not defined in config file: rpr_hardware_interface/config/sd_hardware_interface.yaml, path: %s", motor_path.c_str()");
    ROS_BREAK();
  }
  //ROS_INFO("min high pulse width: %d", min_high_pulse_width);

  //retrieve motor minimum low pulse width from parameter server
  int min_low_pulse_width;
  if (!node_private.getParam(motor_path + "/min_low_pulse_width", min_low_pulse_width))
  {
    ROS_ERROR("[ERROR] min low pulse width not defined in config file: rpr_hardware_interface/config/sd_hardware_interface.yaml, path: %s", motor_path.c_str()");
    ROS_BREAK();
  }
  ROS_INFO("min low pulse width: %d", min_low_pulse_width);

  //retrieve motor pulse pin from parameter server
  int pulse_pin;
  if (!node_private.getParam(motor_path + "/pulse_pin", pulse_pin))
  {
    ROS_ERROR("[ERROR] pulse pin not defined in config file: rpr_hardware_interface/config/sd_hardware_interface.yaml, path: %s", motor_path.c_str()");
    ROS_BREAK();
  }
  ROS_INFO("pulse pin: %d", pulse_pin);

  //retrieve motor refresh rate from parameter server
  int refresh_rate;
  if (!node_private.getParam(motor_path + "/refresh_rate", refresh_rate))
  {
    ROS_ERROR("[ERROR] refresh rate not defined in config file: rpr_hardware_interface/config/sd_hardware_interface.yaml, path: %s", motor_path.c_str()");
    ROS_BREAK();
  }
  ROS_INFO("refresh rate: %d", refresh_rate);

  //create Motor type object using defined values from parameter server
  Motor motor(alarm_pin, direction_pin, enable_pin, pulse_pin, max_rpm, min_high_pulse_width, min_low_pulse_width);

  //create sunscriber to subscribe to drive motor messages message topic with queue size set to 1000
  ros::Subscriber encoder_sub = node_private.subscribe(encoder_topic, 1000, encoderCallback);

  //create service server to handle MotorActuation service requests to this node
  ros::ServiceServer motor_actuation_ss = node_private.advertiseService(motor_name + "_actuation", motorActuationCallback);

  //set refresh rate of ROS loop to defined refresh rate from parameter server
  ros::Rate loop_rate(refresh_rate);

  while (ros::ok())
  {

    //process callback function calls
    ros::spinOnce();

    //sleep until next sensor reading
    loop_rate.sleep();*/

  }

  return 0;
}
