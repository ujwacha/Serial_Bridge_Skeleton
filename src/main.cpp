#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <rclcpp/executors.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/timer.hpp>
#include "std_msgs/msg/string.hpp"
#include <memory>
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"

#include <rclcpp/utilities.hpp>
//#include <string>
#include "serial_class.hpp"


#include "rclcpp_components/register_node_macro.hpp"
namespace Serial_Bridge_Skeleton {
  class Sayer : public rclcpp::Node {
    
    int times = 0;
    public:
    Sayer(const rclcpp::NodeOptions & options): rclcpp::Node("NodeName", options), s("/dev/ttyACM0") {


      rclcpp::QoS currentqol = rclcpp::QoS(10);
      currentqol.best_effort();

      // currentqol.best_effort();

      publ1 = this->create_publisher<geometry_msgs::msg::Twist>("publ1", currentqol);
      publ2 = this->create_publisher<geometry_msgs::msg::Twist>("publ2", currentqol);
      publ3 = this->create_publisher<geometry_msgs::msg::Twist>("publ3", currentqol);
      boolpubl = this->create_publisher<std_msgs::msg::Bool>("boolpublisher", currentqol);
      timer = this->create_wall_timer(std::chrono::microseconds(100),
				      std::bind(&Sayer::get_data, this));

    }

    void get_data() {
      CommunicationData c;
      try {
	c = s.attempt_get_non_blocking();
      } catch (int e) {
	switch (e) {
	case -1:
	  RCLCPP_ERROR(this->get_logger(), "Start Byte Search Ongoing");
	  break;

	case -2:
	  RCLCPP_ERROR(this->get_logger(), "Corrupt Data Found");
	  break;

	default:
	  RCLCPP_ERROR(this->get_logger(), "Some Random Error Thrown, Code: %i", e);
	    }
	return;
      }

      publish_data(c);
      
    }

    void publish_data(CommunicationData c) {
 
      geometry_msgs::msg::Twist message1 = geometry_msgs::msg::Twist();
      geometry_msgs::msg::Twist message2 = geometry_msgs::msg::Twist();
      geometry_msgs::msg::Twist message3 = geometry_msgs::msg::Twist();
      std_msgs::msg::Bool boolmsg = std_msgs::msg::Bool();
      

      message1.linear.x = c.x;
      message1.linear.y = c.y;
      message1.angular.z = c.omega; // parse this later to trurn to radians

      message2.linear.x = c.ax;
      message2.linear.y = c.ay;
      message2.linear.z = c.az;

      message3.linear.z = c.zz;
      message3.angular.z = c.theta; // parse later to turn to radians

      boolmsg.data = (c.on_off == 0)? false : true ;



      // auto message = std_msgs::msg::String();
      // message.data = "Hello";
      times++;
      if (times == 10000){
	times = 0;
	RCLCPP_INFO(this->get_logger(), "sent 10000 messages");
      }

      publ1->publish(message1);
      publ2->publish(message2);
      publ3->publish(message3);

      boolpubl->publish(boolmsg);
      
    }

    private:
    BridgeClass s;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publ1;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publ2;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publ3;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr boolpubl;
    rclcpp::TimerBase::SharedPtr timer;
  };

  } // namespace Serial_Node



// RCLCPP_COMPONENTS_REGISTER_NODE(Serial_Bridge_Skeleton::Sayer);

int main(int argc, char * argv[]) {

  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options = rclcpp::NodeOptions();


  rclcpp::spin(std::make_shared<Serial_Bridge_Skeleton::Sayer>(options));

  rclcpp::shutdown();

  return 0;
}  
