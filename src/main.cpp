#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <rclcpp/executors.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/timer.hpp>
#include "std_msgs/msg/string.hpp"
#include <memory>
#include "geometry_msgs/msg/twist.hpp"

#include <rclcpp/utilities.hpp>
//#include <string>
#include "serial_class.hpp"


#include "rclcpp_components/register_node_macro.hpp"
namespace Serial_Bridge_Skeleton {
  class Sayer : public rclcpp::Node {
    
    int times = 0;
    public:
    Sayer(const rclcpp::NodeOptions & options): rclcpp::Node("NodeName", options), s("/dev/ttyACM0") {
      publ = this->create_publisher<geometry_msgs::msg::Twist>("publ", 10);
      timer = this->create_wall_timer(std::chrono::microseconds(100),
				      std::bind(&Sayer::test, this));

    }

    void test() {

      CommunicationData c = s.get_data();

      auto message = geometry_msgs::msg::Twist();

      message.linear.x = c.x;
      message.linear.y = c.y;
      message.linear.z = c.zz;

      message.angular.z = c.omega;


      
      // auto message = std_msgs::msg::String();
      // message.data = "Hello";
      times++;
      if (times == 10000){
	times = 0;
	RCLCPP_INFO(this->get_logger(), "sent 10000 messages");
      }

      publ->publish(message);
    }

    private:
    SerialBridge s;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publ;
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