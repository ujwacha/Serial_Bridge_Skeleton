#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3.hpp"
//#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include <chrono>
#include <functional>
#include <memory>
#include <rclcpp/executors.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/subscription_base.hpp>
#include <rclcpp/timer.hpp>

#include <rclcpp/utilities.hpp>
// #include <string>
#include "serial_class.hpp"

#include "rclcpp_components/register_node_macro.hpp"
namespace Serial_Bridge_Skeleton {
class Sayer : public rclcpp::Node {

  int times = 0;

public:
  Sayer(const rclcpp::NodeOptions &options)
      : rclcpp::Node("NodeName", options), s("/dev/ttyACM0") {

    const rclcpp::QoS currentqol = rclcpp::QoS(10).best_effort();

    // currentqol.best_effort();

    publ1 =
        this->create_publisher<geometry_msgs::msg::Twist>("publ1", currentqol);
    publ2 = this->create_publisher<geometry_msgs::msg::Vector3>("publ2",
                                                                currentqol);
    publ3 =
        this->create_publisher<geometry_msgs::msg::Twist>("publ3", currentqol);
    boolpubl = this->create_publisher<std_msgs::msg::Bool>("boolpublisher",
                                                           currentqol);

    subscription1 = this->create_subscription<geometry_msgs::msg::Twist>(
        "sub1", currentqol,
        std::bind(&Sayer::sub1fun, this, std::placeholders::_1));

    subscription2 = this->create_subscription<geometry_msgs::msg::Vector3>(
        "sub2", currentqol,
        std::bind(&Sayer::sub2fun, this, std::placeholders::_1));

    subscription3 = this->create_subscription<geometry_msgs::msg::Twist>(
        "sub3", currentqol,
        std::bind(&Sayer::sub3fun, this, std::placeholders::_1));

    boolsub = this->create_subscription<std_msgs::msg::Bool>(
        "boolsub", currentqol,
        std::bind(&Sayer::boolsubfun, this, std::placeholders::_1));

    timer = this->create_wall_timer(std::chrono::milliseconds(1),
                                    std::bind(&Sayer::get_data, this));

    sendtimer = this->create_wall_timer(std::chrono::milliseconds(10),
                                    std::bind(&Sayer::send_data, this));


    
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
        RCLCPP_ERROR(this->get_logger(), "Some Random Error Thrown, Code: %i",
                     e);
      }
      return;
    }

    publish_data(c);
  }

  void publish_data(CommunicationData c) {

    geometry_msgs::msg::Twist message1 = geometry_msgs::msg::Twist();
    geometry_msgs::msg::Vector3 message2 = geometry_msgs::msg::Vector3();
    geometry_msgs::msg::Twist message3 = geometry_msgs::msg::Twist();
    std_msgs::msg::Bool boolmsg = std_msgs::msg::Bool();

    message1.linear.x = c.x;
    message1.linear.y = c.y;
    message1.angular.z = c.omega; // parse this later to trurn to radians

    message2.x = c.ax;
    message2.y = c.ay;
    message2.z = c.az;

    message3.linear.z = c.zz;
    message3.angular.z = c.theta; // parse later to turn to radians

    boolmsg.data = (c.on_off == 0) ? false : true;

    // auto message = std_msgs::msg::String();
    // message.data = "Hello";
    times++;
    if (times == 10000) {
      times = 0;
      RCLCPP_INFO(this->get_logger(), "sent 10000 messages");
    }

    publ1->publish(message1);
    publ2->publish(message2);
    publ3->publish(message3);

    boolpubl->publish(boolmsg);
  }

  void sub1fun(const geometry_msgs::msg::Twist::UniquePtr msg) {
    glosend.x = msg->linear.x;
    glosend.y = msg->linear.y;
    glosend.omega = msg->angular.z;

    edited++;
  }

  void sub2fun(const geometry_msgs::msg::Vector3::UniquePtr msg) {
    glosend.x = msg->x;
    glosend.y = msg->y;
    glosend.omega = msg->z;

    edited++;
  }

  void sub3fun(const geometry_msgs::msg::Twist::UniquePtr msg) {
    glosend.zz = msg->linear.z;
    glosend.theta = msg->angular.z;

    edited++;
  }

  void boolsubfun(const std_msgs::msg::Bool::UniquePtr msg) {
    glosend.on_off = msg->data;

    edited++;
  };

  void send_data() {

    // can be used if we want to make sure data is different
    // if (edited < 4) return;
    
    edited = 0;

    make_sendable_with_metadata(glosend);
    try {
      s.attempt_send_probably_blocking(glosend); // it will always return 1 as it is turned into sendable data already
    } catch (int i) {
      switch (i) {
      case -1:
	RCLCPP_ERROR(this->get_logger(), "Couldn't Send Data");
	break;
      default:
	RCLCPP_ERROR(this->get_logger(), "Unknown Error Occoured While Sending Data");
	break;
      }
    }
  }

private:
  BridgeClass s;
  // A bunch of Publishers
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publ1;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr publ2;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publ3;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr boolpubl;

  // A bunch of Subescribers
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription1;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr subscription2;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription3;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscriptionlol;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr boolsub;


  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr anotherstupidbool; 
  


  
  // Timer
  rclcpp::TimerBase::SharedPtr timer;

  // Send Timer
  rclcpp::TimerBase::SharedPtr sendtimer;

  // Global communicationdata to send to
  int edited = 0;
  CommunicationData glosend;
};

} // namespace Serial_Bridge_Skeleton

// RCLCPP_COMPONENTS_REGISTER_NODE(Serial_Bridge_Skeleton::Sayer);

int main(int argc, char *argv[]) {

  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options = rclcpp::NodeOptions();

  rclcpp::spin(std::make_shared<Serial_Bridge_Skeleton::Sayer>(options));

  rclcpp::shutdown();

 
 
  return 0;
}
