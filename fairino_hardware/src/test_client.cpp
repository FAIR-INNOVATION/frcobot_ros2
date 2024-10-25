#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"
#include "fairino_msgs/srv/remote_cmd_interface.hpp"
using namespace std;

class test_client : public rclcpp::Node
{
public:
    // 构造函数,有一个参数为节点名称
    test_client(std::string name) : Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "%s节点已经启动.", name.c_str());
        //创建客户端
        test_client_ = this->create_client<fairino_msgs::srv::RemoteCmdInterface>("fairino_remote_command_service");
    }
    //发送请求函数
    void send_request(const std::string str){
        RCLCPP_INFO(this->get_logger(), "准备发送数据");
        // 1.等待服务端上线
        while (!test_client_->wait_for_service(std::chrono::seconds(1))) {
        //等待时检测rclcpp的状态
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "等待服务的过程中被打断...");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "等待服务端上线中");
        }
        // 2.构造请求数据
        auto request = std::make_shared<fairino_msgs::srv::RemoteCmdInterface::Request>();
        request->cmd_str = "SetSpeed(20)";
        // 3.发送异步请求，然后等待返回，返回时调用返回结果处理函数
        test_client_->async_send_request(request,std::bind(&test_client::result_callback, this,std::placeholders::_1));
    }

private:
    //返回结果处理函数
    void result_callback(rclcpp::Client<fairino_msgs::srv::RemoteCmdInterface>::SharedFuture result_future){
        //获取返回结果
        auto response = result_future.get();
        std::string res_str = response->cmd_res;
        std::cout << "回复信息为" + res_str << std::endl; 
    }
    //声明客户端
    rclcpp::Client<fairino_msgs::srv::RemoteCmdInterface>::SharedPtr test_client_;
};

int main(int argc, char **argv)
{
    //初始化rclcpp
    rclcpp::init(argc, argv);
    //创建对应节点对象
    auto node = std::make_shared<test_client>("test_server");
    //发送请求
    auto msg = "SetSpeed(20)";
    node->send_request(msg);
    //运行节点，并检测退出信号
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
