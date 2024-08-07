#include "srv_test_client.hpp"

srv_test_node::srv_test_node(const std::string node_name):Node(node_name){
    using namespace std::chrono_literals;
    static int count = 0;
    _client_ptr = this->create_client<fairino_msgs::srv::RemoteCmdInterface>("FR_ROS_API_service");
    auto locktimer_callback = [&]()->void{
        static int counter = 0;
        auto req = std::make_unique<fairino_msgs::srv::RemoteCmdInterface::Request>();
        if(counter == 0){
            write_command(std::string("SetSpeed(5)"));
        }else if(counter == 50){
            write_command(std::string("SetSpeed(10)"));
        }else if(counter == 100){
            write_command(std::string("SetSpeed(15)"));
            this->_locktimer->cancel();
        }
        counter++;
    };
    auto watchdog_callback = [&]()->void{
        this->_client_ptr->prune_pending_requests();//清理所有等待信息
    };
    _locktimer = this->create_wall_timer(10ms,locktimer_callback);
    _watchdog = this->create_wall_timer(5s,watchdog_callback);//5s检查一次队列,如果有等待消息则清除
    if(!_client_ptr->wait_for_service(1s)){
        if(!rclcpp::ok()){
            std::cout << "ROS通讯错误:ROS非正常运行" << std::endl;
            _locktimer->cancel();
        }else{
            std::cout << "ROS通讯等待超时:没有检测到service启动" << std::endl;
                _locktimer->cancel();
        }
    }
}

srv_test_node::~srv_test_node(){   

}

void srv_test_node::_get_response_callback(rclcpp::Client<fairino_msgs::srv::RemoteCmdInterface>::SharedFutureWithRequest future){
    auto request_response_pair = future.get();
    std::string req_str = request_response_pair.first->cmd_str;
    std::string res_str = request_response_pair.second->cmd_res;
    std::cout << "命令" + req_str + "的回复信息为" + res_str << std::endl; 
}

int srv_test_node::write_command(const std::string str){  
    auto req = std::make_unique<fairino_msgs::srv::RemoteCmdInterface::Request>();
    req->cmd_str = str;
    //auto func_ptr = [&](rclcpp::Client<frhal_msgs::srv::ROSCmdInterface>::SharedFutureWithRequest future){};
    //auto back_t = _client_ptr->async_send_request(std::move(req),func_ptr);
    auto back_t = _client_ptr->async_send_request(std::move(req),std::bind(&srv_test_node::_get_response_callback,this,std::placeholders::_1));
    return 0;
}