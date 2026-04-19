#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <chrono>
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include <cstddef>
#include <geometry_msgs/msg/detail/transform_stamped__struct.hpp>
#include <memory>
#include <opencv2/imgcodecs.hpp>
#include <rclcpp/client.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/utilities.hpp>
#include <rclcpp_action/client.hpp>
#include <rclcpp_action/client_goal_handle.hpp>
#include <rclcpp_action/create_client.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/detail/image__struct.hpp>
#include <string>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <vector>
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "autopatrol_interfaces/srv/speech_text.hpp"
#include "sensor_msgs/msg/image.hpp"   
#include "cv_bridge/cv_bridge.h"        
#include "opencv2/opencv.hpp"            
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"  
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

using PoseWithCovarianceStamped = geometry_msgs::msg::PoseWithCovarianceStamped;
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using namespace std::chrono_literals;
using PoseStamped = geometry_msgs::msg::PoseStamped;
using SpeechText = autopatrol_interfaces::srv::SpeechText; 
using ImageMsg = sensor_msgs::msg::Image;
using TransformStamped = geometry_msgs::msg::TransformStamped;

class PatrolNode : public rclcpp::Node
{
public:
    PatrolNode(const std::string &node_name) : Node(node_name)
    {
        RCLCPP_INFO(this->get_logger(), "Patrol node initialized.");

        this->declare_parameter<std::vector<double>>("initial_point", {0.0, 0.0, 0.0});
        this->declare_parameter<std::vector<double>>("target_points", {0.0, 0.0, 0.0, 1.0, 2.0, 3.14});
        this->declare_parameter<std::string>("image_save_path", "");

        init_pose_publisher_ = this->create_publisher<PoseWithCovarianceStamped>("initialpose", 10);        // 发布给/initialpose
        nav_action_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");             // 创建action服务器
        
        speech_client_ = this->create_client<SpeechText>("speech_text");    // 创建语音合成客户端
        
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());              // 创建buffer
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this); // 获取当前位姿

        // 创建图像sub，获取最新图像
        image_subscription_ = this->create_subscription<ImageMsg>("/camera_sensor/image_raw", 10, 
            [this](const ImageMsg::SharedPtr msg)
            {
                this->latest_image_ = msg;
            });
    }

    // 一、多点巡航
    // 构建PoseStamped类型消息
    PoseStamped get_pose_stamped(const std::vector<double> &points)
    {
        PoseStamped pose;
        // 配置header   
        pose.header.stamp = this->get_clock()->now();
        pose.header.frame_id = "map";
        // 配置position
        pose.pose.position.x = points[0];
        pose.pose.position.y = points[1];
        // 从欧拉角转换为四元数
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, points[2]);
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();

        return pose;
    }

    // 获取目标点
    std::vector<std::vector<double>> get_target_points()
    {
        std::vector<std::vector<double>> waypoints;
        auto target_points_ = this->get_parameter("target_points").as_double_array();
        // 填充路点列表
        for (size_t i = 0; i + 2 < target_points_.size(); i += 3)
        {
            waypoints.push_back({target_points_[i], target_points_[i + 1], target_points_[i + 2]});
            // %zu: size_t类型
            RCLCPP_INFO(this->get_logger(), "获取目标点: %zu -> (x: %.2f, y: %.2f, yaw: %.2f)", waypoints.size() - 1, target_points_[i], target_points_[i + 1], target_points_[i + 2]);
        }

        return waypoints;
    }

    // 导航到指定位置
    void nav_to_pose(const PoseStamped &goal_pose)
    {
        // 确保action服务器可用
        if (!nav_action_client_->wait_for_action_server(10s)) 
        {
            RCLCPP_ERROR(this->get_logger(), "连接超时，导航 action server 不可用");
            return;
        }

        // 构建goal消息
        auto goal = NavigateToPose::Goal();
        goal.pose = goal_pose;

        // 1. 创建发送选项，设置feedback_callback以实时输出剩余时间和距离
        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.feedback_callback = 
        [this](rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr, const std::shared_ptr<const NavigateToPose::Feedback> feedback)
        {
            if (feedback)   // 保证feedback不为空
            {
                double remaining_distance = feedback->distance_remaining;

                // 提取并转换时间 (将 sec 和 nanosec 合并为总秒数)
                // 计算公式：秒 + 纳秒 / 1e9
                double remaining_time = feedback->estimated_time_remaining.sec + feedback->estimated_time_remaining.nanosec * 1e-9;

                // 3. 打印 (确保类型匹配)
                RCLCPP_INFO(this->get_logger(), "剩余时间: %.2f seconds, 剩余距离: %.2f meters", remaining_time, remaining_distance);
            }
        };

        // 发送导航目标消息
        auto goal_handle_future = nav_action_client_->async_send_goal(goal, send_goal_options);
        // 此处采用同步方式处理response和result，确保导航到点后再继续执行后续逻辑

        // 2. 处理response
        // 2.1 先检查是否成功发送目标，服务器是否响应了目标请求
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), goal_handle_future) != rclcpp::FutureReturnCode::SUCCESS) 
        {
            RCLCPP_ERROR(this->get_logger(), "服务器未响应");
            return;
        }

        // 2.2 再获取目标句柄，检测服务器是否接受了目标请求
        auto goal_handle = goal_handle_future.get();
        if (!goal_handle) 
        {
            RCLCPP_ERROR(this->get_logger(), "服务器拒绝了目标");
            return;
        }

        // 3. 处理result
        // 3.1 先等待导航完成并获取结果，检测服务器是否成功执行了导航任务
        auto result_future = nav_action_client_->async_get_result(goal_handle);     // 获取未来结果
        auto start_time = this->now();                                        // 设置开始时间
        const auto timeout = 300s;                           // 设置超时时间为5分钟

        // 3.2 再循环等待结果，检测是否超时
        while (rclcpp::ok()) 
        {
            auto status = rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future, 100ms);
            // 检测服务器result，若服务器成功相应则跳出循环继续处理结果
            if (status == rclcpp::FutureReturnCode::SUCCESS) {
                break;
            }

            // 若超时，则取消导航任务并跳出循环
            if ((this->now() - start_time) > timeout) {
                RCLCPP_WARN(this->get_logger(), "导航超时，正在取消任务");
                auto cancel_future = nav_action_client_->async_cancel_goal(goal_handle);
                rclcpp::spin_until_future_complete(this->get_node_base_interface(), cancel_future);
                break;
            }
        }

        // 3.3 最后检查result是否就绪，若就绪则处理导航结果，否则输出错误日志
        if (result_future.wait_for(0s) == std::future_status::ready)        // 检查result是否就绪
        {
            const auto result = result_future.get();       // 获取result
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED) { RCLCPP_INFO(this->get_logger(), "导航结果：成功"); } 
            else if (result.code == rclcpp_action::ResultCode::CANCELED) { RCLCPP_WARN(this->get_logger(), "导航结果：被取消"); }
            else if (result.code == rclcpp_action::ResultCode::ABORTED) { RCLCPP_ERROR(this->get_logger(), "导航结果：被中止"); } 
            else { RCLCPP_ERROR(this->get_logger(), "导航结果：失败"); }
        } 
        else { RCLCPP_ERROR(this->get_logger(), "导航结果未就绪"); }
    }  
    
    // 二、语音播报
    // 发送文本到语音合成服务
    void speech_text(std::string text)
    {
        RCLCPP_INFO(this->get_logger(), "准备调用语音服务: %s", text.c_str());
        
        // 检测服务器可用
        if (!speech_client_->wait_for_service(5s)) {
            RCLCPP_ERROR(this->get_logger(), "语音服务不可用");
            return;
        }
        
        // 构建request消息
        auto request = std::make_shared<SpeechText::Request>();
        request->text = text;
        
        // 发送request并等待结果
        auto future = speech_client_->async_send_request(request);
        
        // 超时时间10 秒
        auto status = rclcpp::spin_until_future_complete(this->get_node_base_interface(), future, 10s);
        
        if (status == rclcpp::FutureReturnCode::SUCCESS) 
        {
            auto response = future.get();
            if (response->result) { RCLCPP_INFO(this->get_logger(), "语音播报成功: %s", text.c_str()); } 
            else { RCLCPP_WARN(this->get_logger(), "语音播报失败: %s", text.c_str()); }
        } 
        else if (status == rclcpp::FutureReturnCode::TIMEOUT) { RCLCPP_ERROR(this->get_logger(), "语音服务调用超时: %s", text.c_str()); } 
        else { RCLCPP_ERROR(this->get_logger(), "语音服务调用失败: %s", text.c_str()); }
    }

    // 三、图像保存
    // 获取当前位姿
    TransformStamped get_current_pose()
    {
        while (rclcpp::ok())
        {
            try 
            {
                // 实例化tf对象
                auto tf = tf_buffer_->lookupTransform("odom", "base_footprint", tf2::TimePointZero, 1s); // 父 子 时间点 超时时间
                auto transform = tf.transform;  // 获取tf数据
                // 四元数转换欧拉角
                tf2::Quaternion q(transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w);
                double roll, pitch, yaw;
                tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
                RCLCPP_INFO(this->get_logger(), "平移: (x: %.2f, y: %.2f, z: %.2f), 欧拉角: (roll: %.2f, pitch: %.2f, yaw: %.2f)", 
                            transform.translation.x, transform.translation.y, transform.translation.z,
                            roll, pitch, yaw);
                return tf; 
                
            } 
            catch (const tf2::TransformException& e) 
            {
                RCLCPP_WARN(this->get_logger(), "TF获取失败: %s", e.what());
                rclcpp::sleep_for(100ms);       // 等待100ms
            }
        }

        return TransformStamped();
    }

    // 获取当前位姿并记录图像
    void record_image()
    {
        // 检测图像是否存在
        if (!latest_image_)
        {
            RCLCPP_ERROR(this->get_logger(), "最新图像不存在，程序退出");
            return;
        }

        // 组织文件名并保存图像
        try
        {
            // 获取当前位姿
            auto tf = get_current_pose();
            double x = tf.transform.translation.x;
            double y = tf.transform.translation.y;

            // 将ROS图像转换为OpenCV图像
            auto cv_ptr_ = cv_bridge::toCvCopy(latest_image_, sensor_msgs::image_encodings::RGB8);

            // 构建文件名
            this->image_save_path_ = this->get_parameter("image_save_path").as_string();    // 获取默认保存路径
            std::string filename = image_save_path_ + "image_" + std::to_string(x).substr(0, 5) + "_" + std::to_string(y).substr(0, 5) + ".png";

            // 保存图像
            cv::imwrite(filename, cv_ptr_->image);
            RCLCPP_INFO(this->get_logger(), "图像已保存: %s (位置: %.2f, %.2f)", filename.c_str(), x, y);
        }
        catch (const cv_bridge::Exception& e) { RCLCPP_ERROR(this->get_logger(), "图像转换失败: %s", e.what()); } 
        catch (const std::exception& e) { RCLCPP_ERROR(this->get_logger(), "保存图像失败: %s", e.what()); }
    }


private:
    rclcpp::Publisher<PoseWithCovarianceStamped>::SharedPtr init_pose_publisher_;   // 声明初始化位姿publihser
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_action_client_;            // 声明导航action客户端
    rclcpp::Client<SpeechText>::SharedPtr speech_client_;                           // 声明语音合成客户端
    rclcpp::Subscription<ImageMsg>::SharedPtr image_subscription_;                  // 声明图像subscription
    ImageMsg::SharedPtr latest_image_;                                              // 声明最新图像成员变量
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;                                    // 声明tf_buffer_
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;                       // 声明tf listener_
    std::string image_save_path_;                                                   // 声明图像保存路径

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto patrol_node = std::make_shared<PatrolNode>("patrol_node");

    // 初始化机器人位姿
    patrol_node->speech_text("正在初始化机器人位姿");
    patrol_node->speech_text("机器人位姿初始化完成");
    while (rclcpp::ok()) 
    {
        // 获取目标点
        auto target_points = patrol_node->get_target_points();
        // 导航到目标点
        for (const auto &point : target_points) {
            auto goal_pose = patrol_node->get_pose_stamped(point);
            patrol_node->speech_text("准备前往目标点" + std::to_string(point[0]) + "," + std::to_string(point[1]));
            patrol_node->nav_to_pose(goal_pose);
            patrol_node->speech_text("已到达目标点" + std::to_string(point[0]) + "," + std::to_string(point[1]) + ",准备记录图像");
            patrol_node->record_image();
            patrol_node->speech_text("图像记录完成");
        }
    }

    rclcpp::shutdown();
    return 0;
}