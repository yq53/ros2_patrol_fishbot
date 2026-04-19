#include "rclcpp/rclcpp.hpp"
#include "autopatrol_interfaces/srv/speech_text.hpp"
#include "espeak-ng/speak_lib.h"
#include <memory>
#include <rclcpp/service.hpp>
#include <rclcpp/utilities.hpp>
#include <string>

using SpeechText = autopatrol_interfaces::srv::SpeechText;

class Speaker : public rclcpp::Node
{
public:
    Speaker(const std::string &node_name) : Node(node_name)
    {
        // 初始化espeak-ng
        int ret = espeak_Initialize(AUDIO_OUTPUT_PLAYBACK, 0, NULL, 0);
        if (ret < 0)    // 初始化检测
        {
            RCLCPP_ERROR(this->get_logger(), "espeak-ng初始化失败");
            return;
        }

        // 设置中文语音
        espeak_SetVoiceByName("zh");

        // 创建语音服务
        speech_service_ = this->create_service<SpeechText>("speech_text", 
            [this](const std::shared_ptr<SpeechText::Request> request, std::shared_ptr<SpeechText::Response> response)
            {
                RCLCPP_INFO(this->get_logger(), "收到服务请求: %s", request->text.c_str());
                
                std::string cmd = "espeak-ng -v zh \"" + request->text + "\"";
                int ret = system(cmd.c_str());
                
                response->result = (ret == 0);
                if (response->result) { RCLCPP_INFO(this->get_logger(), "朗读完成"); } 
                else { RCLCPP_ERROR(this->get_logger(), "朗读失败"); }
            });

        RCLCPP_INFO(this->get_logger(), "Speaker node启动");
    }

    ~Speaker()
    {
        // 清理资源
        espeak_Terminate();
    }


private:
    rclcpp::Service<SpeechText>::SharedPtr speech_service_;
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto speaker_node = std::make_shared<Speaker>("speaker_node");
    rclcpp::spin(speaker_node);
    rclcpp::shutdown();
    return 0;
}

