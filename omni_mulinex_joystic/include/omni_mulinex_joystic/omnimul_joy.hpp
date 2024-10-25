#include <chrono>
#include "sensor_msgs/msg/joy.hpp"
#include "std_srvs/srv/empty.hpp"
#include "pi3hat_moteus_int_msgs/msg/omni_mulinex_command.hpp"
#include "pi3hat_moteus_int_msgs/msg/joints_states.hpp"
#include "pi3hat_moteus_int_msgs/msg/quad_foot_state.hpp"
#include "omni_mulinex_joystic/sgait_managet.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/callback_group.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/event_handler.hpp"
#include <string>
#include <mutex>
#include "rosbag2_cpp/writer.hpp"
#include "rclcpp/serialization.hpp"
#include "std_srvs/srv/set_bool.hpp"

#define MAX_LIN_VEL 2.0
#define MAX_ROT_VEL 5.0
#define MAX_HEIGHT_RATE 0.02
const std::vector<int> step_list = {0,2,4,1,3};
const std::vector<std::string> foot_name = {"LF","LH","RF","RH"};
namespace omni_mulinex_joy
{
    using TransictionService = std_srvs::srv::SetBool;
    using JoyCommand = sensor_msgs::msg::Joy;
    using FootCommand = pi3hat_moteus_int_msgs::msg::QuadFootState;
    using OM_JoyCmd = pi3hat_moteus_int_msgs::msg::OmniMulinexCommand;
    using OM_State = pi3hat_moteus_int_msgs::msg::JointsStates;
    enum JoyState {Homing = 0,Rolling ,Static , Walking};
    class OmniMulinex_Joystic : public rclcpp::Node
    {
        public:
            OmniMulinex_Joystic():
            Node("OmniMulinex_Joystic"),
            srv_req_(std::make_shared<TransictionService::Request>()),
            srv_res_(std::make_shared<TransictionService::Response>())
            {
                this->declare_parameter("bag_folder","omni_mul_experiments");
                this->declare_parameter("sup_vel_x",2.0);
                this->declare_parameter("sup_vel_y",2.0);
                this->declare_parameter("sup_omega",1.0);
                this->declare_parameter("sup_height_rate",0.1);
                this->declare_parameter("save_state",true);
                this->declare_parameter("timer_duration",100);
                this->declare_parameter("state_duration",1);
                this->declare_parameter("deadzone_joy",0.15);
                this->declare_parameter("x_displacement",0.03);
                this->declare_parameter("z_amplitude",0.05);
                this->declare_parameter("base_amplitude",0.1);
                this->declare_parameter("swing_dur",5.0);
                this->declare_parameter("init_x",0.0);
                this->declare_parameter("init_z",-0.1);
                RCLCPP_WARN(this->get_logger(),"get_param");
                get_param();
                RCLCPP_WARN(this->get_logger(),"set_tool");
                set_tools();
                RCLCPP_WARN(this->get_logger(),"set_planner_state");
                plan_state_ = JoyState::Homing;



            }
            void close()
            {
                writer_->close();
            }
        private:
            // function to get the parameter
            void get_param();

            // function to setup sub/pub and service
            void set_tools();

            // callback to listen on the joy node topic 
            void joy_command(const std::shared_ptr<JoyCommand> msg);

            //state subscribe callback 
            // void stt_callback(std::shared_ptr<rclcpp::SerializedMessage> msg);
            void stt_callback(std::shared_ptr<OM_State> msg);
            
            // timer callback to send the message at the desired frequency 
            void main_callback();
            void check_callback();
            std::shared_ptr<rclcpp::Subscription<JoyCommand>> cmd_sub_;
            std::shared_ptr<rclcpp::Publisher<FootCommand>>leg_cmd_pub_;
            std::shared_ptr<rclcpp::Publisher<OM_JoyCmd>> cmd_pub_;
            std::shared_ptr<rclcpp::Subscription<OM_State>> stt_sub_;
            std::shared_ptr<rclcpp::TimerBase> timer_, active_timer_;

            OM_JoyCmd cmd_msg_;
            int timer_dur_,stt_period_;
            double sup_vx_,sup_vy_,sup_omega_,sup_height_rate_, v_x_,v_y_,omega_,h_rate_,deadzone_,ix_,iz_ ;
            bool robot_ready_ = false, register_state_; 
            JoyCommand cmd_value_;
            FootCommand foot_msg_;
            std::string bag_folder_;
            std::unique_ptr<rosbag2_cpp::Writer> writer_;
            std::shared_ptr<rclcpp::Client<TransictionService>> hom_srv_,emrgy_srv_,l_hom_srv_,l_emrgy_srv_ ,l_active_srv_;
            bool old_hom_but_ = false;
            bool old_emg_but_ = false, old_change_but_= false, old_walk_but_=false;
            std::shared_ptr<TransictionService::Request> srv_req_;
            std::shared_ptr<TransictionService::Response> srv_res_;
            int count_=0;
            std::vector<std::string> focused_name_ = {
                     "RF_HFE","RF_KFE", "LF_HFE", "LF_KFE"
            };
            JoyState plan_state_;
            std::mutex gv_mut_;
            std::shared_ptr<Static_Gait_Manager> sg_man_;
            SG_Man_Conf sg_man_conf_;
    };
}