#include "pi3hat_moteus_int_msgs/msg/quad_foot_state.hpp"
#include <vector>
#include <string>
#include <math.h>
#include <iostream>
using FootCommand = pi3hat_moteus_int_msgs::msg::QuadFootState;
namespace omni_mulinex_joy
{
    struct SG_Man_Conf{
        double base_amplitude = 0.0;
        double foot_h_amplitude = 0.1;
        double foot_x_displacement = 0.05;
        double swing_period=1.0;
        bool pitch_comp = false;
        void operator=(const SG_Man_Conf& b)
        {
            base_amplitude = b.base_amplitude;
            foot_h_amplitude = b.foot_h_amplitude;
            foot_x_displacement = b.foot_x_displacement;
            swing_period = b.swing_period;
        };
    };
    class Static_Gait_Manager
    {
        public:
            Static_Gait_Manager(FootCommand* data, SG_Man_Conf conf, std::vector<int> step_list)
            {
                data_ = data;
                leg_list_.resize(4);
                for(int i = 0; i < 4; i++)
                {
                    leg_list_[i] = data->foot_id[i];
                }
                swing_list_ = step_list;
                if(swing_list_.size() != (leg_list_.size()+1))
                {
                    std::cerr<< "[SG_MANAGER::ERROR]The swing list has incorrect size, it will be empty"<<std::endl;
                    swing_list_.resize(0);
                }
                config_ = conf;
            };
            int get_counter()
            {
                return swing_counter_;
            }
            void set_walk_begin(double start)
            {
                t_0_ = start;
                swing_counter_ = 0;
                walking_ = true;
                for(int i = 0; i<4 ;i++)
                {
                    base_z_[i] = data_->position[i].z;
                    base_x_[i] = data_->position[i].x;
                }
            }
            void update(double time)
            {
                double act_t = time - t_0_;
                //  std::cerr << "call update"<<std::endl;
                // check swing end, in case start with the next leg
                if(act_t > config_.swing_period)
                {

                    
                    t_0_ = time;
                    // std::cerr << "compute last swing step"<<std::endl;
                    walking_ = true;
                    // compute last swing timestamp time == swing_period
                    compute_feet_pos(config_.swing_period);
                    swing_counter_ = swing_counter_ + 1;
                    if(swing_counter_ >= 5)
                    {
                        std::cerr << "stop  "<<swing_counter_-1<<" step"<< act_t<<std::endl;
                        walking_ = false;
                    }
                }
                // compute the swing
                else
                {
                    compute_feet_pos(act_t);
                    // std::cerr << "doing swing step"<<std::endl;
                }
                // std::cerr << "the foot index is "<< swing_counter_<<" and the dt is "<< act_t<<std::endl;
            }
            bool complete_step()
            {
                return !walking_;
            }
            void compute_feet_pos(double dt)
            {
                int ind;
                double swing_f_x =  config_.foot_x_displacement*(dt/config_.swing_period);
                
                double swing_f_y = config_.foot_h_amplitude*std::sin((M_PI/config_.swing_period)*dt);
                
                double base_x = config_.base_amplitude*std::sin((M_PI/config_.swing_period)*dt);
                double mult,mult_base;
                ind = swing_list_[swing_counter_];
                std::cerr << "the foot index is "<< ind<<" and the dt is "<< dt<<std::endl;
                if( ind != 4)
                    char sf_fh = leg_list_[ind][1];
                for( int i = 0; i < 4; i++)
                {
                    if(leg_list_[i][1] == 'F')
                        mult = 1;
                    else
                        mult = -1;
                    // std::cerr << "the time  is "<< leg_list_[i] << " base command is "<<mult<<std::endl;
                //    data_->position[i].z = base_z_[i];
                //    data_->position[i].x = base_x_[i];
                    if(ind == 4)
                    {
                        
                        
                        data_->position[i].x = base_x_[i] - swing_f_x * mult;
                        if(dt == config_.swing_period)
                        {
                            base_z_[i] = data_->position[i].z;
                            base_x_[i] = data_->position[i].x;
                        }
                    }   
                    else
                    {
                        // std::cerr << "the time  is "<< dt<<std::endl;
                        // swing the indexed leg 
                        if(leg_list_[ind][1] == 'F')
                            mult_base = 1;
                        else
                            mult_base = -1;
                        // std::cerr << "the pd  is "<< leg_list<<std::endl;
                        if( i == ind)
                        {

                            data_->position[i].x = base_x_[i] + swing_f_x* mult_base;
                            data_->position[i].z = base_z_[i] + swing_f_y;
                            if(dt == config_.swing_period)
                            {
                                base_z_[i] = data_->position[i].z;
                                base_x_[i] = data_->position[i].x;
                            }

                        }
                        else
                        {
                            data_->position[i].z = base_z_[i];
                            data_->position[i].x = base_x_[i] + (base_x* mult_base) *mult;// *mult*-mult;
                        }
                    }
                    // // std::cerr<< "the swing foot value is "<< swing_f_x<<" and time is "<< dt<<" and ind is "<<i<<std::endl;
                    

                }
            }
        private:
            FootCommand* data_;
            double t_0_;
            std::vector<std::string> leg_list_;
            std::vector<int> swing_list_;
            std::vector<double> base_z_ = {0.0,0.0,0.0,0.0},base_x_ = {0.0,0.0,0.0,0.0};
            int swing_counter_ = 0;
            SG_Man_Conf config_;
            SG_Man_Conf sg_man_conf_;
            bool walking_ = false;

    };
};