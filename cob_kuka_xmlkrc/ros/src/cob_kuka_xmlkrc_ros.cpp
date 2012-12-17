// ROS includes
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <cob_kuka_xmlkrc/cob_kuka_xmlkrcConfig.h>

// ROS message includes
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <cob_srvs/Trigger.h>
#include <cob_srvs/Trigger.h>



#include <cob_kuka_xmlkrc_common.cpp>


class cob_kuka_xmlkrc_ros
{
	public:
		ros::NodeHandle n_;
		
		dynamic_reconfigure::Server<cob_kuka_xmlkrc::cob_kuka_xmlkrcConfig> server;
  		dynamic_reconfigure::Server<cob_kuka_xmlkrc::cob_kuka_xmlkrcConfig>::CallbackType f;
		

		ros::Publisher joint_states_;
		ros::Publisher cart_pose_;
		

	ros::ServiceServer Init_;
	ros::ServiceServer MoveLin_BL_;
        
 
        cob_kuka_xmlkrc_data component_data_;
        cob_kuka_xmlkrc_config component_config_;
        cob_kuka_xmlkrc_impl component_implementation_;

        cob_kuka_xmlkrc_ros()
        {
       	
  			f = boost::bind(&cob_kuka_xmlkrc_ros::configure_callback, this, _1, _2);
  			server.setCallback(f);
        	
        	
        		std::string Init_remap;
        		n_.param("Init_remap", Init_remap, (std::string)"Init");
        		Init_ = n_.advertiseService<cob_srvs::Trigger::Request , cob_srvs::Trigger::Response>(Init_remap, boost::bind(&cob_kuka_xmlkrc_impl::callback_Init, &component_implementation_,_1,_2,component_config_));
        		std::string MoveLin_BL_remap;
        		n_.param("MoveLin_BL_remap", MoveLin_BL_remap, (std::string)"MoveLin_BL");
        		MoveLin_BL_ = n_.advertiseService<cob_srvs::Trigger::Request , cob_srvs::Trigger::Response>(MoveLin_BL_remap, boost::bind(&cob_kuka_xmlkrc_impl::callback_MoveLin_BL, &component_implementation_,_1,_2,component_config_));
        
				joint_states_ = 	n_.advertise<sensor_msgs::JointState>("joint_states", 1);
				cart_pose_ = 	n_.advertise<geometry_msgs::PoseStamped>("cart_pose", 1);
  	

				n_.param("KRC_ip_address", component_config_.KRC_ip_address, (std::string)"127.0.0.1");
				n_.param("KRC_ip_port", component_config_.KRC_ip_port, (int)4444);
            
        }
		
		
        
		
		void configure_callback(cob_kuka_xmlkrc::cob_kuka_xmlkrcConfig &config, uint32_t level) 
		{
				component_config_.KRC_ip_address = config.KRC_ip_address;
				component_config_.KRC_ip_port = config.KRC_ip_port;
		}

        void configure()
        {
			component_implementation_.configure(component_config_);
        }

        void update()
        {
            component_implementation_.update(component_data_, component_config_);
				joint_states_.publish(component_data_.out_joint_states);
				cart_pose_.publish(component_data_.out_cart_pose);
    
        }
 
};

int main(int argc, char** argv)
{

	ros::init(argc, argv, "cob_kuka_xmlkrc");

	cob_kuka_xmlkrc_ros node;
    node.configure();

	
 	ros::Rate loop_rate(0.1); // Hz

	while(node.n_.ok())
	{
        node.update();
		loop_rate.sleep();
		ros::spinOnce();
	}
	
    return 0;
}
