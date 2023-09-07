/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <stdio.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped des_pose;
    des_pose.pose.position.x = 0;
    des_pose.pose.position.y = 0;
    des_pose.pose.position.z = 0;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(des_pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    int counter=0; 
    bool offb_set_once=false;
    int m=1;
    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
		if (offb_set_once){
			offb_set_mode.request.custom_mode="STABILIZED";
			if( set_mode_client.call(offb_set_mode) &&
                                offb_set_mode.response.mode_sent)
			{
                                ROS_INFO("STABILIZED enabled");
				return 0;
			}
		}
		else
		{
            		if( set_mode_client.call(offb_set_mode) &&
                		offb_set_mode.response.mode_sent)
			{
                		ROS_INFO("Offboard enabled");
				offb_set_once=true;
            		}
		}
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }
        if ( current_state.mode=="OFFBOARD" && current_state.armed )
	{
		if (counter>1000)
		{
			ROS_INFO("UPDATED SETPOINT");
			counter=0;
			m=-m;
		}
		else
			counter++;
                des_pose.pose.position.z = des_pose.pose.position.z + m*0.001; 
       	 	if (des_pose.pose.position.z>1)
			des_pose.pose.position.z=1;
		if (des_pose.pose.position.z<0.30)
                        des_pose.pose.position.z=0.30;

		printf("counter=%d, setpoint=%.2f\n", counter,des_pose.pose.position.z);
		
		
		/*

		if (counter<50)
		{
			//ROS_INFO("SETPOINT ABOVE DRONE");
			des_pose.pose.position.z=1.7;
		}
		else
		{
			//ROS_INFO("SETPOINT BELOW DRONE");
			des_pose.pose.position.z=0.7;
		}
		*/
	}

        local_pos_pub.publish(des_pose);
	
	
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
