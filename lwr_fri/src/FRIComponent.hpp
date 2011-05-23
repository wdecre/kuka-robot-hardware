// Copyright  (C)  2009  Ruben Smits <ruben dot smits at mech dot kuleuven dot be>, 
// Copyright  (C)  2009  Wilm Decre <wilm dot decre at mech dot kuleuven dot be>

// Author: Ruben Smits, Wilm Decre
// Maintainer: Ruben Smits, Wilm Decre

// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.

// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.

// You should have received a copy of the GNU Lesser General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA


#ifndef _FRI_COMPONENT_HPP_
#define _FRI_COMPONENT_HPP_

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Logger.hpp>
#include <sys/socket.h>
#include <arpa/inet.h>

#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jacobian.hpp>

#include <sensor_msgs/JointState.h>
#include <lwr_fri/FriJointCommand.h>
#include <lwr_fri/FriJointImpedance.h>
#include <lwr_fri/FriJointState.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Wrench.h>

#include <sensor_msgs/typekit/JointState_Types.hpp>
#include <geometry_msgs/typekit/Pose_Types.hpp>
#include <geometry_msgs/typekit/Twist_Types.hpp>
#include <geometry_msgs/typekit/Wrench_Types.hpp>

#include <kuka_lwr_fri/friComm.h>

namespace lwr_fri {

using namespace RTT;

class FRIComponent: public RTT::TaskContext {
public:
	FRIComponent(const std::string& name);
	~FRIComponent();

	virtual bool configureHook();
	virtual bool startHook();

	virtual void updateHook();
	virtual void stopHook();
	virtual void cleanupHook();

private:

	int fri_create_socket();
	int fri_recv();
	int fri_send();

	tFriMsrData m_msr_data;
	tFriCmdData m_cmd_data;

        sensor_msgs::JointState m_joint_states;
        lwr_fri::FriJointState m_fri_joint_state;
  
        /*
        geometry_msgs::Pose m_cartPos;
        geometry_msgs::Twist m_cartTwist;
        geometry_msgs::Wrench m_cartWrench;
	*/
        OutputPort<tFriKrlData> port_from_krl;
        OutputPort<tFriKrlData> port_to_krl;
	//Eigen::Matrix<double,7,7> m_massTmp; Not correct so useless

	/**
	 * events
	 */
	OutputPort<std::string> port_events;

	/**fri_create_socket
	 * statistics
	 */
	OutputPort<tFriRobotState> port_robot_state;
	OutputPort<tFriIntfState> port_fri_state;

	/**
	 * Current robot data
	 */
        OutputPort<sensor_msgs::JointState > port_joint_state;
        OutputPort<lwr_fri::FriJointState> port_fri_joint_state;
        
        //OutputPort<geometry_msgs::Pose>  m_msrCartPosPort;
	//OutputPort<geometry_msgs::Pose>  m_cmdCartPosPort;
	//OutputPort<geometry_msgs::Pose>  m_cmdCartPosFriOffsetPort;
        //OutputPort<geometry_msgs::Wrench> m_estExtTcpWrenchPort;
	//RTT::OutputPort<KDL::Jacobian> jacobianPort;
	//RTT::OutputPort<Eigen::MatrixXd > massMatrixPort;

        lwr_fri::FriJointCommand m_fri_joint_command;
        lwr_fri::FriJointImpedance m_fri_joint_impedance;
        InputPort<lwr_fri::FriJointCommand> port_fri_joint_command;
        InputPort<lwr_fri::FriJointImpedance> port_fri_joint_impedance;

        //InputPort<geometry_msgs::Pose> m_cartPosPort;
        //InputPort<geometry_msgs::Twist> m_cartTwistPort;
        //InputPort<geometry_msgs::Wrench> m_addTcpWrenchPort;
        //InputPort<CartesianImpedance> m_cartImpedancePort;

        int prop_local_port,m_socket,m_remote_port, m_control_mode;
        std::string joint_names_prefix;
	uint16_t counter, fri_state_last;
	struct sockaddr_in m_remote_addr;
	socklen_t m_sock_addr_len;
};

}//Namespace LWR

#endif//_FRI_COMPONENT_HPP_


    


    