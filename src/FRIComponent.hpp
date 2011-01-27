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

#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jacobian.hpp>

#include <brics_actuator/JointPositions.h>
#include <brics_actuator/JointVelocities.h>
#include <brics_actuator/JointTorques.h>
#include <brics_actuator/JointImpedances.h>
#include <brics_actuator/JointValues.h>

#include <brics_actuator/CartesianPose.h>
#include <brics_actuator/CartesianWrench.h>
#include <brics_actuator/CartesianImpedance.h>


#include <friComm.h>

namespace lwr_fri_rtt_2_0 {

using namespace RTT;
using namespace brics_actuator;

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

	tFriMsrData m_msr_data;
	tFriCmdData m_cmd_data;

	JointPositions m_jntPos;
	JointTorques m_jntTorques;

	CartesianPose m_cartPos;
	CartesianWrench m_cartWrench;

	tFriKrlData m_fromKRL;
	tFriKrlData m_toKRL;
	//Eigen::Matrix<double,7,7> m_massTmp; Not correct so useless

	/**
	 * events
	 */
	OutputPort<std::string> m_events;

	/**
	 * statistics
	 */
	OutputPort<tFriRobotState> m_RobotStatePort;
	OutputPort<tFriIntfState> m_FriStatePort;

	/**
	 * Current robot data
	 */
	OutputPort<JointPositions> m_msrJntPosPort;
	OutputPort<JointPositions> m_cmdJntPosPort;
	OutputPort<JointPositions> m_cmdJntPosFriOffsetPort;
	OutputPort<CartesianPose>  m_msrCartPosPort;
	OutputPort<CartesianPose>  m_cmdCartPosPort;
	OutputPort<CartesianPose>  m_cmdCartPosFriOffsetPort;
	OutputPort<JointTorques>   m_msrJntTrqPort;
	OutputPort<JointTorques>   m_estExtJntTrqPort;
	OutputPort<CartesianWrench> m_estExtTcpWrenchPort;
	//RTT::OutputPort<KDL::Jacobian> jacobianPort;
	//RTT::OutputPort<Eigen::MatrixXd > massMatrixPort;
	//RTT::OutputPort<std::vector<double> > gravityPort;

	InputPort<JointPositions> m_jntPosPort;
	InputPort<CartesianPose> m_cartPosPort;
	InputPort<JointTorques> m_addJntTrqPort;
	InputPort<CartesianWrench> m_addTcpWrenchPort;
	InputPort<JointImpedances> m_jntImpedancePort;
	InputPort<CartesianImpedance> m_cartImpedancePort;

	int m_local_port,m_socket,m_remote_port, m_control_mode;

	const char* m_remote_address;
	struct sockaddr m_remote_addr;
	uint16_t counter, fri_state_last;
};

}//Namespace LWR

#endif//_FRI_COMPONENT_HPP_


    


    
