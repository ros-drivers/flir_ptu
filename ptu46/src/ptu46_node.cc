/* 
 * PTU46 ROS Package
 * Copyright (C) 2009 Erik Karulf (erik@cse.wustl.edu)
 *
 */
 
/*
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <string>
#include <ros/ros.h>
#include "ptu46/ptu46_driver.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_util/conversion.h"
#include "tf/transform_broadcaster.h"

namespace PTU46
{

/*
 * PTU46 Node Class
 * Note: this class is *not* thread safe as it is expecting to be called from
 * within a single threaded main loop.
 */
class PTU46_Node
{
  public:
    PTU46_Node(ros::NodeHandle& node_handle);
    ~PTU46_Node();
    
    // Service Control 
    void Connect();
    bool ok() { return m_pantilt != NULL; }
    void Disconnect();
    
    // Service Execution
    void spinOnce();
    
    // Callback Methods
    void SetPosition(const geometry_msgs::Quaternion::ConstPtr& msg);
    void SetVelocity(const geometry_msgs::Quaternion::ConstPtr& msg);

  protected:
    PTU46* m_pantilt;
    ros::NodeHandle m_node;
    ros::Publisher  m_position_pub;
    ros::Publisher  m_velocity_pub;
    ros::Publisher  m_goal_position_pub;
    ros::Subscriber m_cmd_position_sub;
    ros::Subscriber m_cmd_velocity_sub;
};

PTU46_Node::PTU46_Node(ros::NodeHandle& node_handle)
    :m_pantilt(NULL), m_node(node_handle)
{
    // Empty Constructor
}

PTU46_Node::~PTU46_Node()
{
    Disconnect();
}

void PTU46_Node::Connect()
{
    // If we are reconnecting, first make sure to disconnect
    if (ok())
    {
        Disconnect();
    }
    
    // Query for serial configuration
    std::string port;
    m_node.param<std::string>("ptu_port", port, PTU46_DEFAULT_PORT);
    int baud;
    m_node.param("ptu_baud", baud, PTU46_DEFAULT_BAUD);
    
    // Connect to the PTU
    m_pantilt = new PTU46(port.c_str(), baud);
    ROS_ASSERT(m_pantilt != NULL);
    if (! m_pantilt->Open())
    {
        ROS_ERROR("Could not connect to pan/tilt unit [%s]", port.c_str());
        Disconnect();
        return;
    }
    
    // Publishers : Only publish the most recent reading
    m_velocity_pub = m_node.advertise<geometry_msgs::Quaternion>("ptu_velocity", 1);
    m_goal_position_pub = m_node.advertise<geometry_msgs::Quaternion>("ptu_goal", 1);

    // Subscribers : Only subscribe to the most recent instructions
    m_cmd_position_sub = m_node.subscribe<geometry_msgs::Quaternion>("ptu_cmd_position", 1, &PTU46_Node::SetPosition, this);
    m_cmd_velocity_sub = m_node.subscribe<geometry_msgs::Quaternion>("ptu_cmd_velocity", 1, &PTU46_Node::SetVelocity, this);
}

void PTU46_Node::Disconnect()
{
    
    if (m_pantilt != NULL)
    {
        delete m_pantilt;   // Closes the connection
        m_pantilt = NULL;   // Marks the service as disconnected
    }
}

void PTU46_Node::SetVelocity(const geometry_msgs::Quaternion::ConstPtr& msg)
{
    if (! ok())
        return;
    
    double roll, pitch, yaw;
    geometry_util::GetEulerYPR(*msg, roll, pitch, yaw);
    
    m_pantilt->SetSpeed(PTU46_PAN, roll);
    m_pantilt->SetSpeed(PTU46_TILT, pitch);
    
    // TODO: publish an empty goal?
}

void PTU46_Node::SetPosition(const geometry_msgs::Quaternion::ConstPtr& msg)
{
    if (! ok())
        return;
    
    double roll, pitch, yaw;
    geometry_util::GetEulerYPR(*msg, roll, pitch, yaw);

    m_pantilt->SetPos(PTU46_PAN, roll);
    m_pantilt->SetPos(PTU46_TILT, pitch);
    
    m_goal_position_pub.publish(msg);
}

void PTU46_Node::spinOnce()
{
    if (! ok())
        return;
    
    // Read Position & Speed
    double pan  = m_pantilt->GetPos(PTU46_PAN);
    double tilt = m_pantilt->GetPos(PTU46_TILT);
    
    double panspeed  = m_pantilt->GetSpeed(PTU46_PAN);
    double tiltspeed = m_pantilt->GetSpeed(PTU46_TILT);

    // Publish Position & Speed
    static tf::TransformBroadcaster br;
    tf::Quaternion quaternion = tf::Quaternion();
    quaternion.setEuler(pan, tilt, 0.0); // yaw, pitch, roll
    br.sendTransform(tf::Transform(quaternion), ros::Time::now(), "ptu_mount", "ptu_base");

    tf::Quaternion ang_velocity = tf::Quaternion();
    ang_velocity.setEuler(panspeed, tiltspeed, 0.0); // yaw, pitch, roll
    geometry_msgs::Quaternion msg;
    tf::quaternionTFToMsg(ang_velocity, msg);
    m_velocity_pub.publish(msg);
}

} // PTU46 namespace

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ptu46");
    ros::NodeHandle n;
    
	// Connect to PTU
	PTU46::PTU46_Node ptu_node = PTU46::PTU46_Node(n);
	ptu_node.Connect();
	if (! ptu_node.ok())
	    return -1;
	
    // Query for polling frequency
    int hz;
    n.param("ptu_hz", hz, PTU46_DEFAULT_HZ);
	ros::Rate loop_rate(hz);
		
	while (ros::ok() && ptu_node.ok())
	{
	    // Publish position & velocity
	    ptu_node.spinOnce();
	    
	    // Process a round of subscription messages
	    ros::spinOnce();
	    
	    // This will adjust as needed per iteration
	    loop_rate.sleep();
	}
	
	if (! ptu_node.ok())
	{
	    ROS_ERROR("pan/tilt unit disconncted prematurely");
	    return -1;
	}
	
	return 0;
}
