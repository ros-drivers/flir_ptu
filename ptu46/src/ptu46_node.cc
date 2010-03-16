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
#include <ptu46/ptu46_driver.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

namespace PTU46 {

/*
 * PTU46 Node Class
 * Note: this class is *not* thread safe as it is expecting to be called from
 * within a single threaded main loop.
 */
class PTU46_Node {
    public:
        PTU46_Node(ros::NodeHandle& node_handle);
        ~PTU46_Node();

        // Service Control
        void Connect();
        bool ok() {
            return m_pantilt != NULL;
        }
        void Disconnect();

        // Service Execution
        void spinOnce();

        // Callback Methods
        void SetGoal(const sensor_msgs::JointState::ConstPtr& msg);

    protected:
        PTU46* m_pantilt;
        ros::NodeHandle m_node;
        ros::Publisher  m_joint_pub;
        ros::Subscriber m_joint_sub;
};

PTU46_Node::PTU46_Node(ros::NodeHandle& node_handle)
        :m_pantilt(NULL), m_node(node_handle) {
    // Empty Constructor
}

PTU46_Node::~PTU46_Node() {
    Disconnect();
}

void PTU46_Node::Connect() {
    // If we are reconnecting, first make sure to disconnect
    if (ok()) {
        Disconnect();
    }

    // Query for serial configuration
    std::string port;
    m_node.param<std::string>("port", port, PTU46_DEFAULT_PORT);
    int baud;
    m_node.param("baud", baud, PTU46_DEFAULT_BAUD);

    // Connect to the PTU
    ROS_INFO("Attempting to connect to %s...", port.c_str());
    m_pantilt = new PTU46(port.c_str(), baud);
    ROS_ASSERT(m_pantilt != NULL);
    if (! m_pantilt->Open()) {
        ROS_ERROR("Could not connect to pan/tilt unit [%s]", port.c_str());
        Disconnect();
        return;
    }
    ROS_INFO("Connected!");

    float tres = m_pantilt->GetRes(PTU46_TILT),
                 pres = m_pantilt->GetRes(PTU46_PAN);

    m_node.setParam("min_tilt", m_pantilt->TMin*tres);
    m_node.setParam("max_tilt", m_pantilt->TMax*tres);
    m_node.setParam("min_tilt_speed", m_pantilt->TSMin*tres);
    m_node.setParam("max_tilt_speed", m_pantilt->TSMax*tres);
    m_node.setParam("tilt_step", tres);

    m_node.setParam("min_pan", m_pantilt->PMin*pres);
    m_node.setParam("max_pan", m_pantilt->PMax*pres);
    m_node.setParam("min_pan_speed", m_pantilt->PSMin*pres);
    m_node.setParam("max_pan_speed", m_pantilt->PSMax*pres);
    m_node.setParam("pan_step", pres);


    // Publishers : Only publish the most recent reading
    m_joint_pub = m_node.advertise
                  <sensor_msgs::JointState>("state", 1);

    // Subscribers : Only subscribe to the most recent instructions
    m_joint_sub = m_node.subscribe
                  <sensor_msgs::JointState>("cmd", 1, &PTU46_Node::SetGoal, this);

}

void PTU46_Node::Disconnect() {
    if (m_pantilt != NULL) {
        delete m_pantilt;   // Closes the connection
        m_pantilt = NULL;   // Marks the service as disconnected
    }
}

void PTU46_Node::SetGoal(const sensor_msgs::JointState::ConstPtr& msg) {
    if (! ok())
        return;
    double pan = msg->position[0];
    double tilt = msg->position[1];
    double panspeed = msg->velocity[0];
    double tiltspeed = msg->velocity[1];
    m_pantilt->SetPos(PTU46_PAN, pan);
    m_pantilt->SetPos(PTU46_TILT, tilt);
    m_pantilt->SetSpeed(PTU46_PAN, panspeed);
    m_pantilt->SetSpeed(PTU46_TILT, tiltspeed);
}

void PTU46_Node::spinOnce() {
    if (! ok())
        return;

    // Read Position & Speed
    double pan  = m_pantilt->GetPos(PTU46_PAN);
    double tilt = m_pantilt->GetPos(PTU46_TILT);

    double panspeed  = m_pantilt->GetSpeed(PTU46_PAN);
    double tiltspeed = m_pantilt->GetSpeed(PTU46_TILT);

    // Publish Position & Speed
    sensor_msgs::JointState joint_state;
    joint_state.header.stamp = ros::Time::now();
    joint_state.set_name_size(2);
    joint_state.set_position_size(2);
    joint_state.set_velocity_size(2);
    joint_state.name[0] ="head_pan_joint";
    joint_state.position[0] = pan;
    joint_state.velocity[0] = panspeed;
    joint_state.name[1] ="head_tilt_joint";
    joint_state.position[1] = tilt;
    joint_state.velocity[1] = tiltspeed;
    m_joint_pub.publish(joint_state);

    // Publish Transform
    static tf::TransformBroadcaster br;
    tf::Quaternion qx = tf::Quaternion();
    qx.setEuler(0.0, tilt, pan); // yaw, pitch, roll
    geometry_msgs::Quaternion quaternion = geometry_msgs::Quaternion();

    quaternion.w = (double) qx.getW();
    quaternion.x = (double) qx.getX();
    quaternion.y = (double) qx.getY();
    quaternion.z = (double) qx.getZ();

    geometry_msgs::TransformStamped trans;
    trans.header.stamp = ros::Time::now();
    trans.header.frame_id = "ptu_base";
    trans.child_frame_id = "ptu_mount";
    trans.transform.rotation = quaternion;
    br.sendTransform(trans);

}

} // PTU46 namespace

int main(int argc, char** argv) {
    ros::init(argc, argv, "ptu");
    ros::NodeHandle n("~");

    // Connect to PTU
    PTU46::PTU46_Node ptu_node = PTU46::PTU46_Node(n);
    ptu_node.Connect();
    if (! ptu_node.ok())
        return -1;

    // Query for polling frequency
    int hz;
    n.param("hz", hz, PTU46_DEFAULT_HZ);
    ros::Rate loop_rate(hz);

    while (ros::ok() && ptu_node.ok()) {
        // Publish position & velocity
        ptu_node.spinOnce();

        // Process a round of subscription messages
        ros::spinOnce();

        // This will adjust as needed per iteration
        loop_rate.sleep();
    }

    if (! ptu_node.ok()) {
        ROS_ERROR("pan/tilt unit disconncted prematurely");
        return -1;
    }

    return 0;
}
