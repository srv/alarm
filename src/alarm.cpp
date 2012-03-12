/**
 * @file
 * @brief ROS driver base class for Albatros motor board (implementation).
 *
 * This is a ROS node to calculate the depth under the water
from the absolute reading (in psi) given by the pressure sensor included in the motor board provided by Albatros.
 It applies a simple conversion: 1 psi=0.068atmosferes , 1 atmos=10meters. note that before multiplying by 10, the 1 atmosferes has to be substracted from the pressure absolute reading. 
 *
 * @par Advertises
 *
 * - @b emergency_alarm (srv_msgs/emergency_alarm)
 *   alarm_on=true if an emergency has occurrend and the pc must be turn off (water in, battery off, etc...)
 *   
 * @par Subscribes
 *      * - @b humidity topic (srv_msgs/WaterIn.humidity) (at the moment only control the humidity, but more topics can be subscribed to be controled)
 *   Humidity sensor sample.
*  */




#include "ros/ros.h"
#include <srv_msgs/WaterIn.h> // it is necessary to include the packages containing these
// messages in the manifest.xml. Otherwise they wont be found.
#include <srv_msgs/emergency_alarm.h>
#include <alarm.h>
#include <sstream>
 

ros::Publisher alarm_pub; // declare the publisher in global mode

void alarmCallback(const srv_msgs::WaterIn::ConstPtr& msg) // wait for a message type WaterIn and create the pointer to it
{
    int humid;
    srv_msgs::emergency_alarm alarm_msg; // create a message type emergency_alarm called alarm_msg
	humid=msg->humidity;
	if (humid>tolerance){
	alarm_msg.status=true; //activate alarm
	ROS_INFO("Humidity too high activate alarm !!: [%f]", msg->humidity); // log the alarm message
	alarm_pub.publish(alarm_msg); // the message type emergency_alarm is published
	switch_PCOff(); // humidity has been detected inside the cilinder, it has been published the corresponding topic and logged the corresponding message and now the computer must be switched off. 
	}
   
}


int main(int argc, char **argv)
 {
   /**
    * The ros::init() function needs to see argc and argv so that it can perform
      * any ROS arguments and name remapping that were provided at the command line. For programmatic
      * remappings you can use a different version of init() which takes remappings
      * directly, but for most command-line programs, passing argc and argv is the easiest
      * way to do it.  The third argument to init() is the name of the node.
      *
      * You must call one of the versions of ros::init() before using any other
      * part of the ROS system.
      */
     ros::init(argc, argv, "emergency_alarm");
   
     /**
      * NodeHandle is the main access point to communications with the ROS system.
      * The first NodeHandle constructed will fully initialize this node, and the last
      * NodeHandle destructed will close down the node.
      */
     ros::NodeHandle node;
   
     /**
      * The advertise() function is how you tell ROS that you want to
      * publish on a given topic name. This invokes a call to the ROS
      * master node, which keeps a registry of who is publishing and who
      * is subscribing. After this advertise() call is made, the master
      * node will notify anyone who is trying to subscribe to this topic name,
      * and they will in turn negotiate a peer-to-peer connection with this
      * node.  advertise() returns a Publisher object which allows you to
      * publish messages on that topic through a call to publish().  Once
      * all copies of the returned Publisher object are destroyed, the topic
      * will be automatically unadvertised.
      *
      * The second parameter to advertise() is the size of the message queue
      * used for publishing messages.  If messages are published more quickly
      * than we can send them, the number here specifies how many messages to
      * buffer up before throwing some away.
      */
	// subscriber to a topic "humidity" and call the function "alarmCallback" when 
	// a message humidity is available. The topic "humidity" is advertized by the 
	//node "motor_board_node_base"

node.param("tolerance", tolerance, 1500); // param name, variable that will contain the param value, default value. 

     ros::Subscriber sub = node.subscribe("humidity", 1, AlarmCallback);
     alarm_pub = node.advertise<srv_msgs::emergency_alarm>("emergency_alarm", 1);
     // we publish a topic 'emergency' which is a message type srv_msg::emergency_alarm through the object publisher alarm_pub
     ros::spin(); // enters in a infinite loop waiting for message humidity and
     // calls the callback function if one arrives
  
    return 0;
  }
  

