/**
 * @file
 * @brief Node to control several alarm situations. For example, if the humidity sensor gives a signal too high, the node launches a linux shell to automatically shutdown the system. 
 * 
 *
 * @par Advertises
 *
 * - @b emergency_alarm (srv_msgs/emergency_alarm)
 *   alarm_on=true if an emergency has occurrend and the pc must be turn off (water in, battery off, etc...)
 *   
 * @par Subscribes
 *      * - @b humidity topic (srv_msgs/WaterIn.humidity) 
 *   Humidity sensor sample. If the humidity is higher than the tolerance, shutdown the system.
*  */


#include "ros/ros.h"
#include <srv_msgs/WaterIn.h> // it is necessary to include the packages containing these
// messages in the manifest.xml. Otherwise they wont be found.
#include <srv_msgs/emergency_alarm.h>
#include <sstream>
#include <alarm/alarm.h>

using namespace std;

//global variables
int tolerance; 
string actions;


ros::Publisher alarm_pub; // declare the publisher in global mode

/** performActionsCall Run actions when the humidity alarm is launched.
 * @param actions The shell to be executed. It is configured in the alarm.launch file
 */
void performActionsCall(const string& actions)
{
  ROS_INFO("System shut down for security. Integrity Alarm !!"); // log the alarm message
  ostringstream ss(actions,ios_base::ate);
  //out << "System call to '" << ss.str() << "'" << endl; // message shows the shell to be run
  int res = system(ss.str().c_str()); // run the shell
  printf("System Call return: %i", res);
  //out << "System call returned " << res << endl; //result of the execution is a number stored in "res". 0 if ok. 
}

/** alarmCallback. Wait for a message type WaterIn. Evaluate the humidity value and call performActions if it exceeds the tolerance value
 * @param msg WaterIn message 
 */
void alarmCallback(const srv_msgs::WaterIn::ConstPtr& msg) 
{
    int humid;
    srv_msgs::emergency_alarm alarm_msg; // create a message type emergency_alarm called alarm_msg
	humid=msg->humidity;
	//printf("humitat: %i; tolerance: %i",humid, tolerance);
	if (humid>tolerance){
	alarm_msg.status=true; //activate alarm
	alarm_pub.publish(alarm_msg); // the message type emergency_alarm is published
        ROS_INFO("Humidity too high activate alarm !!: [%i]", msg->humidity); // log the alarm message
	cout << "Humidity too high, activate alarm. Actions script is : " << actions << endl;	
        performActionsCall(actions); // humidity has been detected inside the cilinder, it has been published the corresponding topic and logged the corresponding message and now the computer must be switched off. 
	}else{
	alarm_msg.status=false; //status humidity sensor ok, no alarm
   	alarm_pub.publish(alarm_msg); // the message type emergency_alarm is published
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
	/** subscriber to a topic "humidity" and call the function "alarmCallback" when 
	* a message humidity is available. The topic "humidity" is advertized by the 
	*node "motor_board_node_base"
	*/

node.param("tolerance", tolerance, 1500); // param name, variable that will contain the param value, default value. 
node.param<string>("actions", actions,"/home/user/waterinmonitor/init/waterinactions.sh");


     ros::Subscriber sub = node.subscribe("humidity", 1, alarmCallback);
     alarm_pub = node.advertise<srv_msgs::emergency_alarm>("emergency_alarm", 1);
     // we publish a topic 'emergency' which is a message type srv_msg::emergency_alarm through the object publisher alarm_pub
     ros::spin(); // enters in a infinite loop waiting for message humidity and
     // calls the callback function if one arrives
  
    return 0;
  }
  

