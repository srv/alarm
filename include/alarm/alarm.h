/**
 * @file
 * @brief alarm controller.
 * @author Francisco Bonin Font
 * @date 2012-03-12
 *
 * This node subscribes to different topics which values can indicate a hazardous situation inside the
 * cilinder. Today only the humidity value given by the humidity sensors is controlled to
 * see if there is water inside the cilinder. If there is, the Pc is switched off.
 * In the same way, other points can be controlled, such as if the battery if off. 
 * The status of the alarm is continuously published at low freqüency in the tpic emergency_alarm, which must be periodically checked. 
 */

#include <iostream>
#include <fstream>
#include <iomanip>
#include <cstdlib>
#include <string>
#include <string.h>   // Required by strcpy()

//int tolerance; // level at which the alarm is activated. 
//const string actions; // shell to be executed when the alarm activates 
 
