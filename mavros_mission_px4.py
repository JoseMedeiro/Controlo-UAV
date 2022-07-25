#!/usr/bin/env python

##########################################################################
# JdeRobot. PX4 + MAVROS: Load a mission using WPs file + fly fully AUTO
#
# Diego Martin
# 09/02/2019
# v1.0
###########################################################################

from __future__ import print_function

import rospy
import argparse
import threading
import time
import os
import mavros
from mavros import command
from mavros.utils import *
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from mavros import mission as M

mavros.set_namespace()


# MAVWP copy to bypass os.system
no_prettytable = False
try:
    from prettytable import PrettyTable
except ImportError:
    print("Waring: 'show' action disabled. install python-prettytable", file=sys.stderr)
    no_prettytable = True


def get_wp_file_io(args):
    return M.QGroundControlWP()


def _pull(args):
    try:
        ret = M.pull()
    except rospy.ServiceException as ex:
        fault(ex)

    if not ret.success:
        fault("Request failed. Check mavros logs")

    print_if(args.verbose, "Waypoints received:", ret.wp_received)
    return ret


def do_pull(args):
    _pull(args)


def do_show(args):
    str_bool = lambda x: 'Yes' if x else 'No'
    str_frame = lambda f: M.FRAMES.get(f, 'UNK') + ' ({})'.format(f)
    str_command = lambda c: M.NAV_CMDS.get(c, 'UNK') + ' ({})'.format(c)

    done_evt = threading.Event()
    def _show_table(topic):
        pt = PrettyTable(('#', 'Curr', 'Auto',
                          'Frame', 'Command',
                          'P1', 'P2', 'P3', 'P4',
                          'X Lat', 'Y Long', 'Z Alt'))

        for seq, w in enumerate(topic.waypoints):
            pt.add_row((
                seq,
                str_bool(w.is_current),
                str_bool(w.autocontinue),
                str_frame(w.frame),
                str_command(w.command),
                w.param1,
                w.param2,
                w.param3,
                w.param4,
                w.x_lat,
                w.y_long,
                w.z_alt
            ))

        print(pt, file=sys.stdout)
        sys.stdout.flush()
        done_evt.set()

    if args.pull:
        _pull(args)

    # Waypoints topic is latched type, and it updates after pull
    sub = M.subscribe_waypoints(_show_table)
    if args.follow:
        rospy.spin()
    elif not done_evt.wait(30.0):
        fault("Something went wrong. Topic timed out.")


def do_load(args):
    wps = []
    wps_file = get_wp_file_io(args)
    with args.file:
        wps = [w for w in wps_file.read(args.file)]

    def _load_call(start, waypoint_list):
        try:
            ret = M.push(start_index=start, waypoints=waypoint_list)
        except rospy.ServiceException as ex:
            fault(ex)

        if not ret.success:
            fault("Request failed. Check mavros logs")

        print_if(args.verbose, "Waypoints transfered:", ret.wp_transfered)

    done_evt = threading.Event()
    def _fix_wp0(topic):
        if len(topic.waypoints) > 0:
            wps[0] = topic.waypoints[0]
            print_if(args.verbose, "HOME location: latitude:", wps[0].x_lat,
                     "longitude:", wps[0].y_long, "altitude:", wps[0].z_alt)
        else:
            print("Failed to get WP0! WP0 will be loaded from file.", file=sys.stderr)

        done_evt.set()

    if args.start_index > 0:
        # Push partial
        if args.start_index == args.end_index:
            args.end_index += 1

        end_index = args.end_index or len(wps)
        _load_call(args.start_index, wps[args.start_index:end_index])
    else:
        # Full push
        if not args.preserve_home:
            _load_call(0, wps)
        else:
            # Note: _load_call() emit publish on this topic, so callback only changes
            # waypoint 0, and signal done event.
            sub = M.subscribe_waypoints(_fix_wp0)
            if not done_evt.wait(30.0):
                fault("Something went wrong. Topic timed out.")
            else:
                sub.unregister()
                _load_call(0, wps)


def do_dump(args):
    done_evt = threading.Event()
    def _write_file(topic):
        wps_file = get_wp_file_io(args)
        with args.file:
            wps_file.write(args.file, topic.waypoints)
        done_evt.set()

    # Waypoints topic is latched type, and it updates after pull
    _pull(args)
    sub = M.subscribe_waypoints(_write_file)
    if not done_evt.wait(30.0):
        fault("Something went wrong. Topic timed out.")


def do_clear(args):
    try:
        ret = M.clear()
    except rospy.ServiceException as ex:
        fault(ex)

    if not ret.success:
        fault("Request failed, Check mavros logs")
    else:
        print_if(args.verbose, "Waypoints cleared")


def do_set_current(args):
    try:
        ret = M.set_current(wp_seq=args.seq)
    except rospy.ServiceException as ex:
        fault(ex)

    if not ret.success:
        fault("Request failed, Check mavros logs")
    else:
        print_if(args.verbose, "Set current done.")


def mavwp_function():
    parser = argparse.ArgumentParser(description="Command line tool for manipulating missions on MAVLink device.")
    parser.add_argument('-n', '--mavros-ns', help="ROS node namespace", default=mavros.DEFAULT_NAMESPACE)
    parser.add_argument('-v', '--verbose', action='store_true', help="Verbose output")
    subarg = parser.add_subparsers()

    if not no_prettytable:
        show_args = subarg.add_parser('show', help="Show waypoints")
        show_args.add_argument('-f', '--follow', action='store_true', help="Watch and show new data")
        show_args.add_argument('-p', '--pull', action='store_true', help="Pull waypoints from FCU before show")
        show_args.set_defaults(func=do_show)

    load_args = subarg.add_parser('load', help="Load waypoints from file")
    load_args.set_defaults(func=do_load)
    load_args.add_argument('-p', '--preserve-home', action='store_true', help="Preserve home location (WP 0, APM only)")
    load_args.add_argument('-s', '--start-index', type=int, default=0, help="Waypoint start index for partial updating (APM only)")
    load_args.add_argument('-e', '--end-index', type=int, default=0, help="Waypoint end index for partial updating (APM only, default: last element in waypoint list)")
    load_args.add_argument('file', type=argparse.FileType('rb'), help="Input file (QGC/MP format)")

    pull_args = subarg.add_parser('pull', help="Pull waypoints from FCU")
    pull_args.set_defaults(func=do_pull)

    dump_args = subarg.add_parser('dump', help="Dump waypoints to file")
    dump_args.set_defaults(func=do_dump)
    dump_args.add_argument('file', type=argparse.FileType('wb'), help="Output file (QGC format)")

    clear_args = subarg.add_parser('clear', help="Clear waypoints on device")
    clear_args.set_defaults(func=do_clear)

    setcur_args = subarg.add_parser('setcur', help="Set current waypoints on device")
    setcur_args.add_argument('seq', type=int, help="Waypoint seq id")
    setcur_args.set_defaults(func=do_set_current)

    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])

    rospy.init_node("mavwp", anonymous=True)
    mavros.set_namespace(args.mavros_ns)

    args.func(args)


# Class to manage flight mode (PX4 stack).
class px4FlightMode:
    
    def __init__(self):
        pass

    def setTakeoff(self):
    	rospy.wait_for_service('mavros/cmd/takeoff')
    	try:
    		takeoffService = rospy.ServiceProxy('mavros/cmd/takeoff', mavros_msgs.srv.CommandTOL)
    		takeoffService(altitude = 2.5)
		rospy.loginfo("Taking off")
    	except rospy.ServiceException as e:
    		rospy.logerror("Takeoff failed: %s"%e)
    
    def setAutoLandMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='AUTO.LAND')
            rospy.loginfo("Landing")
        except rospy.ServiceException as e:
	    rospy.logerror("Landing failed: %s. Autoland Mode could not be set"%e)

    def setArm(self):
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            	armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            	armService(True)
	    	rospy.loginfo("Arming motors OK")
        except rospy.ServiceException as e:
            	rospy.logerror("Arming motors failed: %s"%e)

    def loadMission(self):
	    # Load mission from file (MP format). 
	    # To do: change to avoid using os.system + mavwp
             # Handle errors pending!
        mavwp_function("load ~/catkin_ws/src/mavros_auto_mission/missions/missionwp_file.txt")
	    #os.system("rosrun mavros mavwp load ~/catkin_ws/src/mavros_auto_mission/missions/missionwp_file.txt") # Load new mission
	    rospy.loginfo("Mission WayPoints LOADED!")
        mavwp_function("show")
	    #os.system("rosrun mavros mavwp show") # Show mission WP loaded

    def setAutoMissionMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='AUTO.MISSION')
	    rospy.loginfo("Entering Auto Mission mode OK")
        except rospy.ServiceException as e:
            rospy.logerror("Entering Auto Mission failed: %s. AUTO.MISSION mode could not be set."%e)


    def read_failsafe(self):
        try:	    
	# Gets the current values of PX4 failsafe-related parameters 
	    get = rospy.ServiceProxy(mavros.get_topic('param', 'get'), ParamGet)
	    DLL_param = get(param_id="NAV_DLL_ACT") # Datalink Failsafe PX4 Parameter
	    RCL_param = get(param_id="NAV_RCL_ACT") # RC Failsafe PX4 Parameter    
	    print(" ")
	    print("----------PX4 FAILSAFE STATUS--------------")
	    print(" Present NAV_DLL_ACT value is", DLL_param.value.integer)
	    print(" Present NAV_RCL_ACT value is", RCL_param.value.integer)
	    print("-------------------------------------------")
	    print(" ")
	    return {'DL':DLL_param.value.integer,'RC':RCL_param.value.integer}
        except rospy.ServiceException as e:
            rospy.logerror("Failsafe Status read failed: %s"%e)

    def remove_failsafe(self):
        try:
	    # Disables both Datalink and RC failsafes  
	    val = ParamValue(integer=0, real=0.0) # Int value for disabling failsafe    
	    set = rospy.ServiceProxy(mavros.get_topic('param', 'set'), ParamSet)
	    new_DLL_param = set(param_id="NAV_DLL_ACT", value=val) 
	    new_RCL_param = set(param_id="NAV_RCL_ACT", value=val)    
	    print(" ")
	    print("----------REMOVING PX4 FAILSAFES ----------")
	    print(" New NAV_DLL_ACT value is", new_DLL_param.value.integer)
	    print(" New NAV_RCL_ACT value is", new_RCL_param.value.integer)
	    print("--------------------------------------------")
	    print(" ")
        except rospy.ServiceException as e:
            rospy.logerror("Failsafe Status change failed: %s"%e)


# WP reached Callback function. Controls spam by publishing once per WP!
def WP_callback(msg):
    
    global last_wp # Maybe there is a more elegant way of doint this :-)
    global starting_time
    global mission_started # Bool handles reaching WP#0 twice (mission beginning and end) 
    
    try: 
	mission_started
    except NameError: # Mission begins
	rospy.loginfo("Starting MISSION: Waypoint #0 reached")
	starting_time = msg.header.stamp.secs
	mission_started = True
    else: # Mission ongoing
        if msg.wp_seq == 0 and msg.wp_seq != last_wp: # Returning to WP #0 (last)
	   elapsed_time = msg.header.stamp.secs-starting_time	
	   rospy.loginfo("Ending MISSION: Total time: %d s", elapsed_time)
        elif msg.wp_seq != last_wp: # Intermediate WPs
	   elapsed_time = msg.header.stamp.secs-starting_time	
	   rospy.loginfo("MISSION Waypoint #%s reached. Elapsed time: %d s", msg.wp_seq, elapsed_time)
   
   # Update last WP reached
    last_wp=msg.wp_seq


# Main Function
def main():
   
    rospy.init_node('auto_mission_node', anonymous=True)

   # Flight mode object
    PX4modes = px4FlightMode()

   # Read Datalink and RC failsafe STATUS. Remove if present (for SITL)!
    failsafe_status = PX4modes.read_failsafe()
    if (failsafe_status['DL'] != 0) or (failsafe_status['RC'] != 0):   
        PX4modes.remove_failsafe() 

   # AUTO MISSION: set mode, read WPs and Arm!  
    PX4modes.loadMission()
    PX4modes.setAutoMissionMode()
    PX4modes.setArm()

   # Subscribe to drone state to publish mission updates
    sub=rospy.Subscriber('mavros/mission/reached', WaypointReached, WP_callback)
   
   # Keep main loop
    rospy.spin()


if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass







