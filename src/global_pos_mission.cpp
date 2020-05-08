#include <cstdlib>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/WaypointPush.h>
#include <mavros_msgs/WaypointClear.h>
#include <mavros_msgs/CommandHome.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandCode.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ExtendedState.h>

#include "UtilitiesGeo.h"

//#include <ros_mavros_wp_mission/FlyMissionAction.h>  // Note: "Action" is appended
#include <tt_mavros_wp_mission/FlyMissionAction.h>  // Note: "Action" is appended
#include <actionlib/server/simple_action_server.h>
#include <tt_mavros_wp_mission/WaypointPush.h>
#include <tt_mavros_wp_mission/WaypointPushRequest.h>
#include <tt_mavros_wp_mission/MissionStatus.h>
#include <tt_mavros_wp_mission/StartMission.h>
#include <tt_mavros_wp_mission/StartMissionRequest.h>

#include <ros/console.h>


//typedef actionlib::SimpleActionServer<tt_mavros_wp_mission::FlyMissionAction> Server;

//*****************************************************************************
//*
//* The Actionlib Execute callback method
//* http://wiki.ros.org/actionlib_tutorials/Tutorials/SimpleActionServer%28ExecuteCallbackMethod%29
//*
//*****************************************************************************

//void execute(const tt_mavros_wp_mission::FlyMissionGoalConstPtr& goal, Server* as)  // Note: "Action" is not appended to DoDishes here
//{
  // Do lots of awesome groundbreaking robot stuff here
//  as->setSucceeded();
//}

//*****************************************************************************
//*
//*
//*
//*****************************************************************************

sensor_msgs::NavSatFix global_pose;
void global_pose_cby(const sensor_msgs::NavSatFixConstPtr& msg)
{
  global_pose = *msg;
  ROS_INFO("global_pose_cby() : got global position [%d]: %f, %f, %f", 
    global_pose.header.seq, global_pose.latitude, 
    global_pose.longitude, global_pose.altitude);
}

//*****************************************************************************
//*
//*
//*
//*****************************************************************************

mavros_msgs::State current_state;
void state_cby(const mavros_msgs::State::ConstPtr& msg)
{
  current_state = *msg;
  ROS_INFO("got status: %d, %d", (int)msg->armed, (int)msg->connected);
}

//*****************************************************************************
//*
//*
//*
//*****************************************************************************

int CalcCoords(double lat1, double lon1, double distanceMeters, 
  double bearing, double* lat2, double* lon2)
{
  destCoordsInDegrees(lat1, lon1, 
						 distanceMeters, bearing,
						 lat2, lon2);

  return 0;
}

//*****************************************************************************
//*
//*
//*
//*****************************************************************************

int ChangeCoords(mavros_msgs::Waypoint* waypoint, double distanceMeters, double bearing)
{
  destCoordsInDegrees(waypoint->x_lat, waypoint->y_long, 
						 distanceMeters, bearing,
						 &(waypoint->x_lat), &(waypoint->y_long));

  return 0;
}

//*****************************************************************************
//*
//*
//*
//*****************************************************************************

int main_orig(int argc, char **argv)
{

  int rate_hz = 10;

  ros::init(argc, argv, "global_pos_mission");
  ros::NodeHandle n;

  ros::NodeHandle& nh = n;
  ros::Rate rate(rate_hz);
  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
                              ("mavros/state", 10, state_cby);
  ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
                                 ("mavros/setpoint_position/local", 10);
  ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
                                     ("mavros/cmd/arming");
  ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
                                       ("mavros/set_mode", 1);
  //ros::Subscriber global_pose_sub = nh.subscribe<sensor_msgs::NavSatFix>("mavros/global_position/global", 1, global_pose_cby);

  ros::Subscriber global_pose_sub = nh.subscribe<sensor_msgs::NavSatFix>("mavros/global_position/raw/fix", 1, global_pose_cby);

  global_pose.header.seq = 0;

  // wait for FCU connection
  while(ros::ok() && (!current_state.connected || global_pose.header.seq == 0))
  {
    ros::spinOnce();
    rate.sleep();
  }

  ROS_INFO("main_orig() : got global position [%d]: %f, %f, %f", global_pose.header.seq, global_pose.latitude, global_pose.longitude, global_pose.altitude);

  //****************************

  double distanceMeters = 500;
  double bearing = 0;
  double lat2;
  double lon2;

  CalcCoords(global_pose.latitude, global_pose.longitude, distanceMeters, bearing, &lat2, &lon2);

  ROS_INFO("NEW global position [%d]: %f, %f, %f *************************************", global_pose.header.seq, lat2, lon2, global_pose.altitude);

  //**************************

  ////////////////////////////////////////////
  ///////////////////ARM//////////////////////
  ////////////////////////////////////////////
  ros::ServiceClient arming_cl = n.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
  mavros_msgs::CommandBool srv;
  srv.request.value = true;
  if(arming_cl.call(srv))
  {
    ROS_INFO("ARM send ok %d", srv.response.success);
  }
  else
  {
    ROS_ERROR("Failed arming or disarming");
  }


  ////////////////////////////////////////////
  /////////////////CLEAR MISSION/////////////////
  ////////////////////////////////////////////


  ros::ServiceClient wp_clear_client = n.serviceClient<mavros_msgs::WaypointClear>("/mavros/mission/clear");
  mavros_msgs::WaypointClear wp_clear_srv;
  if (wp_clear_client.call(wp_clear_srv))
  {
    ROS_INFO("Waypoint list was cleared");
  }
  else
  {
    ROS_ERROR("Waypoint list couldn't been cleared");
  }

  ////////////////////////////////////////////
  /////////////////DO MISSION/////////////////
  ////////////////////////////////////////////
  ros::ServiceClient client_wp = n.serviceClient<mavros_msgs::WaypointPush>("mavros/mission/push");

  mavros_msgs::WaypointPush srv_wp;
  srv_wp.request.start_index = 0;
  mavros_msgs::CommandHome set_home_srv;

  mavros_msgs::Waypoint wp;
  // fill wp
  wp.frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
  wp.command = mavros_msgs::CommandCode::NAV_TAKEOFF;
  wp.is_current   = true;
  wp.autocontinue = true;
  wp.param1       = 5;
  wp.z_alt        = 10;
  wp.x_lat        = global_pose.latitude;
  wp.y_long       = global_pose.longitude;
  srv_wp.request.waypoints.push_back(wp);
  
  wp.command      = mavros_msgs::CommandCode::NAV_WAYPOINT;
  wp.is_current   = false;
  wp.autocontinue = true;
  wp.param1       = 5;          // hold time sec
  wp.z_alt        = 50;
  
  //wp.x_lat        = 47.397606;
  //wp.y_long       = 8.545139;
  ChangeCoords(&wp, 200, 0);
  srv_wp.request.waypoints.push_back(wp);

  //wp.x_lat        = 47.3978;
  //wp.y_long       = 8.5450;
  ChangeCoords(&wp, 300, 90);
  srv_wp.request.waypoints.push_back(wp);

  //wp.x_lat        = 47.398;
  //wp.y_long       = 8.5449;
  ChangeCoords(&wp, 200, 180);
  srv_wp.request.waypoints.push_back(wp);

  wp.command      = mavros_msgs::CommandCode::NAV_LAND;
  wp.z_alt        = 0;
  //wp.x_lat        = 47.397506;
  //wp.y_long       = 8.5458;
  ChangeCoords(&wp, 300, 270);
  srv_wp.request.waypoints.push_back(wp);
  //...

  if (client_wp.call(srv_wp))
  {
    // ok, check srv.response
    ROS_INFO("Uploaded WPs!");
    mavros_msgs::State current_state;
  }
  else
  {
    // roscpp error
    ROS_ERROR("Upload Failed");
  }


  ////////////////////////////////////////////
  ////////////////////////////////////////////
  ///////////SET MODE:AUTO.MISSION////////////
  ////////////////////////////////////////////
  ros::ServiceClient cl = n.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
  mavros_msgs::SetMode srv_setMode;
  srv_setMode.request.base_mode = 0; //(uint8_t)mavros_msgs::SetModeRequest::MAV_MODE_AUTO_ARMED;
  srv_setMode.request.custom_mode = "AUTO.MISSION";
  if(cl.call(srv_setMode))
  {
    ROS_INFO("setmode send ok %d value:", srv_setMode.response.mode_sent);
  }
  else
  {
    ROS_ERROR("Failed SetMode");
    return -1;
  }


  ////////////////////////////////////////////
  ////////////////////////////////////////////
  ///////////WAIT UNTIL DISARMED//////////////
  ////////////////////////////////////////////
  current_state.armed = true;
  while(ros::ok() && current_state.armed )
  {
    ros::spinOnce();
    rate.sleep();
  }
  ROS_INFO("Drone disarmed");


  return 0;
}

//####################################################################################
//####################################################################################

typedef actionlib::SimpleActionServer<tt_mavros_wp_mission::FlyMissionAction> Server;

/*void execute(const tt_mavros_wp_mission::FlyMissionGoalConstPtr& goal, Server* as)  // Note: "Action" is not appended to DoDishes here
{
  ROS_INFO("Executed");
  
  // Do lots of awesome groundbreaking robot stuff here
  as->setSucceeded();
}

int mainy(int argc, char** argv)
{
  ros::init(argc, argv, "GPMission_Server");
  ros::NodeHandle n;
  Server server(n, "GPMission", boost::bind(&execute, _1, &server), false);
  server.start();
  ROS_INFO("Server Started");
  ros::spin();
  return 0;
}*/

//####################################################################################
//####################################################################################

class GPMission
{
  protected:

    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<tt_mavros_wp_mission::FlyMissionAction> as_;
    std::string action_name_;
    int goal_ = 0;
    int rate_hz = 10;
    int statusPublishRate = 1; // 1 Hz

    tt_mavros_wp_mission::MissionStatus missionStatus_;
    std::string landedStateName_ = "Undefined";

    tt_mavros_wp_mission::FlyMissionFeedback feedback_;
    tt_mavros_wp_mission::FlyMissionResult result_;

    ros::ServiceServer WayointPush_service_;
    ros::ServiceServer StartMission_service_;

    ros::ServiceClient arming_client;
    ros::ServiceClient set_mode_client;
    ros::ServiceClient mavros_mission_push_client;

    boost::thread missionThread_;
    boost::thread* statusPublishThread_;

    sensor_msgs::NavSatFix global_pose;
    mavros_msgs::State current_state;
    mavros_msgs::ExtendedState current_extended_state;

    // Register Subscribers
    ros::Subscriber extended_state_sub;
    ros::Subscriber state_sub;
    ros::Subscriber global_pose_sub;

    // Register Publishers                                
    ros::Publisher local_pos_pub;
    ros::Publisher mission_status_pub;

  private:

    //*****************************************************************************
    //*
    //*
    //*
    //*****************************************************************************

    void global_pose_cb(const sensor_msgs::NavSatFixConstPtr& msg)
    {
      global_pose = *msg;
      //ROS_INFO("global_pose_cb() : got global position [%d]: %f, %f, %f", 
      //  global_pose.header.seq, global_pose.latitude, global_pose.longitude, global_pose.altitude);
    }

    //*****************************************************************************
    //*
    //*
    //*
    //*****************************************************************************

    void state_cb(const mavros_msgs::State::ConstPtr& msg)
    {
      current_state = *msg;
      //ROS_INFO("got status: %d, %d", (int)msg->armed, (int)msg->connected);
    }

    //*****************************************************************************
    //*
    //*
    //*
    //*****************************************************************************

    void extended_state_cb(const mavros_msgs::ExtendedState::ConstPtr& msg)
    {
      //if(null != current_extended_state)
        if(current_extended_state.landed_state == msg->landed_state)
          return;

      //std::string landedStateName_ = "Undefined";

      switch(msg->landed_state)
      {
        case mavros_msgs::ExtendedState::LANDED_STATE_UNDEFINED:
          landedStateName_ = "LANDED_STATE_UNDEFINED";
        break;
        case mavros_msgs::ExtendedState::LANDED_STATE_TAKEOFF:
          landedStateName_ = "LANDED_STATE_TAKEOFF";
        break;
        case mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND:
          landedStateName_ = "LANDED_STATE_ON_GROUND";
        break;
        case mavros_msgs::ExtendedState::LANDED_STATE_LANDING:
          landedStateName_ = "LANDED_STATE_LANDING";
        break;
        case mavros_msgs::ExtendedState::LANDED_STATE_IN_AIR:
          landedStateName_ = "LANDED_STATE_IN_AIR";
        break;
      }

      ROS_INFO("extended_state_cb(%d) : %s", (int)msg->landed_state, landedStateName_.c_str());

      current_extended_state = *msg;
    }

    //*************************************************************************
    //*
    //*************************************************************************

    int Arm(bool arming)
    {
      result_.dummy_result = 0;

      ros::ServiceClient arming_cl = 
        nh_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");

      mavros_msgs::CommandBool srv;
      srv.request.value = arming;

      if(arming_cl.call(srv))
      {
        if(arming)
          ROS_INFO("Armed ok %d", srv.response.success);
        else
          ROS_INFO("Disarmed ok %d", srv.response.success);
      }
      else
      {
        if(arming)
          ROS_ERROR("Arming Failed");
        else
          ROS_ERROR("Disarming Failed");
        return -1;
      }

      return 0;
    }

    //*************************************************************************
    //*
    //*************************************************************************

    int ClearWayponts()
    {
      ros::ServiceClient wp_clear_client = 
        nh_.serviceClient<mavros_msgs::WaypointClear>("/mavros/mission/clear");

      mavros_msgs::WaypointClear wp_clear_srv;
      
      if (wp_clear_client.call(wp_clear_srv))
      {
        ROS_INFO("Waypoint list was cleared");
      }
      else
      {
        ROS_ERROR("Waypoint list couldn't been cleared");
        return -1;
      }

      return 0;
    }

    //*************************************************************************
    //*
    //*************************************************************************

    int LoadTestWayponts()
    {
      ROS_INFO("LoadTestWayponts() Entered");

      result_.dummy_result = 0;

      ros::ServiceClient client_wp = 
        nh_.serviceClient<mavros_msgs::WaypointPush>("mavros/mission/push");

      mavros_msgs::WaypointPush srv_wp;
      srv_wp.request.start_index = 0;
      mavros_msgs::CommandHome set_home_srv;

      ROS_INFO("LoadTestWayponts() global position [%d]: %f, %f, %f", 
        global_pose.header.seq, global_pose.latitude, 
        global_pose.longitude, global_pose.altitude);

      mavros_msgs::Waypoint wp;
      
      // takeoff
      wp.frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
      wp.command = mavros_msgs::CommandCode::NAV_TAKEOFF;
      wp.is_current   = true;
      wp.autocontinue = true;
      wp.param1       = 5;
      wp.z_alt        = 10;
      wp.x_lat        = global_pose.latitude;
      wp.y_long       = global_pose.longitude;
      srv_wp.request.waypoints.push_back(wp);

      // waypoints
      wp.command      = mavros_msgs::CommandCode::NAV_WAYPOINT;
      wp.is_current   = false;
      wp.autocontinue = true;
      wp.param1       = 2;          // hold time sec
      wp.z_alt        = 30;
      
      //wp.x_lat        = 47.397606;
      //wp.y_long       = 8.545139;
      ChangeCoords(&wp, 20, 0);
      srv_wp.request.waypoints.push_back(wp);

      //wp.x_lat        = 47.3978;
      //wp.y_long       = 8.5450;
      ChangeCoords(&wp, 20, 90);
      srv_wp.request.waypoints.push_back(wp);

      //wp.x_lat        = 47.398;
      //wp.y_long       = 8.5449;
      ChangeCoords(&wp, 20, 180);
      srv_wp.request.waypoints.push_back(wp);

      // Land
      wp.command      = mavros_msgs::CommandCode::NAV_LAND;
      wp.z_alt        = 0;
      //wp.x_lat        = 47.397506;
      //wp.y_long       = 8.5458;
      ChangeCoords(&wp, 20, 270);
      srv_wp.request.waypoints.push_back(wp);
      //...

      DisplayWaypointMission(srv_wp);

      if (client_wp.call(srv_wp))
      {
        // ok, check srv.response
        ROS_INFO("Uploaded WPs!");
        mavros_msgs::State current_state;
      }
      else
      {
        // roscpp error
        ROS_ERROR("Upload Failed");
        return -1;
      }

      return 0;
    }

    //*************************************************************************
    //*
    //*************************************************************************

    int StartMission()
    {
      result_.dummy_result = 0;
      feedback_.percent_complete = 0.0;

      ros::ServiceClient cl = nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
      mavros_msgs::SetMode srv_setMode;
      srv_setMode.request.base_mode = 0; //(uint8_t)mavros_msgs::SetModeRequest::MAV_MODE_AUTO_ARMED;
      srv_setMode.request.custom_mode = "AUTO.MISSION";

      if(cl.call(srv_setMode))
      {
        ROS_INFO("setmode send ok %d value:", srv_setMode.response.mode_sent);
      }
      else
      {
        ROS_ERROR("Failed SetMode");
        return -1;
      }

      return 0;
    }

    //*************************************************************************
    //*
    //*************************************************************************

    int WaitTillDisarmed()
    {
      ros::Rate rate(rate_hz);
      current_state.armed = true;

      tt_mavros_wp_mission::MissionStatus missionStatus;

      while(ros::ok() && current_state.armed )
      {
        feedback_.percent_complete += .001; //*** TODO * This is a fake increment, make it reflect reality

        if( as_.isActive() ) 
          as_.publishFeedback(feedback_);

        missionStatus.x_lat = global_pose.latitude;
        missionStatus.y_long = global_pose.longitude;
        missionStatus.z_alt = global_pose.altitude;

        mission_status_pub.publish(missionStatus);

        ros::spinOnce();
        rate.sleep();
      }
      ROS_INFO("Drone disarmed");

      feedback_.percent_complete = 100;
      result_.dummy_result = 100;

      return 0;
    }

    //*************************************************************************
    //*
    //*************************************************************************

    bool TotalFake = false;
    
    void RunTestMission()
    {
      result_.dummy_result = 0;
      feedback_.percent_complete = 0.0;

      if(TotalFake)
      {
        ros::Rate rate(rate_hz);
        for(int index = 0; index < 10; index++)
        {
          if( as_.isActive() ) 
            as_.publishFeedback(feedback_);

          feedback_.percent_complete += 1; 
          ros::spinOnce();
          rate.sleep();
        }
        feedback_.percent_complete = 100;
        result_.dummy_result = 100;
      }
      else
      {
        Arm(true);
        ClearWayponts();
        LoadTestWayponts();
        StartMission();
        WaitTillDisarmed();
        Arm(false);
      }

      if( as_.isActive() ) 
        as_.setSucceeded(result_);
    }

    //*************************************************************************
    //*
    //*************************************************************************
    
    void RunMission()
    {
        Arm(true);
        StartMission();
        WaitTillDisarmed();
        Arm(false);
    }

    //*************************************************************************
    //*
    //*************************************************************************
    
    void ActionGoalCB()
    {
      ros::Rate rate(10);
      ROS_INFO("GPMission::ActionGoalCB() called");
      rate.sleep();

      goal_ = as_.acceptNewGoal()->dummy_goal;

      missionThread_ = boost::thread( boost::bind(&GPMission::RunTestMission, this) );
    }

    // https://www.google.com/search?client=ubuntu&channel=fs&q=You+are+attempting+to+call+methods+on+an+uninitialized+goal+handle&ie=utf-8&oe=utf-8
    // https://answers.ros.org/question/266887/you-are-attempting-to-call-methods-on-an-uninitialized-goal-handle/
    /*void ActiongoalCb()
    {
      bool success = true;
      goal_ = *(as_.acceptNewGoal());   
      
      if( as_.isActive() ) 
      {
        if(success)
        {
          result_.text = "ActiongoalCb Succeeded";
          ROS_INFO("%s: ActiongoalCb Succeeded", action_name_.c_str());
          as_.setSucceeded(result_);
        }
      }
    }*/

    //*************************************************************************
    //*
    //*************************************************************************
    
    void ActionPreemptCB()
    {
      ROS_INFO("%s: Preempted", action_name_.c_str());
      as_.setPreempted();
    }

    //***************************************************************************
    //
    //
    //
    //***************************************************************************
   
    void DisplayWaypointMission( mavros_msgs::WaypointPush srv_wp )
    {
      ROS_INFO("--- Waypoint Mission -----------");

      ROS_INFO("start_index = %i, count = %i", 
        srv_wp.request.start_index, srv_wp.request.waypoints.size());

			for (auto &wp : srv_wp.request.waypoints) 
      {
        ROS_INFO("WP : cmd:%i frame:%i current:%i continue:%i lat:%f lon:%f alt:%f p1:%f p2:%f p:3%f p4:%f", 
          wp.command, wp.frame, wp.is_current, wp.autocontinue, 
          wp.x_lat, wp.y_long, wp.z_alt, 
          wp.param1, wp.param2, wp.param3, wp.param4);
			}

      ROS_INFO("-------------------------------");

    }

    //***************************************************************************
    //
    //
    //
    //***************************************************************************
   
    bool WaypointPushServiceCallback( 
      tt_mavros_wp_mission::WaypointPush::Request& request, 
      tt_mavros_wp_mission::WaypointPush::Response& response)
    {
      ROS_INFO("'WaypointPush_service' Service Called");

      mavros_msgs::WaypointPush srv_wp;
      srv_wp.request.start_index = request.start_index;
      //auto wpIn = request.waypoint;
      
      ROS_INFO("'WaypointPush_service' start_index = %i, count = %i", 
        srv_wp.request.start_index, request.waypoints.size());

      ClearWayponts();

			for (auto &wpIn : request.waypoints) 
      {
        mavros_msgs::Waypoint wp;
        // fill wp
        wp.frame        = wpIn.frame;
        wp.command      = wpIn.command;
        wp.is_current   = wpIn.is_current;
        wp.autocontinue = wpIn.autocontinue;
        wp.param1       = wpIn.param1;
        wp.param2       = wpIn.param2;
        wp.param3       = wpIn.param3;
        wp.param4       = wpIn.param4;
        wp.z_alt        = wpIn.z_alt;
        wp.x_lat        = wpIn.x_lat;
        wp.y_long       = wpIn.y_long;

        srv_wp.request.waypoints.push_back(wp);
			}

      DisplayWaypointMission(srv_wp);

      if (mavros_mission_push_client.call(srv_wp))
      {
        // ok, check srv.response
        ROS_INFO("WP Upload Success");
        mavros_msgs::State current_state;
        response.success = true;
        response.wp_transfered = (int)srv_wp.request.waypoints.size();
      }
      else
      {
        // roscpp error
        ROS_ERROR("WP Upload Failed");
        response.success = false;
        response.wp_transfered = 0;
      }

      return true;
    }

    //*************************************************************************
    //*
    //*************************************************************************
    
    bool StartMissionServiceCallback( 
      tt_mavros_wp_mission::StartMission::Request& request, 
      tt_mavros_wp_mission::StartMission::Response& response)
    {
      ROS_INFO("StartMissionServiceCallback() Called");

      missionThread_ = boost::thread( boost::bind(&GPMission::RunMission, this) );

      response.result = "it's ok";

      return true;    
    }

    //*************************************************************************
    //*
    //*************************************************************************
    
    void StatusPublishWorker()
    {
      ros::Rate loop_rate(statusPublishRate);

      while (ros::ok())
      {
        missionStatus_.x_lat = global_pose.latitude;
        missionStatus_.y_long = global_pose.longitude;
        missionStatus_.z_alt = global_pose.altitude;

        missionStatus_.landed_state = landedStateName_;

        mission_status_pub.publish(missionStatus_);

        ros::spinOnce();
        loop_rate.sleep();
      }
    }

    //*************************************************************************
    //*
    //*************************************************************************
    
    void InitStatus()
    {
      current_extended_state.landed_state = mavros_msgs::ExtendedState::LANDED_STATE_UNDEFINED;

      global_pose.latitude = 0;
      global_pose.longitude = 0;
      global_pose.altitude = 0;

      result_.dummy_result = 0;
      feedback_.percent_complete = 0.0;

      // Register Status Publisher
      mission_status_pub = nh_.advertise<tt_mavros_wp_mission::MissionStatus>
        ("tt_mavros_wp_mission/MissionStatus", 10);

      // Start status publisher thread
      statusPublishThread_ = new boost::thread(boost::bind(&GPMission::StatusPublishWorker, this));
    }

    //*************************************************************************
    //*
    //*************************************************************************
    
    int Init()
    {
      ros::Rate rate(10);

      InitStatus();

      if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) 
      {
        ros::console::notifyLoggerLevelsChanged();
      }

      ROS_DEBUG("GPMission::Init() called, Action Name: %s", action_name_.c_str());
      rate.sleep();
      
      // Register action callbacks
      as_.registerGoalCallback(boost::bind(&GPMission::ActionGoalCB, this));
      as_.registerPreemptCallback(boost::bind(&GPMission::ActionPreemptCB, this));
         
      // Register Service Providers  
      WayointPush_service_ = nh_.advertiseService(
        "/tt_mavros_wp_mission/WaypointPush_service", &GPMission::WaypointPushServiceCallback, this);
      StartMission_service_ = nh_.advertiseService(
        "/tt_mavros_wp_mission/StartMission_service", &GPMission::StartMissionServiceCallback, this);

      // Register Service Clients
      arming_client = nh_.serviceClient<mavros_msgs::CommandBool>
        ("mavros/cmd/arming");
      set_mode_client = nh_.serviceClient<mavros_msgs::SetMode>
        ("mavros/set_mode", 1);
      mavros_mission_push_client = nh_.serviceClient<mavros_msgs::WaypointPush>
        ("mavros/mission/push");

      // Register Subscribers
      extended_state_sub = nh_.subscribe<mavros_msgs::ExtendedState>
        ("mavros/extended_state", 10, &GPMission::extended_state_cb, this);

      state_sub = nh_.subscribe<mavros_msgs::State>
        ("mavros/state", 10, &GPMission::state_cb, this);

      //global_pose_sub = nh_.subscribe<sensor_msgs::NavSatFix>
      //  ("mavros/global_position/global", 1, &GPMission::global_pose_cb, this);

      global_pose_sub = nh_.subscribe<sensor_msgs::NavSatFix>
        ("mavros/global_position/raw/fix", 1, &GPMission::global_pose_cb, this);   

      // Register Publishers                                
      local_pos_pub = nh_.advertise<geometry_msgs::PoseStamped>
        ("mavros/setpoint_position/local", 10);


      global_pose.header.seq = 0;
    
      ROS_DEBUG("GPMission::Init() waiting for current_state.connected ...");
      rate.sleep();

      // wait for FCU connection
      while(ros::ok() && (!current_state.connected || global_pose.header.seq == 0))
      {
        ros::spinOnce();
        rate.sleep();
      }

      ROS_INFO("Init() : got global position [%d]: %f, %f, %f", 
        global_pose.header.seq, global_pose.latitude, global_pose.longitude, global_pose.altitude);
      rate.sleep();
      
      as_.start();

      return 0;
    }

  public:

    GPMission(std::string name) :
      as_(nh_, name, false), action_name_(name)
    {
      //ROS_DEBUG("GPMission(%s) called", name);
      Init();

      //missionThread_ = boost::thread( boost::bind(&GPMission::RunTestMission, this) );
    }
};

int main(int argc, char** argv)
{
  std::string ServiceName = "tt_mavros_mission/RunWpMission";
  ros::init(argc, argv, "tt_mavros_mission");
  GPMission gPMission(ServiceName);
  ros::spin();

  return 0;
}