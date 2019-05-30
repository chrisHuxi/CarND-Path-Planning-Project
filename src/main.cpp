#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"

// for spline function to generate smoother trajectory
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;


int main() {
  uWS::Hub h;
  
  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  static double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }
  
  //=============global variable===========//
  double ref_velocity = 0.0; //start velocity: 0 mph
  int lane = 1; //lane：0，1，2
  //=============global variable===========//
  //don't forget add to parameter list in lambda function!!!
  h.onMessage([&ref_velocity, &lane, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
          //============set parameter===========//
          double max_vel = 49.5; //limit max velocity：<=50 kmp
          double max_acc = 10.0; //limit max Acceleration: <=10m/s
          //==========set parameter end==========//
          
          //==========================part1. make path smoother========================//
          int prev_size = previous_path_x.size();
          //previous_path means the trajectory we generated last time
          
          double ref_x;
          double ref_y;
          double ref_yaw;
          ref_x = car_x;
          ref_y = car_y;
          ref_yaw = deg2rad(car_yaw);//deg2rad: helpers function
          //later they will be used to convert Coordinate System
          
          vector<double> anchor_x;
          vector<double> anchor_y;
          //need smooth trajectory ==> need use spline function ==> need anchor points to determine spline
          
          if(prev_size >= 2)//if waypoints of trajectory last time more then 2: use this 2 waypoints as anchor points 
          {
            //careful: firstly add previous points 
            anchor_x.push_back(previous_path_x[prev_size - 2]);
            anchor_y.push_back(previous_path_y[prev_size - 2]);
            
            anchor_x.push_back(previous_path_x[prev_size - 1]);
            anchor_y.push_back(previous_path_y[prev_size - 1]);
            
            double ref_x_prev;
            double ref_y_prev;
            ref_x_prev = previous_path_x[prev_size - 2];
            ref_y_prev = previous_path_y[prev_size - 2];
            
            ref_x = previous_path_x[prev_size - 1];
            ref_y = previous_path_y[prev_size - 1];
            ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);
            //use difference of (x,y) between 2 waypoints, calculate ref_yaw ==> used to convert Coordinate System
            //here set ref_x,ref_y to (x,y) of last point of previous trajectory, 
            //  because this is car's position in future.
          }
          else//if amount of waypoints < 2 (eg. initialization), use current velocity and location to calculate next location
          {
            double current_x = car_x;
            double current_y = car_y;
            double last_x = car_x - cos(car_yaw) * 0.1;
            double last_y = car_y - sin(car_yaw) * 0.1;
            //this calculation from video seems weird. I modified a little bit.
            //my explanation: here we assume average velocity is 5 m/s: 5 *0.02 = 0.1
            
            anchor_x.push_back(last_x);
            anchor_y.push_back(last_y);
            
            anchor_x.push_back(current_x);
            anchor_y.push_back(current_y);
          }
          
          //generate smoother trajectory: set a target far from current position, then add it to spline
          vector<double> next_xy_1 = getXY((car_s + 50),(lane*4 + 2),map_waypoints_s,map_waypoints_x,map_waypoints_y);
          vector<double> next_xy_2 = getXY((car_s + 100),(lane*4 + 2),map_waypoints_s,map_waypoints_x,map_waypoints_y);
          vector<double> next_xy_3 = getXY((car_s + 150),(lane*4 + 2),map_waypoints_s,map_waypoints_x,map_waypoints_y);
          
          anchor_x.push_back(next_xy_1[0]);
          anchor_x.push_back(next_xy_2[0]);
          anchor_x.push_back(next_xy_3[0]);
          
          anchor_y.push_back(next_xy_1[1]);
          anchor_y.push_back(next_xy_2[1]);
          anchor_y.push_back(next_xy_3[1]);
          

          //in order to make calculation more convenient: convert coordinate system of "anchor" to make ego-position as original point
          // and that's why we calculate ref_x & ref_y.
          for(int i =0; i<anchor_x.size(); i++)
          {
            //shift
            double shift_x = anchor_x[i] - ref_x;
            double shift_y = anchor_y[i] - ref_y;
            
            //rotation
            anchor_x[i] = (shift_x*cos(0-ref_yaw) - shift_y*sin(0-ref_yaw));
            anchor_y[i] = (shift_x*sin(0-ref_yaw) + shift_y*cos(0-ref_yaw));
          }
          
          //back to spline
          tk::spline s;
          s.set_points(anchor_x,anchor_y);
          //==========================part1.end========================//
          
          //==================part2. avoid collision check==================//

          double current_car_s = car_s; //record current position, will be used to check feasibility of change lane.
          
          //firstly check if there is a planned trajectory? 
          //  if yes, we use last waypoint of planned trajectory as ego-position in the future.
          if (prev_size > 0)
          {
            car_s = end_path_s;
          }
          
          //flag: ego-car is too close to the car forward
          bool too_close = false;
          
          //flag: find cars around eog-car on other lanes?
          bool find_other_car_left = false;
          bool find_other_car_right = false;
          
          //flag: no possiblity to change lane: default as true, i.e. can't change lane in default case
          bool no_change_lane_left = true;
          bool no_change_lane_right = true;
   
          //read data form sensor fusion, check the distance between other cars and ego-car
          for(int i=0;i<sensor_fusion.size();i++)
          {
            // [ id, x, y, vx, vy, s, d]
            double other_car_d = sensor_fusion[i][6];
            //check if d is in front of us (on the same lane) and too close: slow down
            if( (other_car_d > lane*4+2 -2) &&  (other_car_d < lane*4+2 +2) )
            {
              //use "s" : more convenient
              double other_car_s = sensor_fusion[i][5];
              
              //use velocity to infer position in the future
              double other_car_vx = sensor_fusion[i][3];
              double other_car_vy = sensor_fusion[i][4];
              double other_car_v = sqrt(other_car_vx*other_car_vx + other_car_vy*other_car_vy);
              double other_car_s_future = other_car_s + other_car_v * (double)prev_size * 0.02;
              
              //once the car is too close, set too_close to true
              if((other_car_s_future - car_s) < 25 && (other_car_s_future - car_s) > 0)
              {
                too_close = true;
              }
            }
          }
          
          //once the car in front of ego-car is too close: check if it is possible to change lane?
          if(too_close == true)
          {
            for(int i=0;i<sensor_fusion.size();i++)
            {  
              // [ id, x, y, vx, vy, s, d]
              double other_car_d = sensor_fusion[i][6];
              
              //check cars on the left lane
              if( (other_car_d > (lane - 1) *4+2 -2)  &&  (other_car_d < (lane - 1)*4+2 +2) )
              {
                
                double other_car_s = sensor_fusion[i][5];
                
                //check s-distance
                double distance = other_car_s - current_car_s;
                
                //if s-distance too close( < 35 ), or there is a car behind too close( >= -15) 
                //  we set find-car-flag as true
                // notice: here the distance threshold (>35) must be farther then too-close's threshold (>25)
                //    otherwise, it could cause changing lane back and forth.
                if((distance <= 35) && (distance >= -15) )
                {
                  find_other_car_left = true;
                  break;
                }
              } 
            }
            
            //if we check all the cars on the left lane far away: set non-car-flag to false
            if(find_other_car_left == false)
            {
              no_change_lane_left = false;
            }

            //if left-change impossible, or ego-car is on the leftmost lane, check right lane
            if(no_change_lane_left == true || lane == 0)
            {
              for(int i=0;i<sensor_fusion.size();i++)
              {
                // [ id, x, y, vx, vy, s, d]
                double other_car_d = sensor_fusion[i][6];                
                if( (other_car_d > (lane + 1)*4+2 -2) &&  (other_car_d < (lane + 1)*4+2 +2) )
                {
                  double other_car_s = sensor_fusion[i][5];
                  double distance = other_car_s - current_car_s;
                  
                  //if s-distance too close( < 35 ), or there is a car behind too close( >= -15) 
                  //  we set find-car-flag as true
                  // notice: here the distance threshold (>35) must be farther then too-close's threshold (>25)
                  //    otherwise, it could cause changing lane back and forth.
                  if((distance <= 35) && (distance >= -15) )
                  {
                    find_other_car_right = true;
                    break;
                  }
                } 
              }
              
              //if we check all the cars on the left lane far away: set non-car-flag to false
              if(find_other_car_right == false)
              {
                no_change_lane_right = false;
              }
            }
          }
         
          //==========================part2.end========================//
          
          //===============part3. solve cold start issue and slow down or change lane if too close===============//
          if(too_close == true)
          {
            ref_velocity -= 0.02 * (max_acc * 0.5) * 2.24;
            //acc: meter/s vel:kilo miles per hour ==> 2.24
            
            if(no_change_lane_left == false && lane > 0)
            {
                lane -= 1; 
            }
            else if(no_change_lane_right == false && lane < 2)
            {
                lane += 1;
            }
            
          }
          //solve cold start issue: accelerate from 0 m/s
          else if (ref_velocity < 49.5)
          {
            ref_velocity += 0.02 * (max_acc * 0.5) * 2.24;
          }
          //=======================================part3.end=====================================//
          
          //===================part4. generate waypoints according to spline================//

          //put planned waypoints of last time, which we still didn't go throung, into our new planning waypoints 
          //  so that we can save a lot of calculation
          for(int i=0; i<prev_size; i++)
          {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }
          
          // set target ahead of ego-car 30 meters
          // here we can see the convenience of converting coordinate system
          double target_x = 30; 
          double target_y = s(target_x);
          double target_distance = sqrt(target_x*target_x + target_y*target_y);
          
          //change the ref_vel from miles/h into m/s
          double ref_vel_in_meter = ref_velocity/2.24; // 1 meter per second = 2.236... miles per hour
          
          //determine how many intervals are there from current ego-position to target
          double N = (target_distance/(0.02*ref_vel_in_meter)); //how many intervals in linear distance 
          double next_x = 0;
          double next_y = 0;
          
          for(int i=0; i<50 - prev_size; i++)
          {
            //according to target and spline, generate waypoints
            next_x = (i+1)*(target_x/N);
            next_y = s(next_x);
            
            //convert coordinate system into global coordinate system
            double global_x = ref_x + (next_x*cos(ref_yaw - 0.0) - next_y*sin(ref_yaw - 0.0));
            double global_y = ref_y + (next_x*sin(ref_yaw - 0.0) + next_y*cos(ref_yaw - 0.0));            

            next_x_vals.push_back(global_x);
            next_y_vals.push_back(global_y);
          }
          //==========================part4.end========================//          
          
          //TODO END
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}