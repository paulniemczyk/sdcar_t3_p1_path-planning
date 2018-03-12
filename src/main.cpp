#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2((map_y-y),(map_x-x));

	double angle = fabs(theta-heading);
  angle = min(2*pi() - angle, angle);

  if(angle > pi()/4)
  {
    closestWaypoint++;
  if (closestWaypoint == maps_x.size())
  {
    closestWaypoint = 0;
  }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}

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
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
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

  

  /************* my code ********************/

  // Note that you have to pass these as references in the lambda function def'n in h.onMessage(), below.

  int lane = 1;             // 3 lanes; lane 0 is inside, lane 2 is outside
  double ref_vel = 0;       // reference velocity in mph; start at 0mph
  
  /************* end my *********************/



  h.onMessage([&ref_vel, &lane, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
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

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	//auto sensor_fusion = j[1]["sensor_fusion"];
            vector<vector<double>> sensor_fusion = j[1]["sensor_fusion"];
            

            

            /********************** MY CODE *****************************************/

            // Variables
           
            const double SPEED_LIMIT = 50.0;    // max speed allowed on track
            const double SPEED_MAX = 49.5;      // max speed we want car to go; project track limit is 50mph
            const double START_SPEED = 0.0;     // speed we want car to start (low, to avoid sudden jerk)
  

            int prev_size = previous_path_x.size();     // how many leftover pts were there in previous path plan

          
            /*************************************************************************
              Use sensor fusion list to see if we're too close to other cars.    
              If one is close, flip relevant boolean flags and adjust.

            **************************************************************************/ 

            if (prev_size > 0) 
            {
              car_s = end_path_s;   // change car's s-value to last point of prior path --- why?
            }

            bool car_in_front = false;
            bool car_to_left = false;
            bool car_to_right = false;
            bool car_in_lane_left = false;
            bool car_in_lane_right = false;
            
            // debug
            cout << "car_in_front = " << car_in_front << "   car_in_lane_left = " << car_in_lane_left << "  car_to_left = " << car_to_left << 
                  "  car_in_lane_right = " << car_in_lane_right << "  car_to_right = " << car_to_right << endl;

            
            // Loop through all cars in the current sensor map; figure out what to do

            for (int i=0; i < sensor_fusion.size(); i++)
            {
              
              // If car is in my lane
              float d = sensor_fusion[i][6];    // extract d value for other car
              if (d < (2+4*lane+2) && d > (2+4*lane-2))   // Lanes are exactly 4m wide
              {

                // Is car within a dangerous range in front of me?
                double vx = sensor_fusion[i][3];
                double vy = sensor_fusion[i][4];
                double check_speed = sqrt(vx*vx + vy*vy);
                double check_car_s = sensor_fusion[i][5];

                // We're currently sitting on prior path, and we need to project out where the car will be in the future
                check_car_s += ((double)prev_size*0.02*check_speed);  // if using previous points can project s value outwards in time...
                                                                      // ...b/c our car isn't there yet since we're using previous pathpoints

                // Check if car is in front of me within 30m: s values greater than mine and s gap less than 30 meters
                // i.e., if our car in the future is within 30m of this car in the future, then we need to take some action
                if ((check_car_s > car_s) && ((check_car_s-car_s) < 30))
                {
                  // Take action to avoid car in front of us
                  // Lower speed, maybe change lanes
                  //ref_vel = 29.5;   // mph. Speed is lowered (but never goes back up)
                  car_in_front = true;
                  cout << "car_in_front = " << car_in_front << "   car_in_lane_left = " << car_in_lane_left << "  car_to_left = " << car_to_left << 
                        "  car_in_lane_right = " << car_in_lane_right << "  car_to_right = " << car_to_right << endl;
                  
                  
                  // PRIORITIZE CHANGING LANES, SINCE THERE IS A CAR IN FRONT
                  // Go through the list of cars again and identify if there is a car in the nearest
                  // lane we want to change into, within some s-gap (30m?), and if it's not safe to switch then stay in lane.
                  // What we really want here here is a good cost function and a better finite state machine... look into the future and
                  // try to see what's the best lane to be in in 5 seconds or so
                  // If we can't change lane, then slow down.
                  //

                  // NOTE: The car doesn't pass in lane 2 (i.e., the right-most lane); i.e., if ego is in lane 1,
                  //    it only looks in lane 0 for a passing opportunity. 
                  
                  if (lane > 0)   // we are in right two lanes; try to switch one lane to left
                  {
                    // Loop through all cars to see if they're in one lane to the left and to see if they're in dangerous range
                    for (int j=0; j < sensor_fusion.size(); j++) 
                    {
                      // Is there a car in the lane to my left?
                      double d_left = sensor_fusion[j][6];
                      if (d_left < (2+4*lane-2) && (d_left > 2+4*lane-6)) 
                      {
                        // There is a car to my left
                        car_in_lane_left = true;

                        // See if that car is in a range of a few m behind me and some meters in front of me
                        double vx_left = sensor_fusion[j][3];
                        double vy_left = sensor_fusion[j][4];
                        double check_speed_left = sqrt(vx_left*vx_left + vy_left*vy_left);
                        double check_car_s_left = sensor_fusion[j][5];
                        check_car_s_left += ((double)prev_size*0.02*check_speed_left);  // Calculate at next frame

                        // Check if car on left is in a dangerous range
                        if ((check_car_s_left > car_s - 5) && ((check_car_s_left - car_s) < 30))
                        {
                          car_to_left = true;
                          cout << "car_in_front = " << car_in_front << "   car_in_lane_left = " << car_in_lane_left << "  car_to_left = " << car_to_left << 
                                "  car_in_lane_right = " << car_in_lane_right << "  car_to_right = " << car_to_right << endl;
                        }
                      } // end if car in lane to my left 
                    }   // end loop through all cars
                    
                    if (!car_to_left)
                    {
                      lane--;   // If no car to left, shift to left
                    }
                  }  // end if (lane>0)
                  else
                  {
                    // We must be in lane 0; check cars to the right via same loop as above

                    for (int j=0; j < sensor_fusion.size(); j++)
                    {
                      double d_right = sensor_fusion[j][6];
                      if (d_right > 4 && d_right < 8)   // Car is in lane 1
                      {
                        // There is a car to my right
                        car_in_lane_right = true;

                        // See if that car is in a range of a few m behind me and some meters in front of me
                        double vx_right = sensor_fusion[j][3];
                        double vy_right = sensor_fusion[j][4];
                        double check_speed_right = sqrt(vx_right*vx_right + vy_right*vy_right);
                        double check_car_s_right = sensor_fusion[j][5];
                        check_car_s_right += ((double)prev_size*0.02*check_speed_right);  // Calculate at next frame
                      
                        // Check if car on right is in a dangerous range
                        if ((check_car_s_right > car_s - 5) && ((check_car_s_right - car_s) < 30))
                        {
                          car_to_right = true;
                          cout << "car_in_front = " << car_in_front << "   car_in_lane_left = " << car_in_lane_left << "  car_to_left = " << car_to_left << 
                                "  car_in_lane_right = " << car_in_lane_right << "  car_to_right = " << car_to_right << endl;
                        } // end if car on right s in dangerous range
                      } // end if car in lane to my right in lane 1
                    } // end loop through all cars to see if they're on right and in dangerous range

                    if (!car_to_right)
                    {
                      lane++;
                    }
                  } // end else (must be lane 0)

                } // end check if car is in front within range of me
              } // end check if car in my lane
            } // loop through each car in sensor map

            if (car_in_front)
            {
              ref_vel -= 0.224; // (around 5m/s) slow down a little
            }
            else if (ref_vel < SPEED_MAX)
            {
              ref_vel += 0.224; // if it's not too close and we're under speed limit, bump up speed
            }



            /*************************************************************************
              We are going to build a smooth path from where the car is.
              Look at prior path plan.
              1) Create 5 new waypoints following the path of the car...
                Starting reference will be either where the car is or the prior path's endpoint,
                depending on how many points are left untraversed in the prior path.
              2) Fit the waypoints on a spline.
              3) Add up to 50 points to the spline, equally spaced so that car speed is less than reference velocity (50mph)
                Sim advances at 50cycles/sec, or every 0.02 seconds.
                Always have 50 points, so start with prior path (to keep it smooth), and add points until get to 50

            **************************************************************************/ 

            // (1) CREATE 5 NEW WAYPOINTS

            // Create list of evenly spaced (x,y) waypoints 30m apart
            // Later interpolate with spline

            vector<double> ptsx;
            vector<double> ptsy;

            // Reference x, y, yaw states
            // Reference starting point will be either (a) where ego is, or (b) at previous path's endpoint
            double ref_x = car_x;
            double ref_y = car_y;
            double ref_yaw = deg2rad(car_yaw);

            // If previous size is almost empty, use car as starting reference and build new list
            if (prev_size < 2)      // Hardcode a number
            {

              // Use two points that make path tangent to car
              double prev_car_x = car_x - cos(car_yaw);
              double prev_car_y = car_y - sin(car_yaw);

              ptsx.push_back(prev_car_x);
              ptsx.push_back(car_x);

              ptsy.push_back(prev_car_y);
              ptsy.push_back(car_y);

            }
            else  // Use prior path's endpoint as starting reference
            {
              
              // Redefine reference state as previous path end point
              ref_x = previous_path_x[prev_size-1];
              ref_y = previous_path_y[prev_size-1];

              double ref_x_prev = previous_path_x[prev_size-2];
              double ref_y_prev = previous_path_y[prev_size-2];
              ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);    // Calc angle car was heading in based on last 2 points

              // Use two points that make the path tangent to previous path's endpoint
              ptsx.push_back(ref_x_prev);
              ptsx.push_back(ref_x);
              ptsy.push_back(ref_y_prev);
              ptsy.push_back(ref_y);


            } // endif

            // Create 3 more points - In Frenet add evenly 30m spaced points ahead of starting reference
            // (Note the getXY function require the map waypoints to do the rotation/translation into x,y)
            // When there is a lane change, this works b/c the first two points are in the current lane, but the next 3 below
            // are computed for the new lane, and the spline generates a smooth lane changing trajectory.
            vector<double> next_wp0 = getXY(car_s+30, double(2 + 4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_wp1 = getXY(car_s+60, double(2 + 4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_wp2 = getXY(car_s+90, double(2 + 4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

            ptsx.push_back(next_wp0[0]);
            ptsx.push_back(next_wp1[0]);
            ptsx.push_back(next_wp2[0]);

            ptsy.push_back(next_wp0[1]);
            ptsy.push_back(next_wp1[1]);
            ptsy.push_back(next_wp2[1]);

            // ^ after these push_backs, the ptsxy vectors will have 5 points

            // Transform all the points to ego's coordinate system
            // (specifically, last point of last path is transformed into car coord system) 
            // This will make it eaasier to fit the spline, since the spline will be based on
            // an x-coord system that runs horizonatally (i.e., no change of it being a vertical non-function)
            for (int i=0; i<ptsx.size(); i++)
            {

              // Shift car reference angle to 0 degrees
              double shift_x = ptsx[i]-ref_x;
              double shift_y = ptsy[i]-ref_y;

              // Rotation
              ptsx[i] = (shift_x * cos(0-ref_yaw)-shift_y*sin(0-ref_yaw));
              ptsy[i] = (shift_x * sin(0-ref_yaw)+shift_y*cos(0-ref_yaw));

            }

            
            // (2) FIT POINTS ON SPLINE

            // Create a spline & set (x,y) points to the spline
            // Add the 5 "anchorpoints" (the new waypoints) to the spline
            tk::spline s;

            s.set_points(ptsx, ptsy);

            vector<double> next_x_vals;
            vector<double> next_y_vals;

            // Start with all previous path points from last path
            for (int i=0; i < previous_path_x.size(); i++)
            {
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);

            }

            
            // (3) BREAK UP SPLINE INTO EQUAL STEPS

            // Calculate how to break up spline points so that we travel at desired reference velocity
            // Remember that we move 50 times per second, per the simulator
            
            double target_x = 30.0;           // Target a point 30m in front of car;
            double target_y = s(target_x);    // Spline gives back y value
            double target_dist = sqrt((target_x)*(target_x)+(target_y)*(target_y)); // Calc straight dist to spline point from car (some trig)

            double x_add_on = 0;

            // Fill rest of path planner after filling it with previous points.
            // Here this will always output 50 points.
            // The way this works is that during the prior simulator cycle, 3 of 50 points might have been traversed,
            // leaving 47 points during this current cycle in the previous_path, and we only need to generate 3 more points.
            // This keeps the path nice and smooth without a lot of jerk.
            // Note: N is the number of steps to take to get to the 30m target by keeping under reference velocity;
            // 0.02 is the time step, since sim runs 50 cycles per second

            for (int i=1; i <= 50-previous_path_x.size(); i++) 
            {
              double N = (target_dist/(0.02*ref_vel/2.24));
              double x_point = x_add_on + (target_x)/N;
              double y_point = s(x_point);

              x_add_on = x_point;

              double x_ref = x_point;
              double y_ref = y_point;

              // Shift & rotate back to global coordinates  after rotating it to car's coord system earlier
              x_point = (x_ref * cos(ref_yaw)-y_ref*sin(ref_yaw));
              y_point = (x_ref * sin(ref_yaw)+y_ref*cos(ref_yaw));

              x_point += ref_x;
              y_point += ref_y;

              // Push the new point onto the next_vals

              next_x_vals.push_back(x_point);
              next_y_vals.push_back(y_point);

            }


  
            /************* END MY CODE ******************************************/          	

            /*
            // MY ORIGINAL CODE, commented out now
            // Set points 0.5m apart; since car moves 50 times/sec, velocity will be 25m/s (~50mph)
            
            double dist_inc = 0.5; 
            
            for (int i = 0; i < 50; i++) {
            
              // These lines make the car go straight line in euclidean, not frenet, so it doesn't stay in lane
              //next_x_vals.push_back(car_x+(dist_inc*i)*cos(deg2rad(car_yaw))); 
              //next_y_vals.push_back(car_y+(dist_inc*i)*sin(deg2rad(car_yaw)));

              double next_s = car_s + (i+1)*dist_inc;   // need i+1 to get next point, not where car currently is
              double next_d = 6;                        // Lanes are 4 m wide, there are 3 lanes, and center point 
                                                        // where d=0 is double-yellow line, so so middle of next lane is d=6 meters.

              vector<double> xy = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
              next_x_vals.push_back(xy[0]);
              next_y_vals.push_back(xy[1]);

            }
            */


            json msgJson;
          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

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
