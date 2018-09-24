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
  string map_file_ = "data/highway_map.csv";
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
	
	int lane = 1;
	double ref_vel = 0.0;
	
	
  h.onMessage([&ref_vel, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy, &lane]
		(uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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
          	auto sensor_fusion = j[1]["sensor_fusion"];
						

						bool car_in_front = false;
            bool car_on_left = false;
            bool car_on_right = false;
						int check_car_lane = -1;

						
            const double MAX_SPEED = 49.5;
            const double MAX_ACC = .224;

						int prev_size = previous_path_x.size();
						
						if(prev_size > 0){

							car_s = end_path_s;
						}
						
												
						bool too_close = false;

						//Find the closest vehicles which are in the same lane as the ego car or to the left or right
						for(int i = 0; i < sensor_fusion.size(); i++){
							float d = sensor_fusion[i][6]; //lane of nearby car
							double vx = sensor_fusion[i][3]; 
							double vy = sensor_fusion[i][4];
							double check_speed = sqrt(vx*vx+vy*vy);
							double check_car_s = sensor_fusion[i][5];
							
							check_car_s += ((double)prev_size*.02*check_speed); //project the s value for the nearby car
							
							if(d > 0 && d < 4){ //left lane
								check_car_lane = 0;
							}
							else if(d > 4 && d < 8){ //center lane
								check_car_lane = 1;
							}
							else if(d > 8 && d < 12){ //right lane
								check_car_lane = 2;
							}
							
							
							if(check_car_lane == lane ){ //a car ahead in same lane	and < 30ft						
								if((check_car_s > car_s) && ((check_car_s-car_s) < 30)){	
									
									too_close = true;
									car_in_front = true;
									
								}
								
							}							
							else if(check_car_lane - lane < 0) //another car in left lane 
							{
								//check the distance of the other car in the left lane. First condition checks for trailing car in left lane and 2nd condition 
								//checks for a car in front in the left lane. In either case car is considered close for a lane change if distance < 30m
								if((check_car_s < car_s && car_s - check_car_s < 30) || (check_car_s > car_s && check_car_s - car_s < 30) ){ //TODO: refactor this
									car_on_left = true;
								}
								
							}
							else if(check_car_lane - lane > 0){ //a car in right lane
								//check the distance of the other car in the right lane. Should be greater than 30ft
								if((check_car_s < car_s && car_s - check_car_s < 30) || (check_car_s > car_s && check_car_s - car_s < 30) ){
									car_on_right = true;
								}
							}
							
						}


						if(car_in_front){		//there is a car in front
							if(!car_on_left && lane > 0){	//check if no vehicles in proxmity in the left lane & change to left lane
								lane--;
							}
							else if(!car_on_right && lane != 2){ //check if no vehicles in proxmity in the right lane & change to right lane
								lane++;
							}
							else{
								ref_vel -= MAX_ACC; //too much traffic. chill and stay in lane
							}
						}
						else{ 
							if ( lane != 1 ) { //if not in center lane, try to switch to center lane
                if ( ( lane == 0 && !car_on_right ) || ( lane == 2 && !car_on_left ) ) {
                  lane = 1; 
                }
              }
							if(ref_vel < MAX_SPEED){ //rev up!
								ref_vel += MAX_ACC;
							}
						}
						
						
						// define a path made up of (x,y) points that the car will visit sequentially every .02 seconds

          	
						//vectors to store calculated waypoints, these will be sent to he simulator						
						vector<double> ptsx;
						vector<double> ptsy;

						//ref x,y, yaw states
						//starting point is where the car currently is or it is the previous paths end point
						double ref_x = car_x;
						double ref_y = car_y;
						double ref_yaw = deg2rad(car_yaw);

						//Just getting stated, ref is car's starting point
						if(prev_size < 2){	
							
							/*	
							Lines below create path that's tangential to the angle of the car. Coords in this case are generated using the previous pts which in this case
							are the car's starting pts
							*/
							double prev_car_x = car_x - cos(car_yaw);
							double prev_car_y = car_y - sin(car_yaw);
							
              ptsx.push_back(prev_car_x);
							ptsx.push_back(car_x);

							ptsy.push_back(prev_car_y);
							ptsy.push_back(car_y);
						}
						else{
							//Make sure its tangent by using the last & second to last. Added to ptsx & ptsy
							//change ref x & y to last element in previous paath and then calculate using arctan
							//what were the last couple points & the angle from the prvious points and push them to prev pts x & y
							ref_x = previous_path_x[prev_size-1];
							ref_y = previous_path_y[prev_size-1];

							double ref_x_prev = previous_path_x[prev_size-2];
							double ref_y_prev = previous_path_y[prev_size-2];
							ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

							ptsx.push_back(ref_x_prev);
							ptsx.push_back(ref_x);

							ptsy.push_back(ref_y_prev);
							ptsy.push_back(ref_y);

						}
						
						//Next set of waypoints using frenet
						//lines below make sure frenet is spaced far apart. Instead of creting short spaced these are just 3 pts spaced 30m apart
						
						//d coords calculated using the 2nd parm
						vector<double> next_wp0 = getXY(car_s+30,2+4*lane,map_waypoints_s,map_waypoints_x,map_waypoints_y);
						vector<double> next_wp1 = getXY(car_s+60,2+4*lane,map_waypoints_s,map_waypoints_x,map_waypoints_y);
						vector<double> next_wp2 = getXY(car_s+90,2+4*lane,map_waypoints_s,map_waypoints_x,map_waypoints_y);

						
						ptsx.push_back(next_wp0[0]);
						ptsx.push_back(next_wp1[0]);
						ptsx.push_back(next_wp2[0]);

						ptsy.push_back(next_wp0[1]);
						ptsy.push_back(next_wp1[1]);
						ptsy.push_back(next_wp2[1]);

						//transformation for shift & rotation. from MPC. so that the ref angle from car's perspective is always 0 deg
						for(int i = 0; i < ptsx.size(); i++){
							double shift_x = ptsx[i] - ref_x;
							double shift_y = ptsy[i] - ref_y;

							ptsx[i] = shift_x * cos(0-ref_yaw) - shift_y * sin(0-ref_yaw);
							ptsy[i] = shift_x * sin(0-ref_yaw) + shift_y * cos(0-ref_yaw);
							
						}

						//define spline and set x & y pts
						tk::spline sp;
						sp.set_points(ptsx,ptsy);

						//for holding the future path
						vector<double> next_x_vals;
          	vector<double> next_y_vals;
						
						//add prev pts to the path planner
						for(int i = 0; i < prev_size; i++){
							next_x_vals.push_back(previous_path_x[i]);
							next_y_vals.push_back(previous_path_y[i]);
						}

						//get pts spaced along the line and spaced apart to maintain car speed
						//calculate distance from car to the target, which in this case is 30 pts away
						//spline gives all the y values
						double target_x = 30.0;
						double target_y = sp(target_x);
						//distance calculation from the car to target
						double target_dist = sqrt(target_x*target_x + target_y*target_y);
						//for transformation starting from the origin
						double x_add_on = 0;

						
						for(int i = 1; i < 50 - prev_size; i++)
						{
							
							//Add the pts along the spline
							//N is # of points. times .002 since the car vists a point every .002 sec
							//will get the full distance in meters
							//convert mph to meters/sec
							double N = target_dist/(0.02*ref_vel/2.24);
							double x_point = x_add_on + target_x/N;
							//get the y value for each x from the spline
							double y_point = sp(x_point);
							x_add_on = x_point;

							double x_ref = x_point;
							double y_ref = y_point;

							//go back to global from local
							x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
							y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

							x_point += ref_x;
							y_point += ref_y;

							//push to forward way pts
							next_x_vals.push_back(x_point);
							next_y_vals.push_back(y_point);
						}					

						

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
