#include <iostream>
#include <cstdlib>
#include <string>
#include <uWS/uWS.h>

//#include <chrono>
//#include <thread>

#include "road.h"
#include "vehicle.h"
#include "json.hpp"
#include <map>

#define TRACK_LENGTH 6945.554

using namespace std;
using json = nlohmann::json;



// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned
string hasData(string s)
{
	auto found_null = s.find("null");
	auto b1 = s.find_first_of("[");
	auto b2 = s.find_first_of("}");

	if (found_null != string::npos)
	{
		return "";
	}
	else if (b1 != string::npos && b2 != string::npos)
	{
		return s.substr(b1, b2 - b1 + 2);
	}
	return "";
}

int main(int argc, char *argv[])
{
	if (argc != 2)
	{
		cout << "Usage: <exec_name> map_file_name.csv" << endl;
		exit(-1);
	}

	string file_name_ = string(argv[1]);
	const double max_s = TRACK_LENGTH;

	Road rd(file_name_, max_s);
	Vehicle ego(-1);
	map<int, Vehicle*> cars;

	uWS::Hub h;

	h.onMessage([&rd, &ego, &cars](uWS::WebSocket<uWS::SERVER> *ws, char *data, size_t length, uWS::OpCode opCode)
	{
		// "42" at the start of the message means there's a websocket message event.
		// The 4 signifies a websocket message
		// The 2 signifies a websocket event
		//auto sdata = string(data).substr(0,length);
		//cout << sdata << endl;
		if (length && length > 2 && data[0] == '4' && data[1] == '2')
		{
			auto s = hasData(data);
			if (s != "")
			{
				auto j = json::parse(s);
				string event = j[0].get<string>();
				if (event == "telemetry")
				{
					// j[1] is the actual JSON object

					// Ego car's localization data
					double car_x = j[1]["x"];
					double car_y = j[1]["y"];
					double car_s = j[1]["s"];
					double car_d = j[1]["d"];
					double car_yaw = j[1]["yaw"];
					double car_speed = j[1]["speed"];
					// update own car's parameters with updated telemetry
					ego.update_polar_velocity(car_x, car_y, car_s, car_d, car_yaw, car_speed);
					//ego.display();

					// previous path data fed to the simulator
					auto previous_path_x = j[1]["previous_path_x"];
					auto previous_path_y = j[1]["previous_path_y"];

					// previous path ending s and d values
					double end_path_s = j[1]["end_path_s"];
					double end_path_d = j[1]["end_path_d"];

					// sensor fusion data, a list of all other cars on the same side of the road
					auto sensor_fusion = j[1]["sensor_fusion"];
					// update map variable holding other car's information with updated telemetry
					// this won't perform any kind of management of the validaty of the data (by now)
					for (int idx = 0; idx < sensor_fusion.size(); idx++)
					{
						int car_id = sensor_fusion[idx][0];
						double car_x = sensor_fusion[idx][1];
						double car_y = sensor_fusion[idx][2];
						double car_v_x = sensor_fusion[idx][3];
						double car_v_y = sensor_fusion[idx][4];
						double car_s = sensor_fusion[idx][5];
						double car_d = sensor_fusion[idx][6];
						// check if a car with that ID already exists in the map, if not add it new
						if (cars.count(car_id) == 0)
						{
							Vehicle *car = new Vehicle(car_id);
							car->update_cartesian_velocity(car_x, car_y, car_s, car_d, car_v_x, car_v_y);
							cars.insert(std::pair<int, Vehicle*>(car_id, car));
						}
						else
						{
							cars[car_id]->update_cartesian_velocity(car_x, car_y, car_s, car_d, car_v_x, car_v_y);
						}
						cars[car_id]->display();
					}

					json msgJson;

					vector<double> next_x_vals;
					vector<double> next_y_vals;

					// TODO: define a path made up of (x, y) points that the car will visit sequentially every 0.02 seconds
					msgJson["next_x"] = next_x_vals;
					msgJson["next_y"] = next_y_vals;

					auto msg = "42[\"control\"," + msgJson.dump() + "]";
					//this_thread::sleep_for(chrono::milliseconds(1000));
					ws->send(msg.data(), msg.length(), uWS::OpCode::TEXT);
				}
			}
			else
			{
				// manual driving (supousedly)
				string msg = "42[\"manual\",{}]";
				ws->send(msg.data(), msg.length(), uWS::OpCode::TEXT);
			}
		}
	});

	// we should not need this, since we are not using HTTP, but if it is removed the program will not compile
	h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t)
	{
		const string s = "<h1>Hello world!</h1>";
		if (req.getUrl().valueLength == 1)
		{
			res->end(s.data(), s.length());
		}
		else
		{
			// I guess this should be more graceful ???
			res->end(nullptr, 0);
		}
	});

	h.onConnection([&h](uWS::WebSocket<uWS::SERVER> *ws, uWS::HttpRequest req)
	{
		cout << "Connected!!!" << endl;
	});

	h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> *ws, int code, char *message, size_t length)
	{
		ws->close();
		cout << "Disconnected!!!" << endl;
	});

	int port = 4567;
	if (h.listen(port))
	{
		cout << "Listening to port " << port << endl;
	}
	else
	{
		cerr << "Failed to listen to port " << port << endl;
		return -1;
	}
	
	h.run();

	// let's clean up before exit
	map<int, Vehicle*>::iterator it;
	it = cars.begin();
	while (it != cars.end())
	{
		delete it->second;
		it++;
	}

	return 0;
}

