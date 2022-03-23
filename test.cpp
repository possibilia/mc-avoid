#include "alphabot.h"
#include "a1lidarrpi.h"
#include <thread>
#include <string>
#include <iostream>
#include <curl/curl.h>

class ControlCallback : public AlphaBot::StepCallback {
public:
	virtual void step(AlphaBot &alphabot) {
		if (action == 0) {
			forward(&alphabot, 0.3);
		} else if (action == 1) {
			turnLeft(&alphabot, 0.3);
		} else if (action == 2) {
			turnRight(&alphabot, 0.3);
		}
	}

	void setAction(unsigned action) {
		this->action = action;
	}

private:
	unsigned action = 0;

	void forward(AlphaBot* alphabot, float speed) {
		alphabot->setLeftWheelSpeed(speed);
		alphabot->setRightWheelSpeed(speed);
	}

	void turnLeft(AlphaBot* alphabot, float speed) {
		alphabot->setLeftWheelSpeed(speed);
		alphabot->setRightWheelSpeed(0.0);
	}

	void turnRight(AlphaBot* alphabot, float speed) {
		alphabot->setLeftWheelSpeed(0.0);
		alphabot->setRightWheelSpeed(speed);
	}

};

class DataInterface : public A1Lidar::DataInterface {
public:
	void newScanAvail(float, A1LidarData (&data)[A1Lidar::nDistance]) {
		if ((action == 1) || (action == 2)) {
			action = 0;
		}

		// determine action
		for(A1LidarData &data: data) {
			if ((data.valid) & (data.r < 0.3) & (data.r >= 0.0) & 
				(data.phi > 0.0) & (data.phi < 1.0)) {
				action = 1;
			} else if ((data.valid) & (data.r < 0.3) & (data.r >= 0.0) & 
				(data.phi < 0.0) & (data.phi > -1.0)) {
				action = 2;
			}

			if (data.valid) {
				dumpData(&data);
			}
		}
	}

	unsigned getAction() {	
		return action;
	}

private:
	unsigned action = 0;

	static size_t writeFunction(void *ptr, size_t size, size_t nmemb, std::string* data) {
	    data->append((char*) ptr, size * nmemb);
	    return size * nmemb;
	}

	void dumpData(A1LidarData* data) {
		// initialise curl
		auto curl = curl_easy_init();

		if (curl) {
			std::cout << "here";
			// get timestamp using unix epoch (secs)
			auto now = std::chrono::system_clock::now();
			unsigned t = std::chrono::duration_cast<std::chrono::seconds>(
						 now.time_since_epoch()).count();
			
			// build the query string
			std::string query = "http://192.168.0.111:5000/query?t=" + std::to_string(t) +
			                    "&x=" + std::to_string(data->x) +
			                    "&y=" + std::to_string(data->y) +
			                    "&r=" + std::to_string(data->r) +
			                    "&phi=" + std::to_string(data->phi) +
			                    "&sig=" + std::to_string(data->signal_strength);

			// send data to server
			curl_easy_setopt(curl, CURLOPT_URL, query);
			curl_easy_setopt(curl, CURLOPT_NOPROGRESS, 1L);
	        curl_easy_setopt(curl, CURLOPT_USERPWD, "user:pass");
	        curl_easy_setopt(curl, CURLOPT_USERAGENT, "curl/7.42.0");
	        curl_easy_setopt(curl, CURLOPT_MAXREDIRS, 50L);
	        curl_easy_setopt(curl, CURLOPT_TCP_KEEPALIVE, 1L);

	        std::string response_string;
	        std::string header_string;

	        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, this->writeFunction);
	        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response_string);
	        curl_easy_setopt(curl, CURLOPT_HEADERDATA, &header_string);

	        char* url;
	        long response_code;
	        double elapsed;
	        curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &response_code);
	        curl_easy_getinfo(curl, CURLINFO_TOTAL_TIME, &elapsed);
	        curl_easy_getinfo(curl, CURLINFO_EFFECTIVE_URL, &url);
	        
	        curl_easy_perform(curl);
	        curl_easy_cleanup(curl);
	        curl = NULL;
		}
	}

};

int main(int, char **) { 
	DataInterface data;
	ControlCallback control;

	AlphaBot alphabot;
	alphabot.registerStepCallback(&control);
	alphabot.start();

	A1Lidar lidar;
	lidar.registerInterface(&data);
	lidar.start();

	while(true) {
		unsigned action;
		action = data.getAction();
		control.setAction(action);
	}

	alphabot.stop();
	lidar.stop();
	return 0;
}
