#pragma once

class Params
{
public:
	const std::string YES = "y";
	const std::string NO = "n";
	const std::string READFILE = "f";

	unsigned long long simulation_threshold = 0,
		simulation_v_length = 0,
		simulation_h_length = 0;

	int simulations_count = 0,
		average_count = 0,
		size = 0,
		n_thermals = 0,
		n_drones = 0,
		n_measurments = 0,
		max_range_x = 0,
		max_range_y = 0,
		history_len = 0,
		n_threads = 1;

	float rho_thermals = 0.,
		increment_rho_thermals = 0.,
		max_rho_thermals = 0.,
		lambda = 1.;
		
	bool is_interaction = false,
		is_measure_speed = false,
		is_measure_clustering = false,
		is_go_to_min_thermal = false;
		
	Params(char* arg)
	{
		if(!arg)
		{
			std::cout << "No file to open\n";
			return;
		}

		std::ifstream file(arg);
		if(!file.is_open())
		{
			std::cout << "File not opened\n";
			return;
		}
	
		auto parseBool = [&](std::string var, bool &value)
		{
			if(var.compare(YES) == 0)
				value = true;
			else if(var.compare(NO) == 0)
				value = false;
			else
				std::cout << "Wrong value, running with default <y> \n";
		};
		
		auto getValue = [&](std::ifstream &file, const std::string &delimiter, std::string &line)
		{
			std::getline(file, line);	
			line = line.substr(line.find(delimiter)+1,line.size());
		};

	    std::string line;	
		std::string delimiter = "=";
		getValue(file,delimiter,line);
		parseBool(line, is_interaction);
		getValue(file,delimiter,line);
		parseBool(line, is_measure_speed);
		getValue(file,delimiter,line);
		parseBool(line, is_measure_clustering);
		
		getValue(file,delimiter,line);
		size = std::stod(line);
		getValue(file,delimiter,line);
		n_drones = std::stod(line);
		getValue(file,delimiter,line);
		max_range_x = std::stod(line);
		getValue(file,delimiter,line);
		max_range_y = std::stod(line);
		getValue(file,delimiter,line);
		history_len = std::stod(line);
		getValue(file,delimiter,line);
		parseBool(line, is_go_to_min_thermal);
		getValue(file,delimiter,line);
		rho_thermals = std::stof(line);
		getValue(file,delimiter,line);
		max_rho_thermals = std::stof(line);
		getValue(file,delimiter,line);
		n_measurments = std::stod(line);
		increment_rho_thermals = (max_rho_thermals - rho_thermals)/n_measurments;
		getValue(file,delimiter,line);
		average_count = std::stod(line);
		getValue(file,delimiter,line);
		simulation_threshold = std::stod(line);
		getValue(file,delimiter,line);
		simulation_v_length = std::stod(line);
		getValue(file,delimiter,line);
		simulation_h_length = std::stod(line);
		getValue(file,delimiter,line);
		n_threads = std::stod(line);
		getValue(file,delimiter,line);
		lambda = std::stod(line);
		file.close();
	}
};