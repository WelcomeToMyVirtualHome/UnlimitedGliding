#include <iostream>
#include <vector>
#include <fstream>
#include <chrono>
#include <thread>
#include <dirent.h>
#include <unistd.h>
#include <string>
#include <limits.h>
#include <stdio.h>
#include "./params.h"
#include "./simulation.h"
#include "../lib/objects.h"
#include "../lib/utils.h"

std::string read_path()
{
	char result[PATH_MAX];
	ssize_t count = readlink("/proc/self/exe", result, PATH_MAX);
	std::string path = std::string(result, (count > 0) ? count : 0);
	return path.substr(0, path.size()-5);
}

void concatenate(std::string file_out, std::string folder_in)
{
	std::string exe_path = read_path().c_str();
	std::string output_path = exe_path + std::string("/measure_out/") + file_out;
	
	std::ofstream output(output_path, std::ios_base::binary | std::ios::app | std::ios::out);
	std::string path = exe_path + std::string("/measure_out/") + folder_in;
	
	DIR *dir;
	struct dirent *ent;
	if((dir = opendir(path.c_str())) != NULL)
	{
  		while((ent = readdir(dir)) != NULL) 
  		{
    		std::cout << "Reading " << std::string(path) + "/" + std::string(ent->d_name) << "\n";
			std::ifstream input(std::string(path) + "/" + std::string(ent->d_name), std::ios_base::binary | std::ios::in);
			if(!input.is_open())
				std::cerr << "File " << ent->d_name <<  " not opened\n";	
			std::string line;
			while(getline(input,line))
			{
				output << line << '\n';
			}
			input.close();
  		}
  		closedir(dir);
	} 
	else 
	{
  		perror(" ");
 	}
 	output.close();
}

void delete_files(std::string name, bool recursive = false)
{
	std::string path = read_path() + "/" + name;
	if(!recursive)
	{
		remove(name.c_str());
		return;
	}
	else
	{
		DIR *dir;
		struct dirent *ent;
		if((dir = opendir(name.c_str())) != NULL)
		{
	  		while((ent = readdir(dir)) != NULL) 
	  		{
	  			std::string remove_name = std::string(path) + "/" + std::string(ent->d_name);
	    		remove(remove_name.c_str());
			}
	  		closedir(dir);
		} 
		else 
		{
	  		perror(" ");
	 	}	
	}
}

int main(int argc, char **argv)
{
	auto t1 = std::chrono::high_resolution_clock::now();
	if(argc != 2)
	{
		std::cout << "Usage: <params.dat>\n";
		return 1;
	}

	delete_files("measure_out/clustering.dat");
	delete_files("measure_out/velocity.dat");
	delete_files("measure_out/clustering", true);
	delete_files("measure_out/velocity", true);

	Params *params = new Params(argv[1]);
		
	std::thread *threads = new std::thread[params->n_threads];
	std::vector<Simulation*> simulations(params->n_threads);

	float start = params->rho_thermals;
	float increment = (params->max_rho_thermals - start) / params->n_threads;
	for(int i = 0; i < params->n_threads; i++)
	{
		simulations[i] = new Simulation(i, params, start+i*increment, start + (1+i)*increment);
		simulations[i]->initSimulation();
		threads[i] = std::thread(&Simulation::loop, simulations[i]);
	}
	
	for (int i = 0; i < params->n_threads; i++)
	{
		threads[i].join();
	}

	concatenate("clustering.dat", "clustering");
	concatenate("velocity.dat", "velocity");
	auto t2 = std::chrono::high_resolution_clock::now();
	std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(t2-t1).count()/1000 << " seconds\n";
 	return 0;
}