#pragma once
#include "../lib/objects.h"
#include "../lib/utils.h"
#include "./params.h"
#include <cmath>
#include <iostream>
#include <cstdlib>
#include <ctime>
#include <cstring>
#include <cstdio>
#include <vector>
#include <csignal>	
#include <fstream>
#include <deque>
#include <chrono>
#include <random>
#include <string>

class Simulation
{	
public:
	Params *p;
	std::vector<float> velocity,
		average_velocity,
		average_h;
	
	std::vector<Objects::Cell> drones,
		thermals_list;
	
	std::deque<Objects::Cell> can_go;
	std::vector<std::deque<Objects::Cell> > drones_history;
	std::pair<bool,int> **thermals;
	
	FILE* output_velocity;
	FILE* output_clustering;
	
	float rho_thermals = .0,
		end_rho_thermals = 0.,
		sum_averagev = 0.,
		sum_stddev = 0,
		increment_rho_thermals = 0;

	bool break_loop = false,
		is_exit = false,
		pause = false;

	unsigned long long frame_long = 0;
	int curr_measurment = 0,
		simulations_count = 0,
		id = 0;
	
	Simulation(int n_id, Params *params, float n_start_rho_thermals = 0.001, float n_end_rho_thermals = 1.)
	{
		id = n_id;
		p = params;
		rho_thermals = n_start_rho_thermals;
		end_rho_thermals = n_end_rho_thermals;
		increment_rho_thermals = (end_rho_thermals - rho_thermals) / (std::floor(float(p->n_measurments) / p->n_threads));
		init();
	}

	~Simulation()
	{
		for(int i = 0; i < p->size; ++i) 
		{
        	delete[] thermals[i];   
    	}
    	delete[] thermals;
	}

	void init()
	{
		srand48(time(NULL));
		thermals = new std::pair<bool,int>*[p->size];
		for(int x = 0; x < p->size; x++)
		{
			thermals[x] = new std::pair<bool,int>[p->size];
			for(int y = 0; y < p->size; y++)
			{
				thermals[x][y] = std::pair<bool,int>(false,0);
			}
		}
		drones.resize(p->n_drones);
		drones_history.resize(p->n_drones);
		velocity.resize(p->n_drones);
		average_velocity.resize(p->simulation_v_length);
		average_h.resize(p->size,0.f);	
		if(p->is_measure_speed)
		{
			std::string filename = std::string("measure_out/velocity/") + std::string("output_velocity") + std::to_string(id) + std::string(".dat");
			output_velocity = fopen(filename.c_str(), "w+");
		}
		if(p->is_measure_clustering)
		{
			std::string filename = std::string("measure_out/clustering/") + std::string("output_clustering") + std::to_string(id) + std::string(".dat");
			output_clustering = fopen(filename.c_str(), "w+");
		}
	}

	void initSimulation()
	{
		std::minstd_rand gen(std::random_device{}());
		std::uniform_real_distribution<double> dist(0, 1);
	
		for(int x = 0; x < p->size; x++)
		{
			for(int y = 0; y < p->size; y++)
			{
				thermals[x][y].first = false;
				thermals[x][y].second = 0;
			}
		}

		for(auto &h : drones_history){
			h.clear();
		}

		thermals_list.clear();
		for(int x = 0; x < p->size; x++)
		{
			for(int y = 0; y < p->size; y++)
			{
				if(dist(gen) < rho_thermals)
				{
					thermals_list.push_back(Objects::Cell(x,y));
					thermals[x][y].first = true;
				}
			}
		}

		if(thermals_list.empty())
			return;

		for(int i = 0; i < p->n_drones; i++)
		{
			Objects::Cell thermal = thermals_list[std::floor(dist(gen)*(thermals_list.size()-1))];
			drones[i] = thermal;
			thermals[thermal.x][thermal.y].second++;		
		}
	}


	void moveDrones()
	{
		std::minstd_rand gen(std::random_device{}());
		std::uniform_real_distribution<double> dist(0, 1);
		for(int i = 0; i < p->n_drones; i++)
		{
			// random biased movement within specified range  
			int dx = std::floor(dist(gen)*(float(p->max_range_x)/2+1));
			int dy = std::floor(dist(gen)*(float(p->max_range_y)+1)) - float(p->max_range_y)/2;
			if(dist(gen) > std::exp(-p->lambda*dx))
				continue;

			Objects::Cell &d = drones[i];
			Objects::Cell prev = Objects::Cell(d.x, d.y);
			d.x = Utils::mod(d.x + dx, p->size);
			d.y = Utils::mod(d.y + dy, p->size);
			
			// thermal present in [d.x, d.y]
			if(!thermals[d.x][d.y].first)
			{
				if(p->is_interaction)
				{
					Objects::Cell min_thermal(0,0);
					int min_capacity = p->n_drones;
					// search for thermals within 
					for(int x = -p->max_range_x + dx; x <= p->max_range_x - dx; x++)
					{
						for(int y =	-p->max_range_y + dy; y <= p->max_range_y - dy; y++)
						{
							int n_x = Utils::mod(d.x + x, p->size);
							int n_y = Utils::mod(d.y + y, p->size);
							// check if thermal is occupied by a drone
							if(thermals[n_x][n_y].second > 0)
							{
								if(p->information_radius > 0)
								{
									if(std::abs(n_x - d.x) <= p->information_radius && std::abs(n_y - d.y) <= p->information_radius)
									{
										can_go.push_back(Objects::Cell(n_x,n_y));
									}					
								}
								else
								{
									can_go.push_back(Objects::Cell(n_x,n_y));
								}
								// should a drone go to thermal with least number of drones 
								if(p->is_go_to_min_thermal)
								{
									if(thermals[n_x][n_y].second <= min_capacity)
									{
										min_capacity = thermals[n_x][n_y].second;
										min_thermal.x = n_x;
										min_thermal.y = n_y;
									}
								}
							}
						}
					}

					if(p->is_go_to_min_thermal)
					{
						d.x = min_thermal.x;
						d.y = min_thermal.y;
					}
					else
					{
						int go = std::floor(dist(gen)*(can_go.size()));
						d.x = can_go[go].x;
						d.y = can_go[go].y;		
						can_go.clear();
					}
				}
				// return to previous position
				else
				{
					d.x = prev.x;
					d.y = prev.y;
				}
			}

			thermals[d.x][d.y].second++;
			
			// store history_len previous positions of a drone
			if(p->is_interaction && p->history_len > 0 && drones_history[i].size() < (uint)p->history_len + 1)
			{
				drones_history[i].push_back(d);
			}
			if(p->is_interaction && p->history_len > 0 && drones_history[i].size() == (uint)p->history_len + 1)
			{
				thermals[drones_history[i].front().x][drones_history[i].front().y].second--;	
				drones_history[i].pop_front();	
			}
			else if(p->history_len == 0)	
				thermals[prev.x][prev.y].second--;	
			

			if(p->is_measure_speed)
			{
				int vx = Utils::mod_diff(d.x, prev.x, p->size);
				velocity[i] = vx;
			}	
		}
	}

	void ripleyEstimator()
	{
		for(int r = 1; r < std::floor(float(p->size)/2) + 1; r++)
		{	
			float K_r = 0;
			for(int i = 0; i < p->n_drones; i++)
			{
				for(int j = 0; j < p->n_drones; j++)
				{
					int r_x = Utils::mod_diff(drones[i].x, drones[j].x, p->size);
					int r_y = Utils::mod_diff(drones[i].y, drones[j].y, p->size);
					if(std::abs(r_x) <= r && std::abs(r_y) <= r)
					{
						K_r += 1;
					}
				}
			}
			average_h[r] += std::sqrt(K_r*p->size*p->size/(p->n_drones*p->n_drones)) - 2*r;
		}
	}

	void exit()
	{
		break_loop = true;
		if(p->is_measure_speed)
		{
			fclose(output_velocity);
		}
		if(p->is_measure_clustering)
		{
			fclose(output_clustering);
		}
	}

	void measure()
	{
		if(curr_measurment == std::floor(float(p->n_measurments / p->n_threads)))
		{
			exit();
		}

		if(p->is_measure_clustering)
		{
			if(frame_long > p->simulation_threshold && frame_long <= p->simulation_threshold + p->simulation_h_length)
			{
				ripleyEstimator();
			}
		}

		if(p->is_measure_speed)
		{
			if(frame_long > p->simulation_threshold)
			{
				average_velocity[frame_long - p->simulation_threshold] = Utils::mean(velocity);
			}

			if(frame_long == p->simulation_threshold + p->simulation_v_length)
			{
				std::pair<float, float> sd = Utils::stdev(average_velocity);
				sum_stddev += sd.second;
				sum_averagev += sd.first;
			}			
		}

		if(frame_long >= p->simulation_threshold + std::max(p->simulation_h_length, p->simulation_v_length))
		{
			simulations_count++;
			frame_long = 0;		
			initSimulation();
		}

		if(simulations_count == p->average_count)
		{
			printf("Simulation%d, rho = %.3f\n", id, rho_thermals);
			if(p->is_measure_speed)
			{
				sum_averagev /= p->average_count;
				sum_stddev /= p->average_count;
				fprintf(output_velocity, "%f %f %f\n", rho_thermals, sum_averagev, sum_stddev);
				printf("v = %f, stdev = %f\n", sum_averagev, sum_stddev);
			}

			if(p->is_measure_clustering)
			{	
				for(uint r = 1; r < std::floor(float(p->size)/2) + 1; r++)
				{
					fprintf(output_clustering, "%.3f %u %.3f\n", rho_thermals, r, average_h[r] / p->average_count / p->simulation_h_length);
					average_h[r] = 0;
				}
			}

			sum_averagev = 0;
			sum_stddev = 0;
		
			simulations_count = 0;
			curr_measurment++;	
			if(rho_thermals < 1.)
			{
				rho_thermals += increment_rho_thermals;
			}
			
		}
	}

	void loop()
	{
		while(!break_loop)
		{
			moveDrones();
			measure();
			frame_long++;
		}
	}
};