#include <iostream>
#include <vector>
#include <fstream>
#include <chrono>
#include <thread>
#include "./params.h"
#include "./simulation.h"
#include "../lib/objects.h"
#include "../lib/utils.h"

int main(int argc, char **argv)
{
	if(argc != 2)
	{
		std::cout << "Usage: <params.dat>\n";
		return 1;
	}
	Params *params = new Params(argv[1]);
		
	std::thread *threads = new std::thread[params->n_threads];
	std::vector<Simulation*> simulations(params->n_threads);

	auto t1 = std::chrono::high_resolution_clock::now();
	float start = params->rho_thermals;
	float increment = (params->max_rho_thermals - start) / params->n_threads;
	for(int i = 0; i < params->n_threads; i++)
	{
		printf("%f, %f\n", start+i*increment, start + (1+i)*increment);
		simulations[i] = new Simulation(1, params, start+i*increment, start + (1+i)*increment);
		simulations[i]->initSimulation();
		threads[i] = std::thread(&Simulation::loop, simulations[i]);
	}
	
	for (int i = 0; i < params->n_threads; ++i) {
		threads[i].join();
		delete simulations[i];
	}
	simulations.clear();
	
	auto t2 = std::chrono::high_resolution_clock::now();
	std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(t2-t1).count() << " milliseconds\n";
 
	return 0;

	auto t11 = std::chrono::high_resolution_clock::now();
	Simulation *sim = new Simulation(1, params, 0.001, 1.);
	sim->initSimulation();
	sim->loop();
	auto t22 = std::chrono::high_resolution_clock::now();
	std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(t22-t11).count() << " milliseconds\n";
	
 	return 0;
}