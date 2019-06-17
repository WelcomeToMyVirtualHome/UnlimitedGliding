#include <GL/glut.h>
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
#include <sstream>
#include <omp.h>
#include "../lib/objects.h"
#include "../lib/utils.h"

const int width = 1000;
const int height = 1000;
const char* print_format = "FPS=%4.2f, frame=%u, rho=%.2f";
const std::string YES = "y";
const std::string NO = "n";
const std::string READFILE = "f";
const char* FILENAME = "params.dat";

int reftime = 0,frame = 0, c_time = 0, timebase = 0;

unsigned long long frame_long = 0;

int size = 40, n_thermals = 400, n_drones = 100;
float rho_thermals = n_thermals * std::pow(size,-2);

int max_range_x = 20, max_range_y = 20;
const Objects::Color thermal_color = Objects::Color(1.,0.,0.), drone_color = Objects::Color(1.,1.,1.);
int history_len = 0;

Objects::Rect** lattice;
std::vector<Objects::Cell> drones;
std::vector<std::deque<Objects::Cell> > drones_history;
std::vector<Objects::Cell> thermals_list;
std::pair<bool,int> **thermals;

FILE* output_velocity;
FILE* output_clustering;
int window_id;	

bool is_interaction = false, is_exit = false, pause = false;

volatile sig_atomic_t flag = 0;
void close(int sig)
{
    flag = 1;
}

void init()
{
	signal(SIGINT, close); 
	srand48(time(NULL));
	float rect_size = 1./size;
	lattice = new Objects::Rect*[size];
	thermals = new std::pair<bool,int>*[size];
	for(int x = 0; x < size; x++)
	{
		lattice[x] = new Objects::Rect[size];
		thermals[x] = new std::pair<bool,int>[size];
		for(int y = 0; y < size; y++)
		{
			lattice[x][y] = Objects::Rect(2*float(x)/size - 1.,2*float(y)/size - 1.,2*float(x)/size - 1. + 2*rect_size,2*float(y)/size - 1. + 2*rect_size);
			thermals[x][y] = std::pair<bool,int>(false,0);
		}
	}

	drones.resize(n_drones);
	drones_history.resize(n_drones);
}

void initSimulation()
{
	for(int x = 0; x < size; x++)
	{
		for(int y = 0; y < size; y++)
		{
			thermals[x][y].first = false;
			thermals[x][y].second = 0;
		}
	}
	for(auto &h : drones_history)
		h.clear();
	thermals_list.clear();
	for(int x = 0; x < size; x++)
	{
		for(int y = 0; y < size; y++)
		{
			if(drand48() < rho_thermals)
			{
				thermals_list.push_back(Objects::Cell(x,y));
				thermals[x][y].first = true;
			}
		}
	}
	if(thermals_list.empty())
		return;

	for(int i = 0; i < n_drones; i++)
	{
		Objects::Cell thermal = thermals_list[std::floor(drand48()*(thermals_list.size()-1))];
		drones[i] = thermal;
		thermals[thermal.x][thermal.y].second++;		
	}
}

void exit()
{
	glutDestroyWindow(window_id);
	exit(0);
}

void moveDrones()
{
	if(thermals_list.empty())
		return;
	
	float lambda = 1./3.75; // history = 0
	// float lambda = 1./1.325; //history = 1, m_r = 14
	for(int i = 0; i < n_drones; i++)
	{
		int dx = std::floor(drand48()*(float(max_range_x)/2+1));
		int dy = std::floor(drand48()*(float(max_range_y)/2+1))-float(max_range_y)/4;
		if(drand48() > std::exp(-lambda*std::max(dx,dy)))
			continue;
		
		Objects::Cell &d = drones[i];
		Objects::Cell prev = Objects::Cell(d.x,d.y);
		d.x = Utils::mod(d.x + dx,size);
		d.y = Utils::mod(d.y + dy,size);
		if(!thermals[d.x][d.y].first)
		{
			if(is_interaction)
			{
				std::vector<Objects::Cell> can_go;
				can_go.reserve(max_range_x*max_range_y);
				for(int x = -max_range_x + dx; x <= max_range_x - dx; x++)
				{
					for(int y =	-max_range_y/2 + dy; y <= max_range_y/2 - dy; y++)
					{
						int n_x = Utils::mod(d.x + x,size);
						int n_y = Utils::mod(d.y + y,size);
						if(thermals[n_x][n_y].second > 0)
							can_go.push_back(Objects::Cell(n_x,n_y));
					}
				}
				int go = std::floor(drand48()*(can_go.size()));
				d.x = can_go[go].x;
				d.y = can_go[go].y;
			}
			else
			{
				d.x = prev.x;
				d.y = prev.y;
			}
		}
		thermals[d.x][d.y].second++;
		if(is_interaction && history_len > 0 && drones_history[i].size() < (uint)history_len + 1)
		{
			drones_history[i].push_back(d);
		}
		if(is_interaction && history_len > 0 && drones_history[i].size() == (uint)history_len + 1)
		{
			thermals[drones_history[i].front().x][drones_history[i].front().y].second--;	
			drones_history[i].pop_front();	
		}
		else if(history_len == 0)
		{	
			thermals[prev.x][prev.y].second--;	
		}
	}
}

void print(double x, double y, char *string)
{
	glPushAttrib(GL_CURRENT_BIT);
	glColor4f(1.f,1.f,1.f,1.f);
	glRasterPos2f(x,y);
	for(size_t i = 0; i < strlen(string); i++) 
		glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10,string[i]);
	glPopAttrib();
}

void Timer(int iUnused)
{
	glutPostRedisplay();
	glutTimerFunc(reftime, Timer, 0);
}

void keyboard(int key, int x, int y)
{
	switch(key)
	{
  		case GLUT_KEY_UP:
			if(reftime == 0)
				break;
			reftime -= 1;
			break;
      	case GLUT_KEY_DOWN:
  			reftime += 1;
			break;
		default: 
			break;
	}
}

void keyboardCB(unsigned char key, int x, int y)
{
	switch(key)
	{
		case 27:
			exit();
			break;
		case 112:
			pause = !pause;
			break;
		case 114:
			initSimulation();
			break;
	}
	glutPostRedisplay();
}

void display(void) {
	if(pause)
		return;

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  	
  	glColor4f(thermal_color.r, thermal_color.g, thermal_color.b, 0.6f);
  	for(auto thermal : thermals_list)
  	{
  		int x = thermal.x;
  		int y = thermal.y;
  		glRectf(lattice[x][y].x1,lattice[x][y].y1,lattice[x][y].x2,lattice[x][y].y2);			
	}
  	glColor4f(drone_color.r, drone_color.g, drone_color.b, 0.2f);
	for(auto drone : drones)
  	{
  		int x = drone.x;
  		int y = drone.y;
  		glRectf(lattice[x][y].x1,lattice[x][y].y1,lattice[x][y].x2,lattice[x][y].y2);			
	}

	moveDrones();
	
	frame++;
	frame_long++;
	c_time = glutGet(GLUT_ELAPSED_TIME);
	char buffer[100];
	sprintf(buffer,print_format,frame*1000.0/(c_time-timebase),frame_long,rho_thermals);
	if (c_time - timebase > 1000) 
	{
		timebase = c_time;
		frame = 0;
	}
    print(-1,-1,buffer);

	glutSwapBuffers();
}

void loopGlut(int argc, char **argv)
{
	init();
	initSimulation();
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGBA | GLUT_DEPTH | GLUT_DOUBLE | GLUT_ALPHA);
	glutInitWindowSize(width,height);
	window_id = glutCreateWindow("Unlimited Gliding");
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glClearColor(0.f, 0.f, 0.f, 1.f);
	glutDisplayFunc(display);
	glutSpecialFunc(keyboard);
	glutKeyboardFunc(keyboardCB);
	Timer(0);   
	glutMainLoop();
}

void parseParamsFile(int argc, char** argv)
{
	if(argc != 2)
	{
		std::cout << "Usage: <params.dat>\n";
		return;
	}
	
	std::ifstream file(argv[1]);
	if(!file.is_open())
	{
		std::cout << "No params.dat file\n";
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
	size = std::stof(line);
	getValue(file,delimiter,line);
	n_drones = std::stof(line);
	getValue(file,delimiter,line);
	max_range_x = std::stof(line);
	getValue(file,delimiter,line);
	max_range_y = std::stof(line);
	getValue(file,delimiter,line);
	history_len = std::stof(line);
	getValue(file,delimiter,line);
	rho_thermals = std::stof(line);
	file.close();
}	

int main(int argc, char **argv)
{
	parseParamsFile(argc,argv);
	loopGlut(argc,argv);
	return 0;
}