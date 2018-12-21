#include <GL/glut.h>
#include <cmath>
#include <iostream>
#include <fstream>
#include <cstdlib>
#include <ctime>
#include <cstring>
#include <vector>
#include "structs.h"

const int width = 1000;
const int height = 1000;
int reftime = 0,frame=0,c_time=0,timebase=0;
const char* format_FPS = "FPS=%4.2f";
size_t size = 40;
size_t n_thermals = 400;
size_t n_drones = 100;
float rho_thermals = n_thermals * std::pow(size,-2);
const int max_move = 5;

const Color thermal_color = Color(1.,0.,0.);
const Color drone_color = Color(1.,1.,1.);
Rect **lattice;
std::vector<Cell> drones;
std::vector<Cell> thermals_list;
std::pair<bool,int> **thermals;

void Init()
{
	rho_thermals = n_thermals * std::pow(size,-2);
	srand48(time(NULL));
	float rect_size = 1./size;
	lattice = new Rect*[size];
	thermals = new std::pair<bool,int>*[size];
	for(size_t x = 0; x < size; x++)
	{
		lattice[x] = new Rect[size];
		thermals[x] = new std::pair<bool,int>[size];
		for(size_t y = 0; y < size; y++)
		{
			lattice[x][y] = Rect(2*float(x)/size - 1.,2*float(y)/size - 1.,2*float(x)/size - 1. + 2*rect_size,2*float(y)/size - 1. + 2*rect_size);
			thermals[x][y] = std::pair<bool,int>(false,0);
			if(drand48() < rho_thermals)
			{
				thermals_list.push_back(Cell(x,y));
				thermals[x][y].first = true;
			}
		}
	}
	drones.reserve(n_drones);
	for(size_t i = 0; i < n_drones; i++)
	{
		Cell thermal = thermals_list[std::floor(drand48()*(thermals_list.size()-1))];
		drones.push_back(thermal);
		thermals[thermal.x][thermal.y].second++;		
	}		
}

inline int mod(int x, int divisor)
{
    int m = x % divisor;
    return m + (m < 0 ? divisor : 0);
}

void moveDrones()
{
	for(auto &d : drones)
	{
		int dx = std::floor(drand48()*(max_move+1));
		int dy = std::floor(drand48()*(max_move+1));
		if(drand48() > std::exp(-std::max(dx,dy)/2))
			continue;
		
		int old_x = d.x;
		int old_y = d.y;
		d.x = mod(d.x + dx,size);
		d.y = mod(d.y + dy,size);
		if(!thermals[d.x][d.y].first)
		{
			std::vector<std::pair<int,int> > can_go;
			can_go.reserve(max_move*max_move);
			for(int x = -dx; x <= dx; x++)
			{
				for(int y =	-dy; y <= dy; y++)
				{
					int n_x = mod(d.x + x,size);
					int n_y = mod(d.y + y,size);
					if(thermals[n_x][n_y].second > 0)
						can_go.push_back(std::pair<int,int>(n_x,n_y));
				}
			}
			int go = std::floor(drand48()*(can_go.size()));
			d.x = can_go[go].first;
			d.y = can_go[go].second;
		}
		thermals[old_x][old_y].second--;	
		thermals[d.x][d.y].second++; 	
	}
}

void setup() 
{
	glClearColor(0.f, 0.f, 0.f, 1.f);
}

void print(double x, double y, char *string)
{
	glPushAttrib(GL_CURRENT_BIT);
	glColor3f(1.,1.,1.);
	glRasterPos2f(x,y);
	for (size_t i = 0; i < strlen(string); i++) 
		glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10,string[i]);
	glPopAttrib();
}

void display(void) {
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  	
  	glColor3f(thermal_color.r, thermal_color.g, thermal_color.b);
  	for(auto thermal : thermals_list)
  	{
  		int x = thermal.x;
  		int y = thermal.y;
  		glRectf(lattice[x][y].x1,lattice[x][y].y1,lattice[x][y].x2,lattice[x][y].y2);			
	}
  	glColor3f(drone_color.r, drone_color.g, drone_color.b);
	for(auto drone : drones)
  	{
  		int x = drone.x;
  		int y = drone.y;
  		glRectf(lattice[x][y].x1,lattice[x][y].y1,lattice[x][y].x2,lattice[x][y].y2);			
	}

	moveDrones();
	
	frame++;
	c_time = glutGet(GLUT_ELAPSED_TIME);
	char buffer[50];
	sprintf(buffer,format_FPS,frame*1000.0/(c_time-timebase));
	if (c_time - timebase > 1000) 
	{
		timebase = c_time;
		frame = 0;
	}
    print(-1,-1,buffer);

	glutSwapBuffers();
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
  			reftime += 1;
			break;
      	case GLUT_KEY_DOWN:
			if(reftime == 0)
				break;
			reftime -= 1;
			break;
		default: 
			break;
	}
}

int main(int argc, char *argv[])
{
	if(argc == 4)
	{
		size = atoi(argv[1]);
		n_drones = atoi(argv[2]);
		n_thermals = atoi(argv[3]);
	}
	if (argc<4)
	{
		std::cout<<"usage: <lattice size> <number of drones> <number of thermals>"<<'\n';	
		std::cout<<"running with default: 40, 100, 400"<<"\n";
	}
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE);
	glutInitWindowSize(width,height);
	glutCreateWindow("Unlimited Gliding");
	Init();

	setup();
	glutDisplayFunc(display);
	glutSpecialFunc(keyboard);
	Timer(0);
    
	glutMainLoop();

	return 0;
}