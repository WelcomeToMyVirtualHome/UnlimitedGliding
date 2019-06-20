#include <GL/glut.h>
#include <iostream>
#include <fstream>
#include "../lib/objects.h"
#include "../lib/utils.h"
#include "../lib/params.h"
#include "../lib/simulation.h"

Params *params = NULL;
Simulation *simulation = NULL;

const int width = 1000,
	height = 1000;

const char* print_format = "FPS=%4.2f, frame=%u, rho=%.2f";

const Objects::Color thermal_color = Objects::Color(1., 0., 0.),
	drone_color = Objects::Color(1., 1., 1.);

int reftime = 0,
	frame = 0,
	c_time = 0,
	timebase = 0,
	window_id = 0;

unsigned long long frame_long = 0;

bool pause = false;

Objects::Rect** lattice = NULL;

void init()
{
	float rect_size = 1./params->size;
	lattice = new Objects::Rect*[params->size];
	for(int x = 0; x < params->size; x++)
	{
		lattice[x] = new Objects::Rect[params->size];
		for(int y = 0; y < params->size; y++)
		{
			lattice[x][y] = Objects::Rect(2*float(x)/params->size - 1.,
											2*float(y)/params->size - 1.,
											2*float(x)/params->size - 1. + 2*rect_size,
											2*float(y)/params->size - 1. + 2*rect_size);

		}
	}
}

void exit()
{
	glutDestroyWindow(window_id);
	exit(0);
}

void print(double x, double y, char *string)
{
	glPushAttrib(GL_CURRENT_BIT);
	glColor4f(1.f, 1.f, 1.f, 1.f);
	glRasterPos2f(x, y);
	for(size_t i = 0; i < strlen(string); i++) 
		glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, string[i]);
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
		case GLUT_KEY_LEFT:
			if(simulation->rho_thermals >= 0.02)
  				simulation->rho_thermals -= 0.01;
			break;
		case GLUT_KEY_RIGHT:
  			simulation->rho_thermals += 0.01;
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
			simulation->initSimulation();
			break;
	}
	glutPostRedisplay();
}

void display(void) {
	if(pause)
		return;

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  	
  	glColor4f(thermal_color.r, thermal_color.g, thermal_color.b, 0.6f);
  	for(auto thermal : simulation->thermals_list)
  	{
  		int x = thermal.x;
  		int y = thermal.y;
  		glRectf(lattice[x][y].x1, lattice[x][y].y1, lattice[x][y].x2, lattice[x][y].y2);			
	}
  	glColor4f(drone_color.r, drone_color.g, drone_color.b, 0.2f);
	for(auto drone : simulation->drones)
  	{
  		int x = drone.x;
  		int y = drone.y;
  		glRectf(lattice[x][y].x1, lattice[x][y].y1, lattice[x][y].x2, lattice[x][y].y2);			
	}

	simulation->moveDrones();
	
	frame++;
	frame_long++;
	c_time = glutGet(GLUT_ELAPSED_TIME);
	char buffer[100];
	sprintf(buffer, print_format, frame*1000.0/(c_time-timebase), frame_long, simulation->rho_thermals);
	if (c_time - timebase > 1000) 
	{
		timebase = c_time;
		frame = 0;
	}
    print(-1, -1, buffer);
	glutSwapBuffers();
}

void loopGlut(int argc, char **argv)
{
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGBA | GLUT_DEPTH | GLUT_DOUBLE | GLUT_ALPHA);
	glutInitWindowSize(width, height);
	window_id = glutCreateWindow(" ");
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glClearColor(0.f, 0.f, 0.f, 1.f);
	glutDisplayFunc(display);
	glutSpecialFunc(keyboard);
	glutKeyboardFunc(keyboardCB);
	Timer(0);   
	glutMainLoop();
}	

int main(int argc, char **argv)
{
	if(argc != 2)
	{
		std::cout << "Usage: <params.dat>\n";
		return 1;
	}

	params = new Params(argv[1]);
	simulation = new Simulation(0, params, params->rho_thermals, 1.);
	simulation->initSimulation();
	init();

	loopGlut(argc, argv);
	return 0;
}