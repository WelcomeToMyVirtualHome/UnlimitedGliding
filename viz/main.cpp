#include <GL/glut.h>
#include <iostream>
#include <fstream>
#include <algorithm>
#include "../lib/objects.h"
#include "../lib/utils.h"
#include "../lib/params.h"
#include "../lib/simulation.h"

Params *params = NULL;
Simulation *simulation = NULL;

const int width = 1000,
	height = 1000;

const char* print_format = "FPS = %4.2f, frame = %u, rho = %.2f, lambda = %.3f";

const Objects::Color thermal_color = Objects::Color(1., 0., 0.),
	drone_color = Objects::Color(1., 1., 1.);

int reftime = 0,
	frame = 0,
	c_time = 0,
	timebase = 0,
	window_id = 0;

float dlambda = 0.01,
	drho = 0.01;

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
  			params->lambda += dlambda;
			break;
    	case GLUT_KEY_DOWN:
	    	if(params->lambda > 2*dlambda)
				params->lambda -= dlambda;
			break;
		case GLUT_KEY_LEFT:
			if(simulation->rho_thermals >= 2*drho)
  				simulation->rho_thermals -= drho;
			break;
		case GLUT_KEY_RIGHT:
  			if(simulation->rho_thermals <= 1. - drho)
  				simulation->rho_thermals += drho;
			break;
		default: 
			break;
	}
}

void keyboardCB(unsigned char key, int x, int y)
{
	switch(key)
	{
		// esc
		case 27:
			exit();
			break;
		// p 
		case 112:
			pause = !pause;
			break;
		// r
		case 114:
			simulation->initSimulation();
			break;
		// v 
		case 118:
			params->is_measure_speed = !params->is_measure_speed;
			break;
		// c
		case 99:
			params->is_measure_clustering = !params->is_measure_clustering;
			break;
		// w
		case 119:
			if(reftime == 0)
				break;
			reftime -= 1;
			break;  
		// s
		case 115:
  			reftime += 1;
			break;
	}
	glutPostRedisplay();
}

void draw(std::vector<float> data, unsigned int N, float x, float y)
{
	glPushAttrib(GL_CURRENT_BIT);
	
	glPushMatrix();
	glTranslatef(x, y, 0.0);
	glColor4f(1.f, 1.f, 1.f, .6f);
	
	glBegin(GL_LINE_STRIP);
	if(N > data.size())
		return;

	float max_data = data[std::distance(data.begin(), std::max_element(data.begin(), data.end()))];
	for(unsigned int i = 0; i < N; i++)
	{
		glVertex2f(float(i)/data.size(), data[i]/max_data/5);
	}
	
	glBegin(GL_LINE_STRIP);
	glVertex2f(0, 0);
	glVertex2f(0, 0.2);
	glBegin(GL_LINE_STRIP);
	glVertex2f(0, 0);
	glVertex2f(0.5, 0);
	
	glEnd();

	glPopMatrix();
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
	sprintf(buffer, print_format, frame*1000.0/(c_time-timebase), frame_long, simulation->rho_thermals, params->lambda);
	if (c_time - timebase > 1000) 
	{
		timebase = c_time;
		frame = 0;
	}
    print(-1, -1, buffer);
	
	if(params->is_measure_speed)
	{
		sprintf(buffer, "v = %.3f", Utils::mean(simulation->velocity));
		print(-1, 0.98, buffer);
	}

	
	if(params->is_measure_clustering)
	{
		for(unsigned int i = 0; i < simulation->average_h.size(); i++)
			simulation->average_h[i] = 0;
		simulation->ripleyEstimator();
		draw(simulation->average_h, std::floor(float(params->size)/2), -1, .8);
	}

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