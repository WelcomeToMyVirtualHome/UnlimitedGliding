#pragma once

namespace Objects
{
	class Color
	{
	public:
		Color(float n_r, float n_g, float n_b) : r(n_r), g(n_g), b(n_g) {}
		float r, g, b;
	};

	class Cell
	{
	public:
		Cell() {}
		Cell(int n_x, int n_y) : x(n_x), y(n_y) {}
		int x,y;
	};

	class Rect
	{
	public:
		Rect() {}
		Rect(float n_x1, float n_y1, float n_x2, float n_y2) : x1(n_x1), y1(n_y1), x2(n_x2), y2(n_y2) { }
		float x1,y1,x2,y2;
	};
}