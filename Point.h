#ifndef _POINT_h
#define _POINT_h

#include "Arduino.h"

class Point
{
public:
	Point(float x = 0, float y = 0) : x(x), y(y) {}

	//boolean EQUALS
	bool operator==(Point& rhs)const
	{
		if (x == rhs.x && y == rhs.y)
			return true;
		return false;
	}

	//add
	Point& operator+=(const Point& rhs)
	{
		x += rhs.x;
		y += rhs.y;
		return *this;
	}
	Point operator+(const Point& rhs)
	{
		Point tmp = *this;
		tmp.x += rhs.x;
		tmp.y += rhs.y;
		return tmp;
	}

	//subtract
	Point& operator-=(const Point& rhs)
	{
		x -= rhs.x;
		y -= rhs.y;
		return *this;
	}
	Point operator-(const Point& rhs)
	{
		Point tmp = *this;
		tmp.x -= rhs.x;
		tmp.y -= rhs.y;
		return tmp;
	}

	//negative
	Point& operator-()
	{
		x = -x;
		y = -y;
		return *this;
	}

	//multiply
	Point& operator*=(const float& rhs)
	{
		x *= rhs;
		y *= rhs;
	}
	Point operator*(const float& rhs)
	{
		x *= rhs;
		y *= rhs;
	}

	//dot product
	float operator*(const Point& rhs)
	{
		return rhs.x * x + rhs.y * y;
	}

	//cross product
	float operator%(const Point& rhs)
	{
		return x * rhs.y - y * rhs.x;
	}

	//divide
	Point& operator/=(const float& rhs)
	{
		x /= rhs;
		y /= rhs;
		return *this;
	}

	Point operator/(const float& rhs)
	{
		Point tmp = *this;
		tmp.x /= rhs;
		tmp.y /= rhs;
		return tmp;
	}

	float magnitude()
	{
		return sqrt(x * x + y * y);
	}

	float distance(const Point& rhs)
	{
		return sqrt((x - rhs.x)*(x - rhs.x) + (y - rhs.y)*(y - rhs.y));
	}

	float theta(const Point& rhs)
	{
		return atan2(*this % rhs, *this * rhs);
	}

	float x, y;
};

#endif
