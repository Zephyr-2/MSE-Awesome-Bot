/**
 * A class which stores a [x,y] coordinate, and manages
 * common calculations of vectors
 *
 * Author: Robert Meagher
 * Written for Western's SAE Aero Design team
 * Used in the final project of MSE 2202B 2015
 */ 

#ifndef _POINT_h
#define _POINT_h

#include "Arduino.h"

class Point
{
public:
	/**
	 * Initializes a new Point to [0,0] unless
	 * specified otherwise
	 *
	 * @param	x	The x-coordinate. Default 0.
	 * @param	y	The y-coordinate. Default 0.
	 */
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

	//multiply by a scalar
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

	//divide by a scalar
	Point& operator/=(const float& rhs)
	{
		x /= rhs;
		y /= rhs;
	}
	Point operator/(const float& rhs)
	{
		x /= rhs;
		y /= rhs;
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

	/**
	 * Returns the magnitude of the point from
	 * the origin
	 */
	float magnitude()
	{
		return sqrt(x * x + y * y);
	}

	/**
	 * Returns the distance between two points
	 *
	 * @param	rhs	The point to calculate distance to
	 * @return 	The distance between this vector and rhs
	 */
	float distance(const Point& rhs)
	{
		return sqrt((x - rhs.x)*(x - rhs.x) + (y - rhs.y)*(y - rhs.y));
	}

	/**
	 * Returns the tal-to-tail angle between two points, in radians
	 *
	 * @param	rhs	The point to calculate the angle from
	 * @return 	The angle between this vector and rhs
	 */
	float theta(const Point& rhs)
	{
		return atan2(*this % rhs, *this * rhs);
	}

	float x, y;
};

#endif
