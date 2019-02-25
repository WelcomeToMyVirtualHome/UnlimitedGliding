#pragma once
#include <vector>

namespace Utils
{
	inline int mod(int x, int divisor)
	{
	    int m = x % divisor;
	    return m + (m < 0 ? divisor : 0);
	}

	inline int mod_diff(int newX, int oldX, int mod)
	{
		int diff = newX - oldX;
		if(std::abs(diff) > int(float(mod)/2))
		{
			diff = mod - std::abs(diff);
			newX > oldX ? diff *= -1 : diff *= 1; 
		}
		return diff;
	} 

	inline float mean(std::vector<float> v)
	{
		float sum = 0;
		for(auto it = begin(v); it != end(v); ++it)
			sum += *it;
		return sum/v.size();
	}
}