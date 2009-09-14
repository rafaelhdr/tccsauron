#pragma once
#include <vector>
#include <cmath>
namespace sauron
{
	namespace floating_point
	{
		template<typename T, typename Y>
		inline bool isEqual(T x, Y y) {
			const double epsilon = 1e-5; 
			return std::abs(x - y) <= epsilon * std::abs(x);
		}
	}
	namespace array_math
	{
		template<typename container_t>
		double mean(const container_t& data) {
			double total = 0;
			for(container_t::const_iterator it = data.begin(); it != data.end(); it++) {
				total += *it;
			}
			return total / data.size();
		}

	}
}