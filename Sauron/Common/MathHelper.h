#pragma once
#include <vector>
#include <cmath>
#include <boost/numeric/ublas/vector.hpp>
#include <boost/math/distributions/chi_squared.hpp>

#include "BaseTypes.h"

namespace sauron
{
	namespace floating_point
	{
		template<typename T, typename Y>
		inline bool isEqual(T x, Y y) {
			const double epsilon = 1e-5; 
			return std::abs(x - y) <= epsilon * std::abs(x);
		}

		template<typename T, typename Y>
		inline bool equalOrGreater(T x, Y y) {
			return isEqual(x, y) || x > y;
		}

		template<typename T, typename Y>
		inline bool equalOrLess(T x, Y y) {
			return isEqual(x, y) || x < y;
		}

	}

	namespace trigonometry
	{
		double const PI = 3.14159265358979323846;
		template<typename T> inline double degrees2rads(T degrees) {
			return degrees * PI / 180;
		}

		template<typename T> inline T rads2degrees(T rads) {
			return rads * 180.0 / PI;
		}

	}

	namespace statistics
	{
		template<typename container_t>
		double mean(const container_t& data) {
			double total = 0;
			for(container_t::const_iterator it = data.begin(); it != data.end(); it++) {
				total += *it;
			}
			return total / data.size();
		}

		template<typename container_t>
		double sample_variance(const container_t& data) {
			return sample_variance(data, mean(data));
		}

		template<typename container_t>
		double sample_variance(const container_t& data, double mean) {
			double totalDistance2ToMean = 0;
			for(container_t::const_iterator it = data.begin(); it != data.end(); it++) {
				totalDistance2ToMean += (*it - mean) * (*it - mean);
			}
			return totalDistance2ToMean / (data.size()-1);
		}

		inline bool chiSquareNormalDistributionTest(
			uint sampleSize, // tamanho da amostra
			double sampleVariance, // variância da amostra
			double expectedVariance, // variância da distribuição normal esperada
			double confidence) { // grau de confiança ([0,1])
				if(floating_point::equalOrGreater(confidence, 0.0) &&
					floating_point::equalOrLess(confidence, 1.0)) {
						/**
						Sejam s² a variância da amostra, σ² a variância teórica da distribuição,
						N o tamanho da amostra e α o nível de significância.

						H0: s² = σ²
						H1: s² > σ²

						Seja T = (N-1)s²/σ²
						Aceitamos H1 se T > χ²_(α,N-1). Aceitar H1 é equivalente a rejeitar H0.

						χ²_(α,N-1) é o "upper critical value", e pode ser obtido no
						Boost.Math.uBLAS usando quantile(complement(dist, alpha)).
						**/
						using namespace boost::math;
						chi_squared dist(sampleSize - 1);
						double T = (sampleSize - 1) * sampleVariance / expectedVariance;
						double upperCritical = quantile(complement(dist, confidence));
						bool rejectH0 = T > upperCritical;
						return !rejectH0;
				} else {
					throw std::invalid_argument("confidence deve estar entre 0 e 1");
				}
		}
	}

	namespace algelin
	{
		template<typename T>
		T scalarProduct(const boost::numeric::ublas::vector<T>& v1,
			const boost::numeric::ublas::vector<T>& v2) {
			T total = 0;
			if(v1.size() == v2.size()) {
				for(unsigned int i = 0; i < v1.size(); i++) {
					total += v1[i] * v2[i];
				}
				return total;
			} else {
				throw std::invalid_argument("Os tamanhos dos vetores devem ser iguais");
			}
		}
	}
}