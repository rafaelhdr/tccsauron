#pragma once

namespace sauron
{
class SonarReading;
class Pose;
class Map;
class LineSegment;

class ISonarModel
{
public:
	// retorna true se a leitura for significativa
	virtual bool addReading(const SonarReading& reading, const Pose& estimatedPose) = 0;
	virtual bool validateReadings() = 0;
	// Tenta associar as leituras no buffer com alguma linha do mapa. Retorna true se conseguiu, e preenche as
	// variáveis de saída, se elas não forem nulas. Se não houver associação satisfatória, retorna false e o
	// valor dos ponteiros de saída é indefinido.
	virtual bool tryGetMatchingMapLine(
		Map* map, // o mapa do ambiente
		double sigmaError2, // o erro utilizado no portão de validação
		/*out*/LineSegment* matchedMapLine, // variável de saída: a linha do mapa que foi associada, se alguma
		/*out*/ SonarReading* expectedReading, // variável de saída: a leitura esperada para aquela linha
		/*out*/SonarReading* actualReading, // variável de saída: a leitura de fato obtida para aquela linha
		/*out*/ int* matchScore // variável de saída: a pontuação da associação
		) = 0;

};
}