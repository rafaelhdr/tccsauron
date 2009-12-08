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
	// vari�veis de sa�da, se elas n�o forem nulas. Se n�o houver associa��o satisfat�ria, retorna false e o
	// valor dos ponteiros de sa�da � indefinido.
	virtual bool tryGetMatchingMapLine(
		Map* map, // o mapa do ambiente
		double sigmaError2, // o erro utilizado no port�o de valida��o
		/*out*/LineSegment* matchedMapLine, // vari�vel de sa�da: a linha do mapa que foi associada, se alguma
		/*out*/ SonarReading* expectedReading, // vari�vel de sa�da: a leitura esperada para aquela linha
		/*out*/SonarReading* actualReading, // vari�vel de sa�da: a leitura de fato obtida para aquela linha
		/*out*/ int* matchScore // vari�vel de sa�da: a pontua��o da associa��o
		) = 0;

};
}