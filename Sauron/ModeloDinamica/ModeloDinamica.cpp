// ModeloDinamica.cpp : Defines the exported functions for the DLL application.
//
#include "ModeloDinamica.h"
#include <cassert>
#include "MathHelper.h"

namespace sauron
{

	namespace modeloDinamica
	{
		Pose ModeloDinamica::getNovaPosicao(const MedidaOdometro nova_medida)
		{
			 Pose nova_posicao(calculaX(nova_medida), calculaY(nova_medida), calculaTheta(nova_medida));
			 posicao_anterior = nova_posicao;
			 return nova_posicao;
		}
		


		pose_t ModeloDinamica::calculaX(const MedidaOdometro nova_medida)
		{
			/* x(n-1) + delta distancia * cos (teta médio)
			/* dúvida: teta médio anterior ou atual ? */
			return posicao_anterior.X() + nova_medida.minus(medida_anterior).getDistance()*::cos(medida_anterior.getTheta());
		}

		pose_t ModeloDinamica::calculaY(const MedidaOdometro nova_medida)
		{
			return posicao_anterior.Y() + nova_medida.minus(medida_anterior).getDistance()*::sin(medida_anterior.getTheta());
		}

		pose_t ModeloDinamica::calculaTheta(const MedidaOdometro nova_medida)
		{
			/* odometro traz o delta do theta ou ele acumula o resultado ? 
			 acho que ele acumula, então basta retornar
             se não, o theta médio passa a ser: medida_anterior.getTheta() + nova_medida.getTheta();
			*/
			return nova_medida.getTheta();

		}


	}



}



