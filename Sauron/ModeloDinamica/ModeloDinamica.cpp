// ModeloDinamica.cpp : Defines the exported functions for the DLL application.
//
#include "ModeloDinamica.h"
#include <cassert>
#include "MathHelper.h"

namespace sauron
{

	namespace modeloDinamica
	{
		Pose ModeloDinamica::GetNovaPosicao(const MedidaOdometro nova_medida)
		{
			 Pose nova_posicao(CalculaX(nova_medida), CalculaY(nova_medida), CalculaTheta(nova_medida));
			 posicao_anterior = nova_posicao;
			 medida_anterior = nova_medida;
			 return nova_posicao;
		}
		


		pose_t ModeloDinamica::CalculaX(const MedidaOdometro nova_medida)
		{
			/* x(n-1) + delta distancia * cos (teta médio)
			/* dúvida: teta médio anterior ou atual ? */
			return posicao_anterior.X() + nova_medida.minus(medida_anterior).getDistance()*::cos(medida_anterior.getTheta());
		}

		pose_t ModeloDinamica::CalculaY(const MedidaOdometro nova_medida)
		{
			return posicao_anterior.Y() + nova_medida.minus(medida_anterior).getDistance()*::sin(medida_anterior.getTheta());
		}

		pose_t ModeloDinamica::CalculaTheta(const MedidaOdometro nova_medida)
		{
			/* odometro traz o delta do theta ou ele acumula o resultado ? 
			 acho que ele acumula, então basta retornar
             se não, o theta médio passa a ser: medida_anterior.getTheta() + nova_medida.getTheta();
			*/
			return nova_medida.getTheta();

		}

		void ModeloDinamica::AtualizaPosicao(Pose nova_posicao)
		{
			posicao_anterior = nova_posicao;
		}



	}



}



