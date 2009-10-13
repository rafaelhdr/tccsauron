#include "MedidaOdometro.h"
#include "Pose.h"

namespace sauron 
{

	namespace modeloDinamica
	{
		
		class ModeloDinamica
		{
		private:
			MedidaOdometro medida_anterior;
			Pose posicao_anterior;
			pose_t CalculaX(const MedidaOdometro nova_medida);
			pose_t CalculaY(const MedidaOdometro nova_medida);
			pose_t CalculaTheta(const MedidaOdometro nova_medida);

		public:
			ModeloDinamica(Pose posicao_inicial)
				: posicao_anterior(posicao_inicial), medida_anterior(0, 0){
			}
			ModeloDinamica(Pose posicao_inicial, MedidaOdometro medida_inicial)
				: posicao_anterior(posicao_inicial), medida_anterior(medida_inicial){
			}

			Pose GetNovaPosicao(const MedidaOdometro nova_medida);
			void AtualizaPosicao(Pose nova_posicao);

		};



	}

}