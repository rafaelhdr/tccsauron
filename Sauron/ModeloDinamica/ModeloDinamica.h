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
			pose_t calculaX(const MedidaOdometro nova_medida);
			pose_t calculaY(const MedidaOdometro nova_medida);
			pose_t calculaTheta(const MedidaOdometro nova_medida);

		public:
			ModeloDinamica(Pose posicao_inicial)
				: posicao_anterior(posicao_inicial), medida_anterior(0, 0){
			}
			ModeloDinamica(Pose posicao_inicial, MedidaOdometro medida_inicial)
				: posicao_anterior(posicao_inicial), medida_anterior(medida_inicial){
			}

			Pose getNovaPosicao(const MedidaOdometro nova_medida);
			void atualizaPosicao(Pose nova_posicao);

		};



	}

}