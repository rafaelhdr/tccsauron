#pragma once
#include "Pose.h"
#include <Aria.h>

namespace sauron 
{

	namespace modeloDinamica
	{
		
		class MedidaOdometro
		{
		private:
			pose_t oldDistance;
			pose_t deltaDistance;

			pose_t oldTheta;
			pose_t deltaTheta;

			ArRobot& robot;
	
		public:
			
			MedidaOdometro(ArRobot&   robot);
			
			pose_t getDeltaDistance();
			pose_t getDeltaTheta();
			void atualizaMedida();
			
			/* nos dois métodos abaixo,
			a MedidaOdometro fica responsável por converter as unidades que ela usa para as unidades do pose_t 
			
			pose_t getDistance();
			pose_t getTheta();
			*/

		};



	}

}