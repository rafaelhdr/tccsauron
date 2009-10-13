#include "Pose.h"

namespace sauron 
{

	namespace modeloDinamica
	{
		
		class MedidaOdometro
		{
		private:
			pose_t distance;
			pose_t theta;

		public:
			MedidaOdometro(pose_t _distance, pose_t _theta)
				: distance(_distance), theta(_theta){

			}
			pose_t getDistance() const;
			pose_t getTheta() const;
			MedidaOdometro minus(const MedidaOdometro other) const;

			/* nos dois métodos abaixo,
			a MedidaOdometro fica responsável por converter as unidades que ela usa para as unidades do pose_t 
			
			pose_t getDistance();
			pose_t getTheta();
			*/

		};



	}

}