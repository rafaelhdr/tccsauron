#include "stdafx.h"
#include "Sauron.h"
namespace sauron
{
	using boost::shared_ptr;
	Sauron::Sauron(ArRobot* robot, const std::vector<std::string>& maps,
		const std::string& initialMap, const std::string& marksFile)
		: mp_robot(robot),
		m_isMoving(false)
	{
		mp_mapManager = new MapManager(initialMap, maps);
		mp_planner = new MapPlanner(mp_robot, mp_mapManager);
		mp_localization = new LocalizationManager(mp_robot,	*mp_mapManager, marksFile);
		mp_planner->setLocalizationManager(mp_localization);
	}

	void Sauron::setPose(const Pose& pose)
	{
		mp_localization->setInitialPose(pose);
	}

	bool Sauron::setPose(const std::string& nodeName, pose_t theta)
	{
		sauron::Graph& graph = mp_planner->getGraph(mp_mapManager->getCurrentMap());	
		sauron::Graph::iterator it;

		for(it = graph.begin(); it != graph.end(); it++)
		{
			if((*it).getName() == nodeName)
			{
				Pose pose(it->getPosition().X(), it->getPosition().Y(), theta);
				setPose(pose);
				return true;
			}
		}
		return false;
	}

	void Sauron::halt()
	{
		mp_planner->getPathPlanner().halt();
	}

	void Sauron::freeze()
	{
		halt();
		mp_localization->freeze();
	}

	void Sauron::unfreeze()
	{
		mp_localization->unfreeze();
	}

	void Sauron::continueAfterStop()
	{
		goTo(m_goal);
	}

	Pose Sauron::getPose()
	{
		return mp_localization->getPose();
	}

    void Sauron::goToAsync( const std::string &goalName )
    {
        m_goToThread.detach();
        m_goToThread = boost::thread( &Sauron::goTo, this, goalName );
    }

	bool Sauron::goTo(const std::string& goalName)
	{
		unfreeze();
		m_isMoving = true;
		m_goal = goalName;
		return mp_planner->goTo(m_goal);
	}

	bool Sauron::setInitialMap(const std::string& mapName)
	{
		std::vector<sauron::Map*> maps = mp_planner->getMaps();
		std::vector<sauron::Map*>::iterator mapIt;
		bool alterou = false;
		for(mapIt = maps.begin(); mapIt != maps.end(); mapIt++)
		{
			if((*mapIt)->getOriginalMapFilename() == mapName)
			{
				alterou = true;
				mp_mapManager->setCurrentMap(*mapIt);
				break;
			}
		}
		return alterou;
	}

	Sauron::~Sauron(void)
	{
		delete mp_planner;
		delete mp_mapManager;
		delete mp_localization;
	}
}
