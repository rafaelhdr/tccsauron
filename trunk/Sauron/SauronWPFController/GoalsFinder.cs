using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.IO;

namespace SauronWPFController
{
    public class GoalsFinder
    {

        private string[] mapNames = new string[] { "mezanino", "pavinf", "pavsup_mod" };
        private string extension = ".map";

        private string basePath = "Maps/";


        private Dictionary<string, List<string>> goalsNames;
        private Dictionary<string, List<string>> waypointsNames;

        public GoalsFinder()
        {
            Generate();

        }

        private void Generate()
        {
            goalsNames = new Dictionary<string, List<string>>();
            waypointsNames = new Dictionary<string, List<string>>();
            foreach (string mapName in mapNames)
            {
                goalsNames.Add(mapName, new List<string>());
                waypointsNames.Add(mapName, new List<string>());
                string[] mapLines = File.ReadAllLines(basePath + mapName + extension);

                foreach (string mapLine in mapLines)
                {
                    if (mapLine.StartsWith("Cairn: Goal"))
                    {
                        string goal = mapLine.Split(' ').Last().Replace("\"", null);
                        goalsNames[mapName].Add(goal);
                        waypointsNames[mapName].Add(goal);
                    }
                    else if (mapLine.StartsWith("Cairn: Dock"))
                    {
                        string waypoint = mapLine.Split(' ').Last().Replace("\"", null);
                        waypointsNames[mapName].Add(waypoint);
                    }
                }
                goalsNames[mapName].Sort();
                waypointsNames[mapName].Sort();
            }
           

        }
             
        public Dictionary<string, List<string>> GetGoalsNames()
        {
            return goalsNames;
        }

        public Dictionary<string, List<string>> GetWaypoints()
        {
            return waypointsNames;
        }


    }
}
