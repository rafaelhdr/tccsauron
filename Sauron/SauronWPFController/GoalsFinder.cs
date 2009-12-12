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
             
        public Dictionary<string, List<string>> GetGoalsNames()
        {
            Dictionary<string, List<string>> goals = new Dictionary<string, List<string>>();

            foreach(string mapName in mapNames)
            {
                goals.Add(mapName, new List<string>());
                string[] mapLines =  File.ReadAllLines(basePath + mapName + extension);

                foreach (string mapLine in mapLines)
                {
                    if (mapLine.StartsWith("Cairn: Goal"))
                    {
                        goals[mapName].Add(mapLine.Split(' ').Last().Replace("\"", null));
                    }
                }
            }

            return goals;
        }


    }
}
