#include "SonarTestHelper.h"

using namespace System;
using namespace System::Text;
using namespace System::Collections::Generic;
using namespace	Microsoft::VisualStudio::TestTools::UnitTesting;
using SonarTestHelper::assertExpectedReading;
namespace SonarUnitTests
{
	[TestClass]
	public ref class SonarTest2
	{
	private:
		TestContext^ testContextInstance;

	public: 
		/// <summary>
		///Gets or sets the test context which provides
		///information about and functionality for the current test run.
		///</summary>
		property Microsoft::VisualStudio::TestTools::UnitTesting::TestContext^ TestContext
		{
			Microsoft::VisualStudio::TestTools::UnitTesting::TestContext^ get()
			{
				return testContextInstance;
			}
			System::Void set(Microsoft::VisualStudio::TestTools::UnitTesting::TestContext^ value)
			{
				testContextInstance = value;
			}
		};

		#pragma region Additional test attributes
		//
		//You can use the following additional attributes as you write your tests:
		//
		//Use ClassInitialize to run code before running the first test in the class
		//[ClassInitialize()]
		//static void MyClassInitialize(TestContext^ testContext) {};
		//
		//Use ClassCleanup to run code after all tests in a class have run
		//[ClassCleanup()]
		//static void MyClassCleanup() {};
		//
		//Use TestInitialize to run code before running each test
		//[TestInitialize()]
		//void MyTestInitialize() {};
		//
		//Use TestCleanup to run code after each test has run
		//[TestCleanup()]
		//void MyTestCleanup() {};
		//
		#pragma endregion 

		[TestMethod]
		void ExpectedReadingTest1_S0()
		{
			SonarReadingsLogParser parser("observedLine_MobileSim_C250.log");
			ArMap map;
			map.readFile("pavsup.map");
			
			sauron::Line seenBy012(390, sauron::trigonometry::PI / 2);

			assertExpectedReading(map, parser, 0, seenBy012, 0);

		};

		[TestMethod]
		void ExpectedReadingTest1_S1()
		{
			SonarReadingsLogParser parser("observedLine_MobileSim_C250.log");
			ArMap map;
			map.readFile("pavsup.map");
			
			sauron::Line seenBy012(390, sauron::trigonometry::PI / 2);

			assertExpectedReading(map, parser, 1, seenBy012, 0);
		};

		[TestMethod]
		void ExpectedReadingTest1_S5()
		{
			SonarReadingsLogParser parser("observedLine_MobileSim_C250.log");
			ArMap map;
			map.readFile("pavsup.map");
			
			sauron::Line seenBy567(0, sauron::trigonometry::PI / 2);

			//assertExpectedReading(map, parser, 5, seenBy567, 0);
		};

		[TestMethod]
		void ExpectedReadingTest1_S6()
		{
			SonarReadingsLogParser parser("observedLine_MobileSim_C250.log");
			ArMap map;
			map.readFile("pavsup.map");
			
			sauron::Line seenBy567(0, sauron::trigonometry::PI / 2);

			assertExpectedReading(map, parser, 6, seenBy567, 0);
			assertExpectedReading(map, parser, 7, seenBy567, 0);
		};

		[TestMethod]
		void ExpectedReadingTest1_S7()
		{
			SonarReadingsLogParser parser("observedLine_MobileSim_C250.log");
			ArMap map;
			map.readFile("pavsup.map");
			
			sauron::Line seenBy567(0, sauron::trigonometry::PI / 2);

			assertExpectedReading(map, parser, 7, seenBy567, 0);
		};
	};
}
