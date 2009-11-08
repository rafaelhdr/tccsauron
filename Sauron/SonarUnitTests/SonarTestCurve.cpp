#include "SonarTestHelper.h"

using namespace System;
using namespace System::Text;
using namespace System::Collections::Generic;
using namespace	Microsoft::VisualStudio::TestTools::UnitTesting;

namespace SonarUnitTests
{
	[TestClass]
	public ref class SonarTestCurve
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
		void ClearBufferOnCurve_S0()
		{
			SonarReadingsLogParser parser("curva.log");
			ArMap map;
			map.readFile("linhareta.map");
			
			sauron::Line seenBy01234(0, sauron::trigonometry::PI / 2);
			
			sauron::Pose sonarRelativePose = sauron::configs::sonars::getSonarPose(0);
			sauron::SonarModel sonar(sonarRelativePose);
			parser.addAllReadingsOfOneSonar(0, sonar);
			Assert::IsTrue(sonar.getReadingsBufferCount() <= 1);


		};
	};
}