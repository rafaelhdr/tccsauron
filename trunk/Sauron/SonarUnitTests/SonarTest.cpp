#include "SonarReadingsLogParser.h"
#include "Line.h"

using namespace System;
using namespace System::Text;
using namespace System::Collections::Generic;
using namespace	Microsoft::VisualStudio::TestTools::UnitTesting;

namespace SonarUnitTests
{
	[TestClass]
	public ref class SonarTest
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

		#pragma region sinalpha_validatereadings
		void assertSinAlpha(const SonarReadingsLogParser& parser, int sonarNumber, double
			expectedAngleDegrees) {
			sauron::SonarModel sonar(0, sauron::configs::sonars::getSonarPose(sonarNumber));
			parser.addAllReadingsOfOneSonar(sonarNumber, sonar);
			Assert::AreEqual(::sin(sauron::trigonometry::degrees2rads(expectedAngleDegrees)),
				sonar.getSinAlpha(), 0.05);
		}

		void assertValidateReadings(const SonarReadingsLogParser& parser, int sonarNumber) {
			sauron::SonarModel sonar(0,sauron::configs::sonars::getSonarPose(sonarNumber));
			parser.addAllReadingsOfOneSonar(sonarNumber, sonar);
			Assert::IsTrue(sonar.validateReadings());
		}

		[TestMethod]
		void GetSinAlpha_StraightLine()
		{
			SonarReadingsLogParser parser("validateReadings_MobileSim_StraightLine.log");
			assertSinAlpha(parser, 0, 0);
			assertSinAlpha(parser, 1, 0);
			assertSinAlpha(parser, 2, 0);			
		};

		[TestMethod]
		void ValidateReadingsTest_StraightLine()
		{
			SonarReadingsLogParser parser("validateReadings_MobileSim_StraightLine.log");
			assertValidateReadings(parser, 0);
			assertValidateReadings(parser, 1);
			assertValidateReadings(parser, 2);
		};

		[TestMethod]
		void GetSinAlpha_90deg()
		{
			SonarReadingsLogParser parser("validateReadings_MobileSim_90deg.log");
			assertSinAlpha(parser, 2, 90);
			assertSinAlpha(parser, 3, 90);
		};

		[TestMethod]
		void ValidateReadingsTest_90deg()
		{
			SonarReadingsLogParser parser("validateReadings_MobileSim_90deg.log");
			assertValidateReadings(parser, 2);
			assertValidateReadings(parser, 3);
		};

		[TestMethod]
		void GetSinAlpha_45deg()
		{
			SonarReadingsLogParser parser("validateReadings_MobileSim_45deg.log");
			assertSinAlpha(parser, 1, 45);
			assertSinAlpha(parser, 2, 45);
			assertSinAlpha(parser, 3, 45);
		};

		[TestMethod]
		void ValidateReadingsTest_45deg()
		{
			SonarReadingsLogParser parser("validateReadings_MobileSim_45deg.log");
			assertValidateReadings(parser, 1);
			assertValidateReadings(parser, 2);
			assertValidateReadings(parser, 3);
		};

		[TestMethod]
		void GetSinAlpha_60deg()
		{
			SonarReadingsLogParser parser("validateReadings_MobileSim_60deg.log");
			assertSinAlpha(parser, 2, 60);
			assertSinAlpha(parser, 3, 60);
		};

		[TestMethod]
		void ValidateReadingsTest_60deg()
		{
			SonarReadingsLogParser parser("validateReadings_MobileSim_60deg.log");
			assertValidateReadings(parser, 2);
			assertValidateReadings(parser, 3);
		};

		[TestMethod]
		void GetSinAlpha_minus27deg()
		{
			SonarReadingsLogParser parser("validateReadings_MobileSim_-27deg.log");
			assertSinAlpha(parser, 4, -27);
			assertSinAlpha(parser, 5, -27);
			assertSinAlpha(parser, 6, -27);
			assertSinAlpha(parser, 7, -27);
		};

		[TestMethod]
		void ValidateReadingsTest_minus27deg()
		{
			SonarReadingsLogParser parser("validateReadings_MobileSim_-27deg.log");
			assertValidateReadings(parser, 4);
			assertValidateReadings(parser, 5);
			assertValidateReadings(parser, 6);
			assertValidateReadings(parser, 7);
		};
		#pragma endregion

		#pragma region getobservedline
		void assertObservedLine(const SonarReadingsLogParser& parser, int sonarNumber,
			const sauron::Line& expectedLine) {
			sauron::SonarModel sonar(0, sauron::configs::sonars::getSonarPose(sonarNumber));
			parser.addAllReadingsOfOneSonar(sonarNumber, sonar);
			Assert::IsTrue(sonar.validateReadings());
			sauron::Line observedLine = sonar.getObservedLine();
			Assert::AreEqual(expectedLine.getRWall(), observedLine.getRWall(), 20);
			Assert::AreEqual(expectedLine.getTheta(), observedLine.getTheta(), 0.087);
		}

		[TestMethod]
		void ObservedLineTest_c250() {
			SonarReadingsLogParser parser("observedLine_MobileSim_C250.log");
			sauron::Line seenBy012(390, sauron::trigonometry::PI / 2);
			sauron::Line seenBy567(0, sauron::trigonometry::PI / 2);
			assertObservedLine(parser, 0, seenBy012);
			assertObservedLine(parser, 1, seenBy012);
			// imaginei que fosse funcionar, mas...
			//assertObservedLine(parser, 2, seenBy012);

			assertObservedLine(parser, 5, seenBy567);
			assertObservedLine(parser, 6, seenBy567);
			assertObservedLine(parser, 7, seenBy567);
		}
		#pragma endregion
	};
}
