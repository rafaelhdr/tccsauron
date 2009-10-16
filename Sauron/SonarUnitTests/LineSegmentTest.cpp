#include "MathHelper.h"
#include "LineSegment.h"
#include "ariaUtil.h"
using namespace System;
using namespace System::Text;
using namespace System::Collections::Generic;
using namespace	Microsoft::VisualStudio::TestTools::UnitTesting;

namespace SonarUnitTests
{
	[TestClass]
	public ref class LineSegmentTest
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
		void TestSauronLine()
		{
			// construtor do ArLineSegment em mm
			ArLineSegment lineSegment(36100, 3900, 43750, 3900);
			sauron::LineSegment sauronSegment(lineSegment);
			sauron::Line sauronLine = sauronSegment.getSauronLine();
			Assert::AreEqual(390.0, sauronLine.getRWall(), 0.0001);
			Assert::AreEqual(sauron::trigonometry::PI / 2, sauronLine.getTheta());
		};

		[TestMethod]
		void TestSauronLine_2()
		{
			// construtor do ArLineSegment em mm
			ArLineSegment lineSegment(35970, 400, 35970, 1250);
			sauron::LineSegment sauronSegment(lineSegment);
			sauron::Line sauronLine = sauronSegment.getSauronLine();
			Assert::AreEqual(3597, sauronLine.getRWall(), 0.0001);
			Assert::AreEqual(0, sauronLine.getTheta(), 0.0001);
		};

		[TestMethod]
		void TestContains()
		{
			// construtor do ArLineSegment em mm
			sauron::LineSegment lineSegment(0, 0, 20, 0);
			sauron::LineSegment segmentInside(5, 0, 15, 0);
			Assert::IsTrue(lineSegment.contains(segmentInside));
		};

		[TestMethod]
		void TestContains_2()
		{
			// construtor do ArLineSegment em mm
			sauron::LineSegment lineSegment(0, 0, 20, 0);
			sauron::LineSegment segmentInside(0, 0, 20, 0);
			Assert::IsTrue(lineSegment.contains(segmentInside));
		};

		[TestMethod]
		void TestContains_3()
		{
			// construtor do ArLineSegment em mm
			sauron::LineSegment lineSegment(0, 0, 20, 0);
			sauron::LineSegment segmentInside(0, 5, 21, 0);
			Assert::IsFalse(lineSegment.contains(segmentInside));
		};

		[TestMethod]
		void TestContains_4()
		{
			// construtor do ArLineSegment em mm
			sauron::LineSegment lineSegment(0, 0, 0, 20);
			sauron::LineSegment segmentInside(0, 5, 0, 10);
			Assert::IsTrue(lineSegment.contains(segmentInside));
		};
	};
}
