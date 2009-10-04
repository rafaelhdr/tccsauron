#include "Cone.h"
#include "MathHelper.h"

using namespace System;
using namespace System::Text;
using namespace System::Collections::Generic;
using namespace	Microsoft::VisualStudio::TestTools::UnitTesting;

namespace SonarUnitTests
{
	[TestClass]
	public ref class ConeTest
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
		void BorderTest()
		{
			using namespace sauron;
			sauron::Cone cone(Point2DDouble(3,4), trigonometry::PI/2,
				trigonometry::degrees2rads(15));
			ArPose intersection;
			Assert::IsTrue(cone.getBorder1().intersects(&cone.getBorder2(), &intersection));
			Assert::AreEqual(3.0, intersection.getX(), 0.001);
			Assert::AreEqual(4.0, intersection.getY(), 0.001);
		};

		[TestMethod]
		void BorderTest_2()
		{
			using namespace sauron;
			sauron::Cone cone(Point2DDouble(23,12),
				trigonometry::degrees2rads(130),
				trigonometry::degrees2rads(45));
			ArPose intersection;
			Assert::IsTrue(cone.getBorder1().intersects(&cone.getBorder2(), &intersection));
			Assert::AreEqual(23, intersection.getX(), 0.001);
			Assert::AreEqual(12, intersection.getY(), 0.001);
		};

		[TestMethod]
		void IntersectsLineTest()
		{
			using namespace sauron;
			sauron::Cone cone(Point2DDouble(0,0),
				trigonometry::degrees2rads(90),
				trigonometry::degrees2rads(30));

			ArLineSegment segment(-20, 1, 20, 1);
			Assert::IsTrue(cone.intersectsSegment(segment));
		};

		[TestMethod]
		void IntersectsLineTest_2()
		{
			using namespace sauron;
			sauron::Cone cone(Point2DDouble(0,0),
				trigonometry::degrees2rads(90),
				trigonometry::degrees2rads(90));

			ArLineSegment segment(0, 10, 5, 10);
			Assert::IsTrue(cone.intersectsSegment(segment));
		};

		[TestMethod]
		void IntersectsLineTest_3()
		{
			using namespace sauron;
			sauron::Cone cone(Point2DDouble(0,0),
				trigonometry::degrees2rads(90),
				trigonometry::degrees2rads(90));

			ArLineSegment segment(-3, 10, 7, 15);
			Assert::IsTrue(cone.intersectsSegment(segment));
		};

		[TestMethod]
		void IntersectsLineTest_4()
		{
			using namespace sauron;
			sauron::Cone cone(Point2DDouble(0,0),
				trigonometry::degrees2rads(90),
				trigonometry::degrees2rads(90));

			ArLineSegment segment(5, 4, 15, 15);
			Assert::IsTrue(cone.intersectsSegment(segment));
		};

		[TestMethod]
		void IntersectsLineTest_5()
		{
			using namespace sauron;
			sauron::Cone cone(Point2DDouble(0,0),
				trigonometry::degrees2rads(90),
				trigonometry::degrees2rads(90));

			ArLineSegment segment(9, 8, 15, 8);
			Assert::IsFalse(cone.intersectsSegment(segment));
		};

		[TestMethod]
		void IntersectsLineTest_6()
		{
			using namespace sauron;
			sauron::Cone cone(Point2DDouble(0,0),
				trigonometry::degrees2rads(90),
				trigonometry::degrees2rads(90));

			ArLineSegment segment(-3, 5, -8, 1);
			Assert::IsTrue(cone.intersectsSegment(segment));
		};

		[TestMethod]
		void IntersectsLineTest_7()
		{
			using namespace sauron;
			sauron::Cone cone(Point2DDouble(0,0),
				trigonometry::degrees2rads(90),
				trigonometry::degrees2rads(90));

			ArLineSegment segment(-20, -5, 20, -5);
			Assert::IsFalse(cone.intersectsSegment(segment));
		};

		[TestMethod]
		void IntersectsLineTest_8()
		{
			using namespace sauron;
			sauron::Cone cone(Point2DDouble(0,0),
				trigonometry::degrees2rads(90),
				trigonometry::degrees2rads(90));

			ArLineSegment segment(-20, 5, -10, 5);
			Assert::IsFalse(cone.intersectsSegment(segment));
		};

		[TestMethod]
		void IntersectsLineTest_9()
		{
			using namespace sauron;
			sauron::Cone cone(Point2DDouble(0,0),
				trigonometry::degrees2rads(90),
				trigonometry::degrees2rads(90));

			ArLineSegment segment(20, 5, 10, 5);
			Assert::IsFalse(cone.intersectsSegment(segment));
		};

		[TestMethod]
		void IntersectsLineTest_10()
		{
			using namespace sauron;
			sauron::Cone cone(Point2DDouble(0,0),
				trigonometry::degrees2rads(90),
				trigonometry::degrees2rads(90));

			ArLineSegment segment(-5, 5, 5, 5);
			Assert::IsTrue(cone.intersectsSegment(segment));
		};

		[TestMethod]
		void IntersectsLineTest_11()
		{
			using namespace sauron;
			sauron::Cone cone(Point2DDouble(0,0),
				trigonometry::degrees2rads(90),
				trigonometry::degrees2rads(90));

			ArLineSegment segment(-4, 5, 4, 5);
			Assert::IsTrue(cone.intersectsSegment(segment));
		};

		[TestMethod]
		void IntersectsLineTest_12()
		{
			using namespace sauron;
			sauron::Cone cone(Point2DDouble(0,0),
				trigonometry::degrees2rads(90),
				trigonometry::degrees2rads(90));

			ArLineSegment segment(2, 7, 2, 5);
			Assert::IsTrue(cone.intersectsSegment(segment));
		};

		[TestMethod]
		void IntersectsLineTest_13()
		{
			using namespace sauron;
			sauron::Cone cone(Point2DDouble(0,0),
				trigonometry::degrees2rads(90),
				trigonometry::degrees2rads(90));

			ArLineSegment segment(2, 7, 2, 1);
			Assert::IsTrue(cone.intersectsSegment(segment));
		};

		[TestMethod]
		void IntersectsLineTest_14()
		{
			using namespace sauron;
			sauron::Cone cone(Point2DDouble(0,0),
				trigonometry::degrees2rads(90),
				trigonometry::degrees2rads(90));

			ArLineSegment segment(8, 7, 8, 1);
			Assert::IsFalse(cone.intersectsSegment(segment));
		};

		[TestMethod]
		void IntersectsLineTest_15()
		{
			using namespace sauron;
			sauron::Cone cone(Point2DDouble(0,0),
				trigonometry::degrees2rads(45),
				trigonometry::degrees2rads(90));

			ArLineSegment segment(2, 2, 5, 5);
			Assert::IsTrue(cone.intersectsSegment(segment));
		};

		[TestMethod]
		void IntersectsLineTest_16()
		{
			using namespace sauron;
			sauron::Cone cone(Point2DDouble(0,0),
				trigonometry::degrees2rads(45),
				trigonometry::degrees2rads(90));

			ArLineSegment segment(-1, 2, 5, 5);
			Assert::IsTrue(cone.intersectsSegment(segment));
		};
		
		[TestMethod]
		void IntersectsLineTest_17()
		{
			using namespace sauron;
			sauron::Cone cone(Point2DDouble(0,0),
				trigonometry::degrees2rads(45),
				trigonometry::degrees2rads(90));

			ArLineSegment segment2(-1, -2, -5, -5);
			Assert::IsFalse(cone.intersectsSegment(segment2));
		};

		[TestMethod]
		void IntersectsLineTest_18()
		{
			using namespace sauron;
			sauron::Cone cone(Point2DDouble(0,0),
				trigonometry::degrees2rads(45),
				trigonometry::degrees2rads(90));

			ArLineSegment segment3(0, 70, 0, 90);
			Assert::IsTrue(cone.intersectsSegment(segment3));
		};

		[TestMethod]
		void IntersectsLineTest_19()
		{
			using namespace sauron;
			sauron::Cone cone(Point2DDouble(0,0),
				trigonometry::degrees2rads(45),
				trigonometry::degrees2rads(90));

			ArLineSegment segment4(1, 0, 5, 0);
			Assert::IsTrue(cone.intersectsSegment(segment4));
		};

		[TestMethod]
		void IntersectsLineTest_20()
		{
			using namespace sauron;
			sauron::Cone cone(Point2DDouble(0,0),
				trigonometry::degrees2rads(180),
				trigonometry::degrees2rads(90));

			ArLineSegment segment(-5, -3, 5, 3);
			Assert::IsTrue(cone.intersectsSegment(segment));
		};

		[TestMethod]
		void IntersectsLineTest_21()
		{
			using namespace sauron;
			sauron::Cone cone(Point2DDouble(0,0),
				trigonometry::degrees2rads(90),
				trigonometry::degrees2rads(90));

			ArLineSegment segment(-1, 5, 1, 3);
			Assert::IsTrue(cone.intersectsSegment(segment));
		};

		[TestMethod]
		void IntersectsLineTest_22()
		{
			using namespace sauron;
			sauron::Cone cone(Point2DDouble(0,0),
				trigonometry::degrees2rads(90),
				trigonometry::degrees2rads(90));

			ArLineSegment segment(3, 1, 7, -3);
			Assert::IsFalse(cone.intersectsSegment(segment));
		};

		[TestMethod]
		void IntersectsLineTest_23()
		{
			using namespace sauron;
			sauron::Cone cone(Point2DDouble(0,0),
				trigonometry::degrees2rads(90),
				trigonometry::degrees2rads(90));

			ArLineSegment segment(-3, 5, -8, 0);
			Assert::IsTrue(cone.intersectsSegment(segment));
		};

	};
}
