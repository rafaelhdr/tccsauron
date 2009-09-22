#include <vector>
#include "MathHelper.h"
#include <boost/array.hpp>
#include <boost/numeric/ublas/matrix.hpp>

using namespace System;
using namespace System::Text;
using namespace System::Collections::Generic;
using namespace	Microsoft::VisualStudio::TestTools::UnitTesting;

namespace SonarUnitTests
{
	[TestClass]
	public ref class MathHelperTest
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
		void Mean()
		{
			std::vector<int> vec;
			vec.push_back(1); vec.push_back(2); vec.push_back(5);
			Assert::IsTrue(sauron::floating_point::isEqual(
				8.0 / 3, sauron::statistics::mean(vec)));
		};
		
		[TestMethod]
		void MeanWithArray()
		{
			boost::array<double, 6> vec = { 0.5, 3, 7, 0, 0.5 };
			Assert::IsTrue(sauron::floating_point::isEqual(
				11.0 / 6, sauron::statistics::mean(vec)));
		};

		[TestMethod]
		void IsEqual()
		{
			double x = 0.1;
			float y = 0.05F + 0.05F;
			Assert::IsFalse(x == y);
			Assert::IsTrue(sauron::floating_point::isEqual(x, y));
		}

		[TestMethod]
		void SampleVariance()
		{
			using namespace sauron;
			boost::array<int, 2> vec = {1, 0};
			Assert::IsTrue(floating_point::isEqual(0.5, statistics::sample_variance(vec)));
		}

		[TestMethod]
		void SampleVariance2()
		{
			using namespace sauron;
			boost::array<int, 5> vec = {1, 3, 7, 0, 2};
			double mean = sauron::statistics::mean(vec);
			Assert::IsTrue(floating_point::isEqual(7.3, statistics::sample_variance(vec)));
		}

		[TestMethod]
		void ScalarProductTest()
		{
			using namespace boost::numeric::ublas;		
			vector<int> v1(3);
			vector<int> v2(3);
			v1(0) = 1; v1(1) = 3; v1(2) = -5;
			v2(0) = 4; v2(1) = -2; v2(2) = -1;
			Assert::AreEqual(3, sauron::algelin::scalarProduct(v1, v2));
			Assert::AreEqual(3, sauron::algelin::scalarProduct(v2, v1));
		}

		[TestMethod]
		void ChiSquareTest()
		{
			// http://www.itl.nist.gov/div898/handbook/prc/section2/prc23.htm
			unsigned int N = 10;
			double exp_s2 = 10.0 * 10.0;
			double sample_s2 = 13.97 * 13.97;
			Assert::IsFalse(sauron::statistics::chiSquareNormalDistributionTest(
				N, sample_s2, exp_s2, 0.05));
		}

		[TestMethod]
		void TrigTest_Deg2Rad()
		{	
			int degrees = 180;
			Assert::IsTrue(sauron::floating_point::isEqual(
				sauron::trigonometry::PI,
				sauron::trigonometry::degrees2rads(degrees)));

			degrees = 90;
			Assert::IsTrue(sauron::floating_point::isEqual(
				sauron::trigonometry::PI / 2,
				sauron::trigonometry::degrees2rads(degrees)));

			degrees = 30;
			Assert::IsTrue(sauron::floating_point::isEqual(
				sauron::trigonometry::PI / 6,
				sauron::trigonometry::degrees2rads(degrees)));

			degrees = 270;
			Assert::IsTrue(sauron::floating_point::isEqual(
				sauron::trigonometry::PI * 3.0 / 2.0,
				sauron::trigonometry::degrees2rads(degrees)));
		}

		[TestMethod]
		void TrigTest_Rad2Deg()
		{	
			double rads = sauron::trigonometry::PI;
			Assert::IsTrue(sauron::floating_point::isEqual(
				180,
				sauron::trigonometry::rads2degrees(rads)));

			rads = sauron::trigonometry::PI / 6;
			Assert::IsTrue(sauron::floating_point::isEqual(
				30,
				sauron::trigonometry::rads2degrees(rads)));

			rads = sauron::trigonometry::PI / 4;
			Assert::IsTrue(sauron::floating_point::isEqual(
				45,
				sauron::trigonometry::rads2degrees(rads)));
		}
	};
}
