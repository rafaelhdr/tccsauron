#include <boost/numeric/ublas/matrix.hpp>

using namespace System;
using namespace System::Text;
using namespace System::Collections::Generic;
using namespace	Microsoft::VisualStudio::TestTools::UnitTesting;

namespace SonarUnitTests
{
	[TestClass]
	public ref class uBLASTest
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
		void SimpleMatrixTest()
		{
			boost::numeric::ublas::matrix<int> m(2, 2);
			m(0,0) = 1; m(0,1) = 3; m(1,0) = 7; m(1,1) = 8;
			Assert::AreEqual(m(0,0), 1);
			Assert::AreEqual(m(0,1), 3);
			Assert::AreEqual(m(1,0), 7);
			Assert::AreEqual(m(1,1), 8);
		};

		[TestMethod]
		void MatrixMatrixMultiplicationTest()
		{
			using namespace boost::numeric::ublas;
			matrix<int> A(2, 2);
			A(0,0) = 1; A(0,1) = 3; A(1,0) = 7; A(1,1) = 8;
			matrix<int> B(2, 2);
			B(0,0) = 2; B(0,1) = 2; B(1,0) = 2; B(1,1) = 1;
			matrix<int> C = prod(A, B);
			Assert::AreEqual(C(0,0), 8);
			Assert::AreEqual(C(0,1), 5);
			Assert::AreEqual(C(1,0), 30);
			Assert::AreEqual(C(1,1), 22);
		}

		[TestMethod]
		void MatrixScalarMultiplicationTest()
		{
			using namespace boost::numeric::ublas;
			matrix<int> A(2, 2);
			A(0,0) = 1; A(0,1) = 3; A(1,0) = 7; A(1,1) = 8;
			matrix<int> C = A * 2;
			Assert::AreEqual(C(0,0), 2);
			Assert::AreEqual(C(0,1), 6);
			Assert::AreEqual(C(1,0), 14);
			Assert::AreEqual(C(1,1), 16);
		}

		[TestMethod]
		void MatrixVectorMultiplicationTest()
		{
			using namespace boost::numeric::ublas;
			matrix<int> A(2, 2);
			A(0,0) = 1; A(0,1) = 3; A(1,0) = 7; A(1,1) = 8;
			vector<int> B(2);
			B(0) = 3; B(1) = 2;
			vector<int> C = prod(A, B);
			Assert::AreEqual(C(0), 9);
			Assert::AreEqual(C(1), 37);
		}

		[TestMethod]
		void TransposeTest()
		{
			using namespace boost::numeric::ublas;
			matrix<int> A(2, 2);
			A(0,0) = 1; A(0,1) = 3; A(1,0) = 7; A(1,1) = 8;
			matrix<int> C = trans(A);
			Assert::AreEqual(C(0,0), 1);
			Assert::AreEqual(C(0,1), 7);
			Assert::AreEqual(C(1,0), 3);
			Assert::AreEqual(C(1,1), 8);
		}
	};
}
