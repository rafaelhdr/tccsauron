 #define BOOST_UBLAS_TYPE_CHECK 0 

#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/vector_proxy.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/triangular.hpp>
#include <boost/numeric/ublas/lu.hpp>
#include <boost/numeric/ublas/io.hpp>


namespace ublas = boost::numeric::ublas;


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

		//[TestMethod]
		//void InversionTest()
		//{
		//	using namespace boost::numeric::ublas;
		//	matrix<int> A(3,3);
		//	A(0,0) = -2;
		//	A(0,1) = 1;
		//	A(0,2) = 2;
		//	A(1,0) = -4;
		//	A(1,1) = 3;
		//	A(1,2) = 2;
		//	A(2,0) = -5;
		//	A(2,1) = 1;
		//	A(2,2) = 5;

		//	matrix<int> C(3,3);
		//	InvertMatrix( A, C);

		//	Assert::AreEqual(C(0,0), 2);
		//	Assert::AreEqual(C(0,1), 0);
		//	Assert::AreEqual(C(0,2), 0);
		//	Assert::AreEqual(C(1,0), 1);
		//	Assert::AreEqual(C(1,1), 0);
		//	Assert::AreEqual(C(1,2), 0);
		//	Assert::AreEqual(C(2,0), 1);
		//	Assert::AreEqual(C(2,1), 0);
		//	Assert::AreEqual(C(2,2), 0);
		//}


		/* Matrix inversion routine.
		Uses lu_factorize and lu_substitute in uBLAS to invert a matrix */
		template<class T>
		bool InvertMatrix (const ublas::matrix<T>& input, ublas::matrix<T>& inverse) {
			using namespace boost::numeric::ublas;
			
			typedef permutation_matrix<std::size_t> pmatrix;
			
			// create a working copy of the input
			matrix<T> A(input);
			// create a permutation matrix for the LU-factorization
			pmatrix pm(A.size1());


			// perform LU-factorization
			int res = lu_factorize(A,pm);
			if( res != 0 ) return false;


			// create identity matrix of "inverse"
			inverse.assign(ublas::identity_matrix<T>(A.size1()));


			// backsubstitute to get the inverse
			lu_substitute(A, pm, inverse);


			return true;
		}
	};
}

