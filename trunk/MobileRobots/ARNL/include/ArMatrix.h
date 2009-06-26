/*
MobileRobots Advanced Robotics Navigation and Localization (ARNL)
Version 1.7.1

Copyright (C) 2004, 2005 ActivMedia Robotics LLC
Copyright (C) 2006, 2007, 2008, 2009 MobileRobots Inc.

All Rights Reserved.

MobileRobots Inc does not make any representations about the
suitability of this software for any purpose.  It is provided "as is"
without express or implied warranty.

The license for this software is distributed as LICENSE.txt in the top
level directory.

robots@mobilerobots.com
MobileRobots
10 Columbia Drive
Amherst, NH 03031
800-639-9481

*/
/* ****************************************************************************
 * 
 * File: ArMatrix.h
 * 
 * Function: Header for the matrix class.
 *
 * Created:  George V. Paul. gvp@activmedia.com. January 11 2006.
 *
 *****************************************************************************/
#ifndef ARMATRIX_H
#define ARMATRIX_H
#include <math.h>
#include "Aria.h"

class ArMatrix
{

public:
  /// Constructor.
  ArMatrix(void)
  {
    myRows = 0;
    myCols = 0;
    myElement = NULL;
  }
  /// Constructor.
  ArMatrix(int rows, int cols = 1) 
  {
    myRows = rows;
    myCols = cols;
    myElement = NULL;
    allocate();
    set(0.0);
  }
  /// Constructor.
  ArMatrix(int rows, int cols, double val) 
  {
    myRows = rows;
    myCols = cols;
    myElement = NULL;
    allocate();
    setDiagonal(val);
  }
  /// Copy constructor
  ArMatrix(const ArMatrix& mat) { copy(mat); }
  // Assignment operator function. overloads the equal sign operator
  ArMatrix& operator=(const ArMatrix& mat) 
  {
    if( this == &mat ) return *this;  // If two sides equal, do nothing.
    if(myRows > 0 && myCols > 0 && myElement)
      deallocate();
    copy(mat);                  // Copy right hand side to l.h.s.
    return *this;
  }
  /// Destructor
  ~ArMatrix() 
  {
    if(myRows > 0 && myCols > 0 && myElement)
      deallocate();
  }
  /// Matrix access
  double& operator() (int i, int j = 0)
  {
    if(i < 0 || i >= myRows || j < 0 || j >= myCols)
    {
      ArLog::log(ArLog::Normal, "\nArMatrix::Illegal index %d/%d  %d/%d\n", 
		 i, myRows, j, myCols);
      return myElement[0][0];
//      Aria::exit(1);
    }
    return myElement[i][j];
  }
  const double& operator() (int i, int j = 0) const
  {
    if(i < 0 || i >= myRows || j < 0 || j >= myCols)
    {
      ArLog::log(ArLog::Normal, "\nArMatrix::Illegal index %d/%d  %d/%d\n", 
		 i, myRows, j, myCols);
      return myElement[0][0];
//      Aria::exit(1);
    }
    return myElement[i][j];
  }
  /// Matrix add
  ArMatrix operator+(ArMatrix mat)
  {
    if(mat.getRows() != myRows || mat.getCols() != myCols)
    {
      ArLog::log(ArLog::Normal, "\nArMatrix::Illegal + %d, %d  %d,%d\n", 
	     myRows, myCols, mat.getRows(), mat.getCols());
      return mat;
//      Aria::exit(1);
    }
    ArMatrix sum = *this;
    for(int i = 0; i < myRows; i++)
      for(int j = 0; j < myCols; j++)
	sum(i, j) += mat(i, j);
    return sum;
  }
  /// Matrix subtract
  ArMatrix operator-(ArMatrix mat)
  {
    if(mat.getRows() != myRows || mat.getCols() != myCols)
    {
      ArLog::log(ArLog::Normal, "\nArMatrix::Illegal - %d, %d  %d, %d\n", 
	     myRows, myCols, mat.getRows(), mat.getCols());
      return mat;
//      Aria::exit(1);
    }
    ArMatrix sum = *this;
    for(int i = 0; i < myRows; i++)
      for(int j = 0; j < myCols; j++)
	sum(i, j) -= mat(i, j);
    return sum;
  }
  /// Matrix multiply.
  ArMatrix operator*(ArMatrix mat)
  {
    if(mat.getRows() != myCols)
    {
      ArLog::log(ArLog::Normal, "\nArMatrix::Illegal * %d, %d  %d, %d\n", 
	     myRows, myCols, mat.getRows(), mat.getCols());
      return mat;
//      Aria::exit(1);
    }
    return multiply(mat);
  }
  /// Matrix multiply.
  ArMatrix operator*(double m)
  {
    ArMatrix out(myRows, myCols);
    for(int i = 0; i < myRows; i++)
      for(int j = 0; j < myCols; j++)
	out(i, j) = myElement[i][j] * m;
    return out;
  }
  // Set all elements to val.
  void set(double val)
  {
    for(int i = 0; i < myRows; i++)
      for(int j = 0; j < myCols; j++)
	myElement[i][j] = val;
  }
  /// Constructor for identity matrix.
  void setDiagonal(double val) 
  {
    for(int i = 0; i < myRows; i++)
      for(int j = 0; j < myCols; j++)
	myElement[i][j] = (i == j) ? val : 0.0;
  }
  /// Multiply with another matrix.
  ArMatrix multiply(ArMatrix& in)
  {
    int m = myRows;
    int n = myCols;
    int p = in.getRows();
    int q = in.getCols();
    ArMatrix out(m, q);
    if(n != p)
      return out;
    for(int i = 0; i < m; i++)
    {
      for(int j = 0; j < q; j++)
      {
	double acc = 0.0;
	for(int k = 0; k < n; k++)
	{
	  acc += myElement[i][k]*in(k, j);
	}
	out(i, j) = acc;
      }
    }
    return out;
  }
  /// Transpose the matrix.
  ArMatrix transpose(void)
  {
    ArMatrix out(myCols, myRows);

    for(int i = 0; i < myRows; i++)
    {
      for(int j = 0; j < myCols; j++)
      {
	out(j, i) = myElement[i][j];
      }
    }
    return out;
  }
  /// Print matrix.
  void print(const char* header=NULL, FILE* fp=stdout)
  {
    char buffer[10000];
    buffer[0] = '\0';
    std::string line;
    if(header!=NULL)
    {
      if(myRows == 1)
      {
	sprintf(buffer, "%s [%d %d]\t", header, myRows, myCols);
      }
      else
      {
	sprintf(buffer, "%s [%d %d]\n", header, myRows, myCols);
	line += buffer;
	buffer[0] = '\0';
      }
    }
    for(int i = 0; i < myRows; i++)
    {
      for(int j = 0; j < myCols; j++)
      {
	sprintf(buffer, "%s%5.2f\t", buffer, myElement[i][j]);
      }
      if(i < myRows-1)
      {
	sprintf(buffer, "%s\n", buffer);
	line += buffer;
	buffer[0] = '\0';
      }
    }
    line += buffer;
    ArLog::log(ArLog::Normal, line.c_str());
  }
  /// Print matrix.
  void printDiagonal(char* header=NULL, FILE* fp=stdout)
  {
    char buffer[2064];
    if(header!=NULL)
      sprintf(buffer, "%s [%d %d]\t", header, myRows, myCols);
  
    if(myCols == 1)
    {
      for(int i = 0; i < myRows; i++)
	sprintf(buffer, "%s %5.2f\t", buffer, myElement[i][0]);
//      sprintf(buffer, "%s\n", buffer);
    }
    else if(myRows == 1)
    {
      for(int j = 0; j < myCols; j++)
	sprintf(buffer, "%s%5.2f\t", buffer, myElement[0][j]);
//      sprintf(buffer, "%s\n", buffer);
    }
    else
    {
      for(int i = 0; i < myRows; i++)
      {
	for(int j = 0; j < myCols; j++)
	  if(i == j)
	    sprintf(buffer, "%s%5.2f\t", buffer, myElement[i][j]);
      }
//      sprintf(buffer, "%s\n", buffer);
    }
    ArLog::log(ArLog::Normal, buffer);
  }
  /// Get the i,j element;
  double getElement(int i, int j) { return myElement[i][j];}
  /// Get the no of rows.
  int getRows(void) { return myRows;}
  /// Get the no of cols
  int getCols(void) { return myCols;}
  /// Set the i,j element;
  void setElement(int i, int j, double d) { myElement[i][j] = d;}
  /// Inverse of the matrix if square.
  AREXPORT double inverse(ArMatrix& Ainv);

protected:
  // Private copy function.
  // Copies values from one Matrix object to another.
  void copy(const ArMatrix& mat) 
  {
    myRows = mat.myRows;
    myCols = mat.myCols;
    myElement = new double*[myRows];
    for(int i = 0; i < myRows; i++)
      myElement[i] = new double[myCols];
    for(int i = 0; i < myRows; i++)
      for(int j = 0; j < myCols; j++)
	myElement[i][j] = mat.myElement[i][j];
  }
  void allocate(void)
  {
    if(!(myElement = new double*[myRows]))
    {
      ArLog::log(ArLog::Normal, "\nArMatrix::Cannot allocate memory\n");
      return;
//      Aria::exit(1);
    }
    for(int i = 0; i < myRows; i++)
      if(!(myElement[i] = new double[myCols]))
      {
	ArLog::log(ArLog::Normal, "\nArMatrix::Cannot allocate memory\n");
	return;
//	Aria::exit(1);
      }
  }
  void deallocate(void)
  {
    for(int i = 0; i < myRows; i++)
      if(myElement[i])
	delete [] myElement[i];
    if(myElement)
      delete [] myElement;
  }
protected:
  int myRows, myCols;
  double** myElement;
};

#endif // ARMATRIX_H
