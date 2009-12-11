#pragma once
template<typename T, typename U> class ArFunctor2;
namespace sauron
{
class SonarReading;
class ISonarDataAsyncProvider
{
public:
	typedef ArFunctor2<int,SonarReading> AddReadingCallback;
	
	virtual void setAddReadingCallback(int sonarNumber,
		AddReadingCallback* p_callback) = 0;
	virtual void removeAddReadingCallback(int sonarNumber,
		AddReadingCallback* p_callback) = 0;
	virtual void removeCallback(int sonarNumber) = 0;
	virtual void removeAllCallbacks() = 0;
};
}