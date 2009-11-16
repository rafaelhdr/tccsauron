#pragma once
#include <map>
#include <boost/thread/mutex.hpp>
namespace sauron{

template<typename Functor> class CallbackHandler
{
public:
	CallbackHandler() :
	  m_nextId(0){ }
protected:
	typedef std::map<int, Functor> mapType;

	inline int addCallback(Functor callback){
		boost::unique_lock<boost::mutex> lock(m_callbacksMutex);
		m_callbacks[m_nextId] = callback;
		return m_nextId++;
	}

	inline void removeCallback(int callbackId) {
		boost::unique_lock<boost::mutex> lock(m_callbacksMutex);
		mapType::iterator it = m_callbacks.find(callbackId);
		if(it != m_callbacks.end())
			m_callbacks.erase(it);
	}

	inline void invokeCallbacks() {
		boost::unique_lock<boost::mutex> lock(m_callbacksMutex);
		mapType::iterator it;
		for(it = m_callbacks.begin(); it != m_callbacks.end(); it++) {
			(it->second)();
		}
	}

	template<typename T>
	inline void invokeCallbacks(T t) {
		boost::unique_lock<boost::mutex> lock(m_callbacksMutex);
		mapType::iterator it;
		for(it = m_callbacks.begin(); it != m_callbacks.end(); it++) {
			(it->second)(t);
		}/*
		for(std::vector<Functor>::iterator it = m_callbacks.begin(); it != m_callbacks.end();
			it++) {
				(*it)(t);
		}*/
	}
	template<typename T, typename R>
	inline void invokeCallbacks(T t, R r) {
		boost::unique_lock<boost::mutex> lock(m_callbacksMutex);
		mapType::iterator it;
		for(it = m_callbacks.begin(); it != m_callbacks.end(); it++) {
			(it->second)(t, r);
		}
	}

	template<typename T, typename R, typename S>
	inline void invokeCallbacks(T t, R r, S s) {
		boost::unique_lock<boost::mutex> lock(m_callbacksMutex);
		mapType::iterator it;
		for(it = m_callbacks.begin(); it != m_callbacks.end(); it++) {
			(it->second)(t, r, s);
		}
	}

private:
	std::map<int, Functor> m_callbacks;
	boost::mutex m_callbacksMutex;
	int m_nextId;
};
}