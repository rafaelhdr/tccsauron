#pragma once

namespace sauron{

template<typename Functor> class CallbackHandler
{
public:
protected:
	inline void addCallback(Functor callback){
		m_callbacks.push_back(callback);
	}
	inline void invokeCallbacks() {
		for(std::vector<Functor>::iterator it = m_callbacks.begin(); it != m_callbacks.end();
			it++) {
				(*it)();
		}
	}

	template<typename T>
	inline void invokeCallbacks(T& t) {
		for(std::vector<Functor>::iterator it = m_callbacks.begin(); it != m_callbacks.end();
			it++) {
				(*it)(t);
		}
	}
	template<typename T, typename R>
	inline void invokeCallbacks(T& t, R& r) {
		for(std::vector<Functor>::iterator it = m_callbacks.begin(); it != m_callbacks.end();
			it++) {
				(*it)(t, r);
		}
	}

	template<typename T, typename R, typename S>
	inline void invokeCallbacks(T& t, R& r, S& s) {
		for(std::vector<Functor>::iterator it = m_callbacks.begin(); it != m_callbacks.end();
			it++) {
				(*it)(t, r, s);
		}
	}

private:
	std::vector<Functor> m_callbacks;
};
}