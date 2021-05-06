#include <chrono>

template<typename period = std::micro, typename rep = size_t>
class Timer {
public:
	using count_type = std::chrono::duration<rep,period>;

	Timer() {
		begin = end = std::chrono::high_resolution_clock::time_point::min();
	}
	inline void start(void) {
		begin = std::chrono::high_resolution_clock::now();
	}
	inline void stop(void) {
		end = std::chrono::high_resolution_clock::now();
	}
	inline rep count(void) {
		using namespace std::chrono;
		return duration_cast<count_type>(end - begin).count();
	}
private:
	std::chrono::time_point<std::chrono::high_resolution_clock> begin, end;
};