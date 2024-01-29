#pragma once

#include <vector>

namespace stats {

	/// helper class for statistics collection, keeps the last values inserted over the update method
	template<typename T>
	class scalar_sliding_window {
		std::vector<T> history;
		T history_sum;
		int history_last_ix; // index of last value written to history
		
	public:
		/// @param[in] history_size max. number of stored values 
		scalar_sliding_window(const int history_size = 100) : history_last_ix(history_size), history_sum(0) {
			history.resize(history_size, 0);
		}

		void update(const T& data) {
			history_last_ix = (history_last_ix + 1) % (int)history.size();
			history_sum += data;
			history_sum -= history[history_last_ix];
			history[history_last_ix] = data;

			if (history_last_ix == 0) {
				compute_sum();
			}
		}

		template <typename F>
		F average() const {
			return static_cast<F>(history_sum) / static_cast<F>(history.size());
		}

		void resize(const size_t size) {
			history.resize(size);
		}

		size_t get_size() {
			return history.size();
		}

		/// access to stored values
		T* data() {
			return history.data();
		}

		const T& last() {
			return history[history_last_ix];
		}

		/// returns the cached sum. use compute sum for up to date information
		const T& sum() const {
			return history_sum;
		}

		const T& compute_sum() {
			history_sum = 0;
			for (auto& v : history) {
				history_sum += v;
			}
			return history_sum;
		}
	};
}