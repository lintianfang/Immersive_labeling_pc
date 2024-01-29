#include <cgv/gui/control.h>

namespace pct {


	template <typename T>
	class bitfield_control_provider : public cgv::gui::control_provider<bool> {
		T mask;
		T* data;
	public:
		bitfield_control_provider(const T mask, T* data) : data(data), mask(mask) {}

		bitfield_control_provider() : mask(0), data(nullptr) {};

		void set_value(const bool& value, void* user_data) {
			//T b = ~(~0u << bits_end) & (~0u << bits_start);
			assert(data != nullptr);
			if (value) {
				*data = *data | mask;
			}
			else {
				*data = *data & (~mask);
			}
		}

		const bool get_value(void* user_data) const {
			assert(data != nullptr);
			return (mask & *data) == mask;
		}

		bool controls(const void* ptr, void* user_data) const {
			return ptr == data;
		}
	};

}