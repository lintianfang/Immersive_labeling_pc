#pragma once

#include <cgv/media/axis_aligned_box.h>
#include <limits>

namespace cgv {
	namespace media {
		template <typename T>
		bool update_range(T lb, T ub, T o, T d, unsigned i, unsigned& i_min, unsigned& i_max, T& t_min, T& t_max, T epsilon) {
			// check for case where ray component is parallel to plane slab
			if (std::abs(d) < epsilon)
				return o >= lb && o <= ub;
			// compute intersections
			T f = T(1) / d;
			T t0 = f * (lb - o);
			T t1 = f * (ub - o);
			if (t0 > t1)
				std::swap(t0, t1);

			// update interval
			if (t0 > t_min) {
				i_min = i;
				t_min = t0;
			}
			if (t1 < t_max) {
				i_max = i;
				t_max = t1;
			}
			return true;
		}

		template <typename T, cgv::type::uint32_type N>
		bool ray_axis_aligned_box_intersection(
			const cgv::math::fvec<T,N>& origin, const cgv::math::fvec<T,N>& direction, const axis_aligned_box<T, N>& aabb,
			T& t_result, cgv::math::fvec<T, N>& p_result, cgv::math::fvec<T, N>& n_result,
			T epsilon) {

			const cgv::math::fvec<T, N>& lb = aabb.get_min_pnt();
			const cgv::math::fvec<T, N>& ub = aabb.get_max_pnt();

			T t_min = -std::numeric_limits<T>::max();
			T t_max = std::numeric_limits<T>::max();

			unsigned i_min = 0, i_max = 0;
			for (unsigned i = 0; i < N; ++i)
				if (!update_range(lb[i], ub[i], origin[i], direction[i], i, i_min, i_max, t_min, t_max, epsilon))
					return false;

			if (t_max < 0 || t_min > t_max)
				return false;

			if (t_min < T(0)) {
				t_result = t_max;
				i_min = i_max;
			}
			else
				t_result = t_min;

			p_result = origin + t_result * direction;

			n_result.zeros();
			n_result[i_min] = direction[i_min] > T(0) ? T(-1) : T(1);
		
			return true;
		}
	}
}
