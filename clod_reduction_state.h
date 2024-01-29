#pragma once
#include <vector>
#include <cgv_gl/clod_point_renderer.h>
#include "chunks.h"


namespace pct {
	
// this struct is used to store data required for disributing the point reduction/filtering over multiple frames
template <typename POINT>
struct clod_reduction_state {
	/**
		contains two buffers for reduction, one is written while the other is used for rendering
		buffers are swapped when reduction_demand is processed
	*/
	std::array<cgv::render::clod_point_buffer_manager, 2> reduced_points; 

	//iteraror on visible_chunks
	typename std::vector<chunk<POINT>*>::const_iterator chunk_iterator;

	std::vector<chunk<POINT>*> visible_chunks; //usually chunks after frustum culling

	int visible_buffer_ix = 0;
	int reduction_demand_processed = 0;
	int reduction_demand = 0;
	int reduction_epoch = 0;

	void init(cgv::render::context& ctx) {
		for (auto& b : reduced_points)
			b.init(ctx);
	}

	void clear(cgv::render::context& ctx) {
		for (auto& b : reduced_points)
			b.clear(ctx);
		visible_chunks.clear();
		visible_buffer_ix = 0;
		reduction_demand_processed = 0;
		reduction_demand = 0;
		reduction_epoch = 0;
	}

	// resize point buffers
	void resize(unsigned size) {
		for (auto& b : reduced_points)
			b.resize(size);
	}

	unsigned size() {
		return reduced_points[0].size();
	}
};	

}

