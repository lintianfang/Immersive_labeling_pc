#include "waypoint_navigation.h"
namespace pct {

	cgv::render::render_types::vec3 waypoint_navigation::next_position(vec3 position, float dt)
	{
		if (next_way_point < path.size()) {
			vec3 dist = path[next_way_point] - position;
			vec3 dir = dist;
			dir.normalize(); //breaks if next waypoint is the same as the current
			vec3 step = dir * speed * dt;
			if (step.sqr_length() > dist.sqr_length()) {
				position = path[next_way_point++];
				if (loop_path && next_way_point >= path.size())
					next_way_point = 0;
			}
			else {
				position += step;
			}
		}
		return position;
	}

	void waypoint_navigation::set_path(vec3* path, size_t size)
	{
		this->path.resize(size);
		memcpy(this->path.data(), path, this->path.size());

	}
}

#include <cgv/gui/provider.h>

namespace cgv {
	namespace gui {

		struct waypoint_navigation_gui_creator : public gui_creator {
			/// attempt to create a gui and return whether this was successful
			bool create(provider* p, const std::string& label,
				void* value_ptr, const std::string& value_type,
				const std::string& gui_type, const std::string& options, bool*) {
				if (value_type != cgv::type::info::type_name<pct::waypoint_navigation>::get_name())
					return false;
				pct::waypoint_navigation* rs_ptr = reinterpret_cast<pct::waypoint_navigation*>(value_ptr);
				cgv::base::base* b = dynamic_cast<cgv::base::base*>(p);

				p->add_member_control(b, "loop", rs_ptr->loop_path, "check");
				p->add_member_control(b, "speed", rs_ptr->speed, "value_slider", "min=0.0;max=4.0;log=false;ticks=true");
				p->add_member_control(b, "next way point", rs_ptr->next_way_point, "value_input", "");
				return true;
			}
		};

		cgv::gui::gui_creator_registration<waypoint_navigation_gui_creator> wpngc("waypoint_navigation_gui_creator");
	}
}