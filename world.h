#include <cgv/base/node.h>
#include <cgv/math/fvec.h>
#include <cgv/gui/event_handler.h>
#include <cgv_gl/box_renderer.h>
#include <cgv/gui/provider.h>
#include <cgv/render/drawable.h>
#include <cgv/render/context.h>
#include <libs/cgv_gl/attribute_array_manager.h>

namespace pc_cleaning_tool {

	struct ray_box_intersection_information : public cgv::render::render_types {
		std::vector<vec3> points;
		std::vector<int>  controller_indices;
		std::vector<rgb> colors;

		void clear();
	};

	class world :
		public cgv::base::node,
		public cgv::render::drawable
	{
		bool renderer_data_outdated = false;

		// environment geometry
		std::vector<box3> boxes;
		std::vector<rgb> box_colors;
		cgv::render::box_render_style style;

		cgv::render::box_renderer world_renderer;
		cgv::render::attribute_array_manager aam;

	public:
		// construction control flags
		bool show_environment;
		bool show_table;
		bool show_floor;

	protected:
		void construct_table(float tw, float td, float th, float tW);
		void construct_room(float w, float d, float h, float W, bool walls, bool ceiling);
		void construct_environment(float s, float ew, float ed, float w, float d, float h);

	public:
		void build_scene(float w, float d, float h, float W, float tw, float td, float th, float tW);

		void clear_scene();

		world();

		bool init(cgv::render::context& ctx);

		void clear(cgv::render::context& ctx);

		void draw(cgv::render::context& ctx);

		void prepare_renderer(cgv::render::context& ctx);

		box3& ref_floor_box();
		rgb& ref_floor_box_clr();

		ray_box_intersection_information compute_intersections(const vec3& origin, const vec3& direction, int ci, const rgb& color);

	};
}

