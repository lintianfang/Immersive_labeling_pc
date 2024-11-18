#pragma once
#include <vector>
#include <memory>
#include <string>
#include <point_cloud.h>
#include <point_cloud_provider.h>
#include <ICP.h>
#include <SICP.h>
#include <cgv/base/node.h>
#include <cgv/render/drawable.h>
#include <cgv/math/fmat.h>
#include <cgv/signal/rebind.h>
#include <cgv/signal/signal.h>
#include <cgv_gl/point_renderer.h>
#include <cgv_gl/clod_point_renderer.h>

#include "history.h"
#include "point_cloud_clipboard.h"

struct cgv::render::clod_point_render_style;


namespace pct { //namespace of point cloud cleaning tool

	struct point_cloud_registration_tool_gui_creator;

	enum registration_method {
		RM_ICP,
		RM_SICP,
		RM_GoICP,
		NUM_REGISTRATION_METHODS
	};

	/**	Enables access to pointclouds provided by the rgdb_input plugin.
	*	Captured pointclouds are stored in the clipboard provided by object creation
	*/
	class point_cloud_registration_tool :
		public cgv::signal::tacker, //signal handling for rgbd_input
		public pct::clipboard_event_listener //for clipboard event handling
	{

		std::shared_ptr<point_cloud_clipboard> clipboard;
		std::array<clipboard_record_id, 2> point_cloud_ids = {-1,-1};
		
	public:
		//types in cgv namespace
		using ivec2 = cgv::ivec2;
		using ivec3 = cgv::ivec3;
		using dvec2 = cgv::dvec2;
		using vec2 = cgv::vec2;
		using vec3 = cgv::vec3;
		using vec4 = cgv::vec4;
		using mat3 = cgv::mat3;
		using mat4 = cgv::mat4;
		using mat34 = cgv::mat34;
		using dmat4 = cgv::dmat4;
		using quat = cgv::quat;
		using rgb = cgv::rgb;
		using rgba = cgv::rgba;
		using rgb8 = cgv::rgb8;
		using rgba8 = cgv::rgba8;
		using box3 = cgv::box3;

		// indices of point_cloud_ids
		static constexpr size_t registration_slot = 0;
		static constexpr size_t live_update_slot = 1;
	private:
		cgv::pointcloud::point_cloud_provider* source;

		int num_captured_pointclouds = 2;

		int selected_index;

		std::future<cgv::math::fmat<float, 3, 4>> icp_future;
		int icp_target_index;
		cgv::math::fmat<float, 4, 4> icp_target_transform, icp_initial_transform;


		cgv::render::point_render_style point_style_p;

		bool scheduled_move_in_front;
		cgv::math::fmat<float, 3, 4> scheduled_move_in_front_param_1;

		registration_method reg_mode_p;

		static constexpr float default_scale = 0.001f;
	protected:
		pct::clipboard_record_id allocate_new_pointcloud_record();
	public:

		point_cloud_registration_tool() = default;

		point_cloud_registration_tool(std::shared_ptr<point_cloud_clipboard>& storage);

		void set_provider(cgv::pointcloud::point_cloud_provider* prov);

		// returns index of active point cloud
		int get_index() const;
		// switch to pointcloud with index i
		void set_index(int i);
		int max_index() const;

		void set_registration_pointcloud(clipboard_record_id rid);

		/// reference active point cloud
		point_cloud* get_point_cloud();

		/// reference to record
		point_cloud_record* get_point_cloud_record();

		// create a new empty pointcloud with index max_index() 
		void store();
		
		// check if pointcloud is updated by source
		bool does_live_updates();

		void on_point_cloud_update();
		// check if point cloud registration is running
		bool registration_thread_is_busy() const;

		cgv::pointcloud::point_cloud_provider* get_provider() const;

		void init(cgv::render::context& ctx);

		void draw(cgv::render::context& ctx);

		void clear(cgv::render::context& ctx);

		void copy_point_style(cgv::render::clod_point_render_style& style);
		
		// move active pointcloud to the origin of the coordinate system described by pose
		bool move_in_front(const cgv::math::fmat<float, 3, 4>& pose);

		float& ref_scale();

		cgv::math::fvec<float, 3>& ref_translation();

		cgv::math::quaternion<float>& ref_rotation();

		std::string& ref_name();
		const std::string& get_name() const;

		cgv::math::fmat<float, 4, 4> model_matrix();

		//output status in text form
		std::string info();

		//reg async on selected pointcloud
		void reg_async(const point_cloud& target, const cgv::math::fmat<float, 4, 4>& target_transform);

		void reg_async(const point_cloud& target, const cgv::math::fmat<float, 4, 4>& target_transform, cgv::math::fmat<float, 3, 3>& rotation, cgv::math::fvec<float, 3>& translation);

		bool reg_get_results();

		cgv::render::point_render_style& point_style();

		registration_method& reg_mode();

		/* clipboard_event_listener methods */

		void on_add_pointcloud(point_cloud_clipboard* clipboard, clipboard_record_id rid, point_cloud_record& record);

		void on_erase_pointcloud(point_cloud_clipboard* clipboard, clipboard_record_id rid);

		void on_update_pointcloud(point_cloud_clipboard* clipboard, clipboard_record_id rid);
	};

}