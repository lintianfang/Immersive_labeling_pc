#include "point_cloud_registration_tool.h"
#include <cgv/signal/signal.h>
#include <cgv/signal/rebind.h>
#include <cgv/utils/file.h>
#include <cgv_gl/clod_point_renderer.h>
#include <cgv/math/ftransform.h>
#include <sstream>
#include <stdexcept>

cgv::math::fmat<float, 3, 4> reg_icp(std::unique_ptr<point_cloud> points, std::unique_ptr<point_cloud> target, cgv::math::fmat<float, 3, 3> rotation, cgv::math::fvec<float, 3> translation) {
	cgv::pointcloud::ICP icp;
	icp.set_source_cloud(*points);
	icp.set_target_cloud(*target);
	icp.set_num_random(1000);
	if (target->get_nr_points() > 0)
		icp.reg_icp(rotation, translation);
	cgv::math::fmat<float, 3, 4> pose;
	pose.set_col(0, rotation.col(0));
	pose.set_col(1, rotation.col(1));
	pose.set_col(2, rotation.col(2));
	pose.set_col(3, translation);
	return pose;
};

cgv::math::fmat<float, 3, 4> reg_sicp(std::unique_ptr<point_cloud> points, std::unique_ptr<point_cloud> target, cgv::math::fmat<float, 3, 3> rotation, cgv::math::fvec<float, 3> translation) {
	cgv::pointcloud::SICP sicp;
	sicp.set_source_cloud(*points);
	sicp.set_target_cloud(*target);
	if (target->get_nr_points() > 0)
		sicp.register_point_to_point(rotation, translation);
	cgv::math::fmat<float, 3, 4> pose;
	pose.set_col(0, rotation.col(0));
	pose.set_col(1, rotation.col(1));
	pose.set_col(2, rotation.col(2));
	pose.set_col(3, translation);
	return pose;
};

namespace pct {
	pct::clipboard_record_id point_cloud_registration_tool::allocate_new_pointcloud_record()
	{
		std::unique_ptr<point_cloud_record> record = std::make_unique<point_cloud_record>();
		record->scale = default_scale;
		record->rotation = quat(1.f, 0.f, 0.f, 0.f);
		record->translation = vec3(0.f);
		record->points = std::make_unique<point_cloud>();
		std::stringstream ss_name;
		ss_name << "rgbd input capture " << ++num_captured_pointclouds;
		record->name = ss_name.str();
		return clipboard->move_from(record);
	}

	point_cloud_registration_tool::point_cloud_registration_tool(std::shared_ptr<point_cloud_clipboard>& storage) : source(nullptr)
	{
		clipboard = storage;
		selected_index = 1;
		store();

		scheduled_move_in_front = false;
		icp_target_transform.identity();
		point_style_p.percentual_halo_width = 10.0;
		point_style_p.halo_color_strength = 0.20;
		point_style_p.halo_color = rgba(1.0, 1.0, 1.0, 0.5);
		point_style_p.measure_point_size_in_pixel = false;
		point_style_p.point_size = 0.5;
		reg_mode_p = RM_ICP;
	}

	void point_cloud_registration_tool::set_provider(cgv::pointcloud::point_cloud_provider* prov)
	{
		if (source) {
			cgv::signal::disconnect(source->new_point_cloud_ready(), this, &point_cloud_registration_tool::on_point_cloud_update);
		}

		source = prov;
		if (prov) {
			cgv::signal::connect(source->new_point_cloud_ready(), this, &point_cloud_registration_tool::on_point_cloud_update);
		}
	}

	int point_cloud_registration_tool::get_index() const
	{
		return selected_index;
	}

	void point_cloud_registration_tool::set_index(int i)
	{
		if (i >= 0 && i <= max_index())
			selected_index = i;
	}

	int point_cloud_registration_tool::max_index() const
	{
		return point_cloud_ids.size() - 1;
	}

	void point_cloud_registration_tool::set_registration_pointcloud(clipboard_record_id rid)
	{
		point_cloud_ids[registration_slot] = rid;
	}

	point_cloud* point_cloud_registration_tool::get_point_cloud()
	{
		if (point_cloud_ids[selected_index] != -1) {
			auto* record_ptr = clipboard->get_by_id(point_cloud_ids[selected_index]);
			if (record_ptr && record_ptr->points)
				return record_ptr->points.get();
		}
		return nullptr;
	}

	point_cloud_record* point_cloud_registration_tool::get_point_cloud_record()
	{
		if (point_cloud_ids[selected_index] != -1) {
			return clipboard->get_by_id(point_cloud_ids[selected_index]);
		}
		return nullptr;
	}

	//finds a new empty pointcloud slot to fill by the point cloud provider
	void point_cloud_registration_tool::store()
	{
		auto* prev_record = this->get_point_cloud_record();
		if (prev_record) {
			prev_record->is_dynamic = false; //set previous record static
			clipboard->update(point_cloud_ids[selected_index]);
		}

		std::unique_ptr<point_cloud_record> record = std::make_unique<point_cloud_record>();
		record->scale = default_scale;
		record->rotation = quat(1.f, 0.f, 0.f, 0.f);
		record->translation = vec3(0.f);
		record->points = std::make_unique<point_cloud>();
		std::stringstream ss_name;
		ss_name << "rgbd input capture " << ++num_captured_pointclouds;
		record->name = ss_name.str();

		clipboard_record_id new_id = clipboard->move_from(record);
		point_cloud_ids[registration_slot] = point_cloud_ids[live_update_slot];
		point_cloud_ids[live_update_slot] = new_id;
	}
	/*
	//insert a pointcloud
	void point_cloud_registration_tool::store(std::unique_ptr<point_cloud>&& points, const std::string& name)
	{
		//insert before end
		auto ix = point_cloud_ids.size() - 1;

		//move to back


		point_cloud_ids.push_back(point_cloud_ids[ix]);

		std::unique_ptr<point_cloud_record> record = std::make_unique<point_cloud_record>();
		record->scale = points->ref_point_cloud_scale();
		record->rotation = quat(cgv::math::rotate3<float>(points->ref_point_cloud_rotation()));
		record->translation = points->ref_point_cloud_position();
		record->points = std::move(points);
		record->name = name;

		clipboard_record_id new_id = clipboard->move_from(record);
		point_cloud_ids[ix] = new_id;
	}
	
	bool point_cloud_registration_tool::erase()
	{
		if (point_cloud_ids.size() > 1) {
			auto deleted_id = point_cloud_ids[selected_index];
			point_cloud_ids.erase(point_cloud_ids.begin() + selected_index);
			clipboard->erase(deleted_id);
			return true;
		}
		return false;
	}
	*/
	bool point_cloud_registration_tool::does_live_updates()
	{
		return selected_index == live_update_slot;
	}

	void point_cloud_registration_tool::on_point_cloud_update()
	{
		if (point_cloud_ids[live_update_slot] == -1) {
			// record/clipboard entry was deleted, allocate a new empty record to place captured data in
			point_cloud_ids[live_update_slot] = allocate_new_pointcloud_record();
		}
		
		auto* record_ptr = clipboard->get_by_id(point_cloud_ids[live_update_slot]);
		auto* point_cloud_ptr = (record_ptr && record_ptr->points) ? record_ptr->points.get() : nullptr;
		
		if (point_cloud_ptr) {
			*point_cloud_ptr = source->get_point_cloud();
			record_ptr->is_dynamic = true;
			clipboard->update(point_cloud_ids[live_update_slot]);
		}
	}

	bool point_cloud_registration_tool::registration_thread_is_busy() const
	{
		return icp_future.valid() && icp_future.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready;
	}


	cgv::pointcloud::point_cloud_provider* point_cloud_registration_tool::get_provider() const
	{
		return source;
	}

	void point_cloud_registration_tool::init(cgv::render::context& ctx)
	{
		cgv::render::ref_point_renderer(ctx, 1);
	}

	void point_cloud_registration_tool::draw(cgv::render::context& ctx)
	{
		auto* point_cloud_ptr = get_point_cloud();
		if (point_cloud_ptr) {
			auto& points = *point_cloud_ptr;
			auto& pr = ref_point_renderer(ctx);
			pr.set_render_style(point_style_p);

			ctx.push_modelview_matrix();
			mat4 model = model_matrix();
			ctx.set_modelview_matrix(ctx.get_modelview_matrix() * model);
			size_t nr_points = (size_t)points.get_nr_points();
			if (nr_points > 0) {
				pr.set_position_array(ctx, &points.pnt(0), nr_points);
				pr.set_color_array(ctx, &points.clr(0), nr_points);
				pr.render(ctx, 0, nr_points);
			}
			ctx.pop_modelview_matrix();
		}
	}

	void point_cloud_registration_tool::clear(cgv::render::context& ctx)
	{
		cgv::render::ref_point_renderer(ctx, -1);
	}

	void point_cloud_registration_tool::copy_point_style(cgv::render::clod_point_render_style& style)
	{
		point_style_p.point_size = style.pointSize;
		point_style_p.blend_points = false;
	}


	bool point_cloud_registration_tool::move_in_front(const cgv::math::fmat<float, 3, 4>& pose)
	{
		point_cloud& pc = *get_point_cloud();
		int num_points = pc.get_nr_points();
		mat4 model = model_matrix();
		vec3 centroid(0.f);
		for (int i = 0; i < num_points; ++i) {
			centroid += (vec3)(model * pc.pnt(i).lift());
		}
		if (num_points == 0)
			return false;

		centroid /= num_points;

		vec3 move = pose.col(3) - centroid;

		ref_translation() += move;
		return true;
	}

	float& point_cloud_registration_tool::ref_scale()
	{
		return clipboard->get_by_id(point_cloud_ids[selected_index])->scale;
	}

	cgv::math::fvec<float, 3>& point_cloud_registration_tool::ref_translation()
	{
		return clipboard->get_by_id(point_cloud_ids[selected_index])->translation;
	}

	cgv::math::quaternion<float>& point_cloud_registration_tool::ref_rotation()
	{
		return clipboard->get_by_id(point_cloud_ids[selected_index])->rotation;
	}

	std::string& point_cloud_registration_tool::ref_name()
	{
		if (point_cloud_ids[selected_index] != -1) {
			return clipboard->get_by_id(point_cloud_ids[selected_index])->name;
		}
		throw std::runtime_error("undefined pointcloud id");
	}

	const std::string& point_cloud_registration_tool::get_name() const
	{
		static std::string empty_str = "";
		if (point_cloud_ids[selected_index] != -1) {
			return clipboard->get_by_id(point_cloud_ids[selected_index])->name;
		}
		return empty_str;
	}

	cgv::math::fmat<float, 4, 4> point_cloud_registration_tool::model_matrix()
	{
		mat4 model = cgv::math::translate4(ref_translation()) * ref_rotation().get_homogeneous_matrix() * cgv::math::scale4(vec3(ref_scale()));
		return model;
	}

	std::string point_cloud_registration_tool::info()
	{
		return std::string();
	}

	void point_cloud_registration_tool::reg_async(const point_cloud& target, const cgv::math::fmat<float, 4, 4>& target_transform)
	{
		cgv::math::fmat<float, 3, 3> rotation = ref_rotation().get_matrix();
		cgv::math::fvec<float, 3> translation = ref_translation();
		reg_async(target, target_transform, rotation, translation);
	}

	void point_cloud_registration_tool::reg_async(const point_cloud& target, const cgv::math::fmat<float, 4, 4>& target_transform, cgv::math::fmat<float, 3, 3>& rotation, cgv::math::fvec<float, 3>& translation)
	{
		//icp needs copies due to missing support for transforms
		std::unique_ptr<point_cloud> transformed_target = std::make_unique<point_cloud>(target);
		std::unique_ptr<point_cloud> transformed_pc = std::make_unique<point_cloud>(*get_point_cloud());
		mat4 model_transform = model_matrix();

		int target_num_points = transformed_target->get_nr_points();
		for (int i = 0; i < target_num_points; ++i) {
			transformed_target->pnt(i) = target_transform * transformed_target->pnt(i).lift();
		}

		int num_points = transformed_pc->get_nr_points();

		for (int i = 0; i < num_points; ++i) {
			transformed_pc->pnt(i) = model_transform * transformed_pc->pnt(i).lift();
		}

		cgv::math::fvec<float, 3> trans(0.f);
		cgv::math::fmat<float, 3, 3> rot;
		rot.identity();

		icp_target_index = selected_index;
		icp_target_transform = target_transform;
		icp_initial_transform.identity();
		switch (reg_mode_p) {
		case registration_method::RM_ICP:
			icp_future = std::async(std::launch::async, reg_icp, std::move(transformed_pc), std::move(transformed_target), rot, trans);
			break;
		case registration_method::RM_SICP:
			icp_future = std::async(std::launch::async, reg_sicp, std::move(transformed_pc), std::move(transformed_target), rot, trans);
			break;
		}
	}

	bool point_cloud_registration_tool::reg_get_results()
	{
		if (!icp_future.valid() || icp_future.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready)
			return false;
		auto pose = icp_future.get();
		mat3 rotation;
		for (int i = 0; i < 3; ++i) {
			rotation.set_col(i, pose.col(i));
		}
		vec3 translation = pose.col(3);

		clipboard->get_by_id(point_cloud_ids[icp_target_index])->translation = translation + rotation * clipboard->get_by_id(point_cloud_ids[icp_target_index])->translation;
		clipboard->get_by_id(point_cloud_ids[icp_target_index])->rotation = quat(rotation) * clipboard->get_by_id(point_cloud_ids[icp_target_index])->rotation;

		return true;
	}

	cgv::render::point_render_style& point_cloud_registration_tool::point_style()
	{
		return this->point_style_p;
	}

	registration_method& point_cloud_registration_tool::reg_mode()
	{
		return reg_mode_p;
	}

	void point_cloud_registration_tool::on_add_pointcloud(point_cloud_clipboard* clipboard, clipboard_record_id rid, point_cloud_record& record)
	{
		//nothing to do
	}

	void point_cloud_registration_tool::on_erase_pointcloud(point_cloud_clipboard* clipboard, clipboard_record_id rid)
	{
		//check if one of the active point clouds got deleted
		for (auto& ix : point_cloud_ids) {
			if (rid == ix) {
				ix = -1; //mark as deleted
			}
		}
	}

	void point_cloud_registration_tool::on_update_pointcloud(point_cloud_clipboard* clipboard, clipboard_record_id rid)
	{
		//nothing to do
	}

}

#include <cgv/gui/provider.h>

namespace cgv {
	namespace gui {

		struct point_cloud_registration_tool_gui_creator : public gui_creator {
			/// attempt to create a gui and return whether this was successful
			bool create(provider* p, const std::string& label,
				void* value_ptr, const std::string& value_type,
				const std::string& gui_type, const std::string& options, bool*) {
				if (value_type != cgv::type::info::type_name<pct::point_cloud_registration_tool>::get_name())
					return false;
				pct::point_cloud_registration_tool* rs_ptr = reinterpret_cast<pct::point_cloud_registration_tool*>(value_ptr);
				cgv::base::base* b = dynamic_cast<cgv::base::base*>(p);
				
				std::string mode_defs = "enums='ICP=0;SICP=1,GoICP=2'";
				p->add_member_control(b, "Registration mode", rs_ptr->reg_mode(), "dropdown", mode_defs);
				if (p->begin_tree_node("Point Style", rs_ptr->point_style(), false)) {
					p->align("\a");
					p->add_gui("Point style", rs_ptr->point_style());
					p->align("\b");
					p->end_tree_node(rs_ptr->point_style());
				}				
				return true;
			}
		};

		cgv::gui::gui_creator_registration<point_cloud_registration_tool_gui_creator> cprsgc("point_cloud_registration_tool_gui_creator");
	}
}