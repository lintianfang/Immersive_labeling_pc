#include "point_cloud_palette.h"
#include "octree.h"


namespace vrui {

	namespace {
		const cgv::rgba default_sphere_color(0.38);
		const cgv::rgba highlight_sphere_color(0.05,0.78,0.17,1.0);
	}

	point_cloud_palette_slot* point_cloud_palette::find_next_empty_slot()
	{
		for (auto& h : palette_slots) {
			if (h.record_id == -1)
				return &h;
		}
		return nullptr;
	}

	point_cloud_palette_slot* point_cloud_palette::find_slot_by_record_id(pct::clipboard_record_id rid)
	{
		if (rid == -1) {
			return nullptr;
		}

		for (auto& h : palette_slots) {
			if (h.record_id == rid)
				return &h;
		}
		return nullptr;
	}

	point_cloud_palette_slot* point_cloud_palette::find_slot_by_index(int ix)
	{	
		if (ix >= 0 && ix < palette_slots.size())
			return &palette_slots[ix];
		return nullptr;
	}
	
	point_cloud_palette::point_cloud_palette() : preview_detail_level(0), highlighted_slot(-1)
	{
	}

	void point_cloud_palette::highlight(int slot_ix)
	{
		int size = grid_dimensions.x() * grid_dimensions.y();
		std::vector<cgv::rgba> colors(size, default_sphere_color);
		if (slot_ix >= 0 && slot_ix < size)
			colors[slot_ix] = highlight_sphere_color;
		this->set_colors(colors);
		this->set_palette_changed();
		highlighted_slot = slot_ix;
	}

	void point_cloud_palette::build(int width, int height)
	{
		grid_dimensions = cgv::ivec2(width, height);
		sphere_style().radius = 0.02f;
		box_style().default_extent = cgv::vec3(0.04, 0.04, 0.04);
		wire_box_style().default_extent = cgv::vec3(0.04, 0.04, 0.04);

		//sphere grid
		uintptr_t num_center_spheres = 0;
		int col_i = 0;
		int off = width / 2;
		for (int iz = height; iz > 0; iz--) {
			for (int ix = -off; ix < width-off; ix++) {
				point_cloud_palette_slot entity;
				//add sphere to palette
				int id = this->add_object(
					PaletteObject::PO_SPHERE_FRAME,
					cgv::vec3(ix * step_width, 0.1, -iz * step_width),
					default_sphere_color);
				entity.sphere_object_id = id;

				// allocate space for pointclouds
				cgv::vec3 position = cgv::vec3(ix * step_width, 0.1, -iz * step_width);
				entity.point_cloud_handle = this->add_pointcloud(nullptr, nullptr, 0, position, cgv::quat(1.f, 0.f, 0.f, 0.f));
				entity.record_id = -1;
				entity.preview_generated = false;
				// store handles
				palette_slots.push_back(entity);
			}
		}
		
		set_palette_changed();
	}

	void point_cloud_palette::on_add_pointcloud(pct::point_cloud_clipboard* clipboard, pct::clipboard_record_id rid, pct::point_cloud_record& record)
	{
		//create preview and add to palette
				
		std::vector<cgv::vec3> point_positions;
		std::vector<cgv::rgb8> point_colors;

		if (record.points) {
			generate_preview(*record.points, point_positions, point_colors, preview_detail_level);
		}

		//add preview to palette
		auto* h = find_next_empty_slot();
		if (h) {
			h->record_id = rid;
			this->replace_pointcloud(h->point_cloud_handle, point_positions.data(), point_colors.data(), point_positions.size());
		}
		/*else {
			//TODO handle full palette
		}*/
	}

	void point_cloud_palette::on_erase_pointcloud(pct::point_cloud_clipboard* clipboard, pct::clipboard_record_id rid)
	{
		auto* pcs = find_slot_by_record_id(rid);
		if (pcs) {
			this->replace_pointcloud(pcs->point_cloud_handle, nullptr, nullptr, 0);
			pcs->record_id = -1;
			pcs->preview_generated = false;
			if (find_slot_by_index(highlighted_slot) == pcs) { //pcs != nullptr at this point
				highlight(slot_ix_none);
			}
		}
	}

	void point_cloud_palette::on_update_pointcloud(pct::point_cloud_clipboard* clipboard, pct::clipboard_record_id rid)
	{
		auto* h = find_slot_by_record_id(rid);
		if (h) {
			auto* record_ptr = clipboard->get_by_id(rid);
			if (record_ptr && record_ptr->points) {
				if (!record_ptr->is_dynamic || !h->preview_generated) {
					//update preview
					std::vector<cgv::vec3> point_positions;
					std::vector<cgv::rgb8> point_colors;
					generate_preview(*record_ptr->points, point_positions, point_colors, preview_detail_level);
					h->preview_generated = true;
					this->replace_pointcloud(h->point_cloud_handle, point_positions.data(), point_colors.data(), point_positions.size());
					//this->replace_pointcloud(h->point_cloud_handle, &record_ptr->points->pnt(0), &record_ptr->points->clr(0), record_ptr->points->get_nr_points());
				}

				h->record_id = rid;
			}
		}
		else {
			std::cerr << "point_cloud_palette::on_update_pointcloud: could not find associated slot rid=" << rid << "\n";
		}
	}


}