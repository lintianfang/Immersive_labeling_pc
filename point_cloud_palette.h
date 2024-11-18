#pragma once
#include "palette.h"
#include <vector>
#include "point_cloud_clipboard.h"
#include <utility>

namespace vrui {
	
	/// stores handles associated with a pointcloud
	struct point_cloud_palette_slot {
		int point_cloud_handle;
		int sphere_object_id;
		pct::clipboard_record_id record_id;
		bool preview_generated;
	};

	/** \breif provides view and some control over the clipboard contents
	*/ 
	class point_cloud_palette : public palette, public pct::clipboard_event_listener {

		static constexpr int slot_ix_none = -1;

		std::vector<point_cloud_palette_slot> palette_slots;
		int preview_detail_level;
		int toggle_delete_id; // palette object handle for toggle delete button
	
		cgv::ivec2 grid_dimensions;
		int highlighted_slot; // stores index of highlighted palette slot
	public:
		static double constexpr step_width = 0.05;
		static constexpr double toolbar_gap = -0.013;
	protected:
		/// finds the first unoccupied slot in the palette
		point_cloud_palette_slot* find_next_empty_slot();
		/// finds an occupied slot using the given record identifier
		point_cloud_palette_slot* find_slot_by_record_id(pct::clipboard_record_id rid);
	public:	

		/// acesses ix-th slot of the palette
		point_cloud_palette_slot* find_slot_by_index(int ix);

		point_cloud_palette();

		/// changes color of a sphere , pass -1 to remove the highlighting 
		void highlight(int slot_ix);

		/** \brief builds a grid of spheres
		*/
		void build(int width = 5, int height = 5);

		// -- clipboard_event_listener methods --

		void on_add_pointcloud(pct::point_cloud_clipboard* clipboard, pct::clipboard_record_id rid, pct::point_cloud_record& record);

		void on_erase_pointcloud(pct::point_cloud_clipboard* clipboard, pct::clipboard_record_id rid);

		void on_update_pointcloud(pct::point_cloud_clipboard* clipboard, pct::clipboard_record_id rid);
	};

}