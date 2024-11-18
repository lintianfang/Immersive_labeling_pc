#include "label_palette.h"

namespace vrui {
	label_palette::label_palette()
	{
	}

	void label_palette::build(const std::vector<cgv::rgba>& PALETTE_COLOR_MAPPING, int width, int height, const PaletteObject shape)
	{
		sphere_style().radius = 0.02f;
		box_style().default_extent = cgv::vec3(0.04, 0.04, 0.04);
		wire_box_style().default_extent = cgv::vec3(0.04, 0.04, 0.04);
		box_plane_style().default_extent = cgv::vec3(0.04, 0.04, 0.04);

		// 25 positions for labels = 0.05
		static double constexpr step_width = 0.10;

		//sphere grid
		uintptr_t num_center_spheres = 0;
		int col_i = 0;
		int off = width / 2;
		for (int iz = height; iz > 0; iz--) {
			for (int ix = -off; ix < width-off; ix++) {
				int id = this->add_object(
					shape,
					cgv::vec3(ix * step_width, 0.1, -iz * step_width),
					PALETTE_COLOR_MAPPING[col_i++]);
			}
		}

		set_top_toolbar_visibility(true);

		// text for palette
		set_label_text(0, "delete");
		set_label_text(1, "clear label");
	
		set_palette_changed();
	}


}