#pragma once
#include "palette.h"
#include <vector>


namespace vrui {

	class label_palette : public palette {

	public:
		label_palette();

		/**
			builds a grid of spheres with the ids 0 to width*height-1
			@param PALETTE_COLOR_MAPPING : sphere colors
		*/
		void build(const std::vector<rgba>& PALETTE_COLOR_MAPPING, int width = 5, int height = 5, const PaletteObject shape = PaletteObject::PO_BOX_PLANE);
	};

}