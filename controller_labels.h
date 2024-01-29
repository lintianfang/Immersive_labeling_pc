#include "vr_labels.h"
#include <array>
#include <vector>

namespace pct {
	
	enum controller_label_placement {
		CLP_SIDE,
		CLP_GRIP,
		CLP_TRACKPAD_UP,
		CLP_TRACKPAD_DOWN,
		CLP_TRACKPAD_RIGHT,
		CLP_TRACKPAD_LEFT,
		CLP_TRACKPAD_CENTER,
		CLP_MENU_BUTTON,
		CLP_NUM_LABEL_PLACEMENTS
	};

	/** Stores button labels + their textures for a controller.
	*	This class can store multiple label variants per button which are switched by the set_active method.
	*	
	*/
	class controller_labels : 
		public vr_labels
	{
	public:
		using active_labels_array = std::array<int, CLP_NUM_LABEL_PLACEMENTS>;
	private:
		
		CoordinateSystem label_cs;
		
		std::array<int, CLP_NUM_LABEL_PLACEMENTS> active_labels;

		//indexing:[label pos][label index]
		std::array<std::vector<int>, CLP_NUM_LABEL_PLACEMENTS> label_ids;
	
		// precomputed label positions and orientation

		std::array<vec3, CLP_NUM_LABEL_PLACEMENTS> reference_positions;
		std::array<quat, CLP_NUM_LABEL_PLACEMENTS> reference_orientations;
		std::array<float, CLP_NUM_LABEL_PLACEMENTS> reference_scale;

	public:

		controller_labels();
		
		void set_active_profile(active_labels_array& profile);
		/** sets a label for a button
		    @param location: affected button location
			@param label_idx: variant index. These indices are returned by add_variant when creating the label @see controller_labels::add_variant
		*/
		void set_active(controller_label_placement location, int label_idx);

		void set_controller_label(controller_label_placement label_placement, int variant_ix, const std::string& text, const rgba& bg_clr);
		/// call this in case reference position or orientation changed
		void update_placement(controller_label_placement location, int label_idx);

		bool check_address(controller_label_placement label_placement, int variant_ix) const;
		///adds a button label 
		///@return variant index, which is the total number of variants minus one for the given label_placement
		int add_variant(controller_label_placement label_placement, const std::string& text, const rgba& bg_clr);

		void set_label_color(controller_label_placement location, int label_idx, const rgba& bg_clr);

		/// sets the coordinate system and thereby on which controller the labels are drawn on
		inline void set_coordinate_system(CoordinateSystem cs) { this->label_cs = cs; }
		inline CoordinateSystem get_coordinate_system(CoordinateSystem cs) { return this->label_cs; }

		void draw(cgv::render::context& ctx);
	protected:
		/// special draw method for the grip button labels
		void draw_grip(cgv::render::context& ctx, int li);
	};

}