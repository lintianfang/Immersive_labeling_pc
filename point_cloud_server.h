#pragma once

#include <memory>
#include <cgv/base/node.h>
#include <cgv/render/drawable.h>
#include <cgv/render/context.h>
#include <cgv/render/render_types.h>
#include <point_cloud.h>
#include "chunks.h"
#include <octree.h>
#include <cgv_gl/clod_point_renderer.h>

#include "point_labels.h"
#include "label_shader_manager.h"
#include "clod_reduction_state.h"
#include "statistics_collection.h"
#include "history.h"
#include "selection_tool.h"
#include "point_cloud_registration_tool.h"
#include "pointcloud_transformation_state.h"
#include "gpu_stopwatch.h"
#include "history.h"

namespace pct {

	using LODPoint = cgv::render::clod_point_renderer::Point;

	/// LODPoint with an additional integer field used to keep reference the point's original array position
	struct indexed_point : public cgv::render::clod_point_renderer::Point {
		size_t index;
	};
	
	/// setting for the lod generator
	enum class LoDMode {
		OCTREE = 1,	//good quality lods
		RANDOM_POISSON = 2, //lower quality but generation is way faster
		INVALID = -1
	};

	struct point_cloud_attributes {
		int label_attribute_id = -1;
		int normal_attribute_id = -1;
	};

	/// setting regarding the setup of accelleration data structures and level of detail generation
	struct point_cloud_preparation_settings {
		LoDMode lod_mode;
		bool allow_point_deduplication; // only relevant for LoDMode::OCTREE
		bool auto_chunk_cube_size; // computes chunk sizes for point cloud if enabled
		int auto_chunk_max_num_chunks; ///max number of the whole chunks
		float chunk_cube_size;    //chunk size

		point_cloud_preparation_settings();
	};
	
	struct point_interaction_settings {
		bool use_chunks;
		bool enable_history;

		point_interaction_settings();
	};

	struct box_uniform : public cgv::render::render_types {
		vec4 aabb_min_p;
		vec4 aabb_max_p;
		vec4 translation;
		quat rotation;
	};

	struct labeling_constraint : public cgv::render::render_types {
		box_uniform label_constraint_box;
		GLint label_constraint_bits;
		GLboolean label_constraint_solid_inverted;
	};

	/*	
		class responsibilities:
		access to point labels
		store and provide active pointcloud
		prepare pointcloud for continous lod rendering
		provide data for rendering with level of detail
		swap active pointcloud with other pointclouds
		encapsulate interactions with the active pointcloud that depend on compute shaders

		Some notes about the point labels used by this class:
		
		Point labels are split into two section,
		|persistent Part  |runtime part     |
		|MSB----16 bit----|----16 bit----LSB|
		
		The persistent part is loaded from and stored to files.
		The runtime part (label group) should be used by the application to implement function like selection or point deletion.
		Methods from point_labels.h can be used to create and decode labels
	*/
	class point_cloud_server :
		public cgv::base::node,
		public cgv::render::render_types
	{
		
		cgv::render::context* ctx_ptr;

		std::unordered_map<unsigned, chunks<LODPoint>*> point_cloud_ptrs;
		unsigned last_id;

		chunks<LODPoint> chunked_points;
		float point_cloud_scale;
		vec3 point_cloud_position;
		vec3 point_cloud_rotation;

		bool chunks_disabled; //enables alternative code paths which ignore the chunk accelleration data structure
		float chunk_cube_size;

		//shader programs

		cgv::render::shader_program point_raycast_prog;
		cgv::render::shader_program selection_relabel_prog;

		//cgv::pointcloud::octree_lod_generator<pct::indexed_point> lod_generator;

		//attribute ids

		int label_attribute_id = -1;
		int normal_attribute_id = -1;

		// buffers
		GLuint gp_results_buffer; //general purpose buffer for results from various compute shaders


		//queries
		gpu_stopwatch labeling_stopwatch;

		bool gpu_timers_enabled;
		bool cpu_timers_enabled;
		
		// stores the old labels and indices of points affected by the last labeling action
		std::unique_ptr<history> history_ptr;

		//configuration / settings
		point_interaction_settings interaction_settings;

		labeling_constraint active_constraint;
		GLuint box_constraint_buffer;

	public:
		//gl buffer layout information
		static const GLuint point_labels_layout_pos = 6;
		static const GLuint point_normals_layout_pos = 5;

		point_cloud_server();

		~point_cloud_server();

		void manage_singelton(cgv::render::context& ctx, const std::string& name, int& ref_count, int ref_count_change);
		
		// takes ownership of the history object pointed by ptr, caller get ownership of the previous used history object
		void swap_history(std::unique_ptr<history>& ptr);

		inline chunks<LODPoint>& ref_chunks() {
			return chunked_points;
		}
		
		inline float& ref_point_cloud_scale() {
			return point_cloud_scale;
		}

		inline vec3& ref_point_cloud_position() {
			return point_cloud_position;
		}

		inline vec3& ref_point_cloud_rotation() {
			return point_cloud_rotation;
		}

		inline point_interaction_settings& ref_interaction_settings() {
			return interaction_settings;
		}
		/// computes a model matrix
		mat4 get_point_cloud_model_transform();
		
		pointcloud_transformation_state get_transformation_state();
		/// sets rotation translation and scale of the active point cloud
		void restore_transformation_state(const pointcloud_transformation_state& state);

		/// creates chunk structure filled with points from source_point_cloud, copies or generates lod attribute for every point
		void use_point_cloud(point_cloud& source_point_cloud, const point_cloud_preparation_settings& settings);
		/// find chunk size for given settings
		float compute_chunk_size(point_cloud& source_pc, const point_cloud_preparation_settings& settings);

		void rechunk_point_cloud(float chunk_size);

		// builds a lookup table that can be used to visualize lod values
		std::vector<cgv::render::render_types::rgba> make_lod_to_color_lut(const float lod_min_level_hue = 230.0 / 360.0, const float lod_max_level_hue = 1.0);

		// interaction methods

		/// interprets point cloud points as spheres, returns true if there is a intersection with the given ray
		/// @param in model_matrix: model matrix for the pointcloud
		/// @param in dir, origin: ray to test defined as dir*t+origin
		/// @param out res_t, res_p: intersection point p and t from the equation t*dir+origin=res_p
		bool raycast_points(const vec3& dir, const vec3& origin, const float radius, float& res_t, vec3& res_p);
		
		/// returns number of points found for each level of detail
		std::vector<GLuint> count_points_in_sphere(const vec3& position, const float radius, const bool use_chunks_for_labeling=true);

		/// the following 3 methods write to the point history if enable_label_history == true

		/// assigns labels to points inside the sphere defined by position and radius, points is expected to be a valid opengl buffer id containing the chunked_points
		/// affects only labels where point_label & point_group_mask != 0 and point_label & exclude_point_group_mask == 0
		/// the make_label function creates the expected label format
		void label_points_in_sphere(const GLint label, const GLint point_group_mask, const GLint exclude_point_group_mask, vec3 position, const float radius, const point_label_operation op = point_label_operation::REPLACE);

		/// assigns labels to points inside the defined box
		/// affects only labels where label & point_group_mask != 0 and label & exclude_point_group_mask == 0
		/// the make_label function creates the expected label format
		void label_points_in_box(const GLint label, const GLint point_group_mask, const GLint exclude_point_group_mask, const box3& box, const vec3& position, const quat& box_orientation, const point_label_operation operation = point_label_operation::REPLACE);

		/// assigns labels to points on one side of clipping plane
		/// affects only labels where label & point_group_mask != 0 and label & exclude_point_group_mask == 0
		/// the make_label function creates the expected label format
		void label_points_by_clipping(const GLint label, const GLint point_group_mask, const GLint exclude_point_group_mask, vec3 position, mat3 plane_ori, const bool invert = false, const point_label_operation operation = point_label_operation::REPLACE);

		/// applies label operation on all points
		/// affects only labels where label & point_group_mask != 0 and label & exclude_point_group_mask == 0
		/// the make_label function creates the expected label format
		void label_all_points(const GLint label, const GLint point_group_mask, const GLint exclude_point_group_mask, const point_label_operation operation = point_label_operation::REPLACE);

		// unused method, may contain bugs
		/// processes all point of the point cloud, assigns new labels to points where point_label&point_group_mask == expected_label
		/// this enables for example deletion of marked points (new_label = point_label::DELETED)
		void label_selected_points(const GLint new_label, const GLint expected_label, const int32_t point_group_mask, const GLuint points, const unsigned num_points);

		/// The label_* methods treat labels of points within the defined constraint box as if or_label was added to their label.
		/// This is intended to be used in combination with the point_group_mask and or exclude_point_group_mask parameters of the label_* methods.
		/// For example setting "or_label = exclude_point_group_mask" and "box_inverted = true" restricts labeling to the area within the box
		/// @param box: box dimensions defined in form of an aabb
		/// @param translation: box translation
		/// @param rotation: box rotation
		void set_constraint_box(GLint or_label,box3 box, vec3 translation,quat rotation, bool box_inverted =false);

		void clear_constraints();

		bool poll_labeling_time(GLuint64& time);

		/// merges "source" transformed by "model_matrix" into the currently active point cloud 
		void fuse_point_cloud(point_cloud& source, mat4& model_matrix, bool copy_labels = false);

		bool init(cgv::render::context& ctx);
		void clear(cgv::render::context& ctx);

		void configure_timers(bool gpu_timers_enabled, bool cpu_timers_enabled);
	};

	point_cloud_server& ref_point_cloud_server(cgv::render::context& ctx, int ref_count_change = 0);
}

