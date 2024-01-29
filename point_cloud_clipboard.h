#pragma once

#include <memory>
#include <point_cloud.h>
#include <unordered_map>
#include <vector>
#include <exception>

#include "history.h"

namespace pct {
	/** contains a pointcloud + additional data for the clipboard. 
		This struct should contain all data required for restoring a previous state of point data in the point_cloud_cleaning_tool class
	*/
	struct point_cloud_record {

		std::unique_ptr<point_cloud> points; //used unique_ptr because point_cloud lacks a move constructor

		// these are used for restoring the state of the stored pointcloud, transformation attributes of the point_cloud class are ignored
		cgv::math::fvec<float, 3> translation;
		cgv::math::quaternion<float> rotation;
		float scale;
		// optional name for identification in user interfaces
		std::string name;
		// copy of the rollback history, also contains a copy of GPU buffers used by the rollback feature
		inactive_history stored_history;
		std::vector<cgv::render::clod_point_renderer::Point> cached_lod_points; // generated lod points for faster rendering
		cgv::math::fvec<float, 3> centroid; // cached centroid of points
		// type / how the record is used
		bool is_dynamic; // used for optimizing views, set true if this pointcloud receives many updates

		point_cloud_record();

		point_cloud_record(const point_cloud_record& other);

		point_cloud_record(point_cloud_record&& other) noexcept;

		point_cloud_record& operator=(const point_cloud_record& rhs);

		point_cloud_record& operator=(point_cloud_record&& rhs) noexcept;

		void compute_centroid();
		// returns reference to cached_lod_points, generates cached_lod_points if required
		std::vector<cgv::render::clod_point_renderer::Point>& lod_points();

		cgv::math::fmat<float, 4, 4> model_matrix();
	protected:
		void init_transform();
	};

	using clipboard_record_id = int64_t;
	class point_cloud_clipboard;

	class clipboard_event_listener {
	public:
		virtual void on_add_pointcloud(point_cloud_clipboard* clipboard, clipboard_record_id rid, point_cloud_record& record) = 0;

		virtual void on_erase_pointcloud(point_cloud_clipboard* clipboard, clipboard_record_id rid) = 0;

		virtual void on_update_pointcloud(point_cloud_clipboard* clipboard, clipboard_record_id rid) = 0;
	};

	class record_alloc_error : public std::exception {
	public:
		std::unique_ptr<point_cloud_record> moved_record;

		record_alloc_error() throw();
		record_alloc_error(std::unique_ptr<point_cloud_record>&& record) throw();
	};

	class point_cloud_clipboard {
		std::unordered_map<clipboard_record_id, std::unique_ptr<point_cloud_record>> point_clouds;

		std::vector<clipboard_event_listener*> event_listeners;

		clipboard_record_id next_key;
		unsigned max_size;
	public:
		point_cloud_clipboard();

		/// move a pointcloud record to clipboard, returns an id for access 
		clipboard_record_id move_from(std::unique_ptr<point_cloud_record>& p);

		/// get a pointer to a previously added pointcloud record, 
		point_cloud_record* get_by_id(const clipboard_record_id k);

		std::unique_ptr<point_cloud_record> remove(const clipboard_record_id k);

		void erase(const clipboard_record_id k);

		/// notify listeners that the pointcloud was changed
		void update(const clipboard_record_id k);

		void register_event_listener(clipboard_event_listener* ptr);

		void unregister_event_listener(clipboard_event_listener* ptr);

		void limit_size(unsigned max_size);

		unsigned get_max_size() const;
	protected:
		void emit_add_point_cloud_event(clipboard_record_id id);
		void emit_erase_point_cloud_event(clipboard_record_id id);
		void emit_update_point_cloud_event(clipboard_record_id id);
	};

}