#include "point_cloud_clipboard.h"

#include <cgv/math/ftransform.h>
#include "octree.h"

namespace pct {
	point_cloud_clipboard::point_cloud_clipboard() : max_size(-1), next_key(0)
	{
	}

	clipboard_record_id point_cloud_clipboard::move_from(std::unique_ptr<point_cloud_record>& p)
	{
		if (point_clouds.size() < max_size) {
			point_clouds[next_key] = std::move(p);
			emit_add_point_cloud_event(next_key);
			return next_key++;
		}
		throw record_alloc_error(std::move(p));
	}
	point_cloud_record* point_cloud_clipboard::get_by_id(const clipboard_record_id k)
	{
		try {
			return point_clouds.at(k).get();
		}
		catch (std::out_of_range& ex) {
			return nullptr;
		}
	}
	std::unique_ptr<point_cloud_record> point_cloud_clipboard::remove(const clipboard_record_id k)
	{
		std::unique_ptr<point_cloud_record> r(std::move(point_clouds.at(k)));
		point_clouds.erase(k);
		emit_erase_point_cloud_event(k);
		return r;
	}
	void point_cloud_clipboard::erase(const clipboard_record_id k)
	{
		emit_erase_point_cloud_event(k);
		point_clouds.erase(k);
	}
	
	void point_cloud_clipboard::update(const clipboard_record_id k)
	{
		emit_update_point_cloud_event(k);
	}

	void point_cloud_clipboard::register_event_listener(clipboard_event_listener* ptr)
	{
		event_listeners.emplace_back(ptr);
		// inform listener about already existing pointclouds
		for (auto& kv_pair : point_clouds) {
			ptr->on_add_pointcloud(this, kv_pair.first, *kv_pair.second);
		}
	}
	void point_cloud_clipboard::unregister_event_listener(clipboard_event_listener* ptr)
	{
		// copy all events ptr except the registered one and resize the event_listener
		size_t size = event_listeners.size();
		size_t i = 0;
		while (i < size && event_listeners[i] != ptr) ++i;
		if (i < size) {
			event_listeners[i] = event_listeners[size - 1];
			event_listeners.resize(size - 1);
		}
	}

	void point_cloud_clipboard::emit_add_point_cloud_event(clipboard_record_id id)
	{
		auto* record = get_by_id(id);
		assert(record != nullptr);
		for (pct::clipboard_event_listener* el : event_listeners) {
			el->on_add_pointcloud(this, id, *record);
		}
	}

	void point_cloud_clipboard::emit_erase_point_cloud_event(clipboard_record_id id)
	{
		auto* record = get_by_id(id);
		assert(record != nullptr);
		for (pct::clipboard_event_listener* el : event_listeners) {
			el->on_erase_pointcloud(this, id);
		}
	}

	void point_cloud_clipboard::emit_update_point_cloud_event(clipboard_record_id id)
	{
		auto* record = get_by_id(id);
		assert(record != nullptr);
		for (pct::clipboard_event_listener* el : event_listeners) {
			el->on_update_pointcloud(this, id);
		}
	}

	void point_cloud_clipboard::limit_size(unsigned max_size)
	{
		this->max_size = max_size;
	}

	unsigned point_cloud_clipboard::get_max_size() const
	{
		return this->max_size;
	}
	
	point_cloud_record::point_cloud_record() : scale(0), is_dynamic(false)
	{
		init_transform();
	}
	point_cloud_record::point_cloud_record(const point_cloud_record& other)
	{
		*this = other;
	}
	point_cloud_record::point_cloud_record(point_cloud_record&& other) noexcept
	{
		*this = std::move(other);
	}

	point_cloud_record& point_cloud_record::operator=(const point_cloud_record& rhs)
	{
		translation = rhs.translation;
		rotation = rhs.rotation;
		scale = rhs.scale;
		name = rhs.name;
		stored_history = rhs.stored_history;
		points = std::make_unique<point_cloud>(*rhs.points);
		cached_lod_points = rhs.cached_lod_points;
		return *this;
	}
	point_cloud_record& point_cloud_record::operator=(point_cloud_record&& rhs) noexcept
	{
		translation = rhs.translation;
		rotation = rhs.rotation;
		scale = rhs.scale;
		name = std::move(rhs.name);
		stored_history = std::move(rhs.stored_history);
		points = std::move(rhs.points);
		cached_lod_points.swap(rhs.cached_lod_points);	
		return *this;
	}
	void point_cloud_record::compute_centroid()
	{
		if (points == nullptr) {
			return;
		}
		cgv::math::fvec<float, 3> d_centroid(0);
		int nr_points = points->get_nr_points();
		for (int i = 0; i < nr_points; ++i) {
			d_centroid += points->pnt(i);
		}
		d_centroid /= (double)nr_points;
		centroid = d_centroid;
	}
	std::vector<cgv::render::clod_point_renderer::Point>& point_cloud_record::lod_points()
	{
		if (cached_lod_points.size() != points->get_nr_points()) {
			cgv::pointcloud::octree_lod_generator<cgv::render::clod_point_renderer::Point> lod_generator;

			// generate input for lod_generator
			std::vector<cgv::render::clod_point_renderer::Point> lod_points(points->get_nr_points());
			for (int i = 0; i < points->get_nr_points(); ++i) {
				cgv::render::clod_point_renderer::Point p;
				p.color() = points->clr(i);
				p.position() = points->pnt(i);
				lod_points[i] = p;
			}
			
			cached_lod_points = lod_generator.generate_lods(lod_points);
		}
		return cached_lod_points;
	}

	cgv::math::fmat<float, 4, 4> point_cloud_record::model_matrix()
	{
		cgv::math::fmat<float, 4, 4> rotation = cgv::math::fmat<float, 4, 4>(3, 3, this->rotation.get_matrix().begin());
		cgv::math::fmat<float, 4, 4> model = cgv::math::translate4(translation) * rotation * cgv::math::scale4(scale, scale, scale);
		return model;
	}
	void point_cloud_record::init_transform()
	{
		translation = cgv::math::fvec<float,3>(0.f);
		rotation = cgv::math::quaternion<float>(1.f, 0.f, 0.f, 0.f);
		scale = 1.f;
	}
	record_alloc_error::record_alloc_error() throw()
	{
	}
	record_alloc_error::record_alloc_error(std::unique_ptr<point_cloud_record>&& record) throw() : moved_record(std::move(record))
	{
	}
}