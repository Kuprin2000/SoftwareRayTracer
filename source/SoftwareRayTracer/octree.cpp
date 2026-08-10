#include "octree.h"
#include "bounding_box.h"
#include <cmath>
#include <fstream>

namespace
{
	static const float PI = 3.14159265359f;


	_NODISCARD double sign(double value)
	{
		return value < 0.0 ? -1.0 : 1.0;
	}

	_NODISCARD double solveCubic(double a, double b, double c)
	{
		// x^3 + a * a^2 + b * x + c = 0

		const double q = (a * a - 3.0 * b) / 9.0;
		const double r = (2.0 * a * a - 9.0 * a * b + 27.0 * c) / 54.0;

		const double r_2 = pow(r, 2.0);
		const double q_3 = pow(q, 3.0);

		if (r_2 < q_3)
		{
			const double theta = std::acos(r / sqrt(q_3));
			const double mult = -2.0 * sqrt(q);
			const double add = a / 3.0;

			double x1 = mult * cos(theta / 3.0) - add;
			double x2 = mult * cos((theta + 2.0 * PI) / 3.0) - add;
			double x3 = mult * cos((theta - 2.0 * PI) / 3.0) - add;

			x1 = (x1 < 0.0) ? FLT_MAX : x1;
			x2 = (x2 < 0.0) ? FLT_MAX : x2;
			x3 = (x3 < 0.0) ? FLT_MAX : x3;

			return std::min({ x1, x2, x3 });
		}
		else
		{
			const double A = -sign(r) * pow(abs(r) + sqrt(r_2 - q_3), 1.0 / 3.0);
			const double B = abs(A) > DBL_EPSILON ? q / A : 0.0;
			return A + B - a / 3.0;
		}
	}

	_NODISCARD double solveQuadatic(double a, double b, double c)
	{
		const double q = -0.5f * (b + sign(b) * sqrtf(b * b - 4 * a * c));
		double x1 = q / a;
		double x2 = c / q;

		x1 = (x1 < 0.0) ? FLT_MAX : x1;
		x2 = (x2 < 0.0) ? FLT_MAX : x2;

		return std::min({ x1, x2 });
	}

	_NODISCARD float solveAnalytic(double c0, double c1, double c2, double c3)
	{
		// c3 * x^3 + c2 * x^2 + c1 * x + c0 = 0

		if (fabs(c3) > 1e-2)
		{
			const double a = c2 / c3;
			const double b = c1 / c3;
			const double c = c0 / c3;
			return solveCubic(a, b, c);
		}

		if (fabs(c2) > 1e-4)
		{
			const double a = c2;
			const double b = c1;
			const double c = c0;
			return solveQuadatic(a, b, c);
		}

		if (fabs(c1) > 1e-6)
		{
			return -c0 / c1;
		}

		return FLT_MAX;
	}

	_NODISCARD float trilinearInterpolation(const float3& point, const std::array<float, 8>& values)
	{
		//s000, s001, s010, s011, s100, s101, s110, s111
		float result = 0.0f;
		result += values[0] * (1.0f - point.x) * (1.0f - point.y) * (1.0f - point.z);
		result += values[1] * (1.0f - point.x) * (1.0f - point.y) * point.z;
		result += values[2] * (1.0f - point.x) * point.y * (1.0f - point.z);
		result += values[3] * (1.0f - point.x) * point.y * point.z;
		result += values[4] * point.x * (1.0f - point.y) * (1.0f - point.z);
		result += values[5] * point.x * (1.0f - point.y) * point.z;
		result += values[6] * point.x * point.y * (1.0f - point.z);
		result += values[7] * point.x * point.y * point.z;

		return result;
	}

	_NODISCARD float3 calculateNormal(const float3& point, const std::array<float, 8>& values)
	{

		float3 result;
		{
			const float y0 = LiteMath::lerp(values[4] - values[0], values[6] - values[2], point.y);
			const float y1 = LiteMath::lerp(values[5] - values[1], values[7] - values[3], point.y);
			result.x = LiteMath::lerp(y0, y1, point.z);
		}

		{
			const float x0 = LiteMath::lerp(values[2] - values[0], values[6] - values[4], point.x);
			const float x1 = LiteMath::lerp(values[3] - values[1], values[7] - values[5], point.x);
			result.y = LiteMath::lerp(x0, x1, point.z);
		}

		{
			const float x0 = LiteMath::lerp(values[1] - values[0], values[5] - values[4], point.x);
			const float x1 = LiteMath::lerp(values[3] - values[2], values[7] - values[6], point.x);
			result.z = LiteMath::lerp(x0, x1, point.y);
		}

		return normalize(result);
	}
}

namespace Octree
{
	bool SdfOctree::contains(const float3& point) const
	{
		return std::min({ point.x, point.y, point.z }) >= -1.0f && std::max({ point.x, point.y, point.z }) <= 1.0f;
	}

	SamplingResult SdfOctree::sample(const float3& ray_start, const float3& ray_dir,
		SamplingMode mode, float3& point, float3& normal) const
	{
		float3 ray_start_copy = ray_start;
		float3 ray_dir_copy = ray_dir;

		uint8_t a = 0u;
		if (ray_dir_copy.x < 0.0f)
		{
			ray_start_copy.x = -ray_start_copy.x;
			ray_dir_copy.x = -ray_dir_copy.x;
			a |= 4;
		}
		if (ray_dir_copy.y < 0.0f)
		{
			ray_start_copy.y = -ray_start_copy.y;
			ray_dir_copy.y = -ray_dir_copy.y;
			a |= 2;
		}
		if (ray_dir_copy.z < 0.0f)
		{
			ray_start_copy.z = -ray_start_copy.z;
			ray_dir_copy.z = -ray_dir_copy.z;
			a |= 1;
		}

		const float tx0 = (m_min_corner.x - ray_start_copy.x) / ray_dir_copy.x;
		const float ty0 = (m_min_corner.y - ray_start_copy.y) / ray_dir_copy.y;
		const float tz0 = (m_min_corner.z - ray_start_copy.z) / ray_dir_copy.z;
		const float tx1 = (m_max_corner.x - ray_start_copy.x) / ray_dir_copy.x;
		const float ty1 = (m_max_corner.y - ray_start_copy.y) / ray_dir_copy.y;
		const float tz1 = (m_max_corner.z - ray_start_copy.z) / ray_dir_copy.z;

		const float max = std::max({ tx0, ty0, tz0 });
		const float min = std::min({ tx1, ty1, tz1 });
		if (max > min)
		{
			return SamplingResult::NO_INTERSECTION;
		}

		QueryContext ctx(a);
		SelectedNode selected_node = selectNode(0, tx0, ty0, tz0, tx1, ty1, tz1, ctx);

		if (selected_node.m_node_id == -1)
		{
			return SamplingResult::NO_INTERSECTION;
		}

		return calculateIntersection(selected_node, ray_start, ray_dir, mode, point, normal);
	}

	int SdfOctree::getChildOffset(size_t parent_index, size_t child_index) const
	{
		return m_nodes[parent_index].offset + child_index;
	}

	int SdfOctree::firstNode(float tx0, float ty0, float tz0, float txm, float tym, float tzm) const
	{
		uint8_t result = 0;

		if (tx0 > ty0 && tx0 > tz0)
		{
			if (tym < tx0) result |= 2;
			if (tzm < tx0) result |= 1;
		}
		else if (ty0 > tz0)
		{
			if (txm < ty0) result |= 4;
			if (tzm < ty0) result |= 1;
		}
		else
		{
			if (txm < tz0) result |= 4;
			if (tym < tz0) result |= 2;
		}

		return result;
	}

	int SdfOctree::newNode(float t1, int a, float t2, int b, float t3, int c) const
	{
		return ((t1 < t2 && t1 < t3) ? a : (t2 < t3 ? b : c));
	}

	SelectedNode SdfOctree::selectNode(int32_t idx, float tx0, float ty0, float tz0, float tx1, float ty1, float tz1, QueryContext& ctx) const
	{
		if (tx1 < 0.0f || ty1 < 0.0f || tz1 < 0.0f)
		{
			return SelectedNode();
		}

		if (!getChildOffset(idx, 0))
		{
			return SelectedNode(idx, std::max({ tx0, ty0, tz0 }), std::min({ tx1, ty1, tz1 }),
				ctx.m_min_corner, ctx.m_max_corner, m_nodes[idx].values);
		}

		const float txm = 0.5f * (tx0 + tx1);
		const float tym = 0.5f * (ty0 + ty1);
		const float tzm = 0.5f * (tz0 + tz1);
		unsigned int current_child = firstNode(tx0, ty0, tz0, txm, tym, tzm);

		SelectedNode result;
		size_t child_index;
		QueryContext context_backup = ctx;

		do {
			switch (current_child)
			{
			case 0:
				child_index = ctx.a;
				ctx.goToChild(child_index);
				result = selectNode(getChildOffset(idx, child_index), tx0, ty0, tz0, txm, tym, tzm, ctx);
				current_child = newNode(txm, 4, tym, 2, tzm, 1);
				break;
			case 1:
				child_index = 1 ^ ctx.a;
				ctx.goToChild(child_index);
				result = selectNode(getChildOffset(idx, child_index), tx0, ty0, tzm, txm, tym, tz1, ctx);
				current_child = newNode(txm, 5, tym, 3, tz1, 8);
				break;
			case 2:
				child_index = 2 ^ ctx.a;
				ctx.goToChild(child_index);
				result = selectNode(getChildOffset(idx, child_index), tx0, tym, tz0, txm, ty1, tzm, ctx);
				current_child = newNode(txm, 6, ty1, 8, tzm, 3);
				break;
			case 3:
				child_index = 3 ^ ctx.a;
				ctx.goToChild(child_index);
				result = selectNode(getChildOffset(idx, child_index), tx0, tym, tzm, txm, ty1, tz1, ctx);
				current_child = newNode(txm, 7, ty1, 8, tz1, 8);
				break;
			case 4:
				child_index = 4 ^ ctx.a;
				ctx.goToChild(child_index);
				result = selectNode(getChildOffset(idx, child_index), txm, ty0, tz0, tx1, tym, tzm, ctx);
				current_child = newNode(tx1, 8, tym, 6, tzm, 5);
				break;
			case 5:
				child_index = 5 ^ ctx.a;
				ctx.goToChild(child_index);
				result = selectNode(getChildOffset(idx, child_index), txm, ty0, tzm, tx1, tym, tz1, ctx);
				current_child = newNode(tx1, 8, tym, 7, tz1, 8);
				break;
			case 6:
				child_index = 6 ^ ctx.a;
				ctx.goToChild(child_index);
				result = selectNode(getChildOffset(idx, child_index), txm, tym, tz0, tx1, ty1, tzm, ctx);
				current_child = newNode(tx1, 8, ty1, 8, tzm, 7);
				break;
			case 7:
				child_index = 7 ^ ctx.a;
				ctx.goToChild(child_index);
				result = selectNode(getChildOffset(idx, child_index), txm, tym, tzm, tx1, ty1, tz1, ctx);
				current_child = 8;
				break;
			}

			if (result.m_node_id != -1)
			{
				return result;
			}

			ctx = context_backup;

		} while (current_child < 8u);

		return result;
	}

	SamplingResult SdfOctree::calculateIntersection(SelectedNode selected_node,
		const float3& ray_start, const float3& ray_dir, SamplingMode mode, float3& point, float3& normal) const
	{
		point = ray_start;

		BoundingBox bounding_box;
		bounding_box.setBox(selected_node.m_min_corner, selected_node.m_max_corner);
		if (!bounding_box.contains(point))
		{
			point += selected_node.m_near_intersection * ray_dir;
			selected_node.m_far_intersection -= selected_node.m_near_intersection;
			selected_node.m_near_intersection = 0.0f;
		}

		const float min_value = std::min({
			selected_node.m_values[0], selected_node.m_values[1],
			selected_node.m_values[2], selected_node.m_values[3],
			selected_node.m_values[4], selected_node.m_values[5],
			selected_node.m_values[6], selected_node.m_values[7]
			});

		const float max_value = std::max({
			fabs(selected_node.m_values[0]), fabs(selected_node.m_values[1]),
			fabs(selected_node.m_values[2]), fabs(selected_node.m_values[3]),
			fabs(selected_node.m_values[4]), fabs(selected_node.m_values[5]),
			fabs(selected_node.m_values[6]), fabs(selected_node.m_values[7])
			});

		if (min_value > 999.0f || max_value < FLT_EPSILON)
		{
			point = point + (selected_node.m_far_intersection + 1e-4f) * ray_dir;
			return SamplingResult::STEP;
		}

		if (mode == SamplingMode::SPHERE_TRACING)
		{
			return calculateIntersectionSphereTracing(selected_node, ray_start, ray_dir, point, normal);
		}

		return calculateIntersectionAnalytic(selected_node, ray_start, ray_dir, point, normal);
	}

	_NODISCARD SamplingResult SdfOctree::calculateIntersectionSphereTracing(SelectedNode selected_node, const float3& ray_start,
		const float3& ray_dir, float3& point, float3& normal) const
	{
		BoundingBox bounding_box;
		bounding_box.setBox(selected_node.m_min_corner, selected_node.m_max_corner);

		const float3 cell_size = selected_node.m_max_corner - selected_node.m_min_corner;

		float3 normalized_point = point;
		normalized_point -= selected_node.m_min_corner;
		normalized_point /= cell_size;

		float3 point_backup = point;

		float t = trilinearInterpolation(normalized_point, selected_node.m_values);

		while (fabs(t) > 1e-4)
		{
			point = point + t * ray_dir;

			if (!bounding_box.contains(point))
			{
				point = point_backup + (selected_node.m_far_intersection + 1e-4f) * ray_dir;
				return SamplingResult::STEP;
			}

			normalized_point = point;
			normalized_point -= selected_node.m_min_corner;
			normalized_point /= cell_size;

			t = trilinearInterpolation(normalized_point, selected_node.m_values);
		}

		normal = calculateNormal(normalized_point, selected_node.m_values);

		return SamplingResult::INTERSECTION;
	}

	_NODISCARD SamplingResult SdfOctree::calculateIntersectionAnalytic(SelectedNode selected_node, const float3& ray_start,
		const float3& ray_dir, float3& point, float3& normal) const
	{
		BoundingBox bounding_box;
		bounding_box.setBox(selected_node.m_min_corner, selected_node.m_max_corner);

		const float3 cell_size = selected_node.m_max_corner - selected_node.m_min_corner;

		float3 normalized_point = point;
		normalized_point -= selected_node.m_min_corner;
		normalized_point /= cell_size;

		float3 point_backup = point;

		const float s000 = selected_node.m_values[0];
		const float s001 = selected_node.m_values[1];
		const float s010 = selected_node.m_values[2];
		const float s011 = selected_node.m_values[3];
		const float s100 = selected_node.m_values[4];
		const float s101 = selected_node.m_values[5];
		const float s110 = selected_node.m_values[6];
		const float s111 = selected_node.m_values[7];

		const float a = s101 - s001;

		const float k0 = s000;
		const float k1 = s100 - s000;
		const float k2 = s010 - s000;
		const float k3 = s110 - s010 - k1;
		const float k4 = k0 - s001;
		const float k5 = k1 - a;
		const float k6 = k2 - (s011 - s001);
		const float k7 = k3 - (s111 - s011 - a);

		float3 o = normalized_point;
		float3 d3 = ray_dir;

		const float m0 = o.x * o.y;
		const float m1 = d3.x * d3.y;
		const float m2 = o.x * d3.y + o.y * d3.x;
		const float m3 = k5 * o.z - k1;
		const float m4 = k6 * o.z - k2;
		const float m5 = k7 * o.z - k3;

		// c3 * t^3 + c2 * t^2 + c1 * t + c0 = 0
		const float c0 = (k4 * o.z - k0) + o.x * m3 + o.y * m4 + m0 * m5;
		const float c1 = d3.x * m3 + d3.y * m4 + m2 * m5 + d3.z * (k4 + k5 * o.x + k6 * o.y + k7 * m0);
		const float c2 = m1 * m5 + d3.z * (k5 * d3.x + k6 * d3.y + k7 * m2);
		const float c3 = k7 * m1 * d3.z;

		float t = solveAnalytic(c0, c1, c2, c3);
		t *= cell_size.x;

		point = point + t * ray_dir;

		if (!bounding_box.contains(point))
		{
			point = point_backup + (selected_node.m_far_intersection + 1e-4f) * ray_dir;
			return SamplingResult::STEP;
		}

		normalized_point = point;
		normalized_point -= selected_node.m_min_corner;
		normalized_point /= cell_size;

		normal = calculateNormal(normalized_point, selected_node.m_values);

		return SamplingResult::INTERSECTION;
	}

	SelectedNode::SelectedNode(int node_id, float near_intersection, float far_intersection,
		const float3& min_corner, const float3& max_corner, const std::array<float, 8>& values) :
		m_node_id(node_id), m_near_intersection(near_intersection), m_far_intersection(far_intersection),
		m_min_corner(min_corner), m_max_corner(max_corner)
	{
		m_values = { values[0], values[1], values[2], values[3], values[4], values[5], values[6], values[7] };
	}

	inline Octree::SdfOctree::QueryContext::QueryContext(uint8_t a) : a(a)
	{
	}

	inline void Octree::SdfOctree::QueryContext::goToChild(size_t child_index)
	{
		const float3 center = 0.5f * (m_min_corner + m_max_corner);
		const float3 size = 0.5f * (m_max_corner - m_min_corner);

		switch (child_index)
		{
		case 0:
			m_max_corner = m_min_corner + size;
			break;
		case 1:
			m_min_corner.z = center.z;
			m_max_corner = m_min_corner + size;
			break;
		case 2:
			m_min_corner.y = center.y;
			m_max_corner = m_min_corner + size;
			break;
		case 3:
			m_min_corner.y = center.y;
			m_min_corner.z = center.z;
			m_max_corner = m_min_corner + size;
			break;
		case 4:
			m_min_corner.x = center.x;
			m_max_corner = m_min_corner + size;
			break;
		case 5:
			m_min_corner.x = center.x;
			m_min_corner.z = center.z;
			m_max_corner = m_min_corner + size;
			break;
		case 6:
			m_min_corner.x = center.x;
			m_min_corner.y = center.y;
			m_max_corner = m_min_corner + size;
			break;
		case 7:
			m_min_corner = center;
			m_max_corner = m_min_corner + size;
			break;
		}
	}

	void save_sdf_octree(const SdfOctree& scene, const std::string& path)
	{
		std::ofstream fs(path, std::ios::binary);
		size_t size = scene.m_nodes.size();
		fs.write((const char*)&size, sizeof(unsigned));
		fs.write((const char*)scene.m_nodes.data(), size * sizeof(SdfOctreeNode));
		fs.flush();
		fs.close();
	}

	void load_sdf_octree(SdfOctree& scene, const std::string& path)
	{
		std::ifstream fs(path, std::ios::binary);
		unsigned sz = 0;
		fs.read((char*)&sz, sizeof(unsigned));
		scene.m_nodes.resize(sz);
		fs.read((char*)scene.m_nodes.data(), scene.m_nodes.size() * sizeof(SdfOctreeNode));
		fs.close();
	}
}
