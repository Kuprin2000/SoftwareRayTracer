#pragma once
#include "app_data.h"
#include "camera.h"
#include "grid.h"

namespace DrawGridMinusSphere
{
	class SdfGridMinusSphere : public Grid::SdfGrid
	{
	public:
		SdfGridMinusSphere(const float3& center, float raduis);

		_NODISCARD float sample_value(const float3& point) const final;

	private:

		float3 m_sphere_center;
		float m_sphere_radius;
	};

	struct Data
	{
		Data(const char* grid_path, const float3& center, float raduis);

		SdfGridMinusSphere m_grid;
	};

	void draw_grid_minus_sphere(AppData& app_data, const Camera& camera);
}