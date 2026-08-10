#pragma once
#include "app_data.h"
#include "camera.h"

namespace DrawPlane
{
	struct Data
	{
		Data(float x, float y, float z, float d);

		float m_x = 0.0f;
		float m_y = 0.0f;
		float m_z = 0.0f;
		float m_d = 0.0f;
	};

	void draw_plane(AppData& app_data, const Camera& camera);
}