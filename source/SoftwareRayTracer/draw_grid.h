#pragma once
#include "app_data.h"
#include "camera.h"
#include "grid.h"

namespace DrawGrid
{
	struct Data
	{
		explicit Data(const char* grid_path);

		Grid::SdfGrid m_grid;
	};

	void draw_grid(AppData& app_data, const Camera& camera);
}