#pragma once
#include <string>
#include "app_data.h"
#include "camera.h"
#include "mesh.h"
#include "lbvh.h"

namespace MeshToGrid
{
	struct Data
	{
		Data(int size, const char* mesh_path, const char* grid_path);

		int m_size = 0;
		cmesh4::SimpleMesh m_mesh;
		LinearBVH m_bvh;
		std::string m_grid_path;

	private:
		void getMeshExtent(float4& min_corner, float4& max_corner) const;

		void resizeMesh();
	};

	void mesh_to_grid(AppData& app_data);
}