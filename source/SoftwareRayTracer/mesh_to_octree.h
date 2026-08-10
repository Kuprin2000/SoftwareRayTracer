#pragma once
#include <string>
#include "app_data.h"
#include "camera.h"
#include "mesh.h"
#include "lbvh.h"

namespace MeshToOctree
{
	struct Data
	{
		Data(int d, const char* mesh_path, const char* octree_path);

		int m_d = 0;
		cmesh4::SimpleMesh m_mesh;
		LinearBVH m_bvh;
		std::string m_octree_path;

	private:
		void getMeshExtent(float4& min_corner, float4& max_corner) const;

		void resizeMesh();
	};

	void mesh_to_octree(AppData& app_data);
}