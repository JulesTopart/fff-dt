#pragma once

#include <Merlin.h>

#include <vector>
#include <string>

using namespace Merlin;

class Slicer {
public:
    // Intersect mesh with plane: dot(normal, p) = h
    // Returns flattened polylines (3D points lying on the plane) separated by a NaN sentinel.
    static std::vector<glm::vec3>
        intersectMeshPlane(Mesh_Ptr mesh, const glm::vec3& normal, float h, float eps = 1e-6f);

    // Rebuild polylines from the flattened output.
    static std::vector<std::vector<glm::vec3>>
        deflatten(const std::vector<glm::vec3>& flat);

    // Convert 3D polylines on the plane to a 2D SVG path string.
    static std::string
        toSVG(const std::vector<std::vector<glm::vec3>>& polylines,
            const glm::vec3& planeNormal,
            float strokeWidthPX = 1.5f);
};