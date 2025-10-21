// Slicer.cpp
#include "slicer.h"

#include <glm/gtx/norm.hpp>
#include <unordered_map>
#include <limits>
#include <cmath>
#include <cstdint>
#include <sstream>
#include <algorithm>

namespace {

    // ---- Small utilities ----
    struct Segment { glm::vec3 p0, p1; };

    inline float sgnDist(const glm::vec3& p, const glm::vec3& n, float h) {
        return glm::dot(n, p) - h;
    }

    // Return true and set 'out' if edge [a,b] intersects the plane within tolerance.
    inline bool edgeIntersect(const glm::vec3& a, float da,
        const glm::vec3& b, float db,
        glm::vec3& out, float eps = 1e-7f)
    {
        const bool crosses = (da > eps && db < -eps) || (da < -eps && db > eps);
        const bool touchesEdge = std::fabs(da) <= eps && std::fabs(db) <= eps; // whole edge coplanar
        if (touchesEdge) return false; // coplanar tri ignored here
        if (!crosses) {
            // Exactly one endpoint on plane => include it once
            if (std::fabs(da) <= eps && std::fabs(db) > eps) { out = a; return true; }
            if (std::fabs(db) <= eps && std::fabs(da) > eps) { out = b; return true; }
            return false;
        }
        const float t = da / (da - db); // robust param
        out = a + t * (b - a);
        return true;
    }

    // Quantized key for snapping near-equal points (to stitch segments)
    struct Key { long long x, y, z; bool operator==(const Key& o) const { return x == o.x && y == o.y && z == o.z; } };
    struct KeyHash {
        std::size_t operator()(const Key& k) const {
            auto mix = [](uint64_t z) {
                z += 0x9e3779b97f4a7c15ULL;
                z = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9ULL;
                z = (z ^ (z >> 27)) * 0x94d049bb133111ebULL;
                return z ^ (z >> 31);
                };
            uint64_t h = mix((uint64_t)k.x);
            h ^= mix((uint64_t)k.y + 0x12345678ULL);
            h ^= mix((uint64_t)k.z + 0x9abcdef0ULL);
            return (std::size_t)h;
        }
    };
    inline Key quantize(const glm::vec3& p, float invTol) {
        return {
            (long long)std::llround(p.x * invTol),
            (long long)std::llround(p.y * invTol),
            (long long)std::llround(p.z * invTol)
        };
    }

    // Build a right-handed ONB (u, v, n) for the plane
    inline void planeBasis(const glm::vec3& n, glm::vec3& u, glm::vec3& v) {
        glm::vec3 nn = glm::normalize(n);
        glm::vec3 helper = (std::fabs(nn.z) < 0.9f) ? glm::vec3(0, 0, 1) : glm::vec3(0, 1, 0);
        u = glm::normalize(glm::cross(helper, nn));
        v = glm::cross(nn, u);
    }












    static std::vector<Segment> sliceMesh(const std::vector<Vertex>& vertices, const std::vector<unsigned int>* optIndices, const glm::vec3& normal, float h, float eps) {
        std::vector<Segment> segs;
        if (vertices.size() < 3) return segs;

        const size_t triCount = (optIndices && !optIndices->empty())
            ? optIndices->size() / 3
            : vertices.size() / 3;

        segs.reserve(triCount / 2);

        for (size_t t = 0; t < triCount; ++t) {
            uint32_t i0, i1, i2;
            if (optIndices && !optIndices->empty()) {
                i0 = (*optIndices)[t * 3 + 0];
                i1 = (*optIndices)[t * 3 + 1];
                i2 = (*optIndices)[t * 3 + 2];
            }
            else {
                i0 = static_cast<uint32_t>(t * 3 + 0);
                i1 = static_cast<uint32_t>(t * 3 + 1);
                i2 = static_cast<uint32_t>(t * 3 + 2);
            }

            const glm::vec3 P0 = vertices[i0].position;
            const glm::vec3 P1 = vertices[i1].position;
            const glm::vec3 P2 = vertices[i2].position;

            const float d0 = sgnDist(P0, normal, h);
            const float d1 = sgnDist(P1, normal, h);
            const float d2 = sgnDist(P2, normal, h);

            std::vector<glm::vec3> ip; ip.reserve(3);
            glm::vec3 q;

            if (edgeIntersect(P0, d0, P1, d1, q, eps)) ip.push_back(q);
            if (edgeIntersect(P1, d1, P2, d2, q, eps)) ip.push_back(q);
            if (edgeIntersect(P2, d2, P0, d0, q, eps)) ip.push_back(q);

            if (ip.size() == 2) {
                segs.push_back({ ip[0], ip[1] });
            }
            else if (ip.size() > 2) {
                // Degenerate: keep farthest pair
                float best = -1.0f; glm::vec3 a, b;
                for (size_t i = 0; i < ip.size(); ++i)
                    for (size_t j = i + 1; j < ip.size(); ++j) {
                        float d = glm::length2(ip[i] - ip[j]);
                        if (d > best) { best = d; a = ip[i]; b = ip[j]; }
                    }
                if (best > 0.0f) segs.push_back({ a,b });
            }
            // Coplanar triangles ignored (typical for slicing shells).
        }
        return segs;
    }


    // --- Estimate a robust connection/snap tolerance from the segment endpoints ---
    // Uses 25th percentile of nearest-neighbor distances between all endpoints,
    // clamped to a small fraction of the overall scale.
    inline float estimateSnapTolerance(const std::vector<Segment>& segs)
    {
        if (segs.empty()) return 0.0f;

        std::vector<glm::vec3> pts; pts.reserve(segs.size() * 2);
        glm::vec3 bmin(std::numeric_limits<float>::infinity());
        glm::vec3 bmax(-std::numeric_limits<float>::infinity());
        for (const auto& s : segs) {
            pts.push_back(s.p0); pts.push_back(s.p1);
            bmin = glm::min(bmin, s.p0); bmin = glm::min(bmin, s.p1);
            bmax = glm::max(bmax, s.p0); bmax = glm::max(bmax, s.p1);
        }
        const float diag = glm::length(bmax - bmin);
        const float eps = std::max(1e-7f, 1e-6f * diag);

        std::vector<float> nn; nn.reserve(pts.size());
        for (size_t i = 0; i < pts.size(); ++i) {
            float best = std::numeric_limits<float>::infinity();
            for (size_t j = 0; j < pts.size(); ++j) {
                if (i == j) continue;
                const glm::vec2 di(pts[i].x - pts[j].x, pts[i].y - pts[j].y); // 2D is enough for SVG
                const float d = std::sqrt(di.x * di.x + di.y * di.y);
                if (d < best) best = d;
            }
            nn.push_back(best);
        }

        // 25th percentile (robust “typical gap”)
        const size_t k = nn.size() / 4;
        std::nth_element(nn.begin(), nn.begin() + k, nn.end());
        float p25 = nn[k];

        // Heuristic clamp: between eps and 2% of bbox diag; scale by 2×P25
        float tol = std::max(eps, std::min(p25 * 2.0f, 0.02f * diag));
        return tol;
    }

    // Store the last snap tolerance so toSVG can use the SAME threshold for deciding closure
    static thread_local float g_lastSnapTol = 0.0f;


    // Stitch segments into polylines
    static std::vector<std::vector<glm::vec3>>
        stitchPolylines(const std::vector<Segment>& segs, float tol)
    {
        if (segs.empty()) return {};

        // Snap endpoints -> nodes
        std::unordered_map<Key, int, KeyHash> nodeOf;
        std::vector<glm::vec3> nodes; nodes.reserve(segs.size() * 2);
        auto getNode = [&](const glm::vec3& p)->int {
            Key k = quantize(p, 1.0f / tol);
            auto it = nodeOf.find(k);
            if (it != nodeOf.end()) return it->second;
            int id = (int)nodes.size();
            nodes.push_back(p);
            nodeOf.emplace(k, id);
            return id;
            };

        // Build edges and incidence
        std::vector<std::pair<int, int>> edges; edges.reserve(segs.size());
        for (const auto& s : segs) {
            int a = getNode(s.p0);
            int b = getNode(s.p1);
            if (a == b) continue;
            edges.emplace_back(a, b);
        }

        std::vector<std::vector<int>> inc(nodes.size());
        for (int ei = 0; ei < (int)edges.size(); ++ei) {
            inc[edges[ei].first].push_back(ei);
            inc[edges[ei].second].push_back(ei);
        }

        auto other = [&](int ei, int u)->int {
            return edges[ei].first == u ? edges[ei].second : edges[ei].first;
            };

        std::vector<char> used(edges.size(), 0);
        std::vector<std::vector<glm::vec3>> polylines;

        auto walk_from = [&](int start) {
            // seed with first unused edge
            int seed = -1; for (int ei : inc[start]) if (!used[ei]) { seed = ei; break; }
            if (seed == -1) return;
            used[seed] = 1;
            int cur = start;
            int nxt = other(seed, cur);
            std::vector<glm::vec3> poly; poly.reserve(16);
            poly.push_back(nodes[cur]);
            poly.push_back(nodes[nxt]);
            int prev = cur; cur = nxt;

            while (true) {
                int take = -1;
                for (int ei : inc[cur]) {
                    if (used[ei]) continue;
                    int cand = other(ei, cur);
                    if (cand == prev) continue;
                    take = ei; break;
                }
                if (take == -1) {
                    for (int ei : inc[cur]) if (!used[ei]) { take = ei; break; }
                    if (take == -1) break;
                }
                used[take] = 1;
                int nxt2 = other(take, cur);
                poly.push_back(nodes[nxt2]);
                prev = cur; cur = nxt2;
            }
            polylines.push_back(std::move(poly));
            };

        // Open chains (degree 1) first
        for (int n = 0; n < (int)inc.size(); ++n) {
            if ((int)inc[n].size() == 1) {
                bool has = false; for (int ei : inc[n]) if (!used[ei]) { has = true; break; }
                if (has) walk_from(n);
            }
        }
        // Remaining cycles
        for (int n = 0; n < (int)inc.size(); ++n) {
            bool has = false; for (int ei : inc[n]) if (!used[ei]) { has = true; break; }
            if (has) walk_from(n);
        }
        return polylines;
    }

} // namespace

// --------- Public API ---------

// --- replace Slicer::intersectMeshPlane with this version (normalizes plane + handles empty indices) ---
std::vector<glm::vec3>
Slicer::intersectMeshPlane(Mesh_Ptr mesh, const glm::vec3& normal, float h, float eps)
{
    const auto& vertices = mesh->getVertices();

    // Normalize plane so h is a distance even if normal isn't unit.
    const float nlen = glm::length(normal);
    if (nlen == 0.0f) return {};
    const glm::vec3 nn = normal / nlen;
    const float     hh = h / nlen;

    // Build/convert indices if present (may be empty for procedural meshes)
    const auto& glIdx = mesh->getIndices();          // std::vector<GLuint>
    std::vector<unsigned int> idx; idx.reserve(glIdx.size());
    for (auto id : glIdx) idx.push_back(static_cast<unsigned int>(id));

    // Scale eps to model size
    glm::vec3 bmin(std::numeric_limits<float>::infinity());
    glm::vec3 bmax(-std::numeric_limits<float>::infinity());
    for (const auto& v : vertices) {
        bmin = glm::min(bmin, v.position);
        bmax = glm::max(bmax, v.position);
    }
    const float diag = glm::length(bmax - bmin);
    const float epsScaled = std::max(eps, diag * 1e-6f);

    // Slice (use indices if present; else vertex triplets)
    const std::vector<unsigned int>* opt = idx.empty() ? nullptr : &idx;
    auto segs = sliceMesh(vertices, opt, nn, hh, epsScaled);

    // Stitch
    const float tol = estimateSnapTolerance(segs);
    g_lastSnapTol = tol;
    auto polys = stitchPolylines(segs, tol);

    // Flatten with NaN sentinels
    std::vector<glm::vec3> out;
    const glm::vec3 sentinel(std::numeric_limits<float>::quiet_NaN());
    size_t count = 0; for (auto& pl : polys) count += pl.size();
    out.reserve(count + polys.size());
    for (size_t i = 0; i < polys.size(); ++i) {
        if (i) out.push_back(sentinel);
        out.insert(out.end(), polys[i].begin(), polys[i].end());
    }
    return out;
}


std::vector<std::vector<glm::vec3>>
Slicer::deflatten(const std::vector<glm::vec3>& flat)
{
    std::vector<std::vector<glm::vec3>> polys(1);
    auto isNaN = [](const glm::vec3& p) {
        return std::isnan(p.x) || std::isnan(p.y) || std::isnan(p.z);
        };
    for (const auto& p : flat) {
        if (isNaN(p)) {
            if (!polys.back().empty()) polys.emplace_back();
        }
        else {
            polys.back().push_back(p);
        }
    }
    if (!polys.empty() && polys.back().empty()) polys.pop_back();
    return polys;
}

std::string
Slicer::toSVG(const std::vector<std::vector<glm::vec3>>& polylines,
    const glm::vec3& planeNormal,
    float strokeWidthPX)
{
    if (polylines.empty()) return R"(<svg xmlns="http://www.w3.org/2000/svg"/>)";

    glm::vec3 u, v; planeBasis(planeNormal, u, v);

    // Project & bounds
    std::vector<std::vector<glm::vec2>> proj(polylines.size());
    glm::vec2 minv(std::numeric_limits<float>::infinity());
    glm::vec2 maxv(-std::numeric_limits<float>::infinity());
    for (size_t i = 0; i < polylines.size(); ++i) {
        for (const auto& p3 : polylines[i]) {
            glm::vec2 p2(glm::dot(u, p3), glm::dot(v, p3));
            proj[i].push_back(p2);
            minv = glm::min(minv, p2);
            maxv = glm::max(maxv, p2);
        }
    }

    const float spanX = maxv.x - minv.x;
    const float spanY = maxv.y - minv.y;
    const float span = std::max(1e-6f, std::max(spanX, spanY));
    const float margin = 0.05f * span;

    std::ostringstream ss;
    ss << R"(<svg xmlns="http://www.w3.org/2000/svg" viewBox=")"
        << (minv.x - margin) << " " << (-(maxv.y + margin)) << " "
        << (spanX + 2 * margin) << " " << (spanY + 2 * margin)
        << R"(" fill="none" stroke="black" stroke-width=")" << strokeWidthPX << R"(">)";

    // SVG Y grows down → flip Y
    for (const auto& pl : proj) {
        if (pl.empty()) continue;
        ss << "<path d=\"M " << pl[0].x << " " << (-pl[0].y);
        for (size_t i = 1; i < pl.size(); ++i) ss << " L " << pl[i].x << " " << (-pl[i].y);

        const float closeTol = std::max(1e-7f, g_lastSnapTol);
        const bool closed = glm::length(pl.front() - pl.back()) <= closeTol;

        if (closed && pl.size() >= 2 && glm::length(pl.front() - pl.back()) <= closeTol) {
            // emit path without the duplicated last point
            ss << "<path d=\"M " << pl[0].x << " " << (-pl[0].y);
            for (size_t i = 1; i + 1 < pl.size(); ++i) ss << " L " << pl[i].x << " " << (-pl[i].y);
            ss << " Z\"/>\n";
            continue;
        }

        if (closed) ss << " Z";
        ss << "\"/>\n";
    }
    ss << "</svg>";
    return ss.str();
}
