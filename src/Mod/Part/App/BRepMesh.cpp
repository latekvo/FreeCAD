// SPDX-License-Identifier: LGPL-2.1-or-later

/***************************************************************************
 *   Copyright (c) 2024 Werner Mayer <wmayer[at]users.sourceforge.net>     *
 *                                                                         *
 *   This file is part of FreeCAD.                                         *
 *                                                                         *
 *   FreeCAD is free software: you can redistribute it and/or modify it    *
 *   under the terms of the GNU Lesser General Public License as           *
 *   published by the Free Software Foundation, either version 2.1 of the  *
 *   License, or (at your option) any later version.                       *
 *                                                                         *
 *   FreeCAD is distributed in the hope that it will be useful, but        *
 *   WITHOUT ANY WARRANTY; without even the implied warranty of            *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU      *
 *   Lesser General Public License for more details.                       *
 *                                                                         *
 *   You should have received a copy of the GNU Lesser General Public      *
 *   License along with FreeCAD. If not, see                               *
 *   <https://www.gnu.org/licenses/>.                                      *
 *                                                                         *
 **************************************************************************/


#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <unordered_map>
#include <vector>
#include <Precision.hxx>


#include "BRepMesh.h"
#include <Base/Tools.h>

using namespace Part;

namespace
{
struct MeshVertex
{
    Base::Vector3d p;
    std::size_t i = 0;

    explicit MeshVertex(const Base::Vector3d& p)
        : p(p)
    {}

    Base::Vector3d toPoint() const
    {
        return p;
    }

    bool operator<(const MeshVertex& v) const
    {
        if (p.x != v.p.x) {
            return p.x < v.p.x;
        }
        if (p.y != v.p.y) {
            return p.y < v.p.y;
        }
        if (p.z != v.p.z) {
            return p.z < v.p.z;
        }

        // points are equal
        return false;
    }
};

class MergeVertex
{
public:
    using Facet = BRepMesh::Facet;

    MergeVertex(std::vector<Base::Vector3d> points, std::vector<Facet> faces, double tolerance)
        : points {std::move(points)}
        , faces {std::move(faces)}
        , tolerance {tolerance}
    {
        setDefaultMap();
        check();
    }

    bool hasDuplicatedPoints() const
    {
        return duplicatedPoints > 0;
    }

    void mergeDuplicatedPoints()
    {
        if (!hasDuplicatedPoints()) {
            return;
        }

        redirectPointIndex();
        auto degreeMap = getPointDegrees();
        decrementPointIndex(degreeMap);
        removeUnusedPoints(degreeMap);
        reset();
    }

    std::vector<Base::Vector3d> getPoints() const
    {
        return points;
    }

    std::vector<Facet> getFacets() const
    {
        return faces;
    }

private:
    static constexpr std::size_t noIndex = std::numeric_limits<std::size_t>::max();
    static constexpr double cellsPerTolerance = 64.0;

    void setDefaultMap()
    {
        // by default map point index to itself
        mapPointIndex.resize(points.size());
        std::generate(mapPointIndex.begin(), mapPointIndex.end(), Base::iotaGen<std::size_t>(0));
    }

    void reset()
    {
        mapPointIndex.clear();
        duplicatedPoints = 0;
    }

    // Cell of the uniform lookup grid a point falls into. The grid is deliberately much coarser
    // than the tolerance: the coarser it is, the fewer points sit close enough to a cell border
    // for their neighbouring cells to have to be probed at all.
    struct Cell
    {
        std::int64_t x = 0;
        std::int64_t y = 0;
        std::int64_t z = 0;

        bool operator==(const Cell& other) const
        {
            return x == other.x && y == other.y && z == other.z;
        }
    };

    struct CellHash
    {
        std::size_t operator()(const Cell& cell) const
        {
            std::size_t seed = 0;
            for (std::int64_t value : {cell.x, cell.y, cell.z}) {
                seed ^= std::hash<std::int64_t> {}(value) + 0x9E3779B9U + (seed << 6U) + (seed >> 2U);
            }
            return seed;
        }
    };

    bool sameWithinTolerance(const Base::Vector3d& lhs, const Base::Vector3d& rhs) const
    {
        return std::fabs(lhs.x - rhs.x) < tolerance && std::fabs(lhs.y - rhs.y) < tolerance
            && std::fabs(lhs.z - rhs.z) < tolerance;
    }

    void check()
    {
        // Being within the tolerance of each other is not a transitive relation, so it cannot be
        // expressed as an ordering. The tolerant "less than" this used to sort by is therefore
        // not a strict weak ordering -- with a tolerance of 1, 0.0 ~ 0.6 and 0.6 ~ 1.2 yet
        // 0.0 < 1.2 -- which makes the std::sort call undefined behaviour, and it also parks
        // unrelated points between genuine duplicates so the adjacency scan never merged them.
        // Look the candidates up in a uniform grid instead.
        const double spacing = tolerance * cellsPerTolerance;
        if (!(spacing > 0.0)) {
            return;
        }

        std::unordered_map<Cell, std::size_t, CellHash> firstInCell;
        firstInCell.reserve(points.size());
        std::vector<std::size_t> nextInCell(points.size(), noIndex);

        for (std::size_t index = 0; index < points.size(); ++index) {
            const Base::Vector3d& point = points[index];
            const double coord[3] {point.x, point.y, point.z};
            if (!std::isfinite(coord[0]) || !std::isfinite(coord[1]) || !std::isfinite(coord[2])) {
                continue;
            }

            std::int64_t base[3] {};
            int lower[3] {};
            int upper[3] {};
            for (int axis = 0; axis < 3; ++axis) {
                // Coordinates beyond anything a model can sensibly hold share the border cells.
                constexpr double limit = 4.0e18;
                const double quotient = std::clamp(std::floor(coord[axis] / spacing), -limit, limit);
                base[axis] = static_cast<std::int64_t>(quotient);

                // Probe a neighbour only on the sides this point is within the tolerance of.
                const double offset = (coord[axis] / spacing - quotient) * spacing;
                lower[axis] = offset < tolerance ? -1 : 0;
                upper[axis] = spacing - offset < tolerance ? 1 : 0;
            }

            std::size_t duplicate = noIndex;
            for (int dx = lower[0]; dx <= upper[0] && duplicate == noIndex; ++dx) {
                for (int dy = lower[1]; dy <= upper[1] && duplicate == noIndex; ++dy) {
                    for (int dz = lower[2]; dz <= upper[2] && duplicate == noIndex; ++dz) {
                        auto it = firstInCell.find(Cell {base[0] + dx, base[1] + dy, base[2] + dz});
                        if (it == firstInCell.end()) {
                            continue;
                        }
                        for (std::size_t other = it->second; other != noIndex;
                             other = nextInCell[other]) {
                            if (sameWithinTolerance(point, points[other])) {
                                duplicate = other;
                                break;
                            }
                        }
                    }
                }
            }

            if (duplicate != noIndex) {
                mapPointIndex[index] = duplicate;
                ++duplicatedPoints;
                continue;
            }

            // Only points that survive as representatives go into the grid, so a duplicate always
            // maps directly onto a kept point and the map never needs to be followed twice.
            std::size_t& head
                = firstInCell.try_emplace(Cell {base[0], base[1], base[2]}, noIndex).first->second;
            nextInCell[index] = head;
            head = index;
        }
    }

    void redirectPointIndex()
    {
        for (auto& face : faces) {
            face.I1 = int(mapPointIndex[face.I1]);
            face.I2 = int(mapPointIndex[face.I2]);
            face.I3 = int(mapPointIndex[face.I3]);
        }
    }

    std::vector<std::size_t> getPointDegrees() const
    {
        std::vector<std::size_t> degreeMap;
        degreeMap.resize(points.size());
        for (const auto& face : faces) {
            degreeMap[face.I1]++;
            degreeMap[face.I2]++;
            degreeMap[face.I3]++;
        }

        return degreeMap;
    }

    void decrementPointIndex(const std::vector<std::size_t>& degreeMap)
    {
        std::vector<std::size_t> decrements;
        decrements.resize(points.size());

        std::size_t decr = 0;
        for (std::size_t pos = 0; pos < points.size(); pos++) {
            decrements[pos] = decr;
            if (degreeMap[pos] == 0) {
                decr++;
            }
        }

        for (auto& face : faces) {
            face.I1 -= int(decrements[face.I1]);
            face.I2 -= int(decrements[face.I2]);
            face.I3 -= int(decrements[face.I3]);
        }
    }

    void removeUnusedPoints(const std::vector<std::size_t>& degreeMap)
    {
        // remove unreferenced points
        std::vector<Base::Vector3d> new_points;
        new_points.reserve(points.size() - duplicatedPoints);
        for (std::size_t pos = 0; pos < points.size(); ++pos) {
            if (degreeMap[pos] > 0) {
                new_points.push_back(points[pos]);
            }
        }

        points.swap(new_points);
    }

private:
    std::vector<Base::Vector3d> points;
    std::vector<Facet> faces;
    double tolerance = 0.0;
    std::size_t duplicatedPoints = 0;
    std::vector<std::size_t> mapPointIndex;
};

}  // namespace

void BRepMesh::getFacesFromDomains(
    const std::vector<Domain>& domains,
    std::vector<Base::Vector3d>& points,
    std::vector<Facet>& faces
)
{
    std::size_t numFaces = 0;
    for (const auto& it : domains) {
        numFaces += it.facets.size();
    }
    faces.reserve(numFaces);

    std::set<MeshVertex> vertices;
    auto addVertex = [&vertices](const Base::Vector3d& pnt, uint32_t& pointIndex) {
        MeshVertex vertex(pnt);
        vertex.i = vertices.size();
        auto it = vertices.insert(vertex);
        pointIndex = it.first->i;
    };

    for (const auto& domain : domains) {
        std::size_t numDomainFaces = 0;
        for (const Facet& df : domain.facets) {
            Facet face;

            // 1st vertex
            addVertex(domain.points[df.I1], face.I1);

            // 2nd vertex
            addVertex(domain.points[df.I2], face.I2);

            // 3rd vertex
            addVertex(domain.points[df.I3], face.I3);

            // make sure that we don't insert invalid facets
            if (face.I1 != face.I2 && face.I2 != face.I3 && face.I3 != face.I1) {
                faces.push_back(face);
                numDomainFaces++;
            }
        }

        domainSizes.push_back(numDomainFaces);
    }

    std::vector<Base::Vector3d> meshPoints;
    meshPoints.resize(vertices.size());
    for (const auto& vertex : vertices) {
        meshPoints[vertex.i] = vertex.toPoint();
    }
    points.swap(meshPoints);

    MergeVertex merge(points, faces, Precision::Confusion());
    if (merge.hasDuplicatedPoints()) {
        merge.mergeDuplicatedPoints();
        points = merge.getPoints();
        faces = merge.getFacets();
    }
}

std::vector<BRepMesh::Segment> BRepMesh::createSegments() const
{
    std::size_t numMeshFaces = 0;
    std::vector<Segment> segm;
    for (size_t numDomainFaces : domainSizes) {
        Segment segment(numDomainFaces);
        std::generate(segment.begin(), segment.end(), Base::iotaGen<std::size_t>(numMeshFaces));
        numMeshFaces += numDomainFaces;
        segm.push_back(segment);
    }

    return segm;
}
