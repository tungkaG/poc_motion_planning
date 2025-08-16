#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <iostream>
#include <deque>
#include <vector>

namespace bg = boost::geometry;

// --- Types
using point_type = bg::model::d2::point_xy<double>;
using polygon_type = bg::model::polygon<point_type>;
using mpolygon_type = bg::model::multi_polygon<polygon_type>;
using EigenPolyline = std::vector<point_type>;  // Mocked EigenPolyline

// --- Function: polygonWithinPolygonBoost
std::vector<EigenPolyline> polygonWithinPolygonBoost(const polygon_type& polygon_input,
                                                     const polygon_type& polygon_other) {
    std::vector<EigenPolyline> polygons_within_polygon_other;
    std::deque<polygon_type> parts_in_polygon;

    bg::intersection(polygon_input, polygon_other, parts_in_polygon);

    for (const polygon_type& p : parts_in_polygon) {
        EigenPolyline vertices;
        for (auto it = bg::exterior_ring(p).rbegin(); it != bg::exterior_ring(p).rend(); ++it) {
            double x = bg::get<0>(*it);
            double y = bg::get<1>(*it);
            vertices.emplace_back(x, y);
        }
        polygons_within_polygon_other.push_back(vertices);
    }
    return polygons_within_polygon_other;
}

// --- Entry point
int main() {
    // Define a square polygon (poly_A)
    polygon_type poly_A;
    bg::append(poly_A, point_type(0, 0));
    bg::append(poly_A, point_type(0, 10));
    bg::append(poly_A, point_type(10, 10));
    bg::append(poly_A, point_type(10, 0));
    bg::append(poly_A, point_type(0, 0));
    bg::correct(poly_A);

    // Define a larger polygon domain (poly_domain)
    polygon_type poly_domain;
    bg::append(poly_domain, point_type(-5, -5));
    bg::append(poly_domain, point_type(-5, 15));
    bg::append(poly_domain, point_type(15, 15));
    bg::append(poly_domain, point_type(15, -5));
    bg::append(poly_domain, point_type(-5, -5));
    bg::correct(poly_domain);

    // Define multi-polygon (mpoly)
    mpolygon_type mpoly;
    mpoly.push_back(poly_A);

    // Test covered_by for a point
    point_type test_pt(5, 5);
    bool is_covered = bg::covered_by(test_pt, poly_domain);
    std::cout << "Point (5,5) is covered by domain? " << std::boolalpha << is_covered << "\n";

    // Test within for poly_A in poly_domain
    bool poly_within = bg::within(poly_A, poly_domain);
    std::cout << "poly_A is within domain? " << std::boolalpha << poly_within << "\n";

    // Intersect poly_A with another polygon
    polygon_type poly_B;
    bg::append(poly_B, point_type(5, 5));
    bg::append(poly_B, point_type(5, 20));
    bg::append(poly_B, point_type(20, 20));
    bg::append(poly_B, point_type(20, 5));
    bg::append(poly_B, point_type(5, 5));
    bg::correct(poly_B);

    std::cout << "\nRunning polygonWithinPolygonBoost...\n";
    auto results = polygonWithinPolygonBoost(poly_A, poly_B);

    std::cout << "Intersection result count: " << results.size() << "\n";
    for (size_t i = 0; i < results.size(); ++i) {
        std::cout << "Polygon " << i + 1 << ":\n";
        for (const auto& pt : results[i]) {
            std::cout << "  (" << bg::get<0>(pt) << ", " << bg::get<1>(pt) << ")\n";
        }
    }

    return 0;
}
