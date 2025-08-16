#include "clipper2/clipper.h"
#include <vector>
#include <utility>
#include <iostream>

// pick only what you need from Clipper2, no global using-namespace
using C2Path  = Clipper2Lib::PathD;
using C2Paths = Clipper2Lib::PathsD;
using C2Point = Clipper2Lib::PointD;

// your own point type, do not name it Point to avoid clashes
using FPoint = std::pair<double, double>;
using EigenPolyline = std::vector<FPoint>;

// Ray casting point in polygon
bool pointInPolygon(const FPoint& pt, const EigenPolyline& polygon) {
    const double x = pt.first, y = pt.second;
    bool inside = false;
    const std::size_t n = polygon.size();
    if (n < 2) return false;

    for (std::size_t i = 0, j = n - 1; i < n; j = i++) {
        const double xi = polygon[i].first, yi = polygon[i].second;
        const double xj = polygon[j].first, yj = polygon[j].second;

        const bool straddles = ((yi > y) != (yj > y));
        if (straddles) {
            const double xint = xi + (xj - xi) * (y - yi) / ((yj - yi) + 1e-12);
            if (x < xint) inside = !inside;
        }
    }
    return inside;
}

// Polygon within check, all vertices inside
bool polygonWithin(const EigenPolyline& inner, const EigenPolyline& outer) {
    for (const auto& p : inner) {
        if (!pointInPolygon(p, outer)) return false;
    }
    return true;
}

// conversions
C2Path toClipper(const EigenPolyline& poly) {
    C2Path path;
    path.reserve(poly.size());
    for (const auto& p : poly) path.emplace_back(p.first, p.second);
    return path;
}

static inline bool samePt(const FPoint& a, const FPoint& b, double eps = 1e-12) {
    return std::abs(a.first - b.first) <= eps && std::abs(a.second - b.second) <= eps;
}

EigenPolyline fromClipper(const C2Path& path) {
    EigenPolyline poly;
    poly.reserve(path.size() + 1);
    for (const auto& p : path) poly.emplace_back(p.x, p.y);

    // make orientation counterclockwise to mirror Boost default for exterior rings
    if (Clipper2Lib::Area(path) < 0) {
        std::reverse(poly.begin(), poly.end());
    }

    // close the ring by repeating the first point, like Boost does
    if (!poly.empty() && !samePt(poly.front(), poly.back())) {
        poly.push_back(poly.front());
    }
    return poly;
}

// intersection using Clipper2
std::vector<EigenPolyline>
polygonWithinPolygonClipper(const EigenPolyline& a, const EigenPolyline& b) {
    std::vector<EigenPolyline> out;
    C2Paths subject{ toClipper(a) };
    C2Paths clip{ toClipper(b) };
    C2Paths solution = Clipper2Lib::Intersect(subject, clip, Clipper2Lib::FillRule::NonZero);
    out.reserve(solution.size());
    for (const auto& path : solution) out.push_back(fromClipper(path));
    return out;
}

int main() {
    EigenPolyline poly_A = { {0,0},{0,10},{10,10},{10,0},{0,0} };
    EigenPolyline poly_domain = { {-5,-5},{-5,15},{15,15},{15,-5},{-5,-5} };
    EigenPolyline poly_B = { {5,5},{5,20},{20,20},{20,5},{5,5} };

    FPoint test_pt{5,5};
    bool covered = pointInPolygon(test_pt, poly_domain);
    std::cout << "Point covered by domain? " << std::boolalpha << covered << "\n";

    bool within = polygonWithin(poly_A, poly_domain);
    std::cout << "poly_A within domain? " << std::boolalpha << within << "\n";

    auto results = polygonWithinPolygonClipper(poly_A, poly_B);
    std::cout << "Intersection result count: " << results.size() << "\n";
    for (std::size_t i = 0; i < results.size(); ++i) {
        std::cout << "Polygon " << i + 1 << ":\n";
        for (const auto& p : results[i]) {
            std::cout << "  (" << p.first << ", " << p.second << ")\n";
        }
    }
    return 0;
}
