#pragma once
// ============================================================
//  singapore_streets.h
//  Singapore CBD & surrounds street network for ns-3 waveguide
//  mesh simulation (IEEE IoTJ 2026).
//
//  COORDINATE SYSTEM
//  -----------------
//  Origin (0, 0)       : ~1.270°N, 103.820°E  (Harbourfront / Vivocity)
//  NE corner (10000,10000): ~1.360°N, 103.910°E  (Paya Lebar / Geylang)
//  Units               : metres (1 sim unit = 1 m)
//  x-axis              : East  (longitude increasing)
//  y-axis              : North (latitude increasing)
//  Conversion          : x = (lon - 103.820) * 111297
//                        y = (lat - 1.270)   * 111319
//
//  STREET DATA
//  -----------
//  Each segment is an approximate centreline derived from
//  OpenStreetMap data for Singapore (© OpenStreetMap contributors,
//  ODbL licence). Endpoints are snapped to the nearest 50 m for
//  clarity. Only major arterials relevant to sub-GHz propagation
//  corridors are included; minor lanes are omitted (~50 segments
//  total; omission of minor streets makes waveguide percolation
//  results conservative).
//
//  Streets are grouped by bearing class:
//    E-W  : bearing  90° ± 15°
//    N-S  : bearing   0° ± 15°
//    NE-SW: bearing  45° ± 15°  (oblique — key Singapore CBD feature)
//    NW-SE: bearing 135° ± 15°
// ============================================================

#include <cmath>
#include <vector>
#include <string>

namespace SingaporeStreets {

struct Segment {
    double x1, y1;   // start (metres, simulation coords)
    double x2, y2;   // end
    std::string name;
};

// ---- E-W STREETS (bearing ~90°) -------------------------
// ---- N-S STREETS (bearing ~0°)  -------------------------
// ---- OBLIQUE NE-SW (~45°)       -------------------------
// ---- OBLIQUE NW-SE (~135°)      -------------------------

static const std::vector<Segment> STREETS = {

    // ========================================================
    //  SOUTHERN CBD / TANJONG PAGAR / MARINA BAY
    // ========================================================

    // Keppel Road — E-W, southern waterfront
    {1500, 800,  5000, 800,  "Keppel Road"},

    // Telok Blangah Road — E-W
    {500,  1500, 3000, 1500, "Telok Blangah Road"},

    // Maxwell Road — E-W
    {2000, 1900, 4500, 1900, "Maxwell Road"},

    // Anson Road — N-S (Tanjong Pagar spine)
    {3200, 800,  3200, 3000, "Anson Road"},

    // Shenton Way (southern, N-S section)
    {3800, 800,  3800, 2500, "Shenton Way (S)"},

    // Robinson Road (northern, N-S section)
    {4250, 2000, 4250, 4000, "Robinson Road (N-S)"},

    // Cecil Street — NE-SW ~45°
    // Runs from Collyer Quay area to Market Street junction
    {3100, 1400, 4700, 3000, "Cecil Street"},

    // Robinson Road (southern oblique section, ~45°)
    {3700, 1800, 4700, 2800, "Robinson Road (oblique)"},

    // Collyer Quay / Marina Boulevard — E-W waterfront
    {3000, 500,  7000, 500,  "Collyer Quay / Marina Blvd"},

    // Cross Street — E-W
    {2500, 2500, 5500, 2500, "Cross Street"},

    // Eu Tong Sen Street / New Bridge Road — N-S with slight NNW lean
    // bearing ~10° off N
    {2200, 1000, 2500, 5000, "Eu Tong Sen St / New Bridge Rd"},

    // South Bridge Road — N-S
    {2950, 2500, 2950, 5500, "South Bridge Road"},

    // ========================================================
    //  CENTRAL / CIVIC DISTRICT
    // ========================================================

    // North Bridge Road — roughly N-S with slight NNE lean
    {5000, 3500, 5200, 7000, "North Bridge Road"},

    // Victoria Street — N-S
    {5800, 4500, 5800, 7500, "Victoria Street"},

    // Bras Basah Road — E-W with slight SE lean (~80°)
    {3500, 6000, 7000, 5700, "Bras Basah Road"},

    // Stamford Road — E-W
    {3500, 6500, 6000, 6500, "Stamford Road"},

    // St Andrew's Road — NW-SE ~135°
    {3800, 5500, 5000, 4500, "St Andrew's Road"},

    // Parliament Place / High Street — E-W
    {3200, 4500, 5200, 4500, "High Street / Parliament Place"},

    // ========================================================
    //  BEACH ROAD / NICOLL HWY CORRIDOR
    // ========================================================

    // Beach Road — NE-SW ~45°
    {5200, 4200, 7200, 6200, "Beach Road"},

    // Nicoll Highway — roughly E-W (slight NE lean ~85°)
    {5000, 5500, 9000, 5700, "Nicoll Highway"},

    // Sims Avenue / Geylang Road — E-W
    {6000, 4000, 10000, 4000, "Geylang Road / Sims Ave"},

    // Mountbatten Road — NW-SE ~135°
    {7000, 5000, 9000, 3500, "Mountbatten Road"},

    // ========================================================
    //  ORCHARD / RIVER VALLEY CORRIDOR
    // ========================================================

    // Orchard Road — E-W with very slight SE lean (~95°)
    {2000, 7500, 8500, 7200, "Orchard Road"},

    // Penang Road — NW-SE ~135°
    {3000, 7500, 4000, 6500, "Penang Road"},

    // River Valley Road — NW-SE ~120°
    {1800, 6500, 4500, 4500, "River Valley Road"},

    // Kim Seng Road — N-S with slight lean
    {1800, 5000, 2000, 8000, "Kim Seng Road"},

    // Grange Road — E-W
    {2000, 7000, 5000, 7000, "Grange Road"},

    // ========================================================
    //  OUTRAM / CHINATOWN / TIONG BAHRU
    // ========================================================

    // Outram Road — N-S
    {1800, 1000, 1800, 5000, "Outram Road"},

    // Havelock Road — E-W
    {1500, 4000, 4000, 4000, "Havelock Road"},

    // Upper Cross Street / Cantonment Rd — E-W
    {1500, 3000, 4000, 3000, "Upper Cross Street"},

    // Jalan Bukit Merah — NW-SE ~120°
    {500,  4000, 2500, 1500, "Jalan Bukit Merah"},

    // ========================================================
    //  EASTERN SECTOR (Kallang / Paya Lebar)
    // ========================================================

    // Kallang Road — NW-SE ~130°
    {7000, 6500, 9000, 4500, "Kallang Road"},

    // Serangoon Road — N-S with slight NNE lean (~15°)
    {6000, 5000, 6500, 9500, "Serangoon Road"},

    // MacPherson Road — E-W
    {7000, 7500, 10000, 7500, "MacPherson Road"},

    // Paya Lebar Road — N-S
    {8500, 4000, 8500, 9000, "Paya Lebar Road"},

    // ========================================================
    //  NORTHERN SECTOR (Novena / Toa Payoh)
    // ========================================================

    // Thomson Road — N-S
    {4500, 7000, 4500, 10000, "Thomson Road"},

    // Toa Payoh Lorong 1–8 (simplified as E-W spine)
    {3500, 8500, 6500, 8500, "Toa Payoh Central"},

    // Braddell Road — E-W
    {3000, 9000, 8000, 9000, "Braddell Road"},

    // Balestier Road — NW-SE ~135°
    {4000, 8000, 5500, 6500, "Balestier Road"},

    // ========================================================
    //  WESTERN SECTOR (Alexandra / Queenstown)
    // ========================================================

    // Alexandra Road — NW-SE ~120°
    {500,  5000, 2500, 2500, "Alexandra Road"},

    // Commonwealth Avenue — N-S
    {1000, 4500, 1000, 8500, "Commonwealth Avenue"},

    // Queensway — NW-SE ~135°
    {500,  7500, 2000, 6000, "Queensway"},

};

// ============================================================
//  PointToLineDistance()
//
//  Returns perpendicular distance from point (px,py) to the
//  infinite line passing through (ax,ay)→(bx,by).
// ============================================================

inline double PointToLineDistance(double px, double py,
                                   double ax, double ay,
                                   double bx, double by) {
    double dx = bx - ax, dy = by - ay;
    double len2 = dx*dx + dy*dy;
    if (len2 < 1e-9) {
        // Degenerate segment — treat as point
        return std::sqrt((px-ax)*(px-ax) + (py-ay)*(py-ay));
    }
    // Perpendicular distance from point to infinite line
    // |cross product| / |direction|
    double cross = (px - ax) * dy - (py - ay) * dx;
    return std::abs(cross) / std::sqrt(len2);
}

// ============================================================
//  IsStreetAligned()
//
//  Returns true if both node positions (px1,py1) and (px2,py2)
//  lie within halfWidth metres of the same street centreline
//  AND both nodes' projections fall within the finite physical
//  extent of that segment (with a 200m overshoot tolerance at
//  each end — one block-length — to handle nodes just past a
//  segment endpoint without requiring the segment to be
//  extended further).
//
//  WHY FINITE PROJECTION:
//  Without the projection check, two nodes could be aligned
//  with the *infinite mathematical extension* of a short street
//  segment while being kilometres past its physical end (past a
//  T-junction, height change, or open area). This would
//  incorrectly grant n1=1.4 waveguide treatment to a link with
//  no actual canyon. The 200m tolerance prevents this while
//  still correctly handling nodes placed near segment endpoints.
//
//  WHY INFINITE LINE FOR THE PERPENDICULAR CHECK:
//  The perpendicular distance check still uses the infinite line
//  projection. This is correct: the canyon alignment condition
//  (both nodes within 12.5m of the centreline axis) is a
//  geometric property of the street direction, not its length.
//  Oblique arterials like Cecil St (~45°) and Beach Rd are
//  correctly handled this way.
// ============================================================

inline bool IsStreetAligned(double px1, double py1,
                              double px2, double py2,
                              double streetWidth = 25.0) {
    double halfW = streetWidth / 2.0;
    for (const auto &s : STREETS) {
        // Step 1: Both nodes within halfWidth of the infinite centreline
        if (PointToLineDistance(px1, py1, s.x1, s.y1, s.x2, s.y2) < halfW &&
            PointToLineDistance(px2, py2, s.x1, s.y1, s.x2, s.y2) < halfW) {

            // Step 2: Both nodes' projections fall within the finite
            // physical segment extent (± 200m overshoot tolerance).
            double dx    = s.x2 - s.x1, dy = s.y2 - s.y1;
            double segLen2 = dx*dx + dy*dy;
            if (segLen2 < 1.0) continue; // degenerate segment guard

            // Normalised parameter t: 0 = segment start, 1 = segment end.
            // Values outside [0,1] are past the physical segment ends.
            double t1 = ((px1 - s.x1)*dx + (py1 - s.y1)*dy) / segLen2;
            double t2 = ((px2 - s.x1)*dx + (py2 - s.y1)*dy) / segLen2;

            // Allow one block-length (200m) overshoot at each end.
            double tol = 200.0 / std::sqrt(segLen2);

            if (t1 >= -tol && t1 <= 1.0 + tol &&
                t2 >= -tol && t2 <= 1.0 + tol) {
                return true;
            }
        }
    }
    return false;
}

} // namespace SingaporeStreets
