#pragma once

#include "riptide_mpc/motion_profile.hpp"

#include <cstdint>
#include <memory>
#include <vector>

namespace riptide_mpc {
struct Waypoint {
    Vector3d position = Vector3d::Zero(); // base_link in the odometry frame
    Quaterniond orientation = Quaterniond::Identity();
};

// Same values as riptide_msgs2/PathSegment.
enum class PathShape : std::uint8_t { LINE = 0, ARC = 1 };
enum class PathHeading : std::uint8_t { WAYPOINT = 0, PATH = 1, LOOK_AT = 2 };

// One FollowPath point and how the path gets there from the point before it
// (odometry frame, base_link). See riptide_msgs2/PathSegment.
struct PathPoint {
    Vector3d position = Vector3d::Zero();
    Quaterniond orientation = Quaterniond::Identity();
    PathShape shape = PathShape::LINE;
    // ARC: `sweep` rad about `center` (x/y only) around +z, + = counterclockwise
    // seen from above. `position` sets the end radius (a spiral if it differs
    // from the start's) and depth; the arc ends at that radius, `sweep` round.
    Vector3d center = Vector3d::Zero();
    double sweep = 0;
    PathHeading heading = PathHeading::WAYPOINT;
    Vector3d look_at = Vector3d::Zero();
    double yaw_offset = 0; // added to the PATH / LOOK_AT yaw
    double spin = 0;       // extra yaw over the segment, spread by progress
    // Steady spin: extra yaw per metre of s, one continuous spin across segments.
    // The path's total is rounded to whole turns (at least one), so it ends on its heading.
    double spin_rate = 0;
};

struct PathOptions {
    double corner_radius = 0.3; // m; a sharp join is rounded off within this of the corner
    double lateral_accel = 0.3; // m/s^2; sideways acceleration allowed on arcs and rounded corners
    double kink_speed = 0.03;   // m/s; a join that cannot be rounded is crossed this slowly
};

// Where a reference is along a plan: distance s and its rates.
struct PathProgress {
    double s = 0, v = 0, a = 0;
};

// A path as one curve parameterized by distance s: lines, arcs (spirals,
// helices) and quadratic blends rounding off sharp joins. A rotation in place
// counts as rotation * (linear_speed / angular_speed) metres of s, so turning
// is progress too. The attitude is a function of s: yaw from the segment's
// heading mode, roll and pitch blended between the points. A reference moves
// along s with a jerk-limited profile that slows for curvature (lateral
// acceleration), turn rate and joins, and stops exactly on the last point.
class PathPlan {
  public:
    // From `start` (where the reference is) through `points`. Throws
    // std::invalid_argument for an unusable path.
    static std::shared_ptr<const PathPlan> build(const Waypoint &start, const std::vector<PathPoint> &points,
                                                 const MotionLimits &limits, const PathOptions &options = {});

    double length() const {
        return length_;
    }
    Waypoint end() const {
        return {position(length_), orientation(length_)};
    }
    // Index of the path point being approached at s.
    std::size_t segmentAt(double s) const;

    // Advances the reference by dt. The scales (0, 1] slow its cruise (the
    // reference governor); speed limits from the path itself always apply.
    void step(PathProgress &p, double linear_scale, double angular_scale, double dt) const;
    bool atEnd(const PathProgress &p) const {
        return p.s >= length_ && p.v == 0 && p.a == 0;
    }
    // The reference's pose, velocity and acceleration at p (odometry frame).
    void pose(const PathProgress &p, MotionProfile &out) const;
    Vector3d position(double s) const;
    Vector3d tangent(double s) const; // unit direction of travel; zero while turning in place
    Quaterniond orientation(double s) const;
    // s of the point nearest `p` within `window` of s = `near`.
    double project(const Vector3d &p, double near, double window) const;

  private:
    struct Piece {
        enum Kind { LINE, ARC, BEZIER, TURN } kind = LINE;
        Vector3d a = Vector3d::Zero(), b = Vector3d::Zero(), c = Vector3d::Zero(); // LINE a->b, BEZIER, TURN at a
        double cx = 0, cy = 0, phi0 = 0, sweep = 0, r0 = 0, r1 = 0, z0 = 0, z1 = 0; // ARC
        double t0 = 0, t1 = 1; // parameter range in use
        double s0 = 0, length = 0;
        std::size_t segment = 0;
        std::vector<double> table; // distance from t0 at evenly spaced t; empty: uniform speed in t
        void derivatives(double t, Vector3d &p, Vector3d &d1, Vector3d &d2) const;
        Vector3d at(double t) const;
        void finalize(); // length and table for [t0, t1]
        double tAt(double local) const;
    };
    struct Segment {
        std::size_t first = 0, last = 0; // pieces [first, last)
        double s0 = 0, s1 = 0;
        bool linear = true;   // yaw changes at a constant rate (WAYPOINT heading or a turn in place)
        double start_yaw = 0; // continuous with the previous segment
        double turn = 0;      // linear: yaw change over the segment, before spin
        std::vector<double> mode_yaw; // otherwise: heading-mode yaw at evenly spaced s, unwrapped
        double delta = 0, blend = 1;  // alignment from start_yaw, faded out over `blend` metres
        double spin = 0;
        double phase0 = 0, spin_rate = 0; // steady spin: its yaw at s0, and per metre after
        Quaterniond tilt0 = Quaterniond::Identity(), tilt1 = Quaterniond::Identity();
    };
    struct Sample {
        double s = 0;
        double envelope = 0;   // speed the path allows here, including braking for what is ahead
        double linear_cap = 0; // linear speed limit along the direction of travel
        double angular_cap = 0; // speed at which the attitude turns at the angular speed limit
        double accel = 0, jerk = 0;
        Vector3d position = Vector3d::Zero();
    };

    std::size_t pieceAt(double s, std::size_t begin, std::size_t end) const;
    void geometry(double s, std::size_t begin, std::size_t end, Vector3d &p, Vector3d &t, Vector3d &k) const;
    double yaw(const Segment &g, double s) const;
    std::size_t sampleAt(double s) const;

    std::vector<Piece> pieces_;
    std::vector<Segment> segments_;
    std::vector<Sample> samples_;
    double length_ = 0;
};
} // namespace riptide_mpc
