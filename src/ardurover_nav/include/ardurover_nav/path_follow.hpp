#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <optional>
#include <vector>

#include "ardurover_nav/path_io.hpp"

namespace ardurover_nav {

// Pure-pursuit on a smooth stretch. Progress and 180s are not carrot problems:
// we step one segment at a time, and a cusp is a vertex plus a discrete maneuver.
constexpr double kLookaheadM = 1.4;     // default carrot; 2 m cut corners on path 1/2
constexpr double kLookaheadMinM = 0.6;  // floor in a sharp turn
constexpr double kUnstickLookaheadM = 2.8;  // Unstick-only: around the lip, not progress++
constexpr double kTurnHeadingRad = 0.7; // |ψe| at which look-ahead/speed are fully tightened
constexpr double kMinTurnSpeed = 0.35;  // fraction of cruise when |ψe| is large
constexpr double kCruiseSpeed = 1.0;
constexpr double kHeadingP = 1.0;     // flip sign if BODY_NED steers the wrong way
constexpr double kHeadingD = 0.08;    // yaw-rate damping; not d(ψe)/dt (carrot noise pumps weave)
constexpr double kMaxYawRate = 0.6;   // stop spin-outs when heading error is large
constexpr double kSlowdownM = 4.0;    // bleed speed into the true polyline end (not a stop rule)
constexpr double kCuspDot = -0.5;     // next tangent opposite current → fold / 180 vertex
constexpr double kSegPastEps = 1e-3;  // increment index only once projection is past the end
constexpr double kTinySegM = 1e-4;    // skip degenerate samples when taking a tangent
constexpr double kSkipSegM = 0.12;    // hairpin recordings (path 2 i=103) are 5 cm; projection never passes
constexpr double kYawSameRad = 0.7;   // recorded yaw unchanged across the fold
constexpr double kYawFlipRad = 2.0;   // ~115 deg: recorded yaw flipped (real U-turn)
constexpr double kTurnAlignRad = 0.25;  // leave Turn once heading matches the outgoing yaw
constexpr double kTurnCreep = 0.15;     // slow vx while spinning at a heading-180
constexpr double kCuspApproachM = 1.5;  // bleed speed so we arrive at the fold before stopping / turning
constexpr double kRollbackTailM = 2.0;  // leftover after a fold: stop-rollback, not a mid-path wiggle
constexpr double kCatchUpM = 10.0;      // cut-corner: search this far ahead for a closer segment
constexpr double kCatchUpBetterM = 0.15;  // later segment must beat the current one by this
constexpr double kPi = 3.14159265358979323846;
constexpr double kRadToDeg = 180.0 / kPi;
// Boardwalk hang: reverse to unhook, then a forward+yaw push while the bumper
// is free. Leaving on reverse-distance alone rams the same lip (path 2 i=91).
// GUIDED ramp-up also looks like act≈0 — wait until we have actually rolled.
constexpr int kStuckDetectTicks = 30;       // 1.5 s still commanding and frozen
constexpr int kUnstickMinTicks = 8;         // 0.4 s before a phase may end
constexpr int kUnstickWiggleTicks = 8;      // 0.4 s per reverse yaw side (left then right)
constexpr int kUnstickReverseTicks = 40;    // 2.0 s: at least two left/right wiggle cycles
constexpr int kUnstickDriveTicks = 40;      // 2.0 s forward push around the lip
constexpr int kUnstickMaxAttempts = 4;      // reverse→drive cycles before giving up
constexpr int kUnstickCooldownTicks = 30;   // 1.5 s before another unstick
constexpr double kStuckCmdVx = 0.20;
constexpr double kStuckCmdWz = 0.15;
constexpr double kStuckActVx = 0.04;        // slow slide (0.06) is not a hang yet
constexpr double kStuckActWz = 0.04;
constexpr double kStuckPoseM = 0.04;
constexpr double kUnstickReverseM = 0.30;   // unhook distance after the wiggle, not instead of it
constexpr double kUnstickDriveM = 0.40;     // leave only after a real forward gain
constexpr double kUnstickClearHeadingRad = 0.40;  // ~23 deg: do not resume into the same lip
constexpr double kUnstickReverseSpeed = 0.45;
constexpr double kUnstickDriveSpeed = 0.55;
constexpr double kSeenMotionVel = 0.20;     // have we ever really moved this run?
constexpr double kTurnInPlaceRad = 0.85;    // |ψe| above this → vx=0 (do not drive off the curb)
constexpr double kTiltRad = 0.10;           // ~6 deg: chassis on a lip (roll/pitch)

inline double distance(const Waypoint &a, const Waypoint &b) {
    return std::hypot(a.x - b.x, a.y - b.y);
}

inline double segment_length(const std::vector<Waypoint> &path, std::size_t i) {
    if (i + 1 >= path.size()) {
        return 0.0;
    }
    return distance(path[i], path[i + 1]);
}

// Unclamped projection of pose onto segment [a, b]. t in [0, 1] is on the
// segment; t > 1 means we have passed the end and may step the index.
// Degenerate (zero-length) samples count as already past so we skip them.
inline double project_t(const Waypoint &pose, const Waypoint &a, const Waypoint &b) {
    const double abx = b.x - a.x;
    const double aby = b.y - a.y;
    const double len2 = abx * abx + aby * aby;
    if (len2 < 1e-16) {
        return 1.0;
    }
    return ((pose.x - a.x) * abx + (pose.y - a.y) * aby) / len2;
}

struct Tangent {
    double dx{0.0};
    double dy{0.0};
    double len{0.0};
};

// First non-tiny chord into / out of a vertex. Recordings leave 1e-4 m jitter
// samples that would otherwise give a nonsense heading.
inline std::optional<Tangent> incoming_tangent(const std::vector<Waypoint> &path, std::size_t vertex) {
    if (vertex == 0) {
        return std::nullopt;
    }
    for (std::size_t i = vertex; i-- > 0;) {
        const double dx = path[vertex].x - path[i].x;
        const double dy = path[vertex].y - path[i].y;
        const double len = std::hypot(dx, dy);
        if (len > kTinySegM) {
            return Tangent{dx, dy, len};
        }
    }
    return std::nullopt;
}

inline std::optional<Tangent> outgoing_tangent(const std::vector<Waypoint> &path, std::size_t vertex) {
    for (std::size_t i = vertex + 1; i < path.size(); ++i) {
        const double dx = path[i].x - path[vertex].x;
        const double dy = path[i].y - path[vertex].y;
        const double len = std::hypot(dx, dy);
        if (len > kTinySegM) {
            return Tangent{dx, dy, len};
        }
    }
    return std::nullopt;
}

inline double tangent_dot(const Tangent &a, const Tangent &b) {
    return (a.dx * b.dx + a.dy * b.dy) / (a.len * b.len);
}

// Fold in the polyline: incoming and outgoing travel point opposite ways.
// This is a vertex problem, not a "carrot is behind us" problem.
inline bool is_cusp_at_vertex(const std::vector<Waypoint> &path, std::size_t vertex) {
    const auto in = incoming_tangent(path, vertex);
    const auto out = outgoing_tangent(path, vertex);
    return in && out && tangent_dot(*in, *out) < kCuspDot;
}

// First cusp at or after the end of from_seg. Used both to stop the carrot
// and to decide when we have physically arrived at the fold.
inline std::optional<std::size_t> next_cusp_vertex(const std::vector<Waypoint> &path, std::size_t from_seg) {
    const std::size_t start = from_seg + 1;
    for (std::size_t v = start; v + 1 < path.size(); ++v) {
        if (is_cusp_at_vertex(path, v)) {
            return v;
        }
    }
    return std::nullopt;
}

enum class CuspKind { End, Turn, Pass };

// Classify from the recording, not the vehicle heading. Using vehicle yaw
// turned path 2 i=69 (a 9 cm wiggle) into a Turn after the rover spun.
//
// End: recorded yaw unchanged and XY after the fold goes the other way
//   (path 0/1 ~1 m stop-rollback). Do not track it backwards.
// Turn: recorded yaw flips ~180° — a real U-turn, spin then go forward.
// Pass: recording wiggle, just step through.
inline CuspKind classify_cusp(const std::vector<Waypoint> &path, std::size_t vertex) {
    const auto out = outgoing_tangent(path, vertex);
    if (!out) {
        return CuspKind::Pass;
    }
    const double travel = std::atan2(out->dy, out->dx);
    const std::size_t yaw_i = std::min(vertex + 1, path.size() - 1);
    const double path_yaw_after = path[yaw_i].yaw;
    const double path_yaw_at = path[vertex].yaw;
    const double yaw_delta = std::abs(wrap(path_yaw_after - path_yaw_at));
    const double travel_vs_yaw = std::abs(wrap(travel - path_yaw_after));
    if (yaw_delta < kYawSameRad && travel_vs_yaw > kYawFlipRad) {
        return CuspKind::End;
    }
    if (yaw_delta > kYawFlipRad) {
        return CuspKind::Turn;
    }
    return CuspKind::Pass;
}

// End / Turn must stop the carrot and bleed speed. A Pass fold is a recorded
// wiggle (path 2 i=69 / i=177) — look and drive through it.
inline bool is_hard_cusp(const std::vector<Waypoint> &path, std::size_t vertex) {
    return is_cusp_at_vertex(path, vertex) && classify_cusp(path, vertex) != CuspKind::Pass;
}

inline std::optional<std::size_t> next_hard_cusp_vertex(const std::vector<Waypoint> &path, std::size_t from_seg) {
    const std::size_t start = from_seg + 1;
    for (std::size_t v = start; v + 1 < path.size(); ++v) {
        if (is_hard_cusp(path, v)) {
            return v;
        }
    }
    return std::nullopt;
}

// After cutting a Pass fold, projection t on the inbound segment never
// exceeds 1 (path 2 i=166). Step forward to a later, closer segment.
// Search only ahead, and never across an End/Turn (path 0 rollback).
inline std::size_t catch_up_progress(
    const Waypoint &pose, const std::vector<Waypoint> &path, std::size_t progress
) {
    if (progress + 1 >= path.size()) {
        return progress;
    }
    auto seg_dist = [&](std::size_t i) {
        if (i + 1 >= path.size()) {
            return distance(pose, path.back());
        }
        const double t = std::clamp(project_t(pose, path[i], path[i + 1]), 0.0, 1.0);
        const double x = path[i].x + t * (path[i + 1].x - path[i].x);
        const double y = path[i].y + t * (path[i + 1].y - path[i].y);
        return std::hypot(pose.x - x, pose.y - y);
    };

    std::size_t best = progress;
    double best_d = seg_dist(progress);
    double traveled = 0.0;
    const std::size_t last_seg = path.size() - 2;
    for (std::size_t i = progress; i < last_seg; ++i) {
        if (is_hard_cusp(path, i + 1)) {
            break;
        }
        traveled += segment_length(path, i);
        if (traveled > kCatchUpM) {
            break;
        }
        const double d = seg_dist(i + 1);
        if (d + kCatchUpBetterM < best_d) {
            best = i + 1;
            best_d = d;
        }
    }
    return best;
}

// Recorded heading on the outgoing side of the fold — the yaw we spin toward.
inline double path_yaw_after_vertex(const std::vector<Waypoint> &path, std::size_t vertex) {
    return path[std::min(vertex + 1, path.size() - 1)].yaw;
}

// Arrived = we have stepped onto the vertex, or the projection on the
// inbound segment is past it and we are actually nearby (not a distant
// collinear projection onto an overlapping later stretch).
inline bool arrived_at_vertex(
    const Waypoint &pose, const std::vector<Waypoint> &path, std::size_t progress, std::size_t vertex
) {
    if (progress >= vertex) {
        return true;
    }
    if (vertex == 0) {
        return true;
    }
    if (distance(pose, path[vertex]) > 2.0) {
        return false;
    }
    return project_t(pose, path[vertex - 1], path[vertex]) >= 1.0 - kSegPastEps;
}

inline Waypoint interpolate(const Waypoint &a, const Waypoint &b, double t) {
    t = std::clamp(t, 0.0, 1.0);
    return {a.x + t * (b.x - a.x), a.y + t * (b.y - a.y), wrap(a.yaw + t * wrap(b.yaw - a.yaw))};
}

// Carrot at along-track s + L on this unfolding of the polyline (interpolated,
// not the nearest sample). Stop at End/Turn so a 1.4 m look-ahead does not sit
// on path 0's ~1 m rollback. Pass folds are not a wall.
inline Waypoint lookahead_waypoint(const std::vector<Waypoint> &path, std::size_t from, double t_on_seg, double lookahead_m) {
    if (path.size() < 2) {
        return path.front();
    }
    if (from + 1 >= path.size()) {
        return path.back();
    }
    const double t0 = std::clamp(t_on_seg, 0.0, 1.0);
    const double seg_len = segment_length(path, from);
    const double remain_on_seg = (1.0 - t0) * seg_len;
    if (lookahead_m <= remain_on_seg) {
        const double u = (seg_len < 1e-12) ? 1.0 : t0 + lookahead_m / seg_len;
        return interpolate(path[from], path[from + 1], u);
    }
    lookahead_m -= remain_on_seg;
    for (std::size_t i = from + 1; i + 1 < path.size(); ++i) {
        if (is_hard_cusp(path, i)) {
            return path[i];
        }
        const double len = segment_length(path, i);
        if (lookahead_m <= len) {
            return interpolate(path[i], path[i + 1], (len < 1e-12) ? 1.0 : lookahead_m / len);
        }
        lookahead_m -= len;
    }
    return path.back();
}

// Leftover polyline from the current projection. Used only to bleed speed;
// never as a stop condition (path 0's reverse tail is itself ~1 m).
inline double remaining_path_length(const std::vector<Waypoint> &path, std::size_t from, double t_on_seg) {
    if (from + 1 >= path.size()) {
        return 0.0;
    }
    double length = (1.0 - std::clamp(t_on_seg, 0.0, 1.0)) * segment_length(path, from);
    for (std::size_t i = from + 1; i + 1 < path.size(); ++i) {
        length += segment_length(path, i);
    }
    return length;
}

// Path 0/1 end with a ~1 m recorded rollback. Path 2 i=69 leaves ~62 m —
// that is a wiggle, not a finish.
inline bool is_rollback_tail(const std::vector<Waypoint> &path, std::size_t vertex) {
    return remaining_path_length(path, vertex, 0.0) <= kRollbackTailM;
}

// Signed distance to the current segment. Positive = left of travel direction.
inline double signed_cross_track(const Waypoint &pose, const std::vector<Waypoint> &path, std::size_t seg_i) {
    if (path.size() < 2) {
        return 0.0;
    }
    const std::size_t i0 = (seg_i + 1 < path.size()) ? seg_i : path.size() - 2;
    const Waypoint &a = path[i0];
    const Waypoint &b = path[i0 + 1];
    const double abx = b.x - a.x;
    const double aby = b.y - a.y;
    const double len = std::hypot(abx, aby);
    if (len < 1e-12) {
        return 0.0;
    }
    return (abx * (pose.y - a.y) - aby * (pose.x - a.x)) / len;
}

// True end of this unfolding: last index, or last segment with t past 1.
// Crow-flies to path.back() is the same collinear trap as the old 136→156 jump.
inline bool at_polyline_end(const std::vector<Waypoint> &path, std::size_t progress, double t_on_seg) {
    if (path.size() < 2) {
        return true;
    }
    if (progress + 1 >= path.size()) {
        return true;
    }
    return progress + 1 == path.size() - 1 && t_on_seg >= 1.0 - kSegPastEps;
}

// wz = Kp*ψe − Kd*yaw_rate. Damp measured spin, not carrot-bearing rate.
inline double heading_yaw_pd(double heading_error, double yaw_rate) {
    const double rate = std::clamp(yaw_rate, -kMaxYawRate, kMaxYawRate);
    return kHeadingP * heading_error - kHeadingD * rate;
}

inline double heading_yaw_cmd(double heading_error, double yaw_rate) {
    return std::clamp(heading_yaw_pd(heading_error, yaw_rate), -kMaxYawRate, kMaxYawRate);
}

inline double heading_toward(const Waypoint &pose, const Waypoint &to) {
    return wrap(std::atan2(to.y - pose.y, to.x - pose.x) - pose.yaw);
}

// Step along the current segment only. Never snap to a closer sample on an
// overlapping reverse tail (that was the 136 → 156 jump on path 0).
inline std::size_t advance_progress(
    const Waypoint &pose, const std::vector<Waypoint> &path, std::size_t progress
) {
    if (path.size() < 2) {
        return progress;
    }
    const std::size_t last_pt = path.size() - 1;
    const std::size_t last_seg = path.size() - 2;
    while (progress < last_seg) {
        // Dense hairpin samples (5–8 cm) leave t∈(0,1) forever when we are
        // 1 m beside the apex — that was the i=103 orbit. Skip them; do not
        // jump a distant overlapping tail. Only freeze on a short rollback
        // tail (path 0/1); mid-path wiggles (path 2 i=69) stay skippable.
        if (segment_length(path, progress) < kSkipSegM) {
            if (is_cusp_at_vertex(path, progress) && is_rollback_tail(path, progress)) {
                break;
            }
            ++progress;
            continue;
        }
        const double t = project_t(pose, path[progress], path[progress + 1]);
        if (t > 1.0 - kSegPastEps) {
            ++progress;
        } else {
            break;
        }
    }
    if (progress == last_seg && project_t(pose, path[last_seg], path[last_pt]) > 1.0 - kSegPastEps) {
        progress = last_pt;
    }
    return catch_up_progress(pose, path, progress);
}

struct Pursuit {
    Waypoint target{};
    double heading_error{0.0};
    double lookahead_m{kLookaheadM};
    double turn{0.0};
};

// BODY_NED heading to the carrot. Turn aims at the recorded outgoing yaw.
inline Pursuit pursuit_target(
    const std::vector<Waypoint> &path,
    const Waypoint &pose,
    std::size_t track_i,
    double track_t,
    bool turning,
    bool unsticking,
    double turn_target_yaw
) {
    Pursuit out;
    out.lookahead_m = kLookaheadM;
    out.target = lookahead_waypoint(path, track_i, track_t, out.lookahead_m);
    out.heading_error = heading_toward(pose, out.target);
    if (turning) {
        out.heading_error = wrap(turn_target_yaw - pose.yaw);
        out.target = lookahead_waypoint(path, track_i, track_t, kLookaheadMinM);
    } else if (unsticking) {
        // Same polyline, further along. The 0.6–0.7 m carrot sits in the lip;
        // do not bump progressIndex_ (that was the overlapping-tail snap).
        out.lookahead_m = kUnstickLookaheadM;
        out.target = lookahead_waypoint(path, track_i, track_t, out.lookahead_m);
        out.heading_error = heading_toward(pose, out.target);
    } else {
        // Shorten the carrot in a bend so we do not cut path 1/2 vertices.
        // Do not shrink it globally hoping a 180 falls out of the look-ahead.
        out.turn = std::clamp(std::abs(out.heading_error) / kTurnHeadingRad, 0.0, 1.0);
        out.lookahead_m = kLookaheadM + (kLookaheadMinM - kLookaheadM) * out.turn;
        if (out.turn > 0.0) {
            out.target = lookahead_waypoint(path, track_i, track_t, out.lookahead_m);
            out.heading_error = heading_toward(pose, out.target);
        }
    }

    // Circling the hairpin apex: carrot is behind (|ψe|>90°), vx=0 spin, orbit.
    // Walk the same polyline forward until the target is in the front half-plane.
    if (!turning && std::abs(out.heading_error) > kPi / 2) {
        double L = std::max(out.lookahead_m, kLookaheadM);
        for (int step = 0; step < 10; ++step) {
            L += 0.5;
            const Waypoint cand = lookahead_waypoint(path, track_i, track_t, L);
            const double he = heading_toward(pose, cand);
            out.target = cand;
            out.heading_error = he;
            out.lookahead_m = L;
            if (std::abs(he) < kPi / 2) {
                break;
            }
        }
    }
    return out;
}

inline double tracking_speed(
    const std::vector<Waypoint> &path,
    const Waypoint &pose,
    std::size_t track_i,
    double remaining,
    bool turning,
    double turn,
    double heading_error
) {
    // Bleed into the real end and into a cusp so we do not punch through
    // the vertex still commanding +vx.
    double end_scale = 1.0;
    if (!turning) {
        end_scale = std::clamp(remaining / kSlowdownM, 0.0, 1.0);
        if (const auto vertex = next_hard_cusp_vertex(path, track_i > 0 ? track_i - 1 : 0)) {
            if (!arrived_at_vertex(pose, path, track_i, *vertex)) {
                const double to_cusp = distance(pose, path[*vertex]);
                if (to_cusp < kCuspApproachM) {
                    end_scale = std::min(end_scale, std::clamp(to_cusp / kCuspApproachM, 0.3, 1.0));
                }
            }
        }
    }
    const double turn_scale = turning ? 1.0 : 1.0 - (1.0 - kMinTurnSpeed) * turn;
    double speed = turning ? kTurnCreep : kCruiseSpeed * end_scale * turn_scale;
    // After a hang the heading can be ~90°. Forward vx then drives off the
    // pathwalk; spin (or the unstick rock) until the carrot is ahead.
    if (!turning && std::abs(heading_error) > kTurnInPlaceRad) {
        speed = 0.0;
    }
    return speed;
}

}  // namespace ardurover_nav
