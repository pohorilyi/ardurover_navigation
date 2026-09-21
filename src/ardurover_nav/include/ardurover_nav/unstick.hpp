#pragma once

namespace ardurover_nav {

// Boardwalk hang: reverse to unhook, then a forward+yaw push while the bumper
// is free. Leaving on reverse-distance alone rams the same lip (path 2 i=91).
// GUIDED ramp-up also looks like act≈0 — wait until we have actually rolled.
constexpr int kStuckDetectTicks = 30;       // 1.5 s still commanding and frozen
constexpr int kUnstickMinTicks = 8;         // 0.4 s before a phase may end
constexpr int kUnstickWiggleTicks = 8;      // 0.4 s per reverse yaw side (left then right)
constexpr int kUnstickReverseTicks = 40;    // 2.0 s: at least two left/right wiggle cycles
constexpr int kUnstickReverseMaxTicks = 80; // 4.0 s: leave reverse even if unhook distance missed
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
constexpr double kTiltRad = 0.10;           // ~6 deg: chassis on a lip (roll/pitch)

}  // namespace ardurover_nav
