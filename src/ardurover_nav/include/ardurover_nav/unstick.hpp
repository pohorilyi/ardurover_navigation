#pragma once

namespace ardurover_nav {

// Boardwalk hang. All times below are 20 Hz ticks.
// Order: detect → Reverse → Drive → resume the mode we interrupted (or Reverse again).
// Leaving on reverse-distance alone rams the same lip (path 2 i=91).
// GUIDED ramp-up also looks like act≈0 — wait until we have actually rolled.

// --- detect (MaybeEnterUnstick) ---
constexpr int kStuckDetectTicks = 30;       // 1.5 s still commanding and frozen
constexpr int kUnstickCooldownTicks = 30;   // 1.5 s after LeaveUnstick before another detect
constexpr double kStuckCmdVx = 0.20;        // |commanded vx| above this counts as "asking to move"
constexpr double kStuckCmdWz = 0.15;        // |commanded wz| above this counts as "asking to move"
constexpr double kStuckActVx = 0.04;        // measured vx below this counts as frozen; 0.06 is still a slide
constexpr double kStuckActWz = 0.04;        // measured wz below this counts as frozen
constexpr double kStuckPoseM = 0.04;        // drift past this resets the 1.5 s counter
constexpr double kSeenMotionVel = 0.20;     // latch seenMotion_; hypot(vx, wz) cutoff, not a real speed

// --- Reverse phase (TickUnstickReverse) ---
constexpr int kUnstickWiggleTicks = 8;      // 0.4 s per yaw side (left, then right)
constexpr int kUnstickReverseTicks = 40;    // 2.0 s minimum: two left/right cycles before "unhooked" is allowed
constexpr int kUnstickReverseMaxTicks = 80; // 4.0 s: go to Drive even if 0.30 m was missed
constexpr double kUnstickReverseM = 0.30;   // backup distance required after the wiggle
constexpr double kUnstickReverseSpeed = 0.45;

// --- Drive phase (TickUnstickDrive) ---
constexpr int kUnstickMinTicks = 8;         // 0.4 s before any Drive exit
constexpr int kUnstickDriveTicks = 40;      // 2.0 s: open the "still hung?" exits
constexpr int kUnstickMaxAttempts = 4;      // reverse→drive cycles, then resume anyway
constexpr double kUnstickDriveM = 0.40;     // happy-path exit: forward travel
constexpr double kUnstickClearHeadingRad = 0.40;  // ~23 deg: carrot roughly ahead
constexpr double kUnstickDriveSpeed = 0.55;
constexpr double kTiltRad = 0.10;           // ~6 deg: chassis on a lip (roll/pitch)

}  // namespace ardurover_nav
