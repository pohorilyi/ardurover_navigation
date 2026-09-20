#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include "ardurover_nav/path_follow.hpp"

using ardurover_nav::CuspKind;
using ardurover_nav::Waypoint;
using ardurover_nav::advance_progress;
using ardurover_nav::arrived_at_vertex;
using ardurover_nav::at_polyline_end;
using ardurover_nav::catch_up_progress;
using ardurover_nav::classify_cusp;
using ardurover_nav::heading_yaw_cmd;
using ardurover_nav::interpolate;
using ardurover_nav::is_cusp_at_vertex;
using ardurover_nav::is_hard_cusp;
using ardurover_nav::is_rollback_tail;
using ardurover_nav::kMaxYawRate;
using ardurover_nav::kPi;
using ardurover_nav::lookahead_waypoint;
using ardurover_nav::project_t;
using ardurover_nav::signed_cross_track;
using ardurover_nav::wrap;

namespace {

// Drive +x, same recorded yaw, then fold back ~1 m. Path 0/1 stop-rollback.
std::vector<Waypoint> rollback_end() {
    return {{0.0, 0.0, 0.0}, {5.0, 0.0, 0.0}, {4.0, 0.0, 0.0}};
}

// Same XY fold, recorded yaw flips. Real U-turn, then a long outbound.
std::vector<Waypoint> heading_turn() {
    return {{0.0, 0.0, 0.0}, {5.0, 0.0, 0.0}, {4.0, 0.0, kPi}, {20.0, 0.0, kPi}};
}

// Opposite travel, yaw only partly changes: mid-path wiggle, long tail.
std::vector<Waypoint> pass_wiggle() {
    return {{0.0, 0.0, 0.0}, {5.0, 0.0, 0.0}, {4.7, 0.0, 1.0}, {70.0, 0.0, 1.0}};
}

}  // namespace

TEST(Wrap, MapsNearPiToSmallError) {
    EXPECT_NEAR(wrap(kPi + 0.1), -kPi + 0.1, 1e-9);
    EXPECT_NEAR(wrap(-kPi - 0.1), kPi - 0.1, 1e-9);
    EXPECT_NEAR(std::abs(wrap(3.0 * kPi)), kPi, 1e-9);
}

TEST(ProjectT, OnSegmentAndPastEnd) {
    const Waypoint a{0.0, 0.0, 0.0};
    const Waypoint b{2.0, 0.0, 0.0};
    EXPECT_NEAR(project_t({1.0, 0.5, 0.0}, a, b), 0.5, 1e-9);
    EXPECT_GT(project_t({3.0, 0.0, 0.0}, a, b), 1.0);
    EXPECT_LT(project_t({-1.0, 0.0, 0.0}, a, b), 0.0);
}

TEST(ProjectT, DegenerateSampleCountsAsPast) {
    const Waypoint a{1.0, 2.0, 0.0};
    EXPECT_DOUBLE_EQ(project_t({0.0, 0.0, 0.0}, a, a), 1.0);
}

TEST(Cusp, RollbackIsEnd) {
    const auto path = rollback_end();
    ASSERT_TRUE(is_cusp_at_vertex(path, 1));
    EXPECT_EQ(classify_cusp(path, 1), CuspKind::End);
    EXPECT_TRUE(is_hard_cusp(path, 1));
    EXPECT_TRUE(is_rollback_tail(path, 1));
}

TEST(Cusp, YawFlipIsTurnNotEnd) {
    const auto path = heading_turn();
    ASSERT_TRUE(is_cusp_at_vertex(path, 1));
    EXPECT_EQ(classify_cusp(path, 1), CuspKind::Turn);
    EXPECT_TRUE(is_hard_cusp(path, 1));
    EXPECT_FALSE(is_rollback_tail(path, 1));
}

TEST(Cusp, MidPathWiggleIsPass) {
    const auto path = pass_wiggle();
    ASSERT_TRUE(is_cusp_at_vertex(path, 1));
    EXPECT_EQ(classify_cusp(path, 1), CuspKind::Pass);
    EXPECT_FALSE(is_hard_cusp(path, 1));
    EXPECT_FALSE(is_rollback_tail(path, 1));
}

TEST(Cusp, SmoothCornerIsNotACusp) {
    const std::vector<Waypoint> path{{0.0, 0.0, 0.0}, {5.0, 0.0, 0.0}, {5.0, 5.0, kPi / 2.0}};
    EXPECT_FALSE(is_cusp_at_vertex(path, 1));
    EXPECT_FALSE(is_hard_cusp(path, 1));
}

TEST(CatchUp, DoesNotCrossRollback) {
    const auto path = rollback_end();
    // On the outbound, nearer the overlapping reverse tail (the 136→156 snap).
    const Waypoint pose{3.0, 0.02, 0.0};
    EXPECT_EQ(catch_up_progress(pose, path, 0), 0u);
}

TEST(CatchUp, StepsPastPassFoldToCloserSegment) {
    const auto path = pass_wiggle();
    const Waypoint pose{30.0, 0.05, 0.0};
    EXPECT_EQ(catch_up_progress(pose, path, 0), 2u);
}

TEST(Lookahead, StopsAtHardCusp) {
    const auto path = rollback_end();
    const Waypoint carrot = lookahead_waypoint(path, 0, 0.8, 1.4);
    EXPECT_NEAR(carrot.x, 5.0, 1e-9);
    EXPECT_NEAR(carrot.y, 0.0, 1e-9);
}

TEST(Lookahead, WalksThroughPassFold) {
    const auto path = pass_wiggle();
    const Waypoint carrot = lookahead_waypoint(path, 0, 0.95, 1.4);
    EXPECT_GT(carrot.x, 4.7);
}

TEST(Lookahead, InterpolatesOnCurrentSegment) {
    const std::vector<Waypoint> path{{0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}};
    const Waypoint carrot = lookahead_waypoint(path, 0, 0.0, 1.4);
    EXPECT_NEAR(carrot.x, 1.4, 1e-9);
    EXPECT_NEAR(carrot.y, 0.0, 1e-9);
}

TEST(Finish, PolylineEndIsLastSegmentNotCrowFlies) {
    const auto path = rollback_end();
    EXPECT_FALSE(at_polyline_end(path, 0, 1.0));
    EXPECT_TRUE(at_polyline_end(path, 1, 1.0));
    EXPECT_TRUE(at_polyline_end(path, 2, 0.0));
}

TEST(Arrived, NearbyPastProjectionCounts) {
    const auto path = rollback_end();
    EXPECT_TRUE(arrived_at_vertex({5.1, 0.1, 0.0}, path, 0, 1));
}

TEST(Arrived, DistantCollinearProjectionDoesNotCount) {
    const auto path = rollback_end();
    EXPECT_FALSE(arrived_at_vertex({20.0, 0.0, 0.0}, path, 0, 1));
}

TEST(Arrived, ProgressAtOrPastVertexCounts) {
    const auto path = rollback_end();
    EXPECT_TRUE(arrived_at_vertex({0.0, 0.0, 0.0}, path, 1, 1));
}

TEST(CrossTrack, LeftIsPositive) {
    const std::vector<Waypoint> path{{0.0, 0.0, 0.0}, {2.0, 0.0, 0.0}};
    EXPECT_NEAR(signed_cross_track({1.0, 0.4, 0.0}, path, 0), 0.4, 1e-9);
    EXPECT_NEAR(signed_cross_track({1.0, -0.4, 0.0}, path, 0), -0.4, 1e-9);
}

TEST(Interpolate, YawTakesShortArc) {
    const Waypoint a{0.0, 0.0, 3.0};
    const Waypoint b{0.0, 0.0, -3.0};
    const Waypoint mid = interpolate(a, b, 0.5);
    EXPECT_NEAR(std::abs(mid.yaw), kPi, 0.2);
}

TEST(HeadingCmd, SaturatesLargeError) {
    EXPECT_NEAR(heading_yaw_cmd(2.0, 0.0), kMaxYawRate, 1e-9);
    EXPECT_NEAR(heading_yaw_cmd(-2.0, 0.0), -kMaxYawRate, 1e-9);
}

TEST(AdvanceProgress, StepsWhenProjectionIsPast) {
    const std::vector<Waypoint> path{{0.0, 0.0, 0.0}, {2.0, 0.0, 0.0}, {4.0, 0.0, 0.0}};
    EXPECT_EQ(advance_progress({2.2, 0.0, 0.0}, path, 0), 1u);
}

TEST(AdvanceProgress, DoesNotSnapToOverlappingRollback) {
    const auto path = rollback_end();
    EXPECT_EQ(advance_progress({3.0, 0.02, 0.0}, path, 0), 0u);
}
