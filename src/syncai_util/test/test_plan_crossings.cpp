// The plan_crossings sidecar's format (plan_crossings.hpp): what the planner
// writes is what the followers read, and a message they cannot read is
// refused whole, never half-applied.

#include <gtest/gtest.h>

#include <string>

#include "syncai_util/plan_crossings.hpp"

using syncai_util::fromJson;
using syncai_util::PlanCrossing;
using syncai_util::PlanCrossings;
using syncai_util::toJson;

TEST(PlanCrossings, WhatIsWrittenIsReadBack)
{
  PlanCrossings c;
  c.known = true;
  c.stamp_sec = 1793603729;
  c.stamp_nanosec = 894962360;
  c.frame_id = "map";
  c.poses = 532;
  PlanCrossing s;
  s.kind = "stair";
  s.stair_gait = true;
  s.evidence = "track";
  s.rise = 5.4;
  s.first = 120;
  s.last = 341;
  s.turns = {200, 260};
  s.pts = {{1.0, 2.0, -0.4}, {1.1, 2.0, -0.3}};
  PlanCrossing r;
  r.kind = "ramp";
  r.evidence = "track";
  r.rise = 0.2;
  r.first = 400;
  r.last = 412;
  c.crossings = {s, r};

  PlanCrossings back;
  std::string error;
  ASSERT_TRUE(fromJson(toJson(c), back, error)) << error;
  EXPECT_TRUE(back.known);
  EXPECT_TRUE(back.isFor(1793603729, 894962360, 532));
  EXPECT_FALSE(back.isFor(1793603729, 894962360, 531));
  EXPECT_EQ(back.frame_id, "map");
  ASSERT_EQ(back.crossings.size(), 2u);
  EXPECT_EQ(back.crossings[0].kind, "stair");
  EXPECT_TRUE(back.crossings[0].stair_gait);
  EXPECT_EQ(back.crossings[0].first, 120u);
  EXPECT_EQ(back.crossings[0].last, 341u);
  ASSERT_EQ(back.crossings[0].turns.size(), 2u);
  EXPECT_EQ(back.crossings[0].turns[1], 260u);
  ASSERT_EQ(back.crossings[0].pts.size(), 2u);
  EXPECT_DOUBLE_EQ(back.crossings[0].pts[1].z, -0.3);
  EXPECT_EQ(back.crossings[1].kind, "ramp");
  EXPECT_FALSE(back.crossings[1].stair_gait);
  EXPECT_DOUBLE_EQ(back.crossings[1].rise, 0.2);
}

TEST(PlanCrossings, UnknownAppliesToNoPlanAndEmptyIsStillAnAnswer)
{
  // known: false -- a planner that reports nothing -- is for no plan, so a
  // follower keeps its own judgement; known with no crossings is an answer.
  PlanCrossings unknown;
  PlanCrossings back;
  std::string error;
  ASSERT_TRUE(fromJson(toJson(unknown), back, error)) << error;
  EXPECT_FALSE(back.known);
  EXPECT_FALSE(back.isFor(0, 0, 0));

  PlanCrossings none;
  none.known = true;
  none.stamp_sec = 7;
  none.poses = 10;
  ASSERT_TRUE(fromJson(toJson(none), back, error)) << error;
  EXPECT_TRUE(back.isFor(7, 0, 10));
  EXPECT_TRUE(back.crossings.empty());
}

TEST(PlanCrossings, AMessageItCannotReadLeavesTheLastOneAlone)
{
  PlanCrossings keep;
  keep.known = true;
  keep.poses = 3;
  std::string error;
  for (
    const std::string & bad :
    {std::string("not json"), std::string("[]"), std::string(R"({"version": 2})"),
     std::string(R"({"version": 1, "known": true})"),
     std::string(
       R"({"version": 1, "known": true, "crossings": [{"kind": "stair", "gait": "up", "first": 1, "last": 2}]})"),
     std::string(
       R"({"version": 1, "known": true, "crossings": [{"kind": "stair", "gait": "stair", "first": 5, "last": 2}]})"),
     std::string(
       R"({"version": 1, "known": true, "crossings": [{"kind": "ramp", "gait": "flat", "first": 1, "last": 2, "pts": [[1, 2]]}]})")}) {
    error.clear();
    EXPECT_FALSE(fromJson(bad, keep, error)) << bad;
    EXPECT_FALSE(error.empty()) << bad;
    EXPECT_EQ(keep.poses, 3u) << bad;
  }
}
