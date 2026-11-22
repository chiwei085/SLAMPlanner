#include <gtest/gtest.h>

#include "yahboom_alc_bringup/planner_event.hpp"

namespace yahboom_alc_bringup
{
namespace
{

TEST(PlannerEvent, AlcStartedRequestsPause) {
    EXPECT_EQ(
        classifyPlannerEvent(R"({"event":"alc_started","phase":"navigation"})"),
        CoordinationAction::kPause);
}

TEST(PlannerEvent, AlcFinishedRequestsResume) {
    EXPECT_EQ(
        classifyPlannerEvent(R"({"event":"alc_finished","success":true})"),
        CoordinationAction::kResume);
}

TEST(PlannerEvent, UnrelatedEventsAreIgnored) {
    EXPECT_EQ(classifyPlannerEvent(R"({"event":"goal_sent"})"),
              CoordinationAction::kNone);
    EXPECT_EQ(classifyPlannerEvent(R"({"event":"frontier_paused_for_alc"})"),
              CoordinationAction::kNone);
}

TEST(PlannerEvent, MissingEventFieldIsIgnored) {
    EXPECT_EQ(classifyPlannerEvent(R"({"reason":"timeout"})"),
              CoordinationAction::kNone);
}

TEST(PlannerEvent, ParsesFailureClassWhenPresent) {
    const auto failure = parseFailureClass(
        R"({"event":"route_abandoned","failure_class":"local_hazard"})");

    ASSERT_TRUE(failure);
    EXPECT_EQ(failure.value_or(""), "local_hazard");
}

TEST(PlannerEvent, MalformedFailureClassPayloadIsIgnored) {
    EXPECT_FALSE(parseFailureClass(R"({"event":"route_abandoned","bad":-})"));
    EXPECT_FALSE(parseFailureClass(R"({"event":"route_abandoned"})"));
}

TEST(PlannerEvent, MalformedPayloadIsIgnored) {
    EXPECT_EQ(classifyPlannerEvent("not json"), CoordinationAction::kNone);
    EXPECT_EQ(classifyPlannerEvent(""), CoordinationAction::kNone);
}

TEST(PlannerEvent, GateTransitionsOnceOnPause) {
    AlcCoordinationGate gate;
    EXPECT_FALSE(gate.paused());
    EXPECT_TRUE(gate.applyPause());
    EXPECT_TRUE(gate.paused());
    EXPECT_FALSE(gate.applyPause());
    EXPECT_TRUE(gate.paused());
}

TEST(PlannerEvent, GateTransitionsOnceOnResume) {
    AlcCoordinationGate gate;
    EXPECT_FALSE(gate.applyResume());
    ASSERT_TRUE(gate.applyPause());
    EXPECT_TRUE(gate.applyResume());
    EXPECT_FALSE(gate.paused());
    EXPECT_FALSE(gate.applyResume());
}

TEST(PlannerEvent, MalformedNumberPayloadIsIgnored) {
    EXPECT_EQ(classifyPlannerEvent(R"({"event":"alc_started","bad":-})"),
              CoordinationAction::kNone);
}

TEST(PlannerEvent, ReverseBlockedSeparatesBlindFromBlocked) {
    const auto stale = parsePlannerEvent(
        R"({"event":"reverse_blocked","reason":"stale_costmap"})");
    EXPECT_EQ(stale.observation, PlannerObservation::kReverseUnverified);
    EXPECT_EQ(stale.reason, "stale_costmap");
    EXPECT_EQ(parsePlannerEvent(
                  R"({"event":"reverse_blocked","reason":"tf_unavailable"})")
                  .observation,
              PlannerObservation::kReverseUnverified);
    EXPECT_EQ(parsePlannerEvent(
                  R"({"event":"reverse_blocked","reason":"rear_blocked"})")
                  .observation,
              PlannerObservation::kReverseBlocked);
    EXPECT_EQ(parsePlannerEvent(R"({"event":"reverse_blocked"})").observation,
              PlannerObservation::kReverseBlocked);
}

TEST(PlannerEvent, ParsesStallAndAlcObservations) {
    EXPECT_EQ(parsePlannerEvent(R"({"event":"stall_or_slip"})").observation,
              PlannerObservation::kForwardStall);
    EXPECT_EQ(parsePlannerEvent(R"({"event":"alc_started"})").observation,
              PlannerObservation::kAlcStarted);
    EXPECT_EQ(parsePlannerEvent(R"({"event":"alc_finished"})").observation,
              PlannerObservation::kAlcFinished);
    EXPECT_EQ(parsePlannerEvent(R"({"event":"goal_sent"})").observation,
              PlannerObservation::kNone);
    EXPECT_EQ(parsePlannerEvent("not json").observation,
              PlannerObservation::kNone);
}

}  // namespace
}  // namespace yahboom_alc_bringup
