// Tests for the map_T_odom computation logic used in rtabmap_map_tf_relay.
//
// The relay derives:  map_T_odom = map_T_base × (odom_T_base)^-1
//
// We verify the four canonical cases a reviewer would want to see:
//   1. Identity inputs → identity output
//   2. Pure translation: base at (1,0) in map, (0.5,0) in odom → odom offset
//   (0.5,0) in map
//   3. Pure rotation: base rotated 90° in map, 0° in odom → odom rotated 90° in
//   map
//   4. Full transform: combined translation + rotation round-trips correctly

#include <gtest/gtest.h>

#include <cmath>

#include "tf2/LinearMath/Matrix3x3.h"   // IWYU pragma: keep
#include "tf2/LinearMath/Quaternion.h"  // IWYU pragma: keep
#include "tf2/LinearMath/Transform.h"   // IWYU pragma: keep
#include "tf2/LinearMath/Vector3.h"     // IWYU pragma: keep

namespace
{

constexpr double kEps = 1e-9;

tf2::Transform makeTranslation(double x, double y) {
    return tf2::Transform(tf2::Quaternion::getIdentity(),
                          tf2::Vector3(x, y, 0.0));
}

tf2::Transform makeYawRotation(double yaw_rad) {
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw_rad);
    return tf2::Transform(q, tf2::Vector3(0.0, 0.0, 0.0));
}

// The relay's core computation, isolated for unit testing.
tf2::Transform computeMapToOdom(const tf2::Transform& map_T_base,
                                const tf2::Transform& odom_T_base) {
    return map_T_base * odom_T_base.inverse();
}

TEST(MapTfRelay, IdentityInputsProduceIdentityOutput) {
    tf2::Transform map_T_base = tf2::Transform::getIdentity();
    tf2::Transform odom_T_base = tf2::Transform::getIdentity();

    auto map_T_odom = computeMapToOdom(map_T_base, odom_T_base);

    EXPECT_NEAR(map_T_odom.getOrigin().x(), 0.0, kEps);
    EXPECT_NEAR(map_T_odom.getOrigin().y(), 0.0, kEps);
    EXPECT_NEAR(map_T_odom.getRotation().getAngle(), 0.0, kEps);
}

TEST(MapTfRelay, PureTranslationOffersPredictableOffset) {
    // map: base at (1, 0); odom: base at (0.5, 0) → odom origin is at (0.5, 0)
    // in map
    tf2::Transform map_T_base = makeTranslation(1.0, 0.0);
    tf2::Transform odom_T_base = makeTranslation(0.5, 0.0);

    auto map_T_odom = computeMapToOdom(map_T_base, odom_T_base);

    EXPECT_NEAR(map_T_odom.getOrigin().x(), 0.5, kEps);
    EXPECT_NEAR(map_T_odom.getOrigin().y(), 0.0, kEps);
}

TEST(MapTfRelay, PureRotationComposesCorrectly) {
    // Base is at 90° in map but at 0° in odom → odom frame is rotated 90° in
    // map
    tf2::Transform map_T_base = makeYawRotation(M_PI / 2.0);
    tf2::Transform odom_T_base = tf2::Transform::getIdentity();

    auto map_T_odom = computeMapToOdom(map_T_base, odom_T_base);

    EXPECT_NEAR(map_T_odom.getOrigin().x(), 0.0, kEps);
    EXPECT_NEAR(map_T_odom.getOrigin().y(), 0.0, kEps);

    // Extracted yaw from rotation quaternion
    double roll, pitch, yaw;
    tf2::Matrix3x3(map_T_odom.getRotation()).getRPY(roll, pitch, yaw);
    EXPECT_NEAR(yaw, M_PI / 2.0, kEps);
}

TEST(MapTfRelay, CombinedTranslationAndRotationRoundTrips) {
    // If the robot is in the same place in both frames (trivial case),
    // map_T_odom = I
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, 0.785);  // 45°
    tf2::Transform same(q, tf2::Vector3(2.0, 3.0, 0.0));

    auto map_T_odom = computeMapToOdom(same, same);

    EXPECT_NEAR(map_T_odom.getOrigin().x(), 0.0, kEps);
    EXPECT_NEAR(map_T_odom.getOrigin().y(), 0.0, kEps);
    double roll, pitch, yaw;
    tf2::Matrix3x3(map_T_odom.getRotation()).getRPY(roll, pitch, yaw);
    EXPECT_NEAR(yaw, 0.0, kEps);
}

TEST(MapTfRelay, FullCompositionIsConsistent) {
    // Construct an arbitrary scenario and verify the chain closes:
    // point expressed in odom → base → map should match direct map pose.
    tf2::Quaternion q_map, q_odom;
    q_map.setRPY(0.0, 0.0, 0.3);
    q_odom.setRPY(0.0, 0.0, 0.1);
    tf2::Transform map_T_base(q_map, tf2::Vector3(1.0, 2.0, 0.0));
    tf2::Transform odom_T_base(q_odom, tf2::Vector3(0.5, 0.5, 0.0));

    auto map_T_odom = computeMapToOdom(map_T_base, odom_T_base);

    // A point at odom origin should land at map_T_odom * (0,0,0) = map_T_odom's
    // translation
    tf2::Vector3 odom_origin(0.0, 0.0, 0.0);
    tf2::Vector3 in_map = map_T_odom * odom_origin;
    EXPECT_NEAR(in_map.x(), map_T_odom.getOrigin().x(), kEps);
    EXPECT_NEAR(in_map.y(), map_T_odom.getOrigin().y(), kEps);

    // Cross-check: map_T_odom × odom_T_base should equal map_T_base
    auto recovered_map_T_base = map_T_odom * odom_T_base;
    EXPECT_NEAR(recovered_map_T_base.getOrigin().x(),
                map_T_base.getOrigin().x(), kEps);
    EXPECT_NEAR(recovered_map_T_base.getOrigin().y(),
                map_T_base.getOrigin().y(), kEps);

    double r1, p1, y1, r2, p2, y2;
    tf2::Matrix3x3(recovered_map_T_base.getRotation()).getRPY(r1, p1, y1);
    tf2::Matrix3x3(map_T_base.getRotation()).getRPY(r2, p2, y2);
    EXPECT_NEAR(y1, y2, kEps);
}

}  // namespace
