#include "gtest/gtest.h"
#include "Transform.h"
#include "Cuboid.h"

TEST (TransformTest, rotateAroundCenterOfMass)
{
    Cuboid a({1, 0, 0}, {0, 1, 0}, {0, 0, 1}, {-0.5, -0.5, -0.5});
    Transform::rotateAroundCenterOfMass(a, 90);
    EXPECT_EQ(a, Cuboid({0, 1, 0}, {-1, 0, 0}, {0, 0, 1}, {0.5, -0.5, -0.5}));

    Cuboid b({1, 0, 0}, {0, 1, 0}, {0, 0, 1}, {1, 1, 1});
    Transform::rotateAroundCenterOfMass(b, 180);
    EXPECT_EQ(b, Cuboid({-1, 0, 0}, {0, -1, 0}, {0, 0, 1}, {2, 2, 1}));
}

TEST (TransformTest, rotateAroundPoint)
{
    Cuboid a({1, 0, 0}, {0, 1, 0}, {0, 0, 1}, {1, 1, 1});
    Transform::rotateAroundPoint(a, 180, {1.5, 1.5, 1.5});
    EXPECT_EQ(a, Cuboid({-1, 0, 0}, {0, -1, 0}, {0, 0, 1}, {2, 2, 1}));

    Cuboid b({1, 0, 0}, {0, 1, 0}, {0, 0, 1}, {1, 1, 1});
    Transform::rotateAroundPoint(b, 90);
    EXPECT_EQ(b, Cuboid({0, 1, 0}, {-1, 0, 0}, {0, 0, 1}, {-1, 1, 1}));
}

TEST (TransformTest, scaleToCenterOfMass)
{
    Cuboid a({1, 0, 0}, {0, 1, 0}, {0, 0, 1}, {-0.5, -0.5, -0.5});
    Transform::scaleToCenterOfMass(a, 2);
    EXPECT_EQ(a, Cuboid({2, 0, 0}, {0, 2, 0}, {0, 0, 2}, {-1, -1, -1}));

    Cuboid b({1, 0, 0}, {0, 1, 0}, {0, 0, 1}, {1, 1, 1});
    Transform::scaleToCenterOfMass(b, 0.5);
    EXPECT_EQ(b, Cuboid({0.5, 0, 0}, {0, 0.5, 0}, {0, 0, 0.5}, {1.25, 1.25, 1.25}));
}

TEST (TransformTest, translate)
{
    Cuboid a({1, 0, 0}, {0, 1, 0}, {0, 0, 1}, {-0.5, -0.5, -0.5});
    Transform::translate(a, std::vector<double>({2, 2, 2}));
    EXPECT_EQ(a, Cuboid({1, 0, 0}, {0, 1, 0}, {0, 0, 1}, {1.5, 1.5, 1.5}));
}
