#include "gtest/gtest.h"
#include "Rectangular.h"

TEST (RectangularTest, equalOperator)
{
    Rectangular a({1, 2, 3}, {3, 2, 1}, {5, 6, 7});
    EXPECT_TRUE(a == Rectangular({1, 2, 3}, {3, 2, 1}, {5, 6, 7}));

    Rectangular b({1.000000000001, 2.0000000000002, 3.00000000000000000003}, {4, 5, 6}, {7, 8, 9});
    EXPECT_TRUE(b == Rectangular({1, 2, 3}, {4, 5, 6}, {7, 8, 9}));
}

TEST (RectangularTest, constructor)
{
    Rectangular a({1, 2, 3}, {3, 2, 1}, {5, 6, 7});
    EXPECT_EQ(a, Rectangular({1, 2, 3}, {3, 2, 1}, {5, 6, 7}));

    EXPECT_THROW(Rectangular({1, 2}, {1, 3, 4}, {5, 6, 7}, {1, 2}), std::invalid_argument);
}

TEST (RectangularTest, recMultiMat)
{
    Rectangular a({1, 0, 0}, {0, 1, 0}, {0, 0, 1});
    matrix<double> mat = {{1, 0, 0}, {0, std::cos(M_PI/180 * 90), -std::sin(M_PI/180 * 90)}, {0, std::sin(M_PI/180 * 90), std::cos(M_PI/180 * 90)}};

    EXPECT_EQ(a * mat, Rectangular({1, 0, 0}, {0, -1, 0}, {0, 0, -1}));
}

TEST (RectangularTest, recPlusVec)
{
    Rectangular a({1, 0, 0}, {0, 1, 0}, {0, 0, 1});
    std::vector<double> vec = {1, 1, 1};
    EXPECT_EQ(a + vec, Rectangular({1, 0, 0}, {0, 1, 0}, {0, 0, 1}, {1, 1, 1}));
}

TEST (RectangularTest, indexOperators)
{
    Rectangular a({1, 0, 0}, {0, 1, 0}, {0, 0, 1});
    EXPECT_EQ(a.x(), std::vector<double>({1, 0, 0}));
    EXPECT_EQ(a.y(), std::vector<double>({0, 1, 0}));
    EXPECT_EQ(a.z(), std::vector<double>({0, 0, 1}));
    EXPECT_EQ(a.startPoint(), std::vector<double>({0, 0, 0}));
    EXPECT_EQ(a.centerOfMass(), std::vector<double>({0.5, 0.5, 0.5}));
}