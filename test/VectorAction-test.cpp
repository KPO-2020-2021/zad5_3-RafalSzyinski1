//
// Created by rafik on 21.04.2021.
//

#include "gtest/gtest.h"
#include "VectorAction.h"

#include <sstream>

using namespace VectorAction;

TEST (VectorActionTest, multiScalar)
{
    double a = 3;
    std::vector<double> b{1, 2, 3, 4};

    EXPECT_EQ(b * a, std::vector<double>({3, 6, 9, 12}));
}

TEST (VectorActionTest, scalarProduct)
{
    std::vector<double> a{1, 2, 3};
    std::vector<double> b{1, 2, 3};
    std::vector<double> c(1);

    EXPECT_DOUBLE_EQ(a * b, 14);
    EXPECT_THROW(a * c, std::out_of_range);
}

TEST (VectorActionTest, sum)
{
    std::vector<double> a{1, 2, 3, 4};
    std::vector<double> b{2, 4, 8, 16};
    std::vector<double> c(1);

    EXPECT_EQ(a + b, std::vector<double>({3, 6, 11, 20}));
    EXPECT_THROW(a + c, std::out_of_range);
}

TEST (VectorActionTest, diff)
{
    std::vector<double> a{1, 2, 3, 4};
    std::vector<double> b{2, 4, 8, 16};
    std::vector<double> c(1);

    EXPECT_EQ(b - a, std::vector<double>({1, 2, 5, 12}));
    EXPECT_THROW(a - c, std::out_of_range);
}

TEST (VectorActionTest, negation)
{
    std::vector<double> a{1, 2, 3};
    EXPECT_EQ(-a, std::vector<double>({-1, -2, -3}));
}

TEST (VectorActionTest, div)
{
    std::vector<double> a{4, 8, 16, 32};
    EXPECT_EQ(a / 4.0, std::vector<double>({1, 2, 4, 8}));
    EXPECT_THROW(a / 0.0, std::invalid_argument);
}

TEST (VectorActionTest, abs)
{
    std::vector<double> a{3, 4};
    EXPECT_EQ(abs(a), 5.0);

    std::vector<double> b{1.123, 3.321};
    EXPECT_NEAR(abs(b), 3.50573, 0.001);
}

TEST (VectorActionTest, ostream)
{
    std::vector<int> a{1, 2, 3, 4, 5};
    std::stringstream ss;
    ss << a;
    EXPECT_EQ(ss.str(), "1,2,3,4,5");

    std::vector<double> b{1.1, 2.22, 3.333, 4.4444};
    ss = std::stringstream();
    ss << b;
    EXPECT_EQ(ss.str(), "1.1000000000,2.2200000000,3.3330000000,4.4444000000");
}