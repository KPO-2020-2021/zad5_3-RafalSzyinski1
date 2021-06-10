#include "gtest/gtest.h"
#include "MatrixAction.h"

#define _USE_MATH_DEFINES

#include <sstream>
#include <cmath>

using namespace MatrixAction;

TEST (MatrixActionTest, sumOperator)
{
    matrix<int> a = {{1, 2}, {3, 4}};
    matrix<int> b = {{2, 4}, {8, 16}};
    matrix<int> c = {{1}, {2}};
    matrix<int> d;

    EXPECT_EQ(a + b, matrix<int>({{3, 6}, {11, 20}}));
    EXPECT_THROW(a + c, std::out_of_range);
    EXPECT_THROW(a + d, std::out_of_range);
}

TEST (MatrixActionTest, diffOperator)
{
    matrix<int> a = {{1, 2}, {3, 4}};
    matrix<int> b = {{2, 4}, {8, 16}};
    matrix<int> c = {{1}, {2}};
    matrix<int> d;

    EXPECT_EQ(b - a, matrix<int>({{1, 2}, {5, 12}}));
    EXPECT_THROW(a - c, std::out_of_range);
    EXPECT_THROW(a - d, std::out_of_range);
}

TEST (MatrixActionTest, multiScalar)
{
    matrix<double> a = {{1, 2}, {3, 4}};
    double b = 3;

    EXPECT_EQ(a * b, matrix<double>({{3, 6}, {9, 12}}));
}

TEST (MatrixActionTest, multiVec)
{
    matrix<double> a = {{1, 2}, {3, 4}};
    std::vector<double> b = {5, 6};
    std::vector<double> c;
    matrix<double> d = {{1, 3}, {2}};

    EXPECT_EQ(a * b, std::vector<double>({17, 39}));
    EXPECT_EQ(b * a, std::vector<double>({23, 34}));
    EXPECT_THROW(a * c, std::out_of_range);
    EXPECT_THROW(d * b, std::out_of_range);
    EXPECT_THROW(b * d, std::out_of_range);
}

TEST (MatrixActionTest, multiMat)
{
    matrix<double> a = {{1, 2}, {3, 4}};
    matrix<double> b = {{2, 4}, {8, 16}};

    matrix<double> c = {{1, 2}, {1}};
    matrix<double> d;

    EXPECT_EQ(a * b, matrix<double>({{18, 36}, {38, 76}}));
    EXPECT_THROW(a * c, std::out_of_range);
    EXPECT_THROW(a * d, std::out_of_range);

    matrix<double> e = {{1, 2, 3}, {4, 5, 6}};
    matrix<double> f = {{8, 9}, {10, 11}, {12, 13}};

    EXPECT_EQ(e * f, matrix<double>({{64, 70}, {154, 169}}));


    matrix<double> g = {{std::cos(M_PI/180 * 50), -std::sin(M_PI/180 * 50)}, {std::sin(M_PI/180 * 50), std::cos(M_PI/180 * 50)}};
    matrix<double> h = {{std::cos(M_PI/180 * -50), -std::sin(M_PI/180 * -50)}, {std::sin(M_PI/180 * -50), std::cos(M_PI/180 * -50)}};
    EXPECT_EQ(g * h, matrix<double>({{1, 0}, {0, 1}}));
}

TEST (MatrixActionTest, ostream)
{
    matrix<int> a = {{1, 2}, {3, 4}};
    std::stringstream ss;
    ss << a;
    EXPECT_EQ(ss.str(), "1,2\n3,4\n");

    matrix<double> b = {{1.1, 2.2}, {3.3, 4.4}};
    ss = std::stringstream();
    ss << b;
    EXPECT_EQ(ss.str(), "1.1000000000,2.2000000000\n3.3000000000,4.4000000000\n");
}
