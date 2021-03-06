/*
 * TestPath.cpp
 *
 *  Created on: 08.06.2017
 *      Author: christian@inf-schaefer.de
 */

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "../src/Points.h"

TEST(TestPath, TestPointsConstruction)
{
  std::vector<double> testXValues = {1,2,3,4};
  std::vector<double> testYValues = {0,2,5,7};
  Points sut(testXValues, testYValues);
  EXPECT_THAT(sut.getXStdVector(), ::testing::ContainerEq(testXValues));
  EXPECT_THAT(sut.getYStdVector(), ::testing::ContainerEq(testYValues));
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST(TestPath, TestPointsTranslation)
{
  std::vector<double> testXValues = {1,2,0,3};
  std::vector<double> testYValues = {1,2,1,4};
  Points sut(testXValues, testYValues);
  sut.translation(-1,-1);
  EXPECT_THAT(sut.getXStdVector(), ::testing::ContainerEq(std::vector<double>({0,1,-1,2})));
  EXPECT_THAT(sut.getYStdVector(), ::testing::ContainerEq(std::vector<double>({0,1,0,3})));
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST(TestPath, TestPointsRotation)
{
  std::vector<double> testXValues = {0,1};
  std::vector<double> testYValues = {1,0};
  Points sut(testXValues, testYValues);
  sut.rotation(M_PI/2.0);
  std::vector<double> x = sut.getXStdVector();
  std::vector<double> y = sut.getYStdVector();

  ASSERT_NEAR(x[0], -1, 1e-6);
  ASSERT_NEAR(x[1], 0, 1e-6);

  ASSERT_NEAR(y[0], 0, 1e-6);
  ASSERT_NEAR(y[1], 1, 1e-6);
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST(TestPath, TestPointsCarSpaceTransformation)
{
  std::vector<double> testXValues = {-32.16173,-43.49173,-61.09,-78.29172,-93.05002,-107.7717};
  std::vector<double> testYValues = {113.361,105.941,92.88499,78.73102,65.34102,50.57938};
  Points sut(testXValues, testYValues);
 // sut.rotation(-3.733651);
  //sut.translation(40.62, -108.73);
  sut.translation(-testXValues[0], -testYValues[0]);
  sut.rotation(-3.733651);

  std::cout << "X: ";
  for(size_t i=0; i < sut.getXVector().size(); ++i)
    std::cout << sut.getXVector()[i] << ",";
  std::cout << std::endl;

  std::cout << "Y: ";
  for(size_t i=0; i < sut.getYVector().size(); ++i)
    std::cout << sut.getYVector()[i] << ",";
  std::cout << std::endl;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST(TestPath, TestPointsAccessor)
{
  std::vector<double> testXValues = {-42};
  std::vector<double> testYValues = {42};
  Points sut(testXValues, testYValues);

  ASSERT_NEAR(sut[0][0], -42, 1e-6);
  ASSERT_NEAR(sut[0][1], 42, 1e-6);
}

