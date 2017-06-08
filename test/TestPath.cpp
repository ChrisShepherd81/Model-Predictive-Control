/*
 * TestPath.cpp
 *
 *  Created on: 08.06.2017
 *      Author: christian@inf-schaefer.de
 */

#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "../src/Path.h"

TEST(TestPath, TestPathConstruction)
{
  std::vector<double> testXValues = {1,2,3,4};
  std::vector<double> testYValues = {0,2,5,7};
  Path sut(testXValues, testYValues);
  EXPECT_THAT(sut.getXVector(), ::testing::ContainerEq(testXValues));
  EXPECT_THAT(sut.getYVector(), ::testing::ContainerEq(testYValues));
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST(TestPath, TestPathTranslation)
{
  std::vector<double> testXValues = {1,2,0,3};
  std::vector<double> testYValues = {1,2,1,4};
  Path sut(testXValues, testYValues);
  sut.translation(-1,-1);
  EXPECT_THAT(sut.getXVector(), ::testing::ContainerEq(std::vector<double>({0,1,-1,2})));
  EXPECT_THAT(sut.getYVector(), ::testing::ContainerEq(std::vector<double>({0,1,0,3})));
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TEST(TestPath, TestPathRotation)
{
  std::vector<double> testXValues = {0,1};
  std::vector<double> testYValues = {1,0};
  Path sut(testXValues, testYValues);
  sut.rotation(M_PI/2.0);
  std::vector<double> x = sut.getXVector();
  std::vector<double> y = sut.getYVector();

  ASSERT_NEAR(x[0], -1, 1e-6);
  ASSERT_NEAR(x[1], 0, 1e-6);

  ASSERT_NEAR(y[0], 0, 1e-6);
  ASSERT_NEAR(y[1], 1, 1e-6);
}

