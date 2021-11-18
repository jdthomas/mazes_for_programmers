#include <fmt/ranges.h>
#include <gtest/gtest.h>

#include <vector>

#include "maze.h"

namespace {
bool is_perfect_maze(jt::maze::Grid &grid) {
  return true;
}
};

TEST(mazeTest, TestBinaryTreeGen) {
  jt::maze::Grid grid{50,50};
  binary_tree_maze(grid);

  EXPECT_TRUE(is_perfect_maze(grid));
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
