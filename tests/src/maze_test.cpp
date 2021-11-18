#include <fmt/ranges.h>
#include <gtest/gtest.h>

#include <vector>

#include "maze.h"

namespace {
bool is_perfect_maze(jt::maze::Grid &grid) {
  // Just checks that every cell is accessible
  auto d = dijkstra_distances(grid, {0, 0});
  auto min_distance = *std::min_element(d.begin(), d.end());
  return min_distance != 0;
}
}; // namespace

TEST(mazeTest, TestInaccessibleMaze) {
  jt::maze::Grid grid{50, 50};
  EXPECT_FALSE(is_perfect_maze(grid));
}

TEST(mazeTest, TestBinaryTreeGen) {
  jt::maze::Grid grid{50, 50};
  binary_tree_maze(grid);

  EXPECT_TRUE(is_perfect_maze(grid));
}

TEST(mazeTest, TestSidewinderGen) {
  jt::maze::Grid grid{50, 50};
  sidewinder_maze(grid);

  EXPECT_TRUE(is_perfect_maze(grid));
}

TEST(mazeTest, TestAldousBroderGen) {
  jt::maze::Grid grid{50, 50};
  random_walk_Aldous_Broder_maze(grid);

  EXPECT_TRUE(is_perfect_maze(grid));
}

TEST(mazeTest, TestWilsonGen) {
  jt::maze::Grid grid{50, 50};
  random_walk_Wilson_maze(grid);

  EXPECT_TRUE(is_perfect_maze(grid));
}

TEST(mazeTest, TestHuntAndKillGen) {
  jt::maze::Grid grid{50, 50};
  hunt_and_kill_maze(grid);

  EXPECT_TRUE(is_perfect_maze(grid));
}

TEST(mazeTest, TestRecursiveBacktrackingGen) {
  jt::maze::Grid grid{50, 50};
  recursive_backtracking_maze(grid);

  EXPECT_TRUE(is_perfect_maze(grid));
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
