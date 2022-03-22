
namespace jt::maze {

class Grid;

// Genertion Algorithms
void binary_tree_maze(Grid &grid);
void binary_tree_maze_p(Grid &grid);
void binary_tree_maze_p2(Grid &grid);
void sidewinder_maze(Grid &grid);
void sidewinder_maze_p(Grid &grid);
void random_walk_Aldous_Broder_maze(Grid &grid);
void random_walk_Wilson_maze(Grid &grid);
void hunt_and_kill_maze(Grid &grid);
void recursive_backtracking_maze(Grid &grid);
void kruskel_maze(Grid &grid);
void prims_maze(Grid &grid);
void grow_sample_maze(Grid &grid);
void grow_last_maze(Grid &grid);
void grow_last_or_sample_maze(Grid &grid);
void ellers_maze(Grid &grid);
void recursive_division_maze(Grid &grid);
void no_walls_maze(Grid &grid);
void all_walls_maze(Grid &grid);
};  // namespace jt::maze
