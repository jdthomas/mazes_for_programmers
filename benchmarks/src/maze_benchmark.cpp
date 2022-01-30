#include <benchmark/benchmark.h>
#include <fmt/ranges.h>

#include <array>
#include <vector>

#include "maze.h"

namespace {};  // namespace

#if 0  // Curiousity, are any of these other RNG more efficient?
static void BM_minstd_rand0(benchmark::State &state) {
  std::random_device rd;
  std::minstd_rand0 gen;
  std::uniform_int_distribution<> distrib(0, 60'000'000);

  int x;
  for (auto _ : state) {
    benchmark::DoNotOptimize(x = distrib(gen));
  }
}
BENCHMARK(BM_minstd_rand0);

static void BM_minstd_rand(benchmark::State &state) {
  std::random_device rd;
  std::minstd_rand gen;
  std::uniform_int_distribution<> distrib(0, 60'000'000);

  int x;
  for (auto _ : state) {
    benchmark::DoNotOptimize(x = distrib(gen));
  }
}
BENCHMARK(BM_minstd_rand);

static void BM_mt19937(benchmark::State &state) {
  std::random_device rd;
  std::mt19937 gen;
  std::uniform_int_distribution<> distrib(0, 60'000'000);

  int x;
  for (auto _ : state) {
    benchmark::DoNotOptimize(x = distrib(gen));
  }
}
BENCHMARK(BM_mt19937);

static void BM_ranlux24(benchmark::State &state) {
  std::random_device rd;
  std::ranlux24 gen;
  std::uniform_int_distribution<> distrib(0, 60'000'000);

  int x;
  for (auto _ : state) {
    benchmark::DoNotOptimize(x = distrib(gen));
  }
}
BENCHMARK(BM_ranlux24);
#endif

static void BM_binary_tree(benchmark::State &state) {
  const int width = state.range(0);
  const int height = state.range(0);
  jt::maze::Grid grid{width, height};
  for (auto _ : state) {
    state.PauseTiming();
    grid.reset();
    state.ResumeTiming();
    binary_tree_maze(grid);
  }
  state.SetComplexityN(state.range(0) * state.range(0));
}
BENCHMARK(BM_binary_tree)->RangeMultiplier(2)->Range(8, 8 << 10)->Complexity();

static void BM_binary_tree_p(benchmark::State &state) {
  const int width = state.range(0);
  const int height = state.range(0);
  jt::maze::Grid grid{width, height};
  for (auto _ : state) {
    state.PauseTiming();
    grid.reset();
    state.ResumeTiming();
    binary_tree_maze_p(grid);
  }
  state.SetComplexityN(state.range(0) * state.range(0));
}
BENCHMARK(BM_binary_tree_p)
    ->RangeMultiplier(2)
    ->Range(8, 8 << 10)
    ->Complexity();

static void BM_binary_tree_p2(benchmark::State &state) {
  const int width = state.range(0);
  const int height = state.range(0);
  jt::maze::Grid grid{width, height};
  for (auto _ : state) {
    state.PauseTiming();
    grid.reset();
    state.ResumeTiming();
    binary_tree_maze_p2(grid);
  }
  state.SetComplexityN(state.range(0) * state.range(0));
}
BENCHMARK(BM_binary_tree_p2)
    ->RangeMultiplier(2)
    ->Range(8, 8 << 10)
    ->Complexity();

static void BM_sidewinder(benchmark::State &state) {
  const int width = state.range(0);
  const int height = state.range(0);
  jt::maze::Grid grid{width, height};
  for (auto _ : state) {
    state.PauseTiming();
    grid.reset();
    state.ResumeTiming();
    sidewinder_maze(grid);
  }
  state.SetComplexityN(state.range(0) * state.range(0));
}
BENCHMARK(BM_sidewinder)->RangeMultiplier(2)->Range(8, 8 << 10)->Complexity();

static void BM_sidewinder_p(benchmark::State &state) {
  const int width = state.range(0);
  const int height = state.range(0);
  jt::maze::Grid grid{width, height};
  for (auto _ : state) {
    state.PauseTiming();
    grid.reset();
    state.ResumeTiming();
    sidewinder_maze_p(grid);
  }
  state.SetComplexityN(state.range(0) * state.range(0));
}
BENCHMARK(BM_sidewinder_p)->RangeMultiplier(2)->Range(8, 8 << 10)->Complexity();

static void BM_random_walk_Aldous_Broder(benchmark::State &state) {
  const int width = state.range(0);
  const int height = state.range(0);
  jt::maze::Grid grid{width, height};
  for (auto _ : state) {
    state.PauseTiming();
    grid.reset();
    state.ResumeTiming();
    random_walk_Aldous_Broder_maze(grid);
  }
  state.SetComplexityN(state.range(0) * state.range(0));
}
BENCHMARK(BM_random_walk_Aldous_Broder)
    ->RangeMultiplier(2)
    ->Range(8, 8 << 4)
    ->Complexity();

static void BM_random_walk_Wilson(benchmark::State &state) {
  const int width = state.range(0);
  const int height = state.range(0);
  jt::maze::Grid grid{width, height};
  for (auto _ : state) {
    state.PauseTiming();
    grid.reset();
    state.ResumeTiming();
    random_walk_Wilson_maze(grid);
  }
  state.SetComplexityN(state.range(0) * state.range(0));
}
BENCHMARK(BM_random_walk_Wilson)
    ->RangeMultiplier(2)
    ->Range(8, 8 << 4)
    ->Complexity();

static void BM_hunt_and_kill(benchmark::State &state) {
  const int width = state.range(0);
  const int height = state.range(0);
  jt::maze::Grid grid{width, height};
  for (auto _ : state) {
    state.PauseTiming();
    grid.reset();
    state.ResumeTiming();
    hunt_and_kill_maze(grid);
  }
  state.SetComplexityN(state.range(0) * state.range(0));
}
BENCHMARK(BM_hunt_and_kill)->RangeMultiplier(2)->Range(8, 8 << 4)->Complexity();

static void BM_recursive_backtracking(benchmark::State &state) {
  const int width = state.range(0);
  const int height = state.range(0);
  jt::maze::Grid grid{width, height};
  for (auto _ : state) {
    state.PauseTiming();
    grid.reset();
    state.ResumeTiming();
    recursive_backtracking_maze(grid);
  }
  state.SetComplexityN(state.range(0) * state.range(0));
}
BENCHMARK(BM_recursive_backtracking)
    ->RangeMultiplier(2)
    ->Range(8, 8 << 6)
    ->Complexity();

static void BM_Kruskel(benchmark::State &state) {
  const int width = state.range(0);
  const int height = state.range(0);
  jt::maze::Grid grid{width, height};
  for (auto _ : state) {
    state.PauseTiming();
    grid.reset();
    state.ResumeTiming();
    kruskel_maze(grid);
  }
  state.SetComplexityN(state.range(0) * state.range(0));
}
BENCHMARK(BM_Kruskel)->RangeMultiplier(2)->Range(8, 8 << 3)->Complexity();

static void BM_Prims(benchmark::State &state) {
  const int width = state.range(0);
  const int height = state.range(0);
  jt::maze::Grid grid{width, height};
  for (auto _ : state) {
    state.PauseTiming();
    grid.reset();
    state.ResumeTiming();
    prims_maze(grid);
  }
  state.SetComplexityN(state.range(0) * state.range(0));
}
BENCHMARK(BM_Prims)->RangeMultiplier(2)->Range(8, 8 << 4)->Complexity();

static void BM_GrowSample(benchmark::State &state) {
  const int width = state.range(0);
  const int height = state.range(0);
  jt::maze::Grid grid{width, height};
  for (auto _ : state) {
    state.PauseTiming();
    grid.reset();
    state.ResumeTiming();
    grow_sample_maze(grid);
  }
  state.SetComplexityN(state.range(0) * state.range(0));
}
BENCHMARK(BM_GrowSample)->RangeMultiplier(2)->Range(8, 8 << 4)->Complexity();

static void BM_GrowLast(benchmark::State &state) {
  const int width = state.range(0);
  const int height = state.range(0);
  jt::maze::Grid grid{width, height};
  for (auto _ : state) {
    state.PauseTiming();
    grid.reset();
    state.ResumeTiming();
    grow_last_maze(grid);
  }
  state.SetComplexityN(state.range(0) * state.range(0));
}
BENCHMARK(BM_GrowLast)->RangeMultiplier(2)->Range(8, 8 << 4)->Complexity();

static void BM_GrowLastOrSample(benchmark::State &state) {
  const int width = state.range(0);
  const int height = state.range(0);
  jt::maze::Grid grid{width, height};
  for (auto _ : state) {
    state.PauseTiming();
    grid.reset();
    state.ResumeTiming();
    grow_last_or_sample_maze(grid);
  }
  state.SetComplexityN(state.range(0) * state.range(0));
}
BENCHMARK(BM_GrowLastOrSample)
    ->RangeMultiplier(2)
    ->Range(8, 8 << 4)
    ->Complexity();

static void BM_Ellers(benchmark::State &state) {
  const int width = state.range(0);
  const int height = state.range(0);
  jt::maze::Grid grid{width, height};
  for (auto _ : state) {
    state.PauseTiming();
    grid.reset();
    state.ResumeTiming();
    ellers_maze(grid);
  }
  state.SetComplexityN(state.range(0) * state.range(0));
}
BENCHMARK(BM_Ellers)->RangeMultiplier(2)->Range(8, 8 << 4)->Complexity();

static void BM_RecursiveDivision(benchmark::State &state) {
  const int width = state.range(0);
  const int height = state.range(0);
  jt::maze::Grid grid{width, height};
  for (auto _ : state) {
    state.PauseTiming();
    grid.reset();
    state.ResumeTiming();
    recursive_division_maze(grid);
  }
  state.SetComplexityN(state.range(0) * state.range(0));
}
BENCHMARK(BM_RecursiveDivision)
    ->RangeMultiplier(2)
    ->Range(8, 8 << 4)
    ->Complexity();

static void BM_NoWalls(benchmark::State &state) {
  const int width = state.range(0);
  const int height = state.range(0);
  jt::maze::Grid grid{width, height};
  for (auto _ : state) {
    state.PauseTiming();
    grid.reset();
    state.ResumeTiming();
    no_walls_maze(grid);
  }
  state.SetComplexityN(state.range(0) * state.range(0));
}
BENCHMARK(BM_NoWalls)->RangeMultiplier(2)->Range(8, 8 << 4)->Complexity();

static void BM_AllWalls(benchmark::State &state) {
  const int width = state.range(0);
  const int height = state.range(0);
  jt::maze::Grid grid{width, height};
  for (auto _ : state) {
    state.PauseTiming();
    grid.reset();
    state.ResumeTiming();
    all_walls_maze(grid);
  }
  state.SetComplexityN(state.range(0) * state.range(0));
}
BENCHMARK(BM_AllWalls)->RangeMultiplier(2)->Range(8, 8 << 4)->Complexity();

BENCHMARK_MAIN();
