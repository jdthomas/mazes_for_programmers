#include <fmt/chrono.h>
#include <fmt/format.h>
#include <fmt/ranges.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cxxopts.hpp>
#include <deque>
#include <experimental/mdspan>
#include <iostream>
#include <mutex>
#include <png++/png.hpp>
#if defined(__GLIBCXX__)
#include <execution>
namespace pstl = std;
#else
#include <pstl/algorithm>
#include <pstl/execution>
#endif

#include <imgui-SFML.h>
#include <imgui.h>

#include <SFML/Graphics.hpp>
#include <SFML/Graphics/RenderWindow.hpp>
#include <SFML/System/Clock.hpp>
#include <SFML/Window/Event.hpp>
#include <random>
#include <range/v3/all.hpp>
#include <set>
#include <unordered_map>
#include <vector>

#include "maze.h"

namespace stdex = std::experimental;
using namespace jt::maze;

////////////////////////////////////////////////////////////////////////////////
// GUI output
////////////////////////////////////////////////////////////////////////////////
namespace {
struct GUIViewSettings {
  sf::Color edge_color = sf::Color::Green;
  sf::Color end_marker_color = sf::Color::Magenta;
  sf::Color begin_marker_color = sf::Color::Cyan;
  sf::Color player_path_color = sf::Color::Cyan;
  sf::Color solution_path_color = sf::Color::Red;
  sf::Color bg_color = sf::Color::Black;
  sf::Color text_color = sf::Color::Yellow;
  sf::Color under_cell_color = sf::Color(37, 69, 69);
  sf::Color cell_color = sf::Color(47, 79, 79);
};
const GUIViewSettings &gui_view_settings() {
  static const GUIViewSettings gui_view_settings_{};
  return gui_view_settings_;
}
}  // namespace

struct DrawableMaze {
  struct GenerationSettings {
    float braid_maze_ratio = 0.0f;
    size_t method_idx = 0;
    bool use_polar_widths = false;
    GridSettings grid_settings;
  };
  struct ViewSettings {
    bool show_solution = false;
    bool polar_maze = false;
    bool show_inset = true;
    bool show_as_hex = false;
  };

  DrawableMaze(GenerationSettings gen)
      : gen_settings{gen}, grid{gen.grid_settings} {
    const auto &method =
        GeneratorRegistry::GetMazeGeneratorByIndex(gen.method_idx);
    init(method);
  }
  // DrawableMaze(int width, int height,
  //              const GeneratorRegistry::RegistryConfig &method)
  //     : grid{width, height} {
  //   init(method);
  // }

  void init(const GeneratorRegistry::RegistryConfig &method) {
    using std::chrono::high_resolution_clock;

    if (gen_settings.use_polar_widths) {
      gen_settings.grid_settings.widths =
          calculate_variable_widths(gen_settings.grid_settings.height);

      // FIXME
      gen_settings.grid_settings.width =
          gen_settings.grid_settings.widths.back();
      gen_settings.grid_settings.mask.mask.resize(
          gen_settings.grid_settings.width * gen_settings.grid_settings.height);
      gen_settings.grid_settings.mask.valid_mask = true;
      // Mask out the polar part east of width
      for (auto p :
           ranges::views::enumerate(gen_settings.grid_settings.widths)) {
        auto &[r, c] = p;
        stdex::mdspan<uint8_t, stdex::extents<stdex::dynamic_extent,
                                              stdex::dynamic_extent>>
            m{gen_settings.grid_settings.mask.mask.data(),
              gen_settings.grid_settings.height,
              gen_settings.grid_settings.width};
        for (auto cc : ranges::views::iota(
                 int(c), int(gen_settings.grid_settings.width))) {
          m(r, cc) = 1;
        }
      }

      // Yuck,
      grid = Grid{gen_settings.grid_settings};
      fmt::print("~~~~ Create new grid ({}, {}) ~~~~\n",
                 gen_settings.grid_settings.widths, grid.grid_settings.widths);
    }

    auto t1 = high_resolution_clock::now();
    method.generate(grid);
    auto t2 = high_resolution_clock::now();

    braid_maze(grid, gen_settings.braid_maze_ratio);

    CellCoordinate start_pos =
        grid.random_cell();  // {grid.grid_settings.widths.back() / 2,
                             // grid.grid_settings.height / 2};
    distances = dijkstra_distances(grid, start_pos);
    path = longest_path_(grid, distances);
    method_name = method.name;
    // Player starts at the start of "path"
    player_path.push_back(path.front());

    auto t3 = high_resolution_clock::now();
    std::chrono::duration<double, std::milli> gen_delta_ms = t2 - t1;
    fmt::print("Generated maze in {}\n", gen_delta_ms);

    std::chrono::duration<double, std::milli> path_delta_ms = t3 - t2;
    fmt::print("Compute longest path maze with dijkstra in {}\n",
               path_delta_ms);

    max_path_len = *std::max_element(distances.begin(), distances.end());

    colorizer = [this](auto &g, auto c, auto r) {
      auto d = stdex::mdspan<
          int, stdex::extents<stdex::dynamic_extent, stdex::dynamic_extent>>(
          this->distances.data(), this->grid.grid_settings.height,
          this->grid.grid_settings.widths.back());
      int shade = 255 * d(c, r) / this->max_path_len;
      return sf::Color(shade, shade, shade);
    };

    // Just some stats
    auto p = grid.positions();
    auto dead_ends =
        ranges::accumulate(p | ranges::views::transform([this](auto pos) {
                             return this->grid.is_dead_end_cell(pos) ? 1 : 0;
                           }),
                           0);
    fmt::print("Dead ends: {}\n", dead_ends);
  }

  Grid grid;
  std::vector<int> distances;
  std::vector<CellCoordinate> path;
  std::string method_name;
  size_t max_path_len;
  std::function<sf::Color(const Grid &, int, int)> colorizer;  // FIXME
  std::vector<CellCoordinate> player_path{};

  GenerationSettings gen_settings;
  ViewSettings view_settings;
};

auto draw_path = [](const Grid &grid, sf::RenderWindow &window,
                    const std::vector<CellCoordinate> &path,
                    const auto center_of_cell,
                    const sf::Color path_color = sf::Color::Red) -> void {
  if (path.empty()) return;

  auto x = ranges::adjacent_find(
      path,
      [&window](auto a, auto b) {
        sf::Vertex l[] = {a, b};
        window.draw(l, 2, sf::Lines);
        return false;
      },
      [center_of_cell, path_color, &grid](auto pos) {
        auto &[row, col] = pos;
        assert(row < grid.grid_settings.height &&
               col < grid.grid_settings.widths.back());
        const auto &[draw_pos_y, draw_pos_x] = center_of_cell(pos);
        return sf::Vertex(sf::Vector2f(draw_pos_y, draw_pos_x), path_color);
      });
};

void draw_circle_centered(sf::RenderWindow &window,
                          std::pair<float, float> center, float radius,
                          sf::Color color) {
  sf::CircleShape shape(radius);
  shape.setPosition(center.first - radius, center.second - radius);
  shape.setFillColor(color);
  window.draw(shape);
}

struct CellCorner {
  CellCorner(float xx, float yy) : x{xx}, y{yy} {};
  float x, y;
};
struct CellSegment {
  CellCorner a, b;
  explicit CellSegment(float x1, float y1, float x2, float y2)
      : a{x1, y1}, b{x2, y2} {};
};
struct CellPoly {
  std::vector<CellCorner> fill_poly;
  std::vector<CellSegment> segments;
  // a --------- b
  //   A ----- B |
  //       i   | |
  //   D ----- C |
  // d --------- c
  //
  // Without inset
  // x1                 x4
  // +---+----------+---+ y1
  // |                  |
  // +                  +
  // :         i        :
  // +                  +
  // |                  |
  // +---+----------+---+ y4
  //
  // With Inset:
  // x1  x2         x3  x4
  // +---+----------+---+ y1
  // |   |          |   |
  // +   +          +   + y2
  // :   :     i    :   :
  // +   +          +   + y3
  // |   |          |   |
  // +---+----------+---+ y4
};

auto hex_center_of_cell = [](const auto &cell) {
  const auto &[row, col] = cell;
  const auto cell_height = 1.0f;
  const auto b_size = 0.5f;
  const auto s = 2.0 * b_size / std::sqrt(3.0f);
  const auto a_size = s / 2.0f;

  const bool even_col = col % 2 == 0;
  const float cx = s + 3 * col * a_size;
  const float cy = (even_col ? 1.0f : 2.0f) * b_size + row * cell_height;

  return CellCorner(cx, cy);
};
auto hex_cell_to_segment_coords = [](const auto &cell, const auto &dmaze) {
  const auto cell_height = 1.0f;
  const auto b_size = 0.5f;
  const auto s = 2.0 * b_size / std::sqrt(3.0f);
  const auto a_size = s / 2.0f;
  const auto inset = 0.1f;

  // const bool even_row = cell.row % 2 == 0;
  // const bool even_col = cell.col % 2 == 0;
  const auto [cx, cy] = hex_center_of_cell(cell);

  const auto x_far_west = cx - s;
  const auto x_near_west = cx - a_size;
  const auto x_near_east = cx + a_size;
  const auto x_far_east = cx + s;

  const auto y_north = cy - b_size;
  const auto y_mid = cy;
  const auto y_south = cy + b_size;

  const auto y_north_inset = y_north + inset;
  const auto y_south_inset = y_south - inset;
  const auto x_far_west_inset = x_far_west + inset;
  const auto x_far_east_inset = x_far_east - inset;

  std::vector<CellSegment> segments;
  std::vector<CellCorner> fill_poly;

  if (!dmaze.view_settings.show_inset) {
    if (!dmaze.grid.connected_cell_north(cell)) {
      segments.emplace_back(x_near_west, y_north, x_near_east, y_north);
    }

    if (!dmaze.grid.connected_cell_north_east(cell)) {
      // top '\' wall
      segments.emplace_back(x_near_east, y_north, x_far_east, y_mid);
    }

    if (!dmaze.grid.connected_cell_south_east(cell)) {
      // bottom '/' wall
      segments.emplace_back(x_far_east, y_mid, x_near_east, y_south);
    }

    if (!dmaze.grid.connected_cell_south(cell)) {
      segments.emplace_back(x_near_east, y_south, x_near_west, y_south);
    }

    if (!dmaze.grid.connected_cell_south_west(cell)) {
      segments.emplace_back(x_near_west, y_south, x_far_west, y_mid);
    }

    if (!dmaze.grid.connected_cell_north_west(cell)) {
      segments.emplace_back(x_far_west, y_mid, x_near_west, y_north);
    }

    fill_poly.emplace_back(x_far_west, y_mid);
    fill_poly.emplace_back(x_near_west, y_north);
    fill_poly.emplace_back(x_near_east, y_north);
    fill_poly.emplace_back(x_far_east, y_mid);
    fill_poly.emplace_back(x_near_east, y_south);
    fill_poly.emplace_back(x_near_west, y_south);
  } else {
    // Show inset
    if (!dmaze.grid.connected_cell_north(cell)) {
      segments.emplace_back(x_near_west, y_north_inset, x_near_east,
                            y_north_inset);
    } else {
      // TODO: really only want to draw thesse based on if there is a wall next
      // to them
      segments.emplace_back(x_near_west, y_north, x_near_west, y_north_inset);
      segments.emplace_back(x_near_east, y_north, x_near_east, y_north_inset);
    }

    if (!dmaze.grid.connected_cell_north_east(cell)) {
      segments.emplace_back(x_near_east, y_north_inset, x_far_east_inset,
                            y_mid);
    } else {
      segments.emplace_back(x_far_east, y_mid, x_far_east_inset, y_mid);
      segments.emplace_back(x_near_east, y_south_inset, x_near_east, y_south);
    }
    if (!dmaze.grid.connected_cell_south_east(cell)) {
      segments.emplace_back(x_far_east_inset, y_mid, x_near_east,
                            y_south_inset);
    } else {
      segments.emplace_back(x_far_east, y_mid, x_far_east_inset, y_mid);
      segments.emplace_back(x_near_east, y_south_inset, x_near_east, y_south);
    }
    // if (!dmaze.grid.connected_cell_east(cell)) {
    //   segments.emplace_back(x_near_east, y_north_inset, x_far_east_inset,
    //   y_mid); segments.emplace_back(x_far_east_inset, y_mid, x_near_east,
    //   y_south_inset);
    // } else {
    //   if (even_col) {
    //     segments.emplace_back(x_near_east, y_north_inset, x_far_east_inset,
    //     y_mid);  // SQ Data hack

    //     segments.emplace_back(x_far_east, y_mid, x_far_east_inset, y_mid);
    //     segments.emplace_back(x_near_east, y_south_inset, x_near_east,
    //     y_south);
    //   } else {
    //     segments.emplace_back(x_far_east_inset, y_mid, x_near_east,
    //     y_south_inset);  // SQ Data hack

    //     segments.emplace_back(x_far_east, y_mid, x_far_east_inset, y_mid);
    //     segments.emplace_back(x_near_east, y_north_inset, x_near_east,
    //     y_north);
    //   }
    // }

    if (!dmaze.grid.connected_cell_south(cell)) {
      segments.emplace_back(x_near_east, y_south_inset, x_near_west,
                            y_south_inset);
    } else {
      segments.emplace_back(x_near_west, y_south, x_near_west, y_south_inset);
      segments.emplace_back(x_near_east, y_south, x_near_east, y_south_inset);
    }

    if (!dmaze.grid.connected_cell_north_west(cell)) {
      segments.emplace_back(x_far_west_inset, y_mid, x_near_west,
                            y_north_inset);
    } else {
      segments.emplace_back(x_far_west, y_mid, x_far_west_inset, y_mid);
      segments.emplace_back(x_near_west, y_north_inset, x_near_west, y_north);
    }
    if (!dmaze.grid.connected_cell_south_west(cell)) {
      segments.emplace_back(x_near_west, y_south_inset, x_far_west_inset,
                            y_mid);
    } else {
      segments.emplace_back(x_far_west, y_mid, x_far_west_inset, y_mid);
      segments.emplace_back(x_near_west, y_south_inset, x_near_west, y_south);
    }
    // if (!dmaze.grid.connected_cell_west(cell)) {
    //   segments.emplace_back(x_near_west, y_south_inset, x_far_west_inset,
    //   y_mid); segments.emplace_back(x_far_west_inset, y_mid, x_near_west,
    //   y_north_inset);
    // } else {
    //   if (!even_col) {
    //     segments.emplace_back(x_near_west, y_south_inset, x_far_west_inset,
    //     y_mid);  // SQ Data hack

    //     segments.emplace_back(x_far_west, y_mid, x_far_west_inset, y_mid);
    //     segments.emplace_back(x_near_west, y_north_inset, x_near_west,
    //     y_north);
    //   } else {
    //     segments.emplace_back(x_far_west_inset, y_mid, x_near_west,
    //     y_north_inset);  // SQ Data hack

    //     segments.emplace_back(x_far_west, y_mid, x_far_west_inset, y_mid);
    //     segments.emplace_back(x_near_west, y_south_inset, x_near_west,
    //     y_south);
    //   }
    // }

    fill_poly.emplace_back(x_far_west_inset, y_mid);
    if (dmaze.grid.connected_cell_north_west(cell)) {
      fill_poly.emplace_back(x_far_west, y_mid);
      fill_poly.emplace_back(x_near_west, y_north);
    }
    fill_poly.emplace_back(x_near_west, y_north_inset);
    if (dmaze.grid.connected_cell_north(cell)) {
      fill_poly.emplace_back(x_near_west, y_north);
      fill_poly.emplace_back(x_near_east, y_north);
    }
    fill_poly.emplace_back(x_near_east, y_north_inset);
    if (dmaze.grid.connected_cell_north_east(cell)) {
      fill_poly.emplace_back(x_near_east, y_north);
      fill_poly.emplace_back(x_far_east, y_mid);
    }
    fill_poly.emplace_back(x_far_east_inset, y_mid);
    if (dmaze.grid.connected_cell_south_east(cell)) {
      fill_poly.emplace_back(x_far_east, y_mid);
      fill_poly.emplace_back(x_near_east, y_south);
    }
    fill_poly.emplace_back(x_near_east, y_south_inset);
    if (dmaze.grid.connected_cell_south(cell)) {
      fill_poly.emplace_back(x_near_east, y_south);
      fill_poly.emplace_back(x_near_west, y_south);
    }
    fill_poly.emplace_back(x_near_west, y_south_inset);
    if (dmaze.grid.connected_cell_south_west(cell)) {
      fill_poly.emplace_back(x_near_west, y_south);
      fill_poly.emplace_back(x_far_west, y_mid);
    }
  }

  return std::make_pair(fill_poly, segments);
};

auto triangle_cell_to_segment_coords = [](const auto &cell, const auto &dmaze) {
  // width = s
  // half_width = width / 2.0
  // height = s * Math.sqrt(3) / 2
  // half_height = height / 2.0
  //
  // half_width = size / 2.0
  // height = size * Math.sqrt(3) / 2.0
  // half_height = height / 2.0
  //
  // west_x = cx - half_width
  // mid_x  = cx
  // east_x = cx + half_width
  // if upright?
  // base_y = cy + half_height apex_y = cy - half_height
  // else
  //   base_y = cy - half_height
  //   apex_y = cy + half_height
  // end
  return 0;
};

// This takes a cell and returns the line segments to traw in "cell space" e.g.
// against a grid of 1x1 cells.
auto square_center_of_cell = [](const auto &cell) {
  const auto &[row, col] = cell;
  return CellCorner(col + 0.5f, row + 0.5f);
};
auto square_cell_to_segment_coords = [](const auto &cell, const auto &dmaze) {
  const auto inset = 0.1f;
  const auto &[row, col] = cell;
  const float y1 = row;
  const float y2 = row + inset;
  const float y3 = row + 1 - inset;
  const float y4 = row + 1;
  const float x1 = col;
  const float x2 = col + inset;
  const float x3 = col + 1 - inset;
  const float x4 = col + 1;
  std::vector<CellSegment> segments;
  std::vector<CellCorner> fill_poly;
  if (!dmaze.view_settings.show_inset) {
    if (!dmaze.grid.connected_cell_north(cell)) {
      segments.emplace_back(x1, y1, x4, y1);
    }
    if (!dmaze.grid.connected_cell_east(cell)) {
      segments.emplace_back(x4, y1, x4, y4);
    }
    if (!dmaze.grid.connected_cell_south(cell)) {
      segments.emplace_back(x1, y4, x4, y4);
    }
    if (!dmaze.grid.connected_cell_west(cell)) {
      segments.emplace_back(x1, y1, x1, y4);
    }
    fill_poly.emplace_back(x1, y1);
    fill_poly.emplace_back(x1, y4);
    fill_poly.emplace_back(x4, y4);
    fill_poly.emplace_back(x4, y1);
    fill_poly.emplace_back(x1, y1);
  } else {
    if (dmaze.grid(cell).check_flag(Grid::Cell::Flags::UnderCell)) {
      if (dmaze.grid.is_v_passage_cell(cell)) {
        segments.emplace_back(x2, y1, x2, y2);
        segments.emplace_back(x3, y1, x3, y2);
        segments.emplace_back(x2, y3, x2, y4);
        segments.emplace_back(x3, y3, x3, y4);
      } else {
        // assert(dmaze.grid.is_h_passage_cell(cell));
        segments.emplace_back(x1, y2, x2, y2);
        segments.emplace_back(x1, y3, x2, y3);
        segments.emplace_back(x3, y2, x4, y2);
        segments.emplace_back(x3, y3, x4, y3);
      }
    }

    {
      if (!dmaze.grid.is_connected_directly_north(cell)) {
        segments.emplace_back(x2, y2, x3, y2);
      } else {
        segments.emplace_back(x2, y1, x2, y2);
        segments.emplace_back(x3, y1, x3, y2);
      }

      if (!dmaze.grid.is_connected_directly_east(cell)) {
        segments.emplace_back(x3, y2, x3, y3);
      } else {
        segments.emplace_back(x3, y2, x4, y2);
        segments.emplace_back(x3, y3, x4, y3);
      }

      if (!dmaze.grid.is_connected_directly_south(cell)) {
        segments.emplace_back(x2, y3, x3, y3);
      } else {
        segments.emplace_back(x2, y3, x2, y4);
        segments.emplace_back(x3, y3, x3, y4);
      }

      if (!dmaze.grid.is_connected_directly_west(cell)) {
        segments.emplace_back(x2, y2, x2, y3);
      } else {
        segments.emplace_back(x1, y2, x2, y2);
        segments.emplace_back(x1, y3, x2, y3);
      }

      if (dmaze.grid(cell).check_flag(Grid::Cell::Flags::UnderCell)) {
        fill_poly.emplace_back(x2, y2);
        fill_poly.emplace_back(x2, y1);
        fill_poly.emplace_back(x3, y1);
        fill_poly.emplace_back(x3, y2);
        fill_poly.emplace_back(x4, y2);
        fill_poly.emplace_back(x4, y3);
        fill_poly.emplace_back(x3, y3);
        fill_poly.emplace_back(x3, y4);
        fill_poly.emplace_back(x2, y4);
        fill_poly.emplace_back(x2, y3);
        fill_poly.emplace_back(x1, y3);
        fill_poly.emplace_back(x1, y2);
      } else {
        //
        fill_poly.emplace_back(x2, y2);
        if (dmaze.grid.is_connected_directly_north(cell)) {
          fill_poly.emplace_back(x2, y1);
          fill_poly.emplace_back(x3, y1);
        }
        fill_poly.emplace_back(x3, y2);
        if (dmaze.grid.is_connected_directly_east(cell)) {
          fill_poly.emplace_back(x4, y2);
          fill_poly.emplace_back(x4, y3);
        }
        fill_poly.emplace_back(x3, y3);
        if (dmaze.grid.is_connected_directly_south(cell)) {
          fill_poly.emplace_back(x3, y4);
          fill_poly.emplace_back(x2, y4);
        }
        fill_poly.emplace_back(x2, y3);
        if (dmaze.grid.is_connected_directly_west(cell)) {
          fill_poly.emplace_back(x1, y3);
          fill_poly.emplace_back(x1, y2);
        }
      }
    }
  }
  return std::make_pair(fill_poly, segments);
};

////////////////////////////////////////////////////////////////////////////////
// Coordinate system
//
// Cartesian maze:
//    +---------- x/col ------------>
//    |
//    |
//    |
//    |
//
//  y/row
//
//    |
//    |
//    |
//    |
//    v
//
// Polar maze:
//           <------ x/col -------+
//             , - ~ ~ ~ - ,       \
//         , '               ' ,    \
//       ,                       ,
//      ,                         ,
//     ,                           ,
//     ,            * -- y/row --> ,
//     ,                           ,
//      ,                         ,
//       ,                       ,
//         ,                  , '
//           ' - , _ _ _ ,  '
//
//
////////////////////////////////////////////////////////////////////////////////
auto draw_maze_common = [](sf::RenderWindow &window, const DrawableMaze &dmaze,
                           auto cell_polygon, auto center_of_cell,
                           auto cell_width, auto cell_height) -> void {
  if (dmaze.view_settings.show_solution) {
    for (const auto &cell : dmaze.grid.positions()) {
      auto corners = cell_polygon(cell);
      sf::ConvexShape polygon;
      polygon.setFillColor(dmaze.colorizer(dmaze.grid, cell.row, cell.col));

      polygon.setPointCount(corners.fill_poly.size());
      for (const auto &[pt_ct, pt] :
           ranges::views::enumerate(corners.fill_poly)) {
        polygon.setPoint(pt_ct, sf::Vector2f(pt.x, pt.y));
      }
      window.draw(polygon);
    }
  } else {
    // Easier to debug with a sligthly different color here
    for (const auto &cell : dmaze.grid.positions()) {
      auto corners = cell_polygon(cell);
      sf::ConvexShape polygon;
      if (dmaze.grid(cell).check_flag(Grid::Cell::Flags::UnderCell)) {
        // polygon.setFillColor(sf::Color(sf::Color::Red));  // dark gray
        polygon.setFillColor(
            gui_view_settings().under_cell_color);  // dark gray
      } else {
        polygon.setFillColor(gui_view_settings().cell_color);  // dark gray
      }

      polygon.setPointCount(corners.fill_poly.size());
      for (const auto &[pt_ct, pt] :
           ranges::views::enumerate(corners.fill_poly)) {
        polygon.setPoint(pt_ct, sf::Vector2f(pt.x, pt.y));
      }
      window.draw(polygon);
    }
  }

  // Draw rest of maze
  for (const auto &cell : dmaze.grid.positions()) {
    auto corners = cell_polygon(cell);
    for (const auto &seg : corners.segments) {
      sf::Vertex line[] = {sf::Vertex(sf::Vector2f(seg.a.x, seg.a.y),
                                      gui_view_settings().edge_color),
                           sf::Vertex(sf::Vector2f(seg.b.x, seg.b.y),
                                      gui_view_settings().edge_color)};
      assert(gui_view_settings().edge_color == sf::Color::Green);
      window.draw(line, 2, sf::Lines);
    }
  }
  // Draw start/end
  {
    auto cell = dmaze.path.front();
    auto corners = cell_polygon(cell);
    auto cent = center_of_cell(cell);
    draw_circle_centered(window, {cent.x, cent.y},
                         .8 * std::min(cell_width, cell_height) / 2,
                         gui_view_settings().begin_marker_color);
  }
  {
    auto cell = dmaze.path.back();
    auto corners = cell_polygon(cell);
    auto cent = center_of_cell(cell);
    draw_circle_centered(window, {cent.x, cent.y},
                         .8 * std::min(cell_width, cell_height) / 2,
                         gui_view_settings().end_marker_color);
  }

  if (dmaze.view_settings.show_solution) {
    draw_path(dmaze.grid, window, dmaze.path, center_of_cell,
              gui_view_settings().solution_path_color);
  }
  draw_path(dmaze.grid, window, dmaze.player_path, center_of_cell,
            gui_view_settings().player_path_color);

  // Labels
  static sf::Font font;
  static std::once_flag load_font_flag;
  std::call_once(load_font_flag, []() {
    if (!font.loadFromFile("/Library/Fonts/Roboto-Thin.ttf")) {
      throw std::runtime_error("Cannot find font");
    }
  });
  {
    sf::Text text;
    text.setFont(font);
    text.setString(dmaze.method_name);
    text.setCharacterSize(22);  // in pixels, not points!
    text.setFillColor(gui_view_settings().text_color);
    window.draw(text);
  }

  if (dmaze.path.back() == dmaze.player_path.back()) {
    sf::Text text;
    text.setFont(font);
    text.setString("WINNER");
    text.setCharacterSize(44);  // in pixels, not points!
    text.setFillColor(gui_view_settings().text_color);
    sf::FloatRect textRect = text.getLocalBounds();
    text.setOrigin(textRect.left + textRect.width / 2.0f,
                   textRect.top + textRect.height / 2.0f);
    text.setPosition(sf::Vector2f(800 / 2.0f, 600 / 2.0f));
    window.draw(text);
  }
};

void draw_polar_maze(sf::RenderWindow &window, const DrawableMaze &dmaze) {
  auto window_size = window.getSize();
  const float cell_width =
      800.0 / (2 * dmaze.grid.grid_settings.height + 2);  // yes use height
  const float cell_height = 600.0 / (2 * dmaze.grid.grid_settings.height + 2);
  const auto cell_size = std::min(cell_width, cell_height);
  const auto center_x = 400;
  const auto center_y = 300;
  constexpr double pi = M_PI;
  const auto theta = [&](int row) -> double {
    // assert(row >= 0 && row < dmaze.grid.grid_settings.widths.size());
    auto w = row < dmaze.grid.grid_settings.widths.size()
                 ? dmaze.grid.grid_settings.widths[row]
                 : dmaze.grid.grid_settings.widths.back();
    // Fixme: `w` is not correct when dmaze.view_settings.show_as_hex is set
    return 2 * pi / w;
  };

  const auto cell_to_draw_x = [&](const auto row, const auto col) -> float {
    const auto radius = row * cell_size;
    const auto theta_ccw = col * theta(std::floor(row));
    auto x = center_x + (radius * std::cos(theta_ccw));
    if (!std::isfinite(x) || std::isnan(x)) {
      throw std::runtime_error("Bad polor calc");
    }
    return x;
  };
  const auto cell_to_draw_y = [&](const auto row, const auto col) -> float {
    const auto radius = row * cell_size;
    const auto theta_ccw = col * theta(std::floor(row));
    auto y = center_y + (radius * std::sin(theta_ccw));
    if (!std::isfinite(y) || std::isnan(y)) {
      throw std::runtime_error("Bad polor calc");
    }
    return y;
  };
  const auto center_of_cell = [&](const auto cell) {
    const auto [cx, cy] = dmaze.view_settings.show_as_hex
                              ? hex_center_of_cell(cell)
                              : square_center_of_cell(cell);
    return CellCorner(cell_to_draw_x(cy, cx), cell_to_draw_y(cy, cx));
  };
  auto cell_polygon = [&](auto cell) -> CellPoly {
    // const auto inner_radius = cell.row * cell_size;
    // const auto outer_radius = (cell.row + 1) * cell_size;
    // const auto theta_ccw = cell.col * theta(cell.row);
    // const auto theta_cw = (cell.col + 1) * theta(cell.row);

    auto [fill_poly, segs] = dmaze.view_settings.show_as_hex
                                 ? hex_cell_to_segment_coords(cell, dmaze)
                                 : square_cell_to_segment_coords(cell, dmaze);
    return {
        .fill_poly = fill_poly | ranges::views::transform([&](auto pt) {
                       return CellCorner{cell_to_draw_x(pt.y, pt.x),
                                         cell_to_draw_y(pt.y, pt.x)};
                     }) |
                     ranges::to<std::vector>,
        .segments = segs | ranges::views::transform([&](auto seg) {
                      return CellSegment(cell_to_draw_x(seg.a.y, seg.a.x),
                                         cell_to_draw_y(seg.a.y, seg.a.x),
                                         cell_to_draw_x(seg.b.y, seg.b.x),
                                         cell_to_draw_y(seg.b.y, seg.b.x));
                    }) |
                    ranges::to<std::vector>,
    };
  };

  // Draw outer boundry
  {
    auto rad = cell_size * (dmaze.grid.grid_settings.height);
    sf::CircleShape shape(rad);
    shape.setFillColor(gui_view_settings().bg_color);
    shape.setOutlineColor(gui_view_settings().edge_color);
    shape.setOutlineThickness(1);
    shape.setPosition(center_x - rad, center_y - rad);
    window.draw(shape);
  }

  return draw_maze_common(window, dmaze, cell_polygon, center_of_cell,
                          cell_width, cell_height);
}

void draw_maze(sf::RenderWindow &window, const DrawableMaze &dmaze) {
  auto window_size = window.getSize();
  const float cell_width = 800.0 / (dmaze.grid.grid_settings.widths.back() + 2);
  const float cell_height = 600.0 / (dmaze.grid.grid_settings.height + 2);

  const auto cell_to_draw_x = [&](const auto row,
                                  [[maybe_unused]] const auto col) {
    return cell_height * (row + 1);
  };
  const auto cell_to_draw_y = [&](const auto row, const auto col) {
    // const float cw = 800.0 / (dmaze.grid.grid_settings.widths[row] + 2);
    const float cw = cell_width;
    return cw * (col + 1);
  };
  const auto center_of_cell = [&](const auto cell) {
    const auto [cx, cy] = dmaze.view_settings.show_as_hex
                              ? hex_center_of_cell(cell)
                              : square_center_of_cell(cell);
    return CellCorner(cell_to_draw_y(cy, cx), cell_to_draw_x(cy, cx));
  };
  auto cell_polygon = [&](auto cell) -> CellPoly {
    const auto &[row, col] = cell;

    auto [fill_poly, segs] = dmaze.view_settings.show_as_hex
                                 ? hex_cell_to_segment_coords(cell, dmaze)
                                 : square_cell_to_segment_coords(cell, dmaze);
    return {
        .fill_poly = fill_poly | ranges::views::transform([&](auto pt) {
                       return CellCorner{cell_to_draw_y(pt.y, pt.x),
                                         cell_to_draw_x(pt.y, pt.x)};
                     }) |
                     ranges::to<std::vector>,
        .segments = segs | ranges::views::transform([&](auto seg) {
                      return CellSegment(
                          // swap x,y
                          cell_to_draw_y(seg.a.y, seg.a.x),
                          cell_to_draw_x(seg.a.y, seg.a.x),
                          cell_to_draw_y(seg.b.y, seg.b.x),
                          cell_to_draw_x(seg.b.y, seg.b.x));
                    }) |
                    ranges::to<std::vector>,
    };
  };

  return draw_maze_common(window, dmaze, cell_polygon, center_of_cell,
                          cell_width, cell_height);
}

void gui_main(DrawableMaze::GenerationSettings gen) {
  std::unique_ptr<DrawableMaze> dmaze;
  auto regen_maze = [&dmaze, &gen]() {
    DrawableMaze::ViewSettings view_settings;
    if (dmaze) std::swap(view_settings, dmaze->view_settings);
    dmaze = std::make_unique<DrawableMaze>(gen);
    std::swap(view_settings, dmaze->view_settings);
  };

  regen_maze();

  // create the window
  sf::RenderWindow window(sf::VideoMode(800, 600), "Maze window");
  window.setVerticalSyncEnabled(true);
  ImGui::SFML::Init(window);

  auto player_move = [&](auto next) {
    if (next) {
      if (dmaze->player_path.size() > 1 &&
          next == dmaze->player_path.rbegin()[1]) {
        dmaze->player_path.pop_back();
      } else {
        dmaze->player_path.push_back(*next);
      }
    }
  };
  bool rot45 = false;
  bool show_gui = false;
  sf::Clock deltaClock;

  // run the program as long as the window is open
  while (window.isOpen()) {
    // check all the window's events that were triggered since the last
    // iteration of the loop
    sf::Event event;
    bool need_regen = false;
    while (window.pollEvent(event)) {
      ImGui::SFML::ProcessEvent(event);
      // "close requested" event: we close the window
      switch (event.type) {
        case sf::Event::Closed:
          window.close();
          break;

        case sf::Event::TextEntered:
          switch (event.text.unicode) {
            case '-':
              gen.grid_settings.width =
                  std::max(1, gen.grid_settings.width - 1);
              need_regen = true;
              break;
            case '+':
              gen.grid_settings.width++;
              need_regen = true;
              break;
            case '[':
              gen.grid_settings.height =
                  std::max(1, gen.grid_settings.height - 1);
              need_regen = true;
              break;
            case ']':
              gen.grid_settings.height++;
              need_regen = true;
              break;

            case '?':
              show_gui = !show_gui;
              break;
            case 'j':
            case 'J':
              gen.method_idx = (gen.method_idx + 1) %
                               GeneratorRegistry::GetMazeGeneratorCount();
              need_regen = true;
              break;
            case 'k':
            case 'K':
              gen.method_idx =
                  (gen.method_idx + GeneratorRegistry::GetMazeGeneratorCount() -
                   1) %
                  GeneratorRegistry::GetMazeGeneratorCount();
              need_regen = true;
              break;
            case 'i':
              dmaze->view_settings.show_inset =
                  !dmaze->view_settings.show_inset;
              break;
            case 'r':
              dmaze->view_settings.polar_maze =
                  !dmaze->view_settings.polar_maze;
              break;
            case 'w':
              gen.grid_settings.allow_ew_wrap =
                  !gen.grid_settings.allow_ew_wrap;
              need_regen = true;
              break;
            case 'p':
              fmt::print("{}\n", dmaze->grid);
              break;
            case 's':
              dmaze->view_settings.show_solution =
                  !dmaze->view_settings.show_solution;
              break;
            case 'x':
              dmaze->view_settings.show_as_hex =
                  !dmaze->view_settings.show_as_hex;
              break;
            case 'z':
              rot45 = !rot45;
              break;
            case 'q':
              return;
              break;
          }
        case sf::Event::KeyPressed:
          switch (event.key.code) {
            case sf::Keyboard::Space:
              need_regen = true;
              break;
            case sf::Keyboard::Left:
              player_move(rot45 ? dmaze->grid.connected_cell_north_west(
                                      dmaze->player_path.back())
                                : dmaze->grid.connected_cell_west(
                                      dmaze->player_path.back()));
              break;
            case sf::Keyboard::Right:
              player_move(rot45 ? dmaze->grid.connected_cell_south_east(
                                      dmaze->player_path.back())
                                : dmaze->grid.connected_cell_east(
                                      dmaze->player_path.back()));
              break;
            case sf::Keyboard::Down:
              player_move(rot45 ? dmaze->grid.connected_cell_south_west(
                                      dmaze->player_path.back())
                                : dmaze->grid.connected_cell_south(
                                      dmaze->player_path.back()));
              break;
            case sf::Keyboard::Up:
              player_move(rot45 ? dmaze->grid.connected_cell_north_east(
                                      dmaze->player_path.back())
                                : dmaze->grid.connected_cell_north(
                                      dmaze->player_path.back()));
              break;
            default:
              break;
          }
          break;
        default:
          break;
      }
    }

    // Settings GUI
    if (show_gui) {
      ImGui::SFML::Update(window, deltaClock.restart());

      ImGui::Begin("Settings");  // begin window

      ImGui::BeginGroup();
      ImGui::Text("Generation Settings");
      if (ImGui::Button("Generate New Maze")) {
        fmt::print("");
        // this code gets if user clicks on the button
        // yes, you could have written if(ImGui::InputText(...))
        // but I do this to show how buttons work :)
        need_regen = true;
      }
      auto algo_names =
          GeneratorRegistry::AllMethods() |
          ranges::views::transform([](auto &x) { return x.name.c_str(); }) |
          ranges::to<std::vector>;
      if (ImGui::BeginCombo("Algorithm", algo_names.at(gen.method_idx))) {
        for (int n = 0; n < algo_names.size(); n++) {
          bool is_selected = gen.method_idx == n;
          if (ImGui::Selectable(algo_names[n], is_selected)) {
            gen.method_idx = n;
          }
          if (is_selected) ImGui::SetItemDefaultFocus();
        }
        ImGui::EndCombo();
      }
      ImGui::Checkbox("Weaving", &gen.grid_settings.enable_weaving);
      ImGui::Checkbox("Wrap east/west", &gen.grid_settings.allow_ew_wrap);
      ImGui::Checkbox("Polar Widths", &gen.use_polar_widths);

      ImGui::SliderFloat("Braiding Ratio", &gen.braid_maze_ratio, 0.0f, 1.0f,
                         "ratio = %.3f");
      ImGui::SliderInt("Height", &gen.grid_settings.height, 4, 200);
      ImGui::SliderInt("Width", &gen.grid_settings.width, 4, 200);
      ImGui::SliderInt("Random Seed (0 for random)",
                       &gen.grid_settings.random_seed, 0,
                       std::numeric_limits<int>::max());

      ImGui::EndGroup();

      ImGui::BeginGroup();
      ImGui::Text("View Settings");
      ImGui::Checkbox("Show Solition", &dmaze->view_settings.show_solution);
      ImGui::Checkbox("Show inset", &dmaze->view_settings.show_inset);
      ImGui::Checkbox("Show as hex", &dmaze->view_settings.show_as_hex);
      ImGui::Checkbox("Show as polar", &dmaze->view_settings.polar_maze);
      ImGui::EndGroup();

      ImGui::End();  // end window
    }

    if (need_regen) regen_maze();

    // clear the window with black color
    window.clear(gui_view_settings().bg_color);

    // draw everything here...
    if (dmaze->view_settings.polar_maze) {
      draw_polar_maze(window, *dmaze);
    } else {
      draw_maze(window, *dmaze);
    }

    if (show_gui) {
      ImGui::SFML::Render(window);
    }
    // end the current frame
    window.display();
  }
}

static void gui_usage() {
  std::string description =
      "========================================\n"
      "Solve the maze!\n"
      "Start at the cyan marker. Use the arrow keys to find a path to "
      "the magenta mark.\n"
      "\n"
      "Controls:\n"
      "arrows      move player path in direction of arrow.\n"
      "Space       generate a new maze.\n"
      "'j' / 'k'   cycle through various maze generators (and generate new "
      "maze).\n"
      "'-' / '+'   shrink/grow horizontally by one (and generate new maze).\n"
      "'[' / ']'   shrink/grow vertically by one (and generate new maze).\n"
      "'s'         toggle revealing/hiding the solution.\n"
      "========================================\n";
  fmt::print(description);
}

static auto read_mask(std::string fn, int width, int height, bool invert)
    -> GridMask {
  png::image<png::gray_pixel> image(fn);
  width = width == 0 ? image.get_width() : width;
  height = height == 0 ? image.get_height() : height;
  image.resize(width, height);
  std::vector<uint8_t> mask(width * height);
  auto pix_to_mask = [invert]() {
    return invert ? [](uint8_t p) -> uint8_t { return p > 128; }
                  : [](uint8_t p) -> uint8_t { return p < 128; };
  }();
  auto d = stdex::mdspan<
      uint8_t, stdex::extents<stdex::dynamic_extent, stdex::dynamic_extent>>(
      mask.data(), height, width);
  for (int i = 0; i < height; i++)
    for (int j = 0; j < width; j++) {
      d(i, j) = pix_to_mask(image.get_pixel(j, i));
    }
  fmt::print("{}", mask);
  return GridMask{true, std::move(mask)};
}

int main(int argc, char **argv) {
  cxxopts::Options options("maze_gui", "Play with mazes!");
  // clang-format off
  options.add_options()
    ("w,width", "width of maze to generate", cxxopts::value<int>()->default_value("10"))
    ("v,height", "width of maze to generate", cxxopts::value<int>()->default_value("10"))
    ("m,mask", "File name", cxxopts::value<std::string>())
    ("i,invert", "Invert the mask",  cxxopts::value<bool>()->default_value("false"))
    ("d,method", "Name of generation method",  cxxopts::value<std::string>()->default_value("R"))
    ("h,help", "Print usage")
    ;
  // clang-format on

  auto result = options.parse(argc, argv);
  if (result.count("help")) {
    fmt::print("{}\n", options.help());
    return 0;
  }
  int width = result["width"].as<int>();
  int height = result["height"].as<int>();

  DrawableMaze::GenerationSettings gen;

  // Calculate polar grid widths and set new width to max of that
  auto w = calculate_variable_widths(height);
  if (gen.use_polar_widths) {
    width = w.back();
  }

  gui_usage();

  auto msk = result.count("mask")
                 ? read_mask(result["mask"].as<std::string>(), width, height,
                             result["invert"].as<bool>())
                 : GridMask{false, {}};
  auto method_name = result["method"].as<std::string>();
  size_t method_idx =
      GeneratorRegistry::GetMazeGeneratorIndexByShortName(method_name[0]);

  auto &m = GeneratorRegistry::AllMethods();
  jt::maze::ensure_registry();

  msk.mask.resize(width * height);
  // Mask out the polar part east of width
  if (gen.use_polar_widths) {
    for (auto p : ranges::views::enumerate(w)) {
      auto &[r, c] = p;
      stdex::mdspan<
          uint8_t, stdex::extents<stdex::dynamic_extent, stdex::dynamic_extent>>
          m{msk.mask.data(), height, width};
      // auto m = dmaze.grid.mask_as_mdspan();
      for (auto cc : ranges::views::iota(int(c), int(width))) {
        fmt::print("{} {}\n", r, cc);
        m(r, cc) = 1;
      }
    }
  }

  gen.method_idx = method_idx;
  gen.grid_settings.height = height;
  gen.grid_settings.width = width;
  gen.grid_settings.mask = std::move(msk);
  gui_main(gen);

  return 0;
}
