#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <deque>
#include <experimental/mdspan>
#include <fmt/chrono.h>
#include <fmt/format.h>
#include <fmt/ranges.h>
#include <iostream>
#include <mutex>
#include <pstl/algorithm>
#include <pstl/execution>
#include <random>
#include <range/v3/all.hpp>
#include <set>
#include <unordered_map>
#include <vector>

#include <SFML/Graphics.hpp>

#include "maze.h"

namespace stdex = std::experimental;


////////////////////////////////////////////////////////////////////////////////
// GUI output
////////////////////////////////////////////////////////////////////////////////
void draw_path(const jt::maze::Grid &grid, sf::RenderWindow &window,
               std::vector<jt::maze::CellCoordinate> &path) {
  const float cell_width = 800.0 / (grid.width_ + 2);
  const float cell_height = 600.0 / (grid.height_ + 2);
  const auto line_color = sf::Color::Red;

  const auto center_of_cell = [cell_width, cell_height](auto row, auto col) {
    // Col/row correctly swapped here, i guess i have always been drwing these
    // sideways :P
    return std::pair{(1.5 + col) * cell_width, (1.5 + row) * cell_height};
  };

  if (path.empty())
    return;

  auto x = ranges::adjacent_find(
      path,
      [&window](auto a, auto b) {
        sf::Vertex l[] = {a, b};
        window.draw(l, 2, sf::Lines);
        return false;
      },
      [center_of_cell, line_color, &grid](auto pos) {
        auto &[row, col] = pos;
        assert(row < grid.height_ && col < grid.width_);
        auto draw_pos = center_of_cell(row, col);
        return sf::Vertex(sf::Vector2f(draw_pos.first, draw_pos.second),
                          line_color);
      });
}

void draw_maze(
    const jt::maze::Grid &grid, sf::RenderWindow &window,
    std::function<sf::Color(const jt::maze::Grid &, int, int)> colorizer,
    std::vector<jt::maze::CellCoordinate> &path) {
  auto window_size = window.getSize();
  const float cell_width = 800.0 / (grid.width_ + 2);
  const float cell_height = 600.0 / (grid.height_ + 2);
  const auto line_color = sf::Color::Green;

  // Fill the cells??
  for (const auto &cell : grid.positions()) {
    const auto &[row, col] = cell;
    float x1 = cell_width * (col + 1);
    float x2 = cell_width * (col + 2);

    float y1 = cell_height * (row + 1);
    float y2 = cell_height * (row + 2);

    // Fill?
    {
      sf::RectangleShape cell(sf::Vector2f(cell_width, cell_height));
      cell.setPosition(x1, y1);
      cell.setFillColor(colorizer(grid, row, col));
      window.draw(cell);
    }
  }

  // Draw top border
  {
    float x1 = cell_width * (0 + 1);

    float y1 = cell_height * (0 + 1);
    float y2 = cell_height * (grid.height_ + 1);
    sf::Vertex line[] = {sf::Vertex(sf::Vector2f(x1, y1), line_color),
                         sf::Vertex(sf::Vector2f(x1, y2), line_color)};

    window.draw(line, 2, sf::Lines);
  }

  // Draw left border
  {
    float x1 = cell_width * (0 + 1);
    float x2 = cell_width * (grid.width_ + 1);

    float y1 = cell_height * (0 + 1);
    sf::Vertex line[] = {sf::Vertex(sf::Vector2f(x1, y1), line_color),
                         sf::Vertex(sf::Vector2f(x2, y1), line_color)};

    window.draw(line, 2, sf::Lines);
  }
  // Draw rest of maze
  for (const auto &cell : grid.positions()) {
    const auto &[row, col] = cell;
    float x1 = cell_width * (col + 1);
    float x2 = cell_width * (col + 2);

    float y1 = cell_height * (row + 1);
    float y2 = cell_height * (row + 2);

    // Right wall
    if (grid(cell).right != jt::maze::Wall::Open) {
      sf::Vertex line[] = {sf::Vertex(sf::Vector2f(x2, y1), line_color),
                           sf::Vertex(sf::Vector2f(x2, y2), line_color)};

      window.draw(line, 2, sf::Lines);
    }

    // Bottom wall
    if (grid(cell).down != jt::maze::Wall::Open) {
      sf::Vertex line[] = {sf::Vertex(sf::Vector2f(x1, y2), line_color),
                           sf::Vertex(sf::Vector2f(x2, y2), line_color)};

      window.draw(line, 2, sf::Lines);
    }
  }

  draw_path(grid, window, path);

  // Labels
  static sf::Font font;
  static std::once_flag load_font_flag;
  std::call_once(load_font_flag, []() {
    if (!font.loadFromFile("/Library/Fonts/Roboto-Thin.ttf")) {
      throw std::runtime_error("Cannot find font");
    }
  });
  sf::Text text;
  text.setFont(font);
  text.setString(method_name(all_methods[method]));
  text.setCharacterSize(22); // in pixels, not points!
  text.setFillColor(sf::Color::White);
  window.draw(text);
}

void gui_main(jt::maze::Grid &grid, std::vector<int> distances,
              std::vector<jt::maze::CellCoordinate> path) {

  auto d = stdex::mdspan<
      int, stdex::extents<stdex::dynamic_extent, stdex::dynamic_extent>>(
      distances.data(), grid.height_, grid.width_);
  auto max_path_len = *std::max_element(distances.begin(), distances.end());
  std::function<sf::Color(const jt::maze::Grid &, int, int)> colorizer =
      [&max_path_len, &d](auto &g, auto c, auto r) {
        int shade = 255 * d(c, r) / max_path_len;
        return sf::Color(shade, shade, shade);
      };

  // create the window
  sf::RenderWindow window(sf::VideoMode(800, 600), "Maze window");

  auto regen_maze = [&]() {
    fmt::print("Regenerating maze!\n");
    grid.reset();
    distances = gen_maze(grid);

    path = longest_path_(grid, distances);
    d = stdex::mdspan<
        int, stdex::extents<stdex::dynamic_extent, stdex::dynamic_extent>>(
        distances.data(), grid.height_, grid.width_);
    max_path_len = *std::max_element(distances.begin(), distances.end());
  };

  // run the program as long as the window is open
  while (window.isOpen()) {
    // check all the window's events that were triggered since the last
    // iteration of the loop
    sf::Event event;
    while (window.pollEvent(event)) {
      // "close requested" event: we close the window
      switch (event.type) {
      case sf::Event::Closed:
        window.close();
        break;

      case sf::Event::TextEntered:
        switch (event.text.unicode) {
        case 'j':
        case 'J':
          method = (method + 1) % all_methods.size();
          break;
        case 'k':
        case 'K':
          method = (method + all_methods.size() - 1) % all_methods.size();
          break;
        }
        regen_maze();
        break;
      case sf::Event::KeyPressed:
        if (event.key.code == sf::Keyboard::Space) {
          regen_maze();
        }
        break;
      default:
        break;
      }
    }

    // clear the window with black color
    window.clear(sf::Color::Black);

    // draw everything here...
    draw_maze(grid, window, colorizer, path);

    // end the current frame
    window.display();
  }
}

int main(int argc, char **argv) {

  if (argc < 3)
    return 1;
  jt::maze::Grid grid{static_cast<size_t>(std::strtol(argv[1], nullptr, 10)),
                      static_cast<size_t>(std::strtol(argv[2], nullptr, 10))};
  auto method_id_for_char = [](auto m) {
    return std::distance(
        std::begin(all_methods),
        std::find(std::begin(all_methods), std::end(all_methods), m));
  };
  method = method_id_for_char(argv[3][0]);
  // stats?
  if (argv[3][0] == 'X') {
    std::unordered_map<char, float> averages;
    // we run stats instead!
    for (auto m : all_methods) {
      // if (m != 'B') continue;
      method = method_id_for_char(m);
      std::vector<int> dead_count;
      for (int i = 0; i < 100; i++) {
        grid.reset();
        gen_maze(grid);
        auto p = grid.positions();
        auto dead_ends =
            ranges::accumulate(p | ranges::views::transform([&grid](auto pos) {
                                 return grid.is_dead_end_cell(pos) ? 1 : 0;
                               }),
                               0);
        dead_count.push_back(dead_ends);
      }
      averages[m] =
          float(ranges::accumulate(dead_count, 0)) / dead_count.size();
    }
    auto cell_count = grid.width_ * grid.height_;
    fmt::print("Average dead-ends per {}x#{} maze ({} cells):\n", grid.width_,
               grid.height_, cell_count);
    for (auto kv : averages) {
      auto &[k, v] = kv;
      fmt::print("{}: {}/{} ({}%)\n", method_name(k), v, cell_count,
                 100 * v / cell_count);
      // AldousBroder : 115/400 (28%)
    }
    exit(0);
  }

  auto distances = gen_maze(grid);

  auto path = longest_path_(grid, distances);

  gui_main(grid, distances, path);
  return 0;
}
