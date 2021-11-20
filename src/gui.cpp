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
using namespace jt::maze;

////////////////////////////////////////////////////////////////////////////////
// GUI output
////////////////////////////////////////////////////////////////////////////////
void draw_path(const Grid &grid, sf::RenderWindow &window,
               const std::vector<CellCoordinate> &path) {
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

struct DrawableMaze {
  DrawableMaze(size_t width, size_t height,
               const GeneratorRegistry::RegistryConfig &method)
      : grid{width, height} {
    using std::chrono::high_resolution_clock;
    auto t1 = high_resolution_clock::now();
    method.generate(grid);
    auto t2 = high_resolution_clock::now();

    distances = dijkstra_distances(grid, {grid.width_ / 2, grid.height_ / 2});
    path = longest_path_(grid, distances);
    method_name = method.name;

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
          this->distances.data(), this->grid.height_, this->grid.width_);
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
  std::function<sf::Color(const Grid &, int, int)> colorizer; // FIXME
};

void draw_maze(sf::RenderWindow &window, const DrawableMaze &dmaze) {
  auto &[grid, distances, path, gen_name, max_path_len, colorizer] = dmaze;
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
    if (grid(cell).right != Wall::Open) {
      sf::Vertex line[] = {sf::Vertex(sf::Vector2f(x2, y1), line_color),
                           sf::Vertex(sf::Vector2f(x2, y2), line_color)};

      window.draw(line, 2, sf::Lines);
    }

    // Bottom wall
    if (grid(cell).down != Wall::Open) {
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
  text.setString(gen_name);
  text.setCharacterSize(22); // in pixels, not points!
  text.setFillColor(sf::Color::Yellow);
  window.draw(text);
}

void gui_main(size_t width, size_t height, size_t method_idx) {
  std::unique_ptr<DrawableMaze> dmaze;
  auto regen_maze = [&dmaze, &method_idx, &width, &height]() {
    const auto &method = GeneratorRegistry::GetMazeGeneratorByIndex(method_idx);
    dmaze = std::make_unique<DrawableMaze>(width, height, method);
  };

  regen_maze();

  // create the window
  sf::RenderWindow window(sf::VideoMode(800, 600), "Maze window");

  // run the program as long as the window is open
  while (window.isOpen()) {
    // check all the window's events that were triggered since the last
    // iteration of the loop
    sf::Event event;
    bool need_regen = false;
    while (window.pollEvent(event)) {
      // "close requested" event: we close the window
      switch (event.type) {
      case sf::Event::Closed:
        window.close();
        break;

      case sf::Event::TextEntered:
        switch (event.text.unicode) {
        case '-':
          width--;
          need_regen = true;
          break;
        case '+':
          width++;
          need_regen = true;
          break;
        case '[':
          height--;
          need_regen = true;
          break;
        case ']':
          height++;
          need_regen = true;
          break;

        case 'j':
        case 'J':
          method_idx =
              (method_idx + 1) % GeneratorRegistry::GetMazeGeneratorCount();
          need_regen = true;
          break;
        case 'k':
        case 'K':
          method_idx =
              (method_idx + GeneratorRegistry::GetMazeGeneratorCount() - 1) %
              GeneratorRegistry::GetMazeGeneratorCount();
          need_regen = true;
          break;
        case 'p':
          fmt::print("{}\n", dmaze->grid);
          break;
        }
        break;
      case sf::Event::KeyPressed:
        if (event.key.code == sf::Keyboard::Space) {
          need_regen = true;
        }
        break;
      default:
        break;
      }
    }
    if (need_regen)
      regen_maze();

    // clear the window with black color
    window.clear(sf::Color::Black);

    // draw everything here...
    draw_maze(window, *dmaze);

    // end the current frame
    window.display();
  }
}

int main(int argc, char **argv) {
  if (argc < 3)
    return 1;

  size_t width = static_cast<size_t>(std::strtol(argv[1], nullptr, 10));
  size_t height = static_cast<size_t>(std::strtol(argv[2], nullptr, 10));
  size_t method_idx =
      GeneratorRegistry::GetMazeGeneratorIndexByShortName(argv[3][0]);

  auto &m = GeneratorRegistry::AllMethods();
  jt::maze::ensure_registry();

  gui_main(width, height, method_idx);

  return 0;
}
