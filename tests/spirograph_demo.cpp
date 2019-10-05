// Copyright (c) 2019 Tim Perkins

#include <vector>

#include <SDL.h>

#include <ttfrm/tfrm.hpp>

#include "test_utils.hpp"

constexpr double TAU = 2.0 * M_PI;
constexpr std::size_t SCREEN_WIDTH = 800;
constexpr std::size_t SCREEN_HEIGHT = 800;

ttfrm::Tfrm<std::string> CircleTransform(const std::string& to_frame, const std::string& from_frame,
                                         const double rr, const double tt)
{
  const double xx = rr * std::cos(TAU * tt);
  const double yy = rr * std::sin(TAU * tt);
  const double th = std::atan2(yy, xx);
  const ttfrm::Quat rot = QuatFromEulerXYZ({0.0, 0.0, th});
  const ttfrm::Vec3 trans = {xx, yy, 0.0};
  return ttfrm::Tfrm<std::string>(to_frame, from_frame, rot, trans);
}

std::vector<ttfrm::Vec3> Spirograph()
{
  constexpr std::size_t NUM_STEPS = 1000;
  constexpr double R1 = 200.0;
  constexpr double R2 = 100.0;
  constexpr double R3 = 50.0;
  constexpr double R4 = 10.0;
  ttfrm::Tfrm<std::string> corner_from_center = ttfrm::Tfrm<std::string>::FromTranslation(
      "corner", "center", {SCREEN_WIDTH / 2.0, SCREEN_HEIGHT / 2.0, 0.0});
  std::vector<ttfrm::Vec3> vecs;
  for (std::size_t ii = 0; ii < NUM_STEPS; ++ii) {
    const double tt = static_cast<double>(ii) / (NUM_STEPS - 1);
    const auto center_from_c1 = CircleTransform("center", "c1", R1, tt);
    const auto c1_from_c2 = CircleTransform("c1", "c2", R2, 16.0 * tt);
    const auto c2_from_c3 = CircleTransform("c2", "c3", R3, 16.0 * tt);
    const auto c3_from_c4 = CircleTransform("c3", "c4", R4, 16.0 * tt);
    const auto corner_from_c =
        corner_from_center * center_from_c1 * c1_from_c2 * c2_from_c3 * c3_from_c4;
    vecs.push_back(corner_from_c.Translation());
  }
  return vecs;
}

std::vector<SDL_Point> SdlPointsFromVecs(const std::vector<ttfrm::Vec3>& vecs)
{
  std::vector<SDL_Point> points;
  for (const ttfrm::Vec3& vec : vecs) {
    points.push_back({static_cast<int>(vec.x()), static_cast<int>(vec.y())});
  }
  return points;
}

void SdlDisplay(const std::vector<SDL_Point>& points)
{
  if (SDL_Init(SDL_INIT_VIDEO) == 0) {
    SDL_Window* window = NULL;
    SDL_Renderer* renderer = NULL;
    if (SDL_CreateWindowAndRenderer(SCREEN_WIDTH, SCREEN_HEIGHT, 0, &window, &renderer) == 0) {
      bool is_done = false;
      while (!is_done) {
        SDL_Event event;
        SDL_SetRenderDrawColor(renderer, 0, 0, 0, SDL_ALPHA_OPAQUE);
        SDL_RenderClear(renderer);
        SDL_SetRenderDrawColor(renderer, 255, 255, 255, SDL_ALPHA_OPAQUE);
        SDL_RenderDrawLines(renderer, points.data(), points.size());
        SDL_RenderPresent(renderer);
        while (SDL_PollEvent(&event)) {
          if (event.type == SDL_QUIT) {
            is_done = true;
          }
        }
      }
    }
    if (renderer) {
      SDL_DestroyRenderer(renderer);
    }
    if (window) {
      SDL_DestroyWindow(window);
    }
  }
  SDL_Quit();
}

#define POINTS_COUNT 4

int main(int argc, char** argv)
{
  (void) argc;
  (void) argv;
  const std::vector<ttfrm::Vec3> spiro_vecs = Spirograph();
  const std::vector<SDL_Point> spiro_points = SdlPointsFromVecs(spiro_vecs);
  SdlDisplay(spiro_points);
  return 0;
}
