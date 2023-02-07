// C++ header
#include <limits>

// local header
#include "lidar_object_tracking/cell.hpp"


Cell::Cell()
  : min_z(std::numeric_limits<float>::max()), is_ground(false)
{
}

void Cell::update_min_z(float z)
{
  if (z < min_z) {
    min_z = z;
  }
}

void Cell::update_height(float h)
{
  height = h;
}

void Cell::update_smoothed(float s)
{
  smoothed = s;
}

void Cell::update_hdiff(float hd)
{
  hdiff = hd;
}

void Cell::update_ground()
{
  hground = height;
  is_ground = true;
}

bool Cell::is_this_ground()
{
  return is_ground;
}

float Cell::get_min_z()
{
  return min_z;
}

float Cell::get_height()
{
  return height;
}

float Cell::get_hdiff()
{
  return hdiff;
}

float Cell::get_smoothed()
{
  return smoothed;
}

float Cell::get_hground()
{
  return hground;
}
