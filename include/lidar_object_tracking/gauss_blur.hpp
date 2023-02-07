#pragma once

// C++ header
#include <vector>

// local header
#include "lidar_object_tracking/cell.hpp"

std::vector<double> gauss_kernel(int samples, double sigma);

void gauss_smoothing(std::vector<Cell>& cell, double sigma, int samples);
