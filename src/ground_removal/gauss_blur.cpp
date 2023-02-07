// C++ header
#include <cmath>

// local header
#include "lidar_object_tracking/gauss_blur.hpp"


std::vector<double> gauss_kernel(int samples, double sigma)
{
  std::vector<double> kernel(samples);
  double mu = (samples-1) / 2.0;
  double sum = 0.0;

  for (int x = 0; x < samples; ++x) {
    kernel[x] = std::exp(-0.5 * pow((x - mu) / sigma, 2)) / (std::sqrt(2 * M_PI) * sigma);
    sum += kernel[x];
  }

  // Normalize the kernel
  for (auto& k: kernel) {
    k /= sum;
  }

  return kernel;
}

void gauss_smoothing(std::vector<Cell>& cell, double sigma, int samples)
{
  std::vector<double> kernel {gauss_kernel(samples, sigma)};
  // the middle index of the weighted kernel
  int mid_idx = samples / 2;
  int cell_size = cell.size();
  int kernel_size = kernel.size();

  // applying weighted gaussian kernel with zero padding
  for (int i = 0; i < cell_size; ++i) {
    double smoothed = 0.0;
    for (int j = 0; j < kernel_size; ++j) {
      int idx = i-mid_idx+j;
      if (idx < 0 || idx > cell_size) {
        continue;
      } else {
        smoothed += kernel[j] * cell[idx].get_height();
      }
    }
    cell[i].update_smoothed(smoothed);
  }
}
