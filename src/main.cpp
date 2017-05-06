/*
 * main.cpp
 *
 *  Created on: May 3, 2017
 *      Author: ramiz
 */

#include <iostream>
#include <random>

/**
 * @param gps_x     GPS provided x position
 * @param gps_y     GPS provided y position
 * @param gps_yaw   GPS provided yaw in radians
 */
void PrintSamples(double gps_x, double gps_y, double gps_yaw);

int main(int argc, char* args[]) {
  // Set GPS provided state of the car.
  double gps_x = 4983;
  double gps_y = 5029;
  double gps_yaw = 1.201;

  // Sample from the GPS provided position.
  PrintSamples(gps_x, gps_y, gps_yaw);
  return 0;
}

void PrintSamples(double gps_x, double gps_y, double gps_yaw) {
  //standard deviations for x, y and yaw
  double std_x = 2.0;
  double std_y = 2.0;
  double std_yaw = 0.05; // in radians

  //initialize a random number generator
  std::default_random_engine generator;

  //create a normal distribution for each of x, y and yaw
  std::normal_distribution<double> normal_distribution_x(gps_x, std_x);
  std::normal_distribution<double> normal_distribution_y(gps_y, std_y);
  std::normal_distribution<double> normal_distribution_yaw(gps_yaw, std_yaw);

  for (int i = 0; i < 3; ++i) {
    //randomly generate samples of x, y and yaw from their respective normal distributions
    double sample_x = normal_distribution_x(generator);
    double sample_y = normal_distribution_y(generator);
    double sample_yaw = normal_distribution_yaw(generator);

    std::cout << "Sample " << i + 1 << " " << sample_x << " " << sample_y << " " << sample_yaw << std::endl;
  }
}
