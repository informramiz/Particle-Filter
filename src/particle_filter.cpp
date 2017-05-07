/*
 * particle_filter.cpp
 *
 *  Created on: May 16, 2016
 *      Author: Ramiz Raja
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <cmath>

#include "particle_filter.h"

ParticleFilter::ParticleFilter() {
  num_particles = 1000;
  is_initialized = false;
}

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).

  //standard deviations for x, y and yaw
    double std_x = std[0];
    double std_y = std[1];
    double std_yaw = std[2]; // in radians

    //initialize a random number generator
    std::default_random_engine generator;

    //create a normal distribution for each of x, y and yaw
    std::normal_distribution<double> normal_distribution_x(x, std_x);
    std::normal_distribution<double> normal_distribution_y(y, std_y);
    std::normal_distribution<double> normal_distribution_yaw(theta, std_yaw);

    for (int i = 0; i < num_particles; ++i) {
      Particle particle;
      particle.id = i;

      //randomly generate samples of x, y and yaw from their respective normal distributions
      particle.x = normal_distribution_x(generator);
      particle.y = normal_distribution_y(generator);
      particle.theta = normal_distribution_yaw(generator);

      //initially set weight to 1
      particle.weight = 1;

      particles.push_back(particle);
    }

    is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

  std::default_random_engine generator;

  for (int i = 0; i < num_particles; ++i) {
    //for ease, take reference of current index particle
    Particle &particle = particles[i];

    //predict x, y and yaw by applying eqse of motion for bicycle model when
    //yaw_rate is not zero
    double new_x = particle.x + (velocity/yaw_rate) * (sin(particle.theta + yaw_rate * delta_t) - sin(particle.theta));
    double new_y = particle.y + (velocity/yaw_rate) * (-cos(particle.theta + yaw_rate * delta_t) + cos(particle.theta));
    double new_theta = particle.theta + yaw_rate * delta_t;

    //create normal distribution for each of new x, y, theta
    //this will be used to get Gaussian noised
    std::normal_distribution<double> normal_distribution_x(new_x, std_pos[0]);
    std::normal_distribution<double> normal_distribution_y(new_y, std_pos[1]);
    std::normal_distribution<double> normal_distribution_theta(new_theta, std_pos[2]);

    //get a random Gaussian noised value for each of new x, y, theta
    particle.x = normal_distribution_x(generator);
    particle.y = normal_distribution_y(generator);
    particle.theta = normal_distribution_theta(generator);
  }
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		std::vector<LandmarkObs> observations, Map map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33. Note that you'll need to switch the minus sign in that equation to a plus to account 
	//   for the fact that the map's y-axis actually points downwards.)
	//   http://planning.cs.uiuc.edu/node99.html

std::vector<Map::MapLandmark> ParticleFilter::FilterMapLandmarks(const Particle& particle,
                                                                 const Map &map_landmarks,
                                                                 double sensor_range) {
  //create container to hold filtered map landmarks
  std::vector<Map::MapLandmark> filtered_map_landmarks;

  size_t map_landmars_count = map_landmarks.landmark_list_.size();

  //go throuh each map landmark and filter the ones that are not in range from give particle
  for (int i = 0; i < map_landmars_count; ++i) {
    Map::MapLandmark landmark = map_landmarks.landmark_list_[i];

    //calculate distance between particle and map landmark
    double distance = dist(particle.x, particle.y, landmark.x, landmark.y);

    //check if distance in sensor range, then add to list
    if (distance <= sensor_range) {
      filtered_map_landmarks.push_back(landmark);
    }
  }

  return filtered_map_landmarks;
}

Map::MapLandmark ParticleFilter::FindAssociatedMapLandmark(const LandmarkObs &transformed_observation,
                                                           const std::vector<Map::MapLandmark> &map_landmarks) {

  double min_distance = dist(transformed_observation.x, transformed_observation.y,
      map_landmarks[0].x, map_landmarks[0].y);
  int min_index = 0;

  size_t map_landmarks_count = map_landmarks.size();
  for (int i = 1; i < map_landmarks_count; ++i) {
    //calculate euclidean distance between predicted observation and map landmark
    double distance = dist(transformed_observation.x, transformed_observation.y,
                           map_landmarks[i].x, map_landmarks[i].y);

    //if new landmark is closer than previous nearest then mark
    //the current landmark as nearest
    if(distance < min_distance) {
      min_distance = distance;
      min_index = i;
    }
  }

  return map_landmarks[min_index];
}

LandmarkObs ParticleFilter::TransformToMapCoordinates(const Particle & particle, LandmarkObs observation) {
  //x, y to be transformed
  double x = observation.x;
  double y = observation.y;

  //x and y with with respect to which
  //observation needs to be transformed
  double tx = particle.x;
  double ty = particle.y;
  double theta = particle.theta;

  LandmarkObs transformed_observation;
  //apply rotation to align vehicle coordinate system and map coordinate system
  //followed by translation to translate observation with respect to particle position
  transformed_observation.id = observation.id;
  transformed_observation.x = x*cos(theta) + y*sin(theta) + tx;
  transformed_observation.y = -1*x*sin(theta) + y*cos(theta) + ty;

  return transformed_observation;
}

double ParticleFilter::CalculateLikelihood(LandmarkObs observation,
                                           Map::MapLandmark map_landmark,
                                           double std_landmark[]) {
  double x = observation.x;
  double y = observation.y;
  double ux = map_landmark.x;
  double uy = map_landmark.y;
  double std_ux = std_landmark[0];
  double std_uy = std_landmark[1];

  double c1 = 1.0/(2 * M_PI * std_ux * std_uy);
  double c2_x = ((x-ux) * (x-ux)) / (2 * std_ux * std_ux);
  double c2_y = ((y-uy) * (y-uy)) / (2 * std_uy * std_uy);

  return c1 * exp(-(c2_x + c2_y));
}

void ParticleFilter::resample() {
	// Resample particles with replacement with probability proportional to their weight.
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

  //create a vector of weights with index of vector same as index of particle
  //in particles vector
  std::vector<double> weights(num_particles);
  weights.resize(num_particles);
  for (int i = 0; i < num_particles; ++i) {
    weights[i] = particles[i].weight;
  }

  //create a random number generator engine
  std::default_random_engine generator;
  //create discrete distribution which generates numbers (indexes)
  //according to their weights (weight at that index)
  std::discrete_distribution<int> discrete_distribution(weights.begin(), weights.end());

  //vector to hold resampled_particles
  std::vector<Particle> resampled_particles(num_particles);
  resampled_particles.resize(num_particles);

  //resample particles using discrete distribution
  for (int i = 0; i < num_particles; ++i) {
    int particle_index = discrete_distribution(generator);
    resampled_particles[i] = particles[particle_index];
  }

  //assign resampled_particles back to particles vector
  //so that now particle filter holds updated re-sampled particles vector
  particles = resampled_particles;
}

void ParticleFilter::write(std::string filename) {
	// You don't need to modify this file.
	std::ofstream dataFile;
	dataFile.open(filename, std::ios::app);
	for (int i = 0; i < num_particles; ++i) {
		dataFile << particles[i].x << " " << particles[i].y << " " << particles[i].theta << "\n";
	}
	dataFile.close();
}
