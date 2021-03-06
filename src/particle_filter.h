/*
 * particle_filter.h
 *
 * 2D particle filter class.
 *  Created on: May 16, 2016
 *      Author: Ramiz Raja
 */

#ifndef PARTICLE_FILTER_H_
#define PARTICLE_FILTER_H_

#include "helper_functions.h"
#include "map.h"

struct Particle {

	int id;
	double x;
	double y;
	double theta;
	double weight;
};



class ParticleFilter {
public:
	
	// Constructor
	ParticleFilter();

	// Destructor
	~ParticleFilter() {}

	/**
	 * init Initializes particle filter by initializing particles to Gaussian
	 *   distribution around first position and all the weights to 1.
	 * @param x Initial x position [m] (simulated estimate from GPS)
	 * @param y Initial y position [m]
	 * @param theta Initial orientation [rad]
	 * @param std[] Array of dimension 3 [standard deviation of x [m], standard deviation of y [m]
	 *   standard deviation of yaw [rad]]
	 */
	void Init(double x, double y, double theta, double std[]);

	/**
	 * prediction Predicts the state for the next time step
	 *   using the process model.
	 * @param delta_t Time between time step t and t+1 in measurements [s]
	 * @param std_pos[] Array of dimension 3 [standard deviation of x [m], standard deviation of y [m]
	 *   standard deviation of yaw [rad]]
	 * @param velocity Velocity of car from t to t+1 [m/s]
	 * @param yaw_rate Yaw rate of car from t to t+1 [rad/s]
	 */
	void Prediction(double delta_t, double std_pos[], double velocity, double yaw_rate);
	
	/**
	 * updateWeights Updates the weights for each particle based on the likelihood of the 
	 *   observed measurements. 
	 * @param sensor_range Range [m] of sensor
	 * @param std_landmark[] Array of dimension 2 [standard deviation of x [m],
   *   standard deviation of y [m]]
	 * @param observations Vector of landmark observations
	 * @param map Map class containing map landmarks
	 */
	void UpdateWeights(double sensor_range, double std_landmark[], std::vector<LandmarkObs> observations,
			Map map_landmarks);
	
	/**
	 * resample Resamples from the updated set of particles to form
	 *   the new set of particles.
	 */
	void Resample();
	
	/*
	 * write Writes particle positions to a file.
	 * @param filename File to write particle positions to.
	 */
	void Write(std::string filename);
	
	/**
	 * initialized Returns whether particle filter is initialized yet or not.
	 */
	const bool IsInitialized() const {
		return is_initialized_;
	}

  int GetNumParticles() const {
    return num_particles_;
  }

  const std::vector<Particle>& GetParticles() const {
    return particles_;
  }

private:

  /**
   * Returns likelihood of @param observation given @map_landmark and standard
   * deviation @param std_landmark
   *
   * @param observation     observation for which to calculate likelihood
   * @param map_landmark    actual map landmark as given condition for likelihood
   * @param std_landmark[]  Array of dimension 2 [standard deviation of x [m],
   *   standard deviation of y [m]]
   */
  double CalculateLikelihood(LandmarkObs observation,
                             Map::MapLandmark map_landmark,
                             double std_landmark[]);
  /**
   * Transforms a given observation to map coordinates from vehicle coordinates with respect
   * to given particle position along with heading direction of particle in
   * map coordinates.
   *
   * @param particle, particle position in map coordinates with respect to which observation needs to be transformed
   * @param observation, observation to be transform to map coordinates from vehicle coordinates
   */
  LandmarkObs TransformToMapCoordinates(const Particle & particle, LandmarkObs observation);

  /**
   * Filters landmarks according to sensor range given.
   *
   * @param particle        position of particle on map from which to check distance of landmark
   * @param map_landmarks   list of map landmarks
   * @param sensor_range    range of sensor in meters
   * @return  std::vector<MapLandmark>, list of filtered map landmarks
   */
  std::vector<Map::MapLandmark> FilterMapLandmarks(const Particle& particle,
                                                  const Map& map_landmarks,
                                                  double sensor_range);

  /**
   * Find closest map landmark for the passed transformed observation.
   *
   * @param transformed_observation   sensor observation transformed in map-coordinates.
   * @param map_landmakrs             list of map landmarks which are within sensor range.
   * @return Map::MapLandmark         map landmark closest to @transformed_observation
   */
  Map::MapLandmark FindAssociatedMapLandmark(const LandmarkObs &transformed_observation,
                                 const std::vector<Map::MapLandmark> &map_landmarks);

  /**
   * Calculates the weight for the passed @particle
   *
   * @param particle        particle to calculate weight of
   * @param observations    list of sensor observations
   * @param map             map of landmarks to compare observations to
   * @param sensor_range    range of sensor in meters
   * @param std_landmark[]  Array of dimension 2 [standard deviation of x [m],
   *   standard deviation of y [m]]
   */
  double CalculateParticleWeight(const Particle &particle,
                                 const std::vector<LandmarkObs> &observations,
                                 const Map &map,
                                 double sensor_range,
                                 double std_landmark[]);

  // Set of current particles
  std::vector<Particle> particles_;

  // Number of particles to draw
  int num_particles_;

  // Flag, if filter is initialized
  bool is_initialized_;

  // Vector of weights of all particles
  std::vector<double> weights_;
};



#endif /* PARTICLE_FILTER_H_ */
