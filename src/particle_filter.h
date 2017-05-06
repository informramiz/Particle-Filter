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
	
	// Set of current particles
	std::vector<Particle> particles;

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
	void init(double x, double y, double theta, double std[]);

	/**
	 * prediction Predicts the state for the next time step
	 *   using the process model.
	 * @param delta_t Time between time step t and t+1 in measurements [s]
	 * @param std_pos[] Array of dimension 3 [standard deviation of x [m], standard deviation of y [m]
	 *   standard deviation of yaw [rad]]
	 * @param velocity Velocity of car from t to t+1 [m/s]
	 * @param yaw_rate Yaw rate of car from t to t+1 [rad/s]
	 */
	void prediction(double delta_t, double std_pos[], double velocity, double yaw_rate);
	
	/**
	 * dataAssociation Finds which observations correspond to which landmarks (likely by using
	 *   a nearest-neighbors data association).
	 * @param predicted Vector of predicted landmark observations
	 * @param observations Vector of landmark observations
	 */
	void dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations);
	
	/**
	 * updateWeights Updates the weights for each particle based on the likelihood of the 
	 *   observed measurements. 
	 * @param sensor_range Range [m] of sensor
	 * @param std_landmark[] Array of dimension 2 [standard deviation of range [m],
	 *   standard deviation of bearing [rad]]
	 * @param observations Vector of landmark observations
	 * @param map Map class containing map landmarks
	 */
	void updateWeights(double sensor_range, double std_landmark[], std::vector<LandmarkObs> observations,
			Map map_landmarks);
	
	/**
	 * resample Resamples from the updated set of particles to form
	 *   the new set of particles.
	 */
	void resample();
	
	/*
	 * write Writes particle positions to a file.
	 * @param filename File to write particle positions to.
	 */
	void write(std::string filename);
	
	/**
	 * initialized Returns whether particle filter is initialized yet or not.
	 */
	const bool initialized() const {
		return is_initialized;
	}

	/**
	 * @param x, x position of observation on map coordinates
	 * @param y, y position of observation on map coordinates
	 * @param ux, x position of landmark on map coordinates
	 * @param uy, y position of landmark on map coordinates
	 * @param std_ux, uncertainty/standard deviation in x position of landmark
	 * @param std_uy, uncertainty/standard deviation in y position of landmark
	 */
	double CalculateLikelihood(double x, double y,
	                           double ux, double uy,
	                           double std_ux, double std_uy);
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
private:

	// Number of particles to draw
	  int num_particles;

	  // Flag, if filter is initialized
	  bool is_initialized;

	  // Vector of weights of all particles
	  std::vector<double> weights;
};



#endif /* PARTICLE_FILTER_H_ */
