/*
 * map.h
 *
 *  Created on: May 16, 2016
 *      Author: Ramiz Raja
 */

#ifndef MAP_H_
#define MAP_H_

class Map {
public:
	
	struct MapLandmark{
		int id ; // Landmark ID
		float x; // Landmark x-position in the map (global coordinates)
		float y; // Landmark y-position in the map (global coordinates)
	};

	std::vector<MapLandmark> landmark_list_ ; // List of landmarks in the map

};



#endif /* MAP_H_ */
