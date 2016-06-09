#ifndef MAPPING_H
#define MAPPING_H

#include <Arduino.h>
#include <constant.h>

typedef struct coordinate
{
	float x;
	float y;
} coord;

typedef struct bottlePET
{
	char type;
	coord location;
} bottle;

	void init_waypoints();			//initiliaze waypoints array
	void init_bottlesArray();		//initiliaze bottles array

	bool insertBottle(coord newBottlePos);	//insert newly found bottle to array
	bool checkTargetExist();				//check if target exist
	bool removeTarget();					//remove current target from array
	coord findClosestBottle(float *robotPosition);	//find closest bottle and set it as target

	extern coord waypoints[NB_WAYPOINTS];
	extern char currentWaypoint;
	extern bottle bottlesArray[NB_BOTTLES_ARRAY];
	 /*
class gridMap {
private:
	bottle * head;
	int listLength;
	
public:
	//Constructor
	gridMap();
	

	 
	//Destructor
	 ~gridMap();
	};


	/*Getter Finds the length of the list
	  * Returns the number of bottles
	  */
	//int getListLength();
/*	void init_map();
	void set_map_value_from_pos(coord pos, char value);
	char get_map_value_from_pos(coord pos);
	int find_number_bottles();
	
	

	extern char map_array[64][64];
*/

#endif