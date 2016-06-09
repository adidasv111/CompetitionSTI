#ifndef MAPPING_H
#define MAPPING_H
 
#include <Arduino.h>
#include <constant.h>


typedef struct coordinate{
	float x;
	float y;
} coord;

/********Linked List****************/
struct bottle {
	public:
		//char type;
		coord location;
		bottle* next;
};

class gridMap {
private:
	bottle * head;
	int listLength;
	
public:
	//Constructor
	gridMap();
	
	/*Getter Finds the length of the list
	  * Returns the number of bottles
	  */
	int getListLength();
	
	/*Setter adds a note to the list at a given position.
	 * Takes a node and list position as parameters
	 * Position must be between 1 and the total number of nodes.
	 * Returns true if the operation is successful.
	 */
	bool insertBottle(bottle *newBottle, int position, coord argLocation);
	
	/*Setter removes a node by its given position
	 * Returns true if the operation is successful
	 */
	bool removeBottle(int position);
	  
	/* Checks if a target is present,
	 * Returns true if a target is present
	 */
	coord findClosestBottle(float *robotPosition);
	 
	//Destructor
	~gridMap();
};



void init_map();
void set_map_value_from_pos(coord pos, char value);
char get_map_value_from_pos(coord pos);
int find_number_bottles();
bool check_target();
coord find_closest_bottle(coord robot);
void set_target(coord new_target);

extern coord waypoints[NB_WAYPOINTS];
extern char currentWaypoint;
extern char map_array[64][64];
#endif