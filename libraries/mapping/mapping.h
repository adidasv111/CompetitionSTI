#ifndef MAPPING_H
#define MAPPING_H
 
#include <Arduino.h>
#include <constant.h>


typedef struct coordinate{
	float x;
	float y;
} coord;

void init_map();
void set_map_value_from_pos(coord pos, char value);
char get_map_value_from_pos(coord pos);
int find_number_bottles();
bool check_target();
coord find_closest_bottle(coord robot);
void set_target(coord new_target);

extern char map_array[64][64];
#endif
