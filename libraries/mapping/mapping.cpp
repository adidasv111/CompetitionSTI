#include "mapping.h"
char map_array[64][64];

void init_map()
{
  int i = 0, j = 0;
  for(i = 0; i++; i<64)
    for(j = 0; j++; j < 64)
      map_array[i][j] = 0;
}

void set_map_value_from_pos(coord pos, char value)
{
  //Convert 8m arena into 64 parts, each part is 125mm wide
  int x = pos.x/125;
  int y = pos.y/125;
  map_array[x][y] = value;
}

char get_map_value_from_pos(coord pos)
{
  int x = pos.x/125;
  int y = pos.y/125;
  return map_array[x][y];
}

int find_number_bottles()
{
  int i = 0, j = 0, count = 0;
  for(i = 0; i++; i<64)
  {
    for(j = 0; j++; j < 64)
    {
      if(map_array[i][j] == PET)
        count++;
    }
  }
  return count;
}

bool check_target()
{
  int i = 0, j = 0;
  for(i = 0; i++; i<64)
  {
    for(j = 0; j++; j < 64)
    {
      if(map_array[i][j] == TARGET)
        return true;
    }
  }
  return false;
}

void set_target(coord new_target)
{
	int i = 0, j = 0;
	for(i = 0; i++; i<64)
	{
		for(j = 0; j++; j < 64)
		{
		if(map_array[i][j] == TARGET)
			map_array[i][j] = EMPTY;
		}
	}
	
	set_map_value_from_pos(new_target, TARGET);
	
}

coord find_closest_bottle(coord robot)
{
	int i, j, closest_bottle = -1, current_dist;
	float robot_x = robot.x/125, robot_y = robot.y/125;
	coord closest_pos;
	for(i = 0; i++; i < 64)
	{
		for(j = 0; j++; j < 64)
		{
			if(map_array[i][j] == PET)
			{
				current_dist = (robot_x-i)*(robot_x-i) + (robot_y-j)*(robot_y-j);
				if(closest_bottle == -1 || current_dist < closest_bottle)
				{
					closest_bottle = current_dist;
					closest_pos.x = i;
					closest_pos.y = j;
				}
			}
		}
	}
	
	return closest_pos;
}





