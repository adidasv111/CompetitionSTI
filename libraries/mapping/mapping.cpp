#include "mapping.h"
char map_array[64][64];
coord waypoints[NB_WAYPOINTS];
char currentWaypoint;

void init_waypoint()
{
	waypoints[0].x = 4;
	waypoints[0].y = 0.5;
	waypoints[1].x = 7.5;
	waypoints[1].y = 0.5;
	waypoints[2].x = 7.5;
	waypoints[2].y = 1.5;
	waypoints[3].x = 4;
	waypoints[3].y = 1.5;
	waypoints[4].x = 0.5;
	waypoints[4].y = 1.5;
	waypoints[5].x = 0.5;
	waypoints[5].y = 2.5;
	waypoints[6].x = 4;
	waypoints[6].y = 2.5;
	waypoints[7].x = 7.5;
	waypoints[7].y = 2.5;
	waypoints[8].x = 7.5;
	waypoints[8].y = 3.5;
	waypoints[9].x = 4;
	waypoints[9].y = 3.5;
	waypoints[10].x = 0.5;
	waypoints[10].y = 3.5;
	waypoints[11].x = 0.5;
	waypoints[11].y = 4.5;
	waypoints[12].x = 4;
	waypoints[12].y = 4.5;
	waypoints[13].x = 7.5;
	waypoints[13].y = 4.5;
	waypoints[14].x = 7.5;
	waypoints[14].y = 5.5;
	waypoints[15].x = 4;
	waypoints[15].y = 5.5;
	waypoints[16].x = 4.5;
	waypoints[16].y = 6.5;
	waypoints[17].x = 4.5;
	waypoints[17].y = 7.5;
	waypoints[18].x = 3;
	waypoints[18].y = 7.5;
	waypoints[19].x = 3;
	waypoints[19].y = 4.5;
	waypoints[20].x = 0.5;
	waypoints[20].y = 0.5;
	waypoints[21].x = 0.5;
	waypoints[21].y = 4.5;
	waypoints[22].x = 0.5;
	waypoints[22].y = 7.5;
	waypoints[23].x = 1.5;
	waypoints[23].y = 7.5;
	waypoints[24].x = 1.5;
	waypoints[24].y = 4;
	waypoints[25].x = 0.5;
	waypoints[25].y = 0.5;
}

/*******Linked List**********/
//Default constructor creates the head node
gridMap::gridMap()
{
	//head -> type = 0;
	head -> location.x = 0;
	head -> location.y = 0;
	head -> next = NULL;
	listLength = 0;
}

bool gridMap::insertBottle(bottle * newBottle, int position, coord argLocation)
{
	//newBottle -> type = argType;
	newBottle -> location 	 = argLocation;
	if((position <= 0) || (position > listLength + 1))
	{
		return false;
	}
	if(head -> next == NULL)
	{
		head -> next = newBottle;
		listLength++;
		return true;
	}
	
	int count = 0;
	bottle *p = head;
	bottle * q = head;
	while(q)
	{
		if(count == position)
		{
			p -> next = newBottle;
			newBottle -> next = q;
			listLength++;
			return true;
		}
		p = q;
		q = p -> next;
		count++;
	}
	if(count == position)
	{
		p -> next = newBottle;
		newBottle -> next = q;
		listLength++;
		return true;
	}
	return false;
}

bool gridMap::removeBottle(int position)
{
	if ((position <= 0) || (position > listLength + 1))
    {
        return false;
    }
    if (head -> next == NULL)
    {
       return false;
    }
    int count = 0;
    bottle * p = head;
    bottle * q = head;
    while (q) 
    {
        if (count == position)
        {
            p -> next = q -> next;
            delete q;
            listLength--;
            return true;
        }
        p = q;
        q = p -> next;
        count++;
    }
    return false;
}

// Destructor de-allocates memory used by the list.
gridMap::~gridMap() 
{
    bottle * p = head;
    bottle * q = head;
    while (q)
    {
        p = q;
        q = p -> next;
        if (q) delete p;
    }
}

int gridMap::getListLength()
{
	return listLength;
}

coord gridMap::findClosestBottle(float *robotPosition)
{
	bottle *current = head;
	float closestDistance = -1, currentDistance;
	coord closestBottle;
	if(listLength == 0)
	{
		closestBottle.x = -1;
		closestBottle.y = -1;
	}
	else
	{
		while(current)
		{
			currentDistance = (robotPosition[0] - current->location.x)*(robotPosition[0] - current->location.x) + (robotPosition[1] - current->location.y)*(robotPosition[1] - current->location.y);
			if(closestDistance == -1 || currentDistance < closestDistance)
			{
				closestDistance = currentDistance;
				closestBottle = current->location;
			}
		}
	}
	return closestBottle;
}



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





