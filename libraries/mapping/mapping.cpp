#include "mapping.h"
char map_array[64][64];

coord waypoints[NB_WAYPOINTS];
char currentWaypoint;

bottle bottlesArray[NB_BOTTLES_ARRAY];

//initiliaze waypoints array
void init_waypoints()
{
	//waypoints without full
	waypoints[0].x = 500;		waypoints[0].y = 5000; 	//calibrate left wall left
	waypoints[1].x = 1500;		waypoints[1].y = 5000;
	waypoints[2].x = 1500;		waypoints[2].y = 2500;	//goHome
	//Home
	waypoints[3].x = 2500;		waypoints[3].y = 500;	//calibrate right wall bottom
	waypoints[4].x = 2500;		waypoints[4].y = 5000;
	waypoints[5].x = 3500;		waypoints[5].y = 5000;
	waypoints[6].x = 3500;		waypoints[6].y = 2500;
	waypoints[7].x = 500;		waypoints[7].y = 2500;
	waypoints[8].x = 500;		waypoints[8].y = 1500;	//calibrate right wall left, goHome
	//Home
	waypoints[9].x = 3000;		waypoints[9].y = 4500; 
	waypoints[10].x = 3000;		waypoints[10].y = 7500;
	waypoints[11].x = 4500;		waypoints[11].y = 7500; //calibrate left wall top
	waypoints[12].x = 4000;		waypoints[12].y = 4500;	//goHome
	//Home
	waypoints[13].x = 500;		waypoints[13].y = 2500;	//calibrate left wall left
	waypoints[14].x = 7500;		waypoints[14].y = 3000;
	waypoints[15].x = 7500;		waypoints[15].y = 3500; //calibrate right wall right
	waypoints[16].x = 4500;		waypoints[16].y = 3500;	//goHome
	//Home
	waypoints[17].x = 1500;		waypoints[17].y = 500;	//calibrate right wall bottom
	waypoints[18].x = 4000;		waypoints[18].y = 4500;
	waypoints[19].x = 7500;		waypoints[19].y = 4500;
	waypoints[20].x = 7500;		waypoints[20].y = 5500; //calibrate right wall right
	waypoints[21].x = 4000;		waypoints[21].y = 5500;	//goHome
	//Home
	//--- grass ---
	waypoints[22].x = 7500;		waypoints[22].y = 500;	//calibrate right wall bottom
	waypoints[23].x = 7500;		waypoints[23].y = 1500;	//calibrate right wall right
	waypoints[24].x = 4000;		waypoints[24].y = 1500; //goHome
	//Home
	//--- stone motherf*@#errs!!!
	waypoints[25].x = 500;		waypoints[25].y = 5000;	//calibrate left wall left
	waypoints[26].x = 500;		waypoints[26].y = 6500;	//calibrate left wall left
	waypoints[27].x = 500;		waypoints[27].y = 7500;	//calibrate left wall left
	waypoints[28].x = 1000;		waypoints[28].y = 7500;	//calibrate left wall top
	waypoints[29].x = 1500;		waypoints[29].y = 7500;	//calibrate left wall top
	waypoints[30].x = 1500;		waypoints[30].y = 6500;
	//stop at this point!!!!!

	/******Waypoints starting with grass*********
	waypoints[0].x = 4000;		waypoints[0].y = 700;
	//--------------
	/*waypoints[1].x = 4000;		waypoints[1].y = 1500;
	waypoints[2].x = 4000;		waypoints[2].y = 3000;
	waypoints[3].x = 500;		waypoints[3].y = 3000;
	waypoints[4].x = 500;		waypoints[4].y = 1500;

	waypoints[1].x = 7500;		waypoints[1].y = 500;
	waypoints[2].x = 7500;		waypoints[2].y = 1500;
	waypoints[3].x = 4000;		waypoints[3].y = 1500;
	waypoints[4].x = 500;		waypoints[4].y = 1500;

	waypoints[5].x = 500;		waypoints[5].y = 2500;
	waypoints[6].x = 4000;		waypoints[6].y = 2500;
	waypoints[7].x = 7500;		waypoints[7].y = 2500;
	waypoints[8].x = 7500;		waypoints[8].y = 3500;
	waypoints[9].x = 4000;		waypoints[9].y = 3500;
	waypoints[10].x = 500;		waypoints[10].y = 3500;
	waypoints[11].x = 500;		waypoints[11].y = 4500;
	waypoints[12].x = 4000;		waypoints[12].y = 4500;
	waypoints[13].x = 7500;		waypoints[13].y = 4500;
	waypoints[14].x = 7500;		waypoints[14].y = 5500;
	waypoints[15].x = 4000;		waypoints[15].y = 5500;
	waypoints[16].x = 4500;		waypoints[16].y = 6500;
	waypoints[17].x = 4500;		waypoints[17].y = 7500;
	waypoints[18].x = 3000;		waypoints[18].y = 7500;
	waypoints[19].x = 3000;		waypoints[19].y = 4500;
	waypoints[20].x = 500;		waypoints[20].y = 500;
	waypoints[21].x = 500;		waypoints[21].y = 4500;
	waypoints[22].x = 500;		waypoints[22].y = 7500;
	waypoints[23].x = 1500;		waypoints[23].y = 7500;
	waypoints[24].x = 1500;		waypoints[24].y = 4000;
	waypoints[25].x = 500;		waypoints[25].y = 500;*/
	
	/*********Waypoints ending with grass*************/
	/*waypoints[0].x = 500;		waypoints[0].y = 2500; //calibrate left wall left
	waypoints[1].x = 500;		waypoints[1].y = 5000; //calibrate left wall left
	waypoints[2].x = 1500;		waypoints[2].y = 5000;
	waypoints[3].x = 2500;		waypoints[3].y = 5000;
	waypoints[4].x = 2500;		waypoints[4].y = 2500;
	waypoints[5].x = 3000;		waypoints[5].y = 2500;
	waypoints[6].x = 3000;		waypoints[6].y = 5500;
	waypoints[7].x = 3000;		waypoints[7].y = 7500;
	waypoints[8].x = 4500;		waypoints[8].y = 7500;//calibrate left wall top
	waypoints[9].x = 4500;		waypoints[9].y = 5000; 
	waypoints[10].x = 7500;		waypoints[10].y = 5000;
	waypoints[11].x = 7500;		waypoints[11].y = 4500; //calibrate left wall right
	waypoints[12].x = 4000;		waypoints[12].y = 4500;
	waypoints[13].x = 4000;		waypoints[13].y = 3500;
	waypoints[14].x = 7500;		waypoints[14].y = 3500;
	waypoints[15].x = 7500;		waypoints[15].y = 2500; //calibrate left wall right
	waypoints[16].x = 2500;		waypoints[16].y = 2500;
	waypoints[17].x = 2500;		waypoints[17].y = 500;
	waypoints[18].x = 4000;		waypoints[18].y = 500; //calibrate right wall bottom
	waypoints[19].x = 7500;		waypoints[19].y = 500;
	waypoints[20].x = 7500;		waypoints[20].y = 1500; //calibrate right wall bottom
	waypoints[21].x = 4000;		waypoints[21].y = 1500;
	waypoints[22].x = 2000;		waypoints[22].y = 1500;
	waypoints[23].x = 2000;		waypoints[23].y = 500;
	waypoints[24].x = 500;		waypoints[24].y = 500; //calibrate left wall bottom
	waypoints[25].x = 500;		waypoints[25].y = 7500;*/
}

//initiliaze bottles array
void init_bottlesArray()
{
	for (int i = 0; i < NB_BOTTLES_ARRAY; i++)
	{
		bottlesArray[i].type = EMPTY;
		bottlesArray[i].location.x = -1;
		bottlesArray[i].location.y = -1;
	}
}

//insert newly found bottle to array
bool insertBottle(coord newBottlePos)
{
	for (int i = 0; i < NB_BOTTLES_ARRAY; i++)
	{
		if (bottlesArray[i].type == EMPTY)
		{
			bottlesArray[i].type = BOTTLE;
			bottlesArray[i].location.x = newBottlePos.x;
			bottlesArray[i].location.y = newBottlePos.y;
			return true;
		}
	}
	Serial.println("Error: Could not add bottle to bottleArray");
	return false;
}

//check if target exist
//	Returns true if a target is present
bool checkTargetExist()	
{
	for (int i = 0; i < NB_BOTTLES_ARRAY; i++)
	{
		if (bottlesArray[i].type == TARGET)
		{
			return true;
		}
	}
	return false;
}

//remove current target from array, set cell to empty
bool removeTarget()
{
	for (int i = 0; i < NB_BOTTLES_ARRAY; i++)
	{
		if (bottlesArray[i].type == TARGET)
		{
			bottlesArray[i].type = EMPTY;
			bottlesArray[i].location.x = -1;
			bottlesArray[i].location.y = -1;
			return true;
		}
	}
	Serial.println("Error: No target present");
	return false;
}

//find closest bottle and set it as target
coord findClosestBottle(float *robotPosition)
{
	float closestDistance = 9999999999999;
	float currentDistance;
	coord closestBottleCoord;
	char closestIndex = 0;
	for (int i = 0; i < NB_BOTTLES_ARRAY; i++)
	{
		currentDistance = (robotPosition[0] - bottlesArray[i].location.x)*(robotPosition[0] - bottlesArray[i].location.x) + 
							(robotPosition[1] - bottlesArray[i].location.y)*(robotPosition[1] - bottlesArray[i].location.x);
		if(currentDistance < closestDistance)
		{
			closestDistance = currentDistance;
			closestBottleCoord = bottlesArray[i].location;
			closestIndex = i;
		}
	}
	if (closestDistance == 9999999999999)
	{
		closestBottleCoord.x = -1;
		closestBottleCoord.y = -1;
	}
	else
	{
		bottlesArray[closestIndex].type = TARGET;
	}
	return closestBottleCoord;
}
/*******Linked List**********/
/*
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

*/
/*
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
*/