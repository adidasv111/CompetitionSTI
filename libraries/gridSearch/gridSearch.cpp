#include "gridSearch.h"

float current_grid[2] = {0, 0};

bool out_of_grid()
{
	if(robotPosition[0] != current_grid[0] && robotPosition[1] != current_grid[1])
		return true;
	else
		return false;
}

void grid_search(float* current_grid_1, float* current_grid_2)
{
	if(!out_of_grid()) //if true go to the current grid
	{
		//state = GO_TO_NEXT_GRID;
		*current_grid_1 += 125;
		*current_grid_2 += 125;
	}
  
	current_grid[0] = *current_grid_1;
	current_grid[1] = *current_grid_2;
}