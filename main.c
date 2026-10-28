#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#define MAX_AIR_ROUTE 5
#define MAX_NEIGHBOURS 6
#define MAX_NODES 4096 // Max value needed of nodes needed for the project
#define DIST_DIJKSTRA (-1)

// -- STRUCT AND GLOBAL VARIABLES  --

typedef struct {
    int x_end, y_end;
    int cost;
} air_route;

typedef struct {
    int cost, distance;
    air_route *route;
} hex_map;

// Dynamic hex grid
hex_map **grid = NULL;
int current_rows = 0, current_columns = 0;

typedef struct {
    int x, y;
} axial_coordinates;

// Neighbor offsets for even rows
axial_coordinates directions_even[6] = {
    {-1,-1}, {-1,0}, {-1,1},
    {0,1},{1,0}, {0,-1}
};

// Neighbor offsets for odd rows
axial_coordinates directions_odd[6] = {
    {0,-1}, {-1,0}, {0,1},
    {1,1},{1,0}, {1,-1}
};

axial_coordinates neighbours[MAX_NEIGHBOURS] = {0};

typedef struct {
    axial_coordinates coordinates;
    int distance;
} heap_node;
heap_node min;

typedef struct {
    int size;
    heap_node node[MAX_NODES];
} heap;

// Linked list to cache computed travel costs
typedef struct cache {
    int xp, yp, xd, yd, cost;
    struct cache *next;
} cache;

cache *route = NULL;

// -- FUNCTIONS --

void init(int x, int y);
void change_cost(int x, int y, int v, int range);
void calc_ring(int x, int y, int distance, axial_coordinates *ring);
void toggle_air_route(int x1, int y1, int x2, int y2);
void travel_cost(int xp, int yp, int xd, int yd);
int find_neighbours(int x, int y);
void insert_node(heap* min_heap, int x, int y, int distance);
void order_heap_up(heap* min_heap, int i);
void extract_node(heap* min_heap);
void order_heap_down(heap* min_heap, int i);
void clear_cache();

// Parse and route standard input commands
int main() {
    char command[20];
    int x, y, v, range, x1, y1, x2, y2;

    while (scanf("%s", command) != EOF) {
        if (strcmp(command, "travel_cost") == 0) {
            if (scanf("%d %d %d %d", &x1, &y1, &x2, &y2) == 4)
                travel_cost(x1, y1, x2, y2);
        }
        else if (strcmp(command, "change_cost") == 0) {
            if (scanf("%d %d %d %d", &x, &y, &v, &range) == 4)
                change_cost(x, y, v, range);
        }
        else if (strcmp(command, "init") == 0) {
            if (scanf("%d %d", &x, &y) == 2)
                init(x, y);
        }
        else {
            if (scanf("%d %d %d %d", &x1, &y1, &x2, &y2) == 4)
                toggle_air_route(x1, y1, x2, y2);
        }
    }
    return 0;
}

// Initialize or re-initialize the odd-r hex grid
void init(int x, int y) {
    if (grid != NULL) {
        for (int k = 0; k < current_rows; k++) {
            for (int l = 0; l < current_columns; l++) {
                if (grid[k][l].route != NULL) {
                    free(grid[k][l].route);
                }
            }
            free(grid[k]);
            }
        free(grid);
        grid = NULL;
    }
    clear_cache();

    current_rows = y;
    current_columns = x;
    grid = malloc(current_rows * sizeof(hex_map *));
    for (int i = 0; i < current_rows; i++) {
        grid[i] = malloc(current_columns * sizeof(hex_map));
        for (int j = 0; j < current_columns; j++) {
            grid[i][j].cost = 1;
            grid[i][j].route = NULL;
        }
    }
    printf("OK\n");
}

// Update costs for the central hex and its surrounding rings
void change_cost(int x, int y, int v, int range) {
    int row, column, result, distance = 1, i = 0, j = 0;
    float part_result, max;

    if (grid == NULL || range <= 0 || v < -10 || v > 10 || x < 0 || y < 0 || x >= current_columns || y >= current_rows) {
        printf("KO\n");
        return;
    }

    // set the cost for the central hex and its air routes
    grid[y][x].cost = grid[y][x].cost + v;
    if (grid[y][x].cost < 0)
        grid[y][x].cost = 0;
    if (grid[y][x].cost > 100)
        grid[y][x].cost = 100;
    if (grid[y][x].route != NULL) {
        while (grid[y][x].route[j].cost != -1 && j < MAX_AIR_ROUTE) {
            grid[y][x].route[j].cost = grid[y][x].cost;
            j++;
        }
    }

    // array to contain the rings around the central hex, sized for the max possible distance (range)
    axial_coordinates ring[range * 6];

    // Process outward rings up to the specified range
    while (distance < range) {

        part_result = (float)(range - distance)/(float)range;
        if (part_result > 0)
            max = part_result;
        else max = 0;
        result = (int)floorf((float)v * max);

        calc_ring(x, y, distance, ring);
        for (int k = 0; k < distance * 6; k++) {
            row = ring[k].y;
            column = ring[k].x;
            if (row < 0 || row >= current_rows || column < 0 || column >= current_columns)
                continue;

            grid[row][column].cost = grid[row][column].cost + result;
            if (grid[row][column].cost < 0)
                grid[row][column].cost = 0;
            if (grid[row][column].cost > 100)
                grid[row][column].cost = 100;
            if (grid[row][column].route != NULL) {
                i = 0;
                while (grid[row][column].route[i].cost != -1 && i < MAX_AIR_ROUTE) {
                    grid[row][column].route[i].cost = grid[row][column].cost;
                    i++;
                }
            }
        }
        distance++;
    }
    clear_cache();
    printf("OK\n");
}

// Return an array of all hex coordinates forming a ring at a given distance
void calc_ring(int x, int y, int distance, axial_coordinates *ring) {
    int current_x = x + distance, current_y = y, neighbours = 0;

    for (int i = 0; i < MAX_NEIGHBOURS; i++) {
        for (int j = 0; j < distance; j++) {
            if (current_y % 2 == 0) {
                ring[neighbours].x = current_x + directions_even[i].x;
                ring[neighbours].y = current_y + directions_even[i].y;
            }
            else {
                ring[neighbours].x = current_x + directions_odd[i].x;
                ring[neighbours].y = current_y + directions_odd[i].y;
            }
            current_x = ring[neighbours].x;
            current_y = ring[neighbours].y;
            neighbours++;
        }
    }
}

// Toggle an air route between two hexes, max 5 routes per hex
void toggle_air_route(int x1, int y1, int x2, int y2) {
    int i = 0, j = 0;

    if (grid == NULL || x1 < 0 || x2 < 0 || y1 < 0 || y2 < 0 || y1 >= current_rows || y2 >= current_rows || x1 >= current_columns || x2 >= current_columns) {
        printf("KO\n");
        return;
    }

    if (grid[y1][x1].route == NULL) {
        grid[y1][x1].route = malloc (MAX_AIR_ROUTE * sizeof(air_route));
        for (int k = 0; k < MAX_AIR_ROUTE; k++)
            grid[y1][x1].route[k].cost = -1;
    }

    while (i < MAX_AIR_ROUTE && grid[y1][x1].route[i].cost != -1) {
        // Remove existing route and shift remaining elements
        if (grid[y1][x1].route[i].x_end == x2 && grid[y1][x1].route[i].y_end == y2) {
            j = i;
            while (j+1 < MAX_AIR_ROUTE && grid[y1][x1].route[j+1].cost != -1) {
                grid[y1][x1].route[j].x_end = grid[y1][x1].route[j+1].x_end;
                grid[y1][x1].route[j].y_end = grid[y1][x1].route[j+1].y_end;
                grid[y1][x1].route[j].cost = grid[y1][x1].route [j+1].cost;
                j++;
            }
            grid[y1][x1].route[j].x_end = 0;
            grid[y1][x1].route[j].y_end = 0;
            grid[y1][x1].route[j].cost = -1;

            if (grid[y1][x1].route[0].cost == -1) {
                free(grid[y1][x1].route);
                grid[y1][x1].route = NULL;
            }
            clear_cache();
            printf("OK\n");
            return;
        }
        i++;
    }

    if (i == 5) {
        printf("KO\n");
        return;
    }

    grid[y1][x1].route[i].y_end = y2;
    grid[y1][x1].route[i].x_end = x2;
    grid[y1][x1].route[i].cost = grid[y1][x1].cost;
    clear_cache();
    printf("OK\n");
}

// Dijkstra implementation to find the shortest path cost
void travel_cost(int xp, int yp, int xd, int yd) {
    int distance = 0, x = xp, y = yp, k = 0, current_neighbours = 0;
    cache *current_route = route;

    if (grid == NULL || xp < 0 || yp < 0 || xd < 0 || yd < 0 || yp >= current_rows || yd >= current_rows || xp >= current_columns || xd >= current_columns) {
        printf("-1\n");
        return;
    }

    if (xp == xd && yp == yd) {
        printf("0\n");
        return;
    }

    // Check cached routes
    while (current_route != NULL) {
        if (xp == current_route->xp && yp == current_route->yp && xd == current_route->xd && yd == current_route->yd){
            printf("%d\n", current_route->cost);
            return;
        }
        current_route = current_route->next;
    }

    // Set all distances to '-1'
    for (int i = 0; i < current_rows; i++) {
        for (int j = 0; j < current_columns; j++) {
            grid[i][j].distance = DIST_DIJKSTRA;
        }
    }
    heap* min_heap = malloc(sizeof(heap));
    min_heap->size = 0;
    insert_node(min_heap, xp, yp, distance);

    // Cycle to check all the routes until the destination
    while (x != xd || y != yd) {
        // Check if the heap is empty: the destination hex is unreachable
        if (min_heap->size <= 0) {
            printf("-1\n");
            current_route = route;
            route = malloc(sizeof(cache));
            route->xp = xp;
            route->yp = yp;
            route->xd = xd;
            route->yd = yd;
            route->cost = -1;
            route->next = current_route;
            free(min_heap);
            return;
        }

        // Extract the node with minimum distance and insert its neighbor and its air routes in the heap only if the distance is '-1'
        extract_node(min_heap);
        x = min.coordinates.x;
        y = min.coordinates.y;
        distance = min.distance;

        if (grid[y][x].cost == 0)
            continue;
        current_neighbours = find_neighbours(x, y);
        for (int l = 0; l < current_neighbours; l++) {
            if (grid[neighbours[l].y][neighbours[l].x].distance == DIST_DIJKSTRA)
                insert_node(min_heap, neighbours[l].x, neighbours[l].y, distance);
        }

        if (grid[y][x].route != NULL) {
            k = 0;
            while (grid[y][x].route[k].cost != -1){
                if (grid[grid[y][x].route[k].y_end][grid[y][x].route[k].x_end].distance == DIST_DIJKSTRA)
                    insert_node(min_heap, grid[y][x].route[k].x_end, grid[y][x].route[k].y_end, distance);
                k++;
            }
        }
    }

    // Destination found: save the route in the cache
    distance = distance - grid[y][x].cost;
    printf("%d\n", distance);
    current_route = route;
    route = malloc(sizeof(cache));
    route->xp = xp;
    route->yp = yp;
    route->xd = xd;
    route->yd = yd;
    route->cost = distance;
    route->next = current_route;
    free(min_heap);
}

// Calculate valid grid neighbors for a given target hex
int find_neighbours(int x, int y) {
    int current_x = x + 1, current_y = y, current_neighbours = 0;

    for (int i = 0; i < MAX_NEIGHBOURS; i++) {
        if (current_y % 2 == 0) {
            current_x = current_x + directions_even[i].x;
            current_y = current_y + directions_even[i].y;
        }
        else {
            current_x = current_x + directions_odd[i].x;
            current_y = current_y + directions_odd[i].y;
        }
        if (current_y < 0 || current_y >= current_rows || current_x < 0 || current_x >= current_columns) {
            continue;
        }
        neighbours[current_neighbours].x = current_x;
        neighbours[current_neighbours].y = current_y;
        current_neighbours++;
    }
    return current_neighbours;
}

// Insert the node at the end of the heap, then reorder it
void insert_node(heap* min_heap, int x, int y, int distance) {
    int i = min_heap->size;

    distance = distance + grid[y][x].cost;
    min_heap->node[i].coordinates.x = x;
    min_heap->node[i].coordinates.y = y;
    min_heap->node[i].distance = distance;
    grid[y][x].distance = distance;
    min_heap->size++;
    order_heap_up(min_heap, i);
}

// Sorts the heap upwards
void order_heap_up(heap* min_heap, int i) {
    if (i == 0)
        return;
    int parent = (i - 1) / 2;
    heap_node swap_node;

    if (min_heap->node[parent].distance > min_heap->node[i].distance) {
        swap_node = min_heap->node[parent];
        min_heap->node[parent] = min_heap->node[i];
        min_heap->node[i] = swap_node;
        order_heap_up(min_heap, parent);
    }
}

// Extract node with minimum distance (at index 0) and reorder heap
void extract_node(heap* min_heap){
    min = min_heap->node[0];
    min_heap->node[0] = min_heap->node[min_heap->size-1];
    min_heap->size--;
    order_heap_down(min_heap, 0);
}

// Sorts the heap downwards
void order_heap_down(heap* min_heap, int i) {
    int left_child = 2 * i + 1, right_child = 2 * i + 2, parent = i;
    heap_node swap_node;

    if (left_child < min_heap->size && min_heap->node[left_child].distance < min_heap->node[parent].distance)
        parent = left_child;
    if (right_child < min_heap->size && min_heap->node[right_child].distance < min_heap->node[parent].distance)
        parent = right_child;

    if (parent != i) {
        swap_node = min_heap->node[parent];
        min_heap->node[parent] = min_heap->node[i];
        min_heap->node[i] = swap_node;
        order_heap_down(min_heap, parent);
    }
}

// Free all the costs in the cache
void clear_cache() {
    cache* current_route;
    while (route != NULL) {
        current_route = route;
        route = route->next;
        free(current_route);
    }
}