# File contains helper functions used for frontier detection
from collections import deque
import math


## Find the cell value at a row and column
#  -100 = invalid/outside our current grid
#  -1   = unknown but actually inside the grid
#  0    = free
#  1–100 = occupancy value
def get_cell(map_msg, row, column):
    width = map_msg.info.width
    height = map_msg.info.height

    if 0 <= row < height and 0 <= column < width:
        index = row * width + column
        return map_msg.data[index]  # return cell value if valid

    return -100  # invalid/outside the map


## Find the four surrounding cells
#   N
#  NCN
#   N
def get_four_neighbours(row, column):
    return [
        (row + 1, column),
        (row - 1, column),
        (row, column + 1),
        (row, column - 1)
    ]


## Find the eight surrounding cells
#  NNN
#  NCN
#  NNN
def get_eight_neighbours(row, column):
    return [
        (row + 1, column - 1),
        (row + 1, column),
        (row + 1, column + 1),
        (row, column + 1),
        (row, column - 1),
        (row - 1, column - 1),
        (row - 1, column),
        (row - 1, column + 1)
    ]


## Check if a certain cell is a frontier
def is_frontier(map_msg, row, column):
    if get_cell(map_msg, row, column) != 0:
        return False

    neighbours = get_four_neighbours(row, column)

    for neighbour_row, neighbour_column in neighbours:
        if get_cell(map_msg, neighbour_row, neighbour_column) == -1:
            return True

    return False


## Find all current frontiers on the grid and return them
def find_frontiers(map_msg):
    frontiers = []
    width = map_msg.info.width
    height = map_msg.info.height

    for row in range(height):
        for column in range(width):
            if is_frontier(map_msg, row, column):
                frontiers.append((row, column))

    return frontiers


## Group frontiers into clusters of frontiers for motion planning
def group_frontiers(frontiers):
    clusters = []
    clustered = set()
    frontier_set = set(frontiers)

    for frontier in frontiers:
        if frontier in clustered:
            continue

        cluster = [frontier]
        clustered.add(frontier)
        to_check = deque([frontier])

        while to_check:
            current_frontier = to_check.popleft()

            neighbours = get_eight_neighbours(
                current_frontier[0],
                current_frontier[1]
            )

            for neighbour in neighbours:
                if neighbour in frontier_set and neighbour not in clustered:
                    to_check.append(neighbour)
                    cluster.append(neighbour)
                    clustered.add(neighbour)

        clusters.append(cluster)

    return clusters


## Return list of clusters at least the minimum size
def filter_clusters(clusters, minimum_size):
    filtered_clusters = []

    for cluster in clusters:
        if len(cluster) >= minimum_size:
            filtered_clusters.append(cluster)

    return filtered_clusters


## Find the centre cell in a cluster
def cluster_centre(cluster):
    length = len(cluster)
    sum_rows = 0
    sum_columns = 0

    for cell in cluster:
        sum_rows += cell[0]
        sum_columns += cell[1]

    average_row = sum_rows / length
    average_column = sum_columns / length

    # Find the frontier cell closest to the average position
    min_square_distance = float('inf')
    centre_cell = None

    for cell in cluster:
        square_distance = ((cell[0] - average_row) ** 2 + (cell[1] - average_column) ** 2)

        if square_distance < min_square_distance:
            min_square_distance = square_distance
            centre_cell = cell

    return centre_cell


## Get the centres of all the frontier clusters
def get_cluster_centres(clusters):
    centres = []

    for cluster in clusters:
        centres.append(cluster_centre(cluster))

    return centres


## Convert a row and column to map coordinates
def grid_to_world(map_msg, row, column):
    resolution = map_msg.info.resolution
    origin = map_msg.info.origin

    # Position relative to the map origin
    grid_x = (column + 0.5) * resolution
    grid_y = (row + 0.5) * resolution

    # Get yaw from the origin quaternion
    q = origin.orientation
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y ** 2 + q.z ** 2))

    # Rotate into the world/map coordinate system
    world_x = (origin.position.x+ grid_x * math.cos(yaw)- grid_y * math.sin(yaw))

    world_y = (origin.position.y+ grid_x * math.sin(yaw)+ grid_y * math.cos(yaw))

    return (world_x, world_y)


## Sort frontier cluster centres from closest to furthest from the robot
def sorted_frontiers(map_msg, clusters, robot_position):
    cluster_centres = get_cluster_centres(clusters)

    frontiers_with_distance = []

    for row, column in cluster_centres:
        # Convert into the same world coordinate system as the robot
        world_centre = grid_to_world(map_msg,row, column)

        square_distance = ((world_centre[0] - robot_position[0]) ** 2 + (world_centre[1] - robot_position[1]) ** 2)

        frontiers_with_distance.append((square_distance, (row, column)))

    # Sort using the distance stored in the first element
    frontiers_with_distance.sort()

    sorted_frontier_cells = []

    for distance, frontier in frontiers_with_distance:
        sorted_frontier_cells.append(frontier)

    return sorted_frontier_cells

## Find a stand-off goal in known free space near a frontier
# Adapted from the waypoint cycler stand-off logic
def offset_frontier_goal(map_msg, frontier, offset):
    if frontier is None:
        return None

    row = frontier[0]
    column = frontier[1]

    resolution = map_msg.info.resolution

    # Convert the stand-off distance from metres into map cells
    standoff_cells = max(1,int(offset / resolution))

    # Determine which direction the unknown space lies in
    # Relative directions are stored as (row change, column change)
    unknown_row_direction = 0
    unknown_column_direction = 0

    neighbours = [
        (row + 1, column, 1, 0),
        (row - 1, column, -1, 0),
        (row, column + 1, 0, 1),
        (row, column - 1, 0, -1)
    ]

    for neighbour_row, neighbour_column, row_direction, column_direction in neighbours:
        if get_cell(map_msg, neighbour_row, neighbour_column) == -1:
            unknown_row_direction += row_direction
            unknown_column_direction += column_direction

    # If the directions cancel out, there is no clear safe offset direction
    direction_length = math.sqrt(unknown_row_direction ** 2 + unknown_column_direction ** 2)

    if direction_length == 0:
        return None

    # Normalise the direction so diagonal offsets have the same magnitude
    unknown_row_direction /= direction_length
    unknown_column_direction /= direction_length

    # Move away from the unknown space and back into known space.
    # Start at the requested stand-off distance and move closer
    # to the frontier until a free goal cell is found.
    for distance in range(standoff_cells, 0, -1):

        goal_row = round(row - unknown_row_direction * distance)

        goal_column = round(column - unknown_column_direction * distance)

        # Ignore positions outside the current map
        if get_cell(map_msg, goal_row, goal_column) == -100:
            continue

        if get_cell(map_msg, goal_row, goal_column) != 0:
            continue

        # A valid free stand-off cell was found
        return grid_to_world(map_msg, goal_row, goal_column)

    # No free stand-off position could be found
    return None

## Find the closest frontier that produces a valid stand-off goal
def choose_frontier_goal(map_msg, clusters, robot_position, offset, failed_goals):
    frontiers = sorted_frontiers(map_msg, clusters, robot_position)

    # Try the closest frontier first, then continue until
    # a valid stand-off goal is found
    for frontier in frontiers:
        goal = offset_frontier_goal(map_msg, frontier, offset)

        if goal is None:
            continue

        # Check whether the goal is close to a previously failed goal
        failed = False

        for failed_goal in failed_goals:
            square_distance = (
                (goal[0] - failed_goal[0]) ** 2
                + (goal[1] - failed_goal[1]) ** 2
            )

            if square_distance < 0.5 ** 2:
                failed = True
                break

        if failed:
            continue

        # Do not select goals that the robot is already effectively at
        square_distance = ((goal[0] - robot_position[0]) ** 2 + (goal[1] - robot_position[1]) ** 2)

        if square_distance < 0.5 ** 2:
            continue

        return frontier, goal

    return None, None  