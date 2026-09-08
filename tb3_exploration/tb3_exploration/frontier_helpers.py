# File containers helper functions used for frontier detection

def get_cell(map_msg, row, column):
    width = map_msg.info.width
    height = map_msg.info.height

    if 0 <= row < height and 0 <= column < width:
        index = row * width + column
        return map_msg.data[index]  # return cell value if valid

    return -100  # invalid/outside the map

def is_frontier(map_msg, row, column):

    if get_cell(map_msg, row, column) != 0:
        return False

    if (get_cell(map_msg, row+1, column) == -1 or
        get_cell(map_msg, row-1, column) == -1 or
        get_cell(map_msg, row, column+1) == -1 or
        get_cell(map_msg, row, column-1) == -1):
        return True

    return False

def find_frontiers(map_msg):
    frontiers = []

    for row in range(height):
        for column in range(width):
            if is_frontier(row, column):
                frontiers.append((row, column))