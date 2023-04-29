import yaml
from nav_msgs.msg import OccupancyGrid

def load_map(file_name):
    with open("/home/jnkimmelman/ros2_ws/src/project_4/"+file_name) as f:
        map = yaml.safe_load(f)
    return map

def parse_map(file_name):
    file = load_map(file_name)
    O_grid = OccupancyGrid()
    j = 0;
    k = 0;
    O_grid.info.resolution = file['resolution']
    # width_found = False
    for i in file['map']:
        if i == "#":
            O_grid.data.append(1)
        elif i == ".":
            O_grid.data.append(0)
        elif i == "\n":
            O_grid.info.width = j-1
            j = 0
            k += 1
            # width_found = True
        j += 1
    # print(self.O_grid.info.width)
    O_grid.info.height = k+1
    # print(self.O_grid.info.height)
    return O_grid
    print(f"height: {O_grid.info.height},width: {O_grid.info.width}")