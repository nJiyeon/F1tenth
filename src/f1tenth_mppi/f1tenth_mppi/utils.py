import csv
import numpy as np
from ament_index_python.packages import get_package_share_directory
from visualization_msgs.msg import Marker, MarkerArray

def load_waypoints(file_path: str) -> np.ndarray:
    '''
    Load waypoints from specified file

    Args:
        file_path (str): The path to the waypoints file
    Returns:
        waypoints (ndarray): The waypoints (x, y, psi, vx)
    '''

    # Get file path
    package_path = get_package_share_directory('f1tenth_mppi')
    file_path = package_path.split("install")[0] + "src/ICRA2026-Paper/maps/" + file_path

    # Load waypoints
    waypoints = []
    with open(file_path, 'r') as file:
        csv_reader = csv.reader(file, delimiter=';')

        for i in range(3):
            next(csv_reader)

        for i, row in enumerate(csv_reader):
            waypoints.append([float(row[1]), float(row[2]), float(row[3]), float(row[5])])

    return np.array(waypoints)


def color(code='r'):
    '''
    
    '''
    code = code.lower().strip()
    if code in ['r', 'red']:
        return np.array([1.0, 0.0, 0.0])
    elif code in ['g', 'green']:
        return np.array([0.0, 1.0, 0.0])
    elif code in ['b', 'blue']:
        return np.array([0.0, 0.0, 1.0])
    elif code in ['w', 'white']:
        return np.array([1.0, 1.0, 1.0])
    else:
        return np.array([0.0, 0.0, 0.0])
    
    
