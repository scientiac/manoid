#! /usr/bin/env python

import cv2
import numpy as np
import threading
import paho.mqtt.client as mqtt
import math
import heapq
import time
import cv2.aruco as aruco

# Define constants and setup
ARENA_WIDTH = 300
ARENA_HEIGHT = 300
GRID_SIZE = 2  # Adjust based on your setup
MQTT_BROKER = "192.168.1.117"
MQTT_PORT = 1883
CORNER_MARKERS = {0, 1, 2, 3}
INORGANIC_DROP_OFF_ID = 4
ORGANIC_DROP_OFF_ID = 5
ROBOT_IDS = [6]
INORGANIC_WASTE_ID = [7, 9]
ORGANIC_WASTE_ID = [8, 10]

# Define PID constants and speeds for each robot
robot_settings = {
    6: {  # Robot ID 6
        "P_left": 0.8,
        "P_right": 0.8,
        "P_center": 0.4,
        "I_left": 0.01,
        "I_right": 0.01,
        "I_center": 0.01,
        "D_left": 0.001,
        "D_right": 0.001,
        "D_center": 0.001,
        "backward_speed_left": 10,
        "backward_speed_right": 10,
        "left_prev_error": 0,
        "right_prev_error": 0,
        "center_prev_error": 0,
        "dt": 0.3,
    }
}

client = mqtt.Client()

# Initialize shared resources and a lock
shared_resources = {
    "frame": None,
    "markers": {},
    "drop_off_locations": {},
    "paths": {},
    "goal_positions": {},
    "processed_markers": set(),  # Blacklist of processed markers
}
resources_lock = threading.Lock()

# Define the dictionary to use
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)

# Create ArUco parameters
parameters = cv2.aruco.DetectorParameters()

def get_warped_frame(input_frame, marker_ids, PAD):
    # Detect markers in the frame
    corners, ids, _ = cv2.aruco.detectMarkers(
        input_frame, aruco_dict, parameters=parameters
    )

    marker_corners_dict = {marker_id: None for marker_id in marker_ids}

    if ids is not None:
        # Extract corners of the specified markers
        for marker_id in marker_ids:
            index = np.where(ids == marker_id)
            if len(index[0]) > 0:
                marker_corners_dict[marker_id] = corners[index[0][0]][0]

    # Check if all specified markers are detected
    if all(value is not None for value in marker_corners_dict.values()):
        # Get the width and height of the frame
        frame_width, frame_height = input_frame.shape[1], input_frame.shape[0]

        # Define the corners of the big marker
        big_marker_corners = [
            marker_corners_dict[marker_ids[3]][2],  # TL = 0
            marker_corners_dict[marker_ids[1]][1],  # BL = 3
            marker_corners_dict[marker_ids[0]][0],  # BR = 2
            marker_corners_dict[marker_ids[2]][3],  # TR = 1
        ]

        square_size = min(frame_width, frame_height)
        destination_points = np.float32(
            [
                [square_size - PAD, square_size - PAD],
                [square_size - PAD, 0 + PAD],
                [0 + PAD, 0 + PAD],
                [0 + PAD, square_size - PAD],
            ]
        )

        # Calculate perspective transformation matrix
        matrix = cv2.getPerspectiveTransform(
            np.float32(big_marker_corners), destination_points
        )

        # Warp the frame using the perspective transformation matrix
        warped_frame = cv2.warpPerspective(
            input_frame, matrix, (square_size, square_size)
        )

        return warped_frame, marker_corners_dict

    return None, None  # Return None if not all markers are detected

def calculate_scale(corners, marker_physical_size_cm):
    # Calculate the distance between the first and second corner (top-left and top-right) in pixels
    pixel_distance = np.linalg.norm(np.array(corners[0]) - np.array(corners[1]))
    # Scale is pixels per cm
    return pixel_distance / marker_physical_size_cm


def adjust_marker_corners(
    corners,
    offset_x_cm=0,
    offset_y_cm=0,
    adjust_width_cm=0,
    adjust_height_cm=0,
    marker_physical_size_cm=15,
):
    # Calculate the center of the marker
    center = np.mean(corners, axis=0)

    # Calculate the vectors from the center to the corners
    vectors = corners - center

    # Calculate the scale based on the physical size of the marker
    scale = np.linalg.norm(vectors[0]) / (marker_physical_size_cm / 2)

    # Convert cm adjustments to pixels
    offset_x_pixels = offset_x_cm * scale
    offset_y_pixels = offset_y_cm * scale
    adjust_width_pixels = adjust_width_cm * scale
    adjust_height_pixels = adjust_height_cm * scale

    # Adjust the width in the marker's local coordinate system
    width_adjustment_factor = 1 + adjust_width_cm / marker_physical_size_cm
    vectors[:, 0] *= width_adjustment_factor

    # Adjust the height in the marker's local coordinate system
    height_adjustment_factor = 1 + adjust_height_cm / marker_physical_size_cm
    vectors[:, 1] *= height_adjustment_factor

    # Rotate the offset to the marker's coordinate system if needed
    # This is optional and depends on whether you want the offset to rotate with the marker
    # If not, you can simply add the offset to the center point
    rotated_offset = center + np.array([offset_x_pixels, offset_y_pixels])

    # Apply the adjustments and offsets to get the new corners
    adjusted_corners = vectors + rotated_offset

    return adjusted_corners


def detect_aruco_markers(frame, aruco_dict_type=cv2.aruco.DICT_6X6_250):
    aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dict_type)
    parameters = cv2.aruco.DetectorParameters()  # Updated for some versions of OpenCV
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(
        frame, aruco_dict, parameters=parameters
    )

    markers = {}
    if ids is not None:
        ids = ids.flatten()
        for id, corner in zip(ids, corners):
            # Process corners to a more readable format
            processed_corners = [
                tuple(map(int, corner_point)) for corner_point in corner[0]
            ]

            # Apply adjustments for specific markers
            if id in ROBOT_IDS:
                processed_corners = adjust_marker_corners(
                    processed_corners,
                    adjust_width_cm= 15,
                    adjust_height_cm= 15
                )

            # Recalculate the center based on the processed corners
            recalculated_center = tuple(map(int, np.mean(processed_corners, axis=0)))

            # Store both center and corners
            marker_data = {"center": recalculated_center, "corners": processed_corners}

            if id in markers:
                markers[id].append(
                    marker_data
                )  # Append the new marker data to the list for this ID
            else:
                markers[id] = [marker_data]  # Start a new list with this marker data

    return markers

# Manhattan Distance
def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def astar(start, goal, obstacles, grid_size):
    # Define the neighbors function
    def neighbors(position, obstacles, grid_size, obstacle_radius):
        x, y = position
        candidates = [
            (x - 1, y),
            (x + 1, y),
            (x, y - 1),
            (x, y + 1),
        ]

        valid_neighbors = []
        for neighbor in candidates:
            nx, ny = neighbor
            if (
                0 <= nx < grid_size
                and 0 <= ny < grid_size
                and neighbor not in obstacles
                and all(
                    abs(ox - nx) > obstacle_radius or abs(oy - ny) > obstacle_radius
                    for ox, oy in obstacles
                )
            ):
                valid_neighbors.append(neighbor)
        return valid_neighbors

    # Initialize the data structures
    open_set = []
    closed_set = set()
    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}

    # Add the start position to the open set
    heapq.heappush(open_set, (f_score[start], start))

    while open_set:
        # Get the position with the lowest f-score from the open set
        current = heapq.heappop(open_set)[1]

        if current == goal:
            # Reconstruct the path
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            path.reverse()
            return path

        # Add the current position to the closed set
        closed_set.add(current)

        # Explore the neighbors
        for neighbor in neighbors(current, obstacles, grid_size, obstacle_radius=2):
            neighbor_g_score = g_score[current] + 1

            if neighbor in closed_set:
                continue

            if neighbor not in open_set or neighbor_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = neighbor_g_score
                f_score[neighbor] = neighbor_g_score + heuristic(neighbor, goal)
                if neighbor not in open_set:
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))

    # No path found
    return None

def connect_mqtt():
    client.connect(MQTT_BROKER, MQTT_PORT, 60)
    client.loop_start()


def send_mqtt_command(topic, command):
    client.publish(topic, command)


def get_bot_position(bot_id, markers):
    """Helper function to get the current position of a bot based on its marker ID."""
    if bot_id in markers:
        marker_data = markers[bot_id]
        if marker_data:
            # Assuming the first marker data contains the position
            center_x, center_y = marker_data[0]["center"]
            return center_x, center_y
    return None

def calculate_distances(robot_corners, next_position):
    center, tl, tr = robot_corners
    goal = next_position
    d_center = math.hypot(center[0] - goal[0], center[1] - goal[1])
    d_left = math.hypot(tl[0] - goal[0], tl[1] - goal[1])
    d_right = math.hypot(tr[0] - goal[0], tr[1] - goal[1])
    return d_right, d_left, d_center

def move_towards_goal(robot_id, path, threshold=10):
    """
    Move the robot towards the goal following the path.
    """
    settings = robot_settings[robot_id]
    P_left = settings["P_left"]
    P_right = settings["P_right"]
    P_center = settings["P_center"]
    I_left = settings["I_left"]
    I_right = settings["I_right"]
    I_center = settings["I_center"]
    D_left = settings["D_left"]
    D_right = settings["D_right"]
    D_center = settings["D_center"]
    backward_speed_left = settings["backward_speed_left"]
    backward_speed_right = settings["backward_speed_right"]
    left_prev_error = settings["left_prev_error"]
    right_prev_error = settings["right_prev_error"]
    center_prev_error = settings["center_prev_error"]
    dt = settings["dt"]

    for next_position in path:
        position_reached = False
        while not position_reached:
            with resources_lock:
                # Extracting robot head position, top left, top right corners, and robot center
                _, tl, tr, robot_center = get_head_position(
                    robot_id, shared_resources["markers"]
                )
                if tl is None or tr is None or robot_center is None:
                    time.sleep(0)
                    continue

                # Update the goal position for the current robot
                shared_resources["goal_positions"][robot_id] = next_position

            # Pass the robot center along with corners to calculate distances
            d_right, d_left, d_center = calculate_distances(
                (robot_center, tl, tr), next_position
            )

            # Determine movement command based on distances
            if d_center < min(d_right, d_left):
                send_mqtt_command(
                    f"/robot{robot_id}_left_backward", backward_speed_left
                )
                send_mqtt_command(
                    f"/robot{robot_id}_right_backward", backward_speed_right
                )
                print("backwards")
            else:
                left_error = d_left - d_right
                right_error = d_right - d_left
                d_center_error = d_center

                left_P_gain = P_left * left_error
                right_P_gain = P_right * right_error
                center_P_gain = P_center * d_center_error
                left_I_gain = I_left * (left_error + left_prev_error) * dt
                right_I_gain = I_right * (right_error + right_prev_error) * dt
                center_I_gain = P_center * (d_center_error + center_prev_error) * dt
                left_D_gain = (D_left * (left_error - left_prev_error)) / dt
                right_D_gain = (D_right * (right_error - right_prev_error)) / dt
                center_D_gain = (D_center * (d_center_error - center_prev_error)) / dt
                center_prev_error = d_center_error
                right_prev_error = right_error
                left_prev_error = left_error

                left_speed = (
                    left_P_gain
                    + left_I_gain
                    + left_D_gain
                    + center_P_gain
                    + center_I_gain
                    + center_D_gain
                )
                right_speed = (
                    right_P_gain
                    + right_I_gain
                    + right_D_gain
                    + center_P_gain
                    + center_I_gain
                    + center_D_gain
                )
                print(f"Left Speed:{left_speed} Right Speed:{right_speed}")

                if left_speed >= 0:
                    send_mqtt_command(f"/robot{robot_id}_left_forward", left_speed)
                if left_speed < 0:
                    send_mqtt_command(
                        f"/robot{robot_id}_left_backward", abs(left_speed)
                    )
                if right_speed >= 0:
                    send_mqtt_command(f"/robot{robot_id}_right_forward", right_speed)
                if right_speed < 0:
                    send_mqtt_command(
                        f"/robot{robot_id}_right_backward", abs(right_speed)
                    )

            # Check if the robot has reached the next position
            with resources_lock:
                current_position, _, _, _ = get_head_position(
                    robot_id, shared_resources["markers"]
                )

                # Update the goal position for the current robot
                shared_resources["goal_positions"][robot_id] = next_position

                if (
                    current_position
                    and math.hypot(
                        current_position[0] - next_position[0],
                        current_position[1] - next_position[1],
                    )
                    < 20
                ):
                    position_reached = True

            time.sleep(0.3)  # Adjust sleep time as needed


def draw_lines_to_goal(
    frame, robot_corners, goal_position, color=(255, 0, 0), thickness=2
):
    # Unpack the robot_corners tuple
    center, tl, tr = robot_corners

    # Check if tl or tr is None and return the frame unmodified if true
    if tl is None or tr is None:
        print("Skipping drawing lines to goal due to missing corner information.")
        return frame

    # Draw lines from each corner to the goal
    cv2.line(
        frame,
        (int(tl[0]), int(tl[1])),
        (int(goal_position[0]), int(goal_position[1])),
        color,
        thickness,
    )
    cv2.line(
        frame,
        (int(tr[0]), int(tr[1])),
        (int(goal_position[0]), int(goal_position[1])),
        color,
        thickness,
    )

    # Also draw a line from the center to the goal
    cv2.line(
        frame,
        (int(center[0]), int(center[1])),
        (int(goal_position[0]), int(goal_position[1])),
        (0, 0, 255),
        thickness,
    )

    # Optionally, mark the goal position and the corners
    cv2.circle(
        frame, (int(goal_position[0]), int(goal_position[1])), 5, (0, 0, 255), -1
    )  # Goal position in red
    cv2.circle(
        frame, (int(tl[0]), int(tl[1])), 5, (255, 0, 0), -1
    )  # Top-left corner in blue
    cv2.circle(
        frame, (int(tr[0]), int(tr[1])), 5, (255, 0, 0), -1
    )  # Top-right corner in blue

    return frame


def draw_path(frame, path, color, thickness=2, grid_size=15):
    """Draws a path on the frame."""
    if not path:
        return

    # Draw each segment of the path
    for i in range(len(path) - 1):
        cv2.line(frame, path[i], path[i + 1], color, thickness)
        cv2.circle(frame, path[i], 2, (0, 0, 0), 1)


def get_head_position(robot_id, markers):
    """
    Return the head position, the top left and top right corner positions,
    and the center of the marker based on ArUco marker detection.
    """
    if robot_id in markers:
        for marker_data in markers[robot_id]:
            # Assuming corners are provided in the order: tl, tr, br, bl
            corners = marker_data["corners"]
            tl, tr, br, bl = corners[0], corners[1], corners[2], corners[3]

            # Calculate the midpoint between tl and tr for the head position
            head_position = (int((tl[0] + tr[0]) / 2), int((tl[1] + tr[1]) / 2))

            # Calculate the center of the marker as the average of all corners
            center_x = int((tl[0] + tr[0] + br[0] + bl[0]) / 4)
            center_y = int((tl[1] + tr[1] + br[1] + bl[1]) / 4)
            marker_center = (center_x, center_y)

            # Ensure tl and tr are tuples of integers
            tl = (int(tl[0]), int(tl[1]))
            tr = (int(tr[0]), int(tr[1]))

            return head_position, tl, tr, marker_center
    return None, None, None, None


def get_waste_positions(markers, waste_id):
    """Filter and return positions of a specific waste type."""
    return [
        data["center"]
        for marker_id, marker_list in markers.items()
        if marker_id == waste_id
        for data in marker_list
    ]


def fill_grid_cells_from_corners(corners, grid_size=5):
    """Given corners of a rectangular area, returns all grid cells covered by the rectangle."""
    # Convert each corner into grid coordinates
    grid_corners = [
        convert_to_grid_coordinates(corner, cell_size=grid_size) for corner in corners
    ]

    min_x = min(corner[0] for corner in grid_corners)
    max_x = max(corner[0] for corner in grid_corners)
    min_y = min(corner[1] for corner in grid_corners)
    max_y = max(corner[1] for corner in grid_corners)

    # Fill in the grid cells
    covered_cells = set()
    for x in range(min_x, max_x + 1):
        for y in range(min_y, max_y + 1):
            covered_cells.add((x, y))

    return covered_cells


def update_obstacles(markers, target_waste_ids, robot_head_pos):
    obstacles = set()
    nearest_waste_pos = None
    nearest_waste_dist = float("inf")
    nearest_waste_id = None  # Track the ID of the nearest waste

    for marker_id, marker_data_list in markers.items():
        if (
            marker_id in target_waste_ids
            and marker_id not in shared_resources["processed_markers"]
        ):
            for marker_data in marker_data_list:
                corners = marker_data["corners"]
                tl, tr = corners[0], corners[1]
                head_center = ((tl[0] + tr[0]) / 2, (tl[1] + tr[1]) / 2)
                obstacles.add(head_center)
                distance = np.linalg.norm(
                    np.array(robot_head_pos) - np.array(head_center)
                )
                if distance < nearest_waste_dist:
                    nearest_waste_dist = distance
                    nearest_waste_pos = head_center
                    nearest_waste_id = marker_id

    if nearest_waste_pos:
        obstacles.discard(nearest_waste_pos)

    return obstacles, nearest_waste_pos, nearest_waste_id


def convert_to_grid_coordinates(position, cell_size=15):
    """Converts position to grid coordinates."""
    if not isinstance(position, tuple) or len(position) != 2:
        raise ValueError("Position must be a tuple of (x, y).")
    grid_x = int(position[0] / cell_size)
    grid_y = int(position[1] / cell_size)
    return (grid_x, grid_y)


def convert_obstacles_to_grid(obstacles, cell_size=15):
    """Converts a set of positions to grid coordinates."""
    grid_obstacles = set()
    for position in obstacles:
        if not isinstance(position, tuple) or len(position) != 2:
            raise ValueError("Each position must be a tuple of (x, y).")
        grid_x = int(position[0] / cell_size)
        grid_y = int(position[1] / cell_size)
        grid_obstacles.add((grid_x, grid_y))
    return grid_obstacles


def convert_grid_to_actual(path, cell_size=15):
    """Converts a path of grid coordinates back to actual coordinates."""
    actual_path = [
        (x * cell_size + cell_size // 2, y * cell_size + cell_size // 2)
        for x, y in path
    ]
    return actual_path


def plan_path(start, goal, obstacles):
    """Wrapper for the A* pathfinding."""
    start_grid = convert_to_grid_coordinates(start)
    goal_grid = convert_to_grid_coordinates(goal)

    obstacles = convert_obstacles_to_grid(obstacles)

    # Assuming your grid size is the width/height of the arena divided by GRID_SIZE
    grid_size = ARENA_WIDTH // GRID_SIZE

    path = astar(start_grid, goal_grid, obstacles, grid_size)
    return path  # This will be a list of grid coordinates representing the path


def find_nearest_edge_midpoint_to_robot(robot_pos, marker_id, markers):
    nearest_edge_midpoint = None
    nearest_edge_label = None
    min_distance = float("inf")

    # Define labels for edges based on their midpoints
    edge_labels = ["Top", "Right", "Bottom", "Left"]

    if marker_id in markers:
        for data in markers[marker_id]:
            corners = data["corners"]
            # Calculate midpoints of edges
            midpoints = [
                (
                    (corners[0][0] + corners[1][0]) / 2,
                    (corners[0][1] + corners[1][1]) / 2,
                ),  # Top edge
                (
                    (corners[1][0] + corners[2][0]) / 2,
                    (corners[1][1] + corners[2][1]) / 2,
                ),  # Right edge
                (
                    (corners[2][0] + corners[3][0]) / 2,
                    (corners[2][1] + corners[3][1]) / 2,
                ),  # Bottom edge
                (
                    (corners[3][0] + corners[0][0]) / 2,
                    (corners[3][1] + corners[0][1]) / 2,
                ),  # Left edge
            ]

            # Determine which midpoint is closest to the robot
            for i, midpoint in enumerate(midpoints):
                distance = math.hypot(
                    midpoint[0] - robot_pos[0], midpoint[1] - robot_pos[1]
                )
                if distance < min_distance:
                    min_distance = distance
                    nearest_edge_midpoint = midpoint
                    nearest_edge_label = edge_labels[i]

    return nearest_edge_midpoint, nearest_edge_label


def pickup_waste(robot_id):
    send_mqtt_command(f"/robot{robot_id}_gripper_close", 1)
    time.sleep(3)
    print("waste picked")


def drop_off_waste(robot_id, waste_id):
    # Simulate dropping off the waste
    # After successful drop off, add the marker ID to the blacklist
    send_mqtt_command(f"/robot{robot_id}_gripper_open", 1)
    time.sleep(3)
    print("waste dropped")
    with resources_lock:
        shared_resources["processed_markers"].add(waste_id)


def robot_control_loop(robot_id):
    global shared_resources, resources_lock
    # Connect to MQTT
    connect_mqtt()

    while True:
        # Acquire frame and markers
        with resources_lock:
            frame = shared_resources.get("frame", None)
            markers = shared_resources.get("markers", {})
            drop_off_locations = shared_resources.get("drop_off_locations", {})

        if frame is None:
            continue

        (
            robot_head_pos,
            robot_top_left_corner,
            robot_top_right_corner,
            _,
        ) = get_head_position(robot_id, markers)

        if not robot_head_pos:
            time.sleep(0)
            continue

        # Determine target waste and calculate path to waste
        target_waste_ids = ORGANIC_WASTE_ID if robot_id == 6 else INORGANIC_WASTE_ID
        obstacles, nearest_waste_pos, nearest_waste_id = update_obstacles(
            markers, target_waste_ids, robot_head_pos
        )

        # Inside your robot_control_loop, after obtaining nearest_waste_id
        if nearest_waste_id is not None:
            (
                nearest_edge_center,
                nearest_edge_label,
            ) = find_nearest_edge_midpoint_to_robot(
                robot_head_pos, nearest_waste_id, markers
            )

        path_to_waste, path_to_drop_off = [], []
        if nearest_waste_pos:
            path_to_waste = plan_path(robot_head_pos, nearest_edge_center, obstacles)
            path_to_waste = (
                convert_grid_to_actual(path_to_waste) if path_to_waste else []
            )

            # Calculate path to drop-off only if waste is found
            drop_off_id = (
                ORGANIC_DROP_OFF_ID
                if target_waste_ids == ORGANIC_WASTE_ID
                else INORGANIC_DROP_OFF_ID
            )
            drop_off_location = drop_off_locations.get(drop_off_id)
            if drop_off_location:
                path_to_drop_off = plan_path(
                    nearest_waste_pos, drop_off_location, obstacles
                )
                path_to_drop_off = (
                    convert_grid_to_actual(path_to_drop_off) if path_to_drop_off else []
                )

        # Update shared resources with calculated paths and head position
        with resources_lock:
            shared_resources["paths"][robot_id] = {
                "path_to_waste": path_to_waste,
                "path_to_drop_off": path_to_drop_off,
            }

        if path_to_waste:
            move_towards_goal(robot_id, path_to_waste)  # Move towards waste
            send_mqtt_command(f"/robot{robot_id}_right_forward", 0)
            send_mqtt_command(f"/robot{robot_id}_left_forward", 0)
            time.sleep(5)
            pickup_waste(robot_id)  # Simulate waste pickup
        else:
            send_mqtt_command(f"/robot{robot_id}_right_forward", 0)
            send_mqtt_command(f"/robot{robot_id}_left_forward", 0)

        if path_to_drop_off:
            move_towards_goal(robot_id, path_to_drop_off)  # Move towards drop-off
            send_mqtt_command(f"/robot{robot_id}_right_forward", 0)
            send_mqtt_command(f"/robot{robot_id}_left_forward", 0)
            drop_off_waste(robot_id, nearest_waste_id)

        # Loop with a delay to prevent constant recalculating
        time.sleep(0)

    # Disconnect MQTT when done
    disconnect_mqtt()


def capture_and_update_shared_resources(url):
    global shared_resources, resources_lock
    cap = cv2.VideoCapture(url)
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break

        # # Perform frame correction here
        # PAD = 8
        # markerTL = 0
        # markerTR = 2
        # markerBL = 1
        # markerBR = 3
        #
        # # Define the markers and their positions
        # marker_ids = [markerTL, markerTR, markerBL, markerBR]
        # corrected_frame, marker_corners_dict = get_warped_frame(frame, marker_ids, PAD)
        #
        # if corrected_frame is not None:
        #     frame = corrected_frame  # Use the corrected frame for further processing

        markers = detect_aruco_markers(frame)  # Detect ArUco markers in the frame
        with resources_lock:
            shared_resources["frame"] = frame
            shared_resources["markers"] = markers
            shared_resources["drop_off_locations"] = {
                INORGANIC_DROP_OFF_ID: markers.get(INORGANIC_DROP_OFF_ID)[0]["center"]
                if markers.get(INORGANIC_DROP_OFF_ID)
                else None,
                ORGANIC_DROP_OFF_ID: markers.get(ORGANIC_DROP_OFF_ID)[0]["center"]
                if markers.get(ORGANIC_DROP_OFF_ID)
                else None,
            }


def visualize_robot_behavior():
    global shared_resources, resources_lock
    while True:
        with resources_lock:
            frame = shared_resources.get("frame", None)
            paths = shared_resources.get("paths", {})
            markers = shared_resources.get("markers", {})
            goal_positions = shared_resources.get("goal_positions", {})

            if frame is None:
                continue

            frame_copy = frame.copy()

            for robot_id in ROBOT_IDS:
                (
                    robot_head_pos,
                    robot_top_left_corner,
                    robot_top_right_corner,
                    _,
                ) = get_head_position(robot_id, markers)
                if robot_head_pos:
                    # Draw robot head position
                    cv2.circle(
                        frame_copy,
                        robot_head_pos,
                        radius=5,
                        color=(255, 0, 0),
                        thickness=-1,
                    )

            for robot_id, path_info in paths.items():
                draw_path(
                    frame_copy,
                    path_info["path_to_waste"],
                    (125, 125, 255),
                    2,
                    GRID_SIZE,
                )
                draw_path(
                    frame_copy,
                    path_info["path_to_drop_off"],
                    (125, 155, 125),
                    2,
                    GRID_SIZE,
                )

            for robot_id in ROBOT_IDS:
                (
                    robot_head_pos,
                    robot_top_left_corner,
                    robot_top_right_corner,
                    robot_center,
                ) = get_head_position(robot_id, markers)

                # Check if there is a current goal position for the robot
                if robot_id in goal_positions:
                    goal_position = goal_positions[robot_id]
                    draw_lines_to_goal(
                        frame_copy,
                        (robot_center, robot_top_left_corner, robot_top_right_corner),
                        goal_position,
                    )

            for marker_id, marker_data in markers.items():
                for data in marker_data:
                    corners = data["corners"]
                    cv2.polylines(
                        frame_copy,
                        [np.array(corners, np.int32).reshape((-1, 1, 2))],
                        isClosed=True,
                        color=(0, 255, 0),
                        thickness=2,
                    )
                    cv2.circle(
                        frame_copy,
                        data["center"],
                        radius=2,
                        color=(0, 0, 255),
                        thickness=-1,
                    )

            for marker_id, marker_data in markers.items():
                for data in marker_data:
                    corners = data["corners"]
                    center = data["center"]

                   # Set color based on marker_id
                    if marker_id == INORGANIC_DROP_OFF_ID:
                        color = (0, 0, 255) 
                    elif marker_id == ORGANIC_DROP_OFF_ID:
                        color = (0, 255, 0) 
                    elif marker_id in INORGANIC_WASTE_ID:
                        color = (255, 0, 0) 
                    elif marker_id in ORGANIC_WASTE_ID:
                        color = (255, 255, 0) 
                    elif marker_id in CORNER_MARKERS:
                        color = (100, 100, 100) 
                    else:
                        color = (255, 0, 255) 

                    cv2.polylines(
                        frame_copy,
                        [np.array(corners, np.int32).reshape((-1, 1, 2))],
                        isClosed=True,
                        color=color,
                        thickness=2,
                    )
                    cv2.circle(
                        frame_copy, center, radius=2, color=(0, 0, 255), thickness=-1
                    )
                    # Annotate marker ID
                    cv2.putText(
                        frame_copy,
                        str(marker_id),
                        center,
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1,
                        (255, 255, 0),
                        2,
                    )

            cv2.imshow("Robot Visualization", frame_copy)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break


def main():
    # Start the video capture and shared resources update in a separate thread
    capture_thread = threading.Thread(
        target=capture_and_update_shared_resources,
        # args=("http://127.0.0.1:5000/video_feed",),
        args=("http://192.168.1.185:8080/video",),
        daemon=True,
    )
    capture_thread.start()

    # Start a thread for each robot
    robot_threads = [
        threading.Thread(target=robot_control_loop, args=(robot_id,), daemon=True)
        for robot_id in ROBOT_IDS
    ]
    for thread in robot_threads:
        thread.start()

    # Visualization thread
    visualization_thread = threading.Thread(
        target=visualize_robot_behavior, daemon=True
    )
    visualization_thread.start()

    # Wait for the capture thread to finish
    capture_thread.join()

    # Threads are daemon threads, so they will exit when the main thread exits
    # Ensure all windows are closed properly
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
