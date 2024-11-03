import numpy as np
import heapq
import math
import tkinter as tk
from tkinter import messagebox

# Initialize default parking layout
default_parking_layout = [
    ['A', ' ', ' ', ' ', ' ', ' ', ' ', 'B'],
    [' ', ' ', 'P', 'P', 'P', 'P', ' ', ' '],
    ['X', 'P', 'P', 'X', 'P', 'X', 'P', 'X'],
    [' ', ' ', ' ', ' ', ' ', ' ', ' ', ' '],
    [' ', ' ', ' ', ' ', ' ', ' ', ' ', ' '],
    [' ', 'P', 'P', 'P', 'P', 'P', 'P', ' '],
    [' ', ' ', 'P', 'P', 'P', 'P', ' ', ' '],
    ['C', ' ', ' ', ' ', ' ', ' ', ' ', 'D']
]
parking_layout = [row[:] for row in default_parking_layout]

# Define entry and exit points
entry_points = {'A': (0, 0), 'B': (0, 7), 'C': (7, 0), 'D': (7, 7)}
parking_spots = {(r, c) for r in range(len(parking_layout)) for c in range(len(parking_layout[r])) if parking_layout[r][c] == 'P'}
occupied_spots = {}
car_data_file = "car_data_astar.txt"  # File to store car data
edit_mode = False

def heuristic(a, b):
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)

def astar(start, goal, layout):
    """A* search algorithm to find the optimal path."""
    rows, cols = len(layout), len(layout[0])
    open_set = []
    heapq.heappush(open_set, (0, start))
    
    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}
    
    while open_set:
        current = heapq.heappop(open_set)[1]
        
        if current == goal:
            return reconstruct_path(came_from, current)
        
        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:  # Up, down, left, right
            neighbor = (current[0] + dx, current[1] + dy)
            if (0 <= neighbor[0] < rows) and (0 <= neighbor[1] < cols):
                if layout[neighbor[0]][neighbor[1]] == 'X' or neighbor in occupied_spots:
                    continue
                
                tentative_g_score = g_score[current] + 1  # Distance to neighbor
                if neighbor in g_score and tentative_g_score >= g_score[neighbor]:
                    continue
                
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                
                if neighbor not in [i[1] for i in open_set]:
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))
    
    return None  # No path found

def reconstruct_path(came_from, current):
    total_path = [current]
    while current in came_from:
        current = came_from[current]
        total_path.append(current)
    return total_path[::-1]  # Reverse the path

def find_nearest_parking_spot(entry_point):
    empty_spots = [spot for spot in parking_spots if spot not in occupied_spots]
    if not empty_spots:
        return None

    shortest_path = None
    nearest_spot = None
    shortest_distance = float('inf')
    
    for spot in empty_spots:
        path = astar(entry_point, spot, parking_layout)
        if path:  # If a valid path exists
            path_length = len(path)
            if path_length < shortest_distance:
                shortest_distance = path_length
                nearest_spot = spot
                shortest_path = path

    return nearest_spot, shortest_path

def park_car(car_number, entry_gate):
    if car_number in occupied_spots.values():
        return None, None, "Car already parked."

    entry_point = entry_points[entry_gate]
    nearest_spot, path = find_nearest_parking_spot(entry_point)

    if nearest_spot and path:
        total_cost = len(path)
        occupied_spots[nearest_spot] = car_number
        with open(car_data_file, "a") as file:
            file.write(f"{car_number}, {path}, {nearest_spot}, Cost: {total_cost}\n")
        
        return nearest_spot, path, None
    else:
        return None, None, "No available parking spots,Try Entering from other gateways."

def find_nearest_exit(car_spot):
    return min(entry_points.values(), key=lambda x: heuristic(car_spot, x))

def unpark_car(car_number):
    for spot, number in occupied_spots.items():
        if number == car_number:
            nearest_exit = find_nearest_exit(spot)
            path = astar(spot, nearest_exit, parking_layout)
            del occupied_spots[spot]
            with open(car_data_file, "a") as file:
                file.write(f"{car_number} unparked from {spot} to exit at {nearest_exit}, Path: {path}\n")
            return spot, nearest_exit, path
    return None, None, None

def draw_parking_space(canvas, show_path=False, path=[]):
    canvas.delete("all")
    for r in range(len(parking_layout)):
        for c in range(len(parking_layout[r])):
            value = parking_layout[r][c]
            spot = (r, c)
            color = "white"
            display_value = ""

            if spot in path:
                color = "light blue" if show_path else "white"
            elif value in entry_points:
                color = "yellow"
                display_value = value
            elif spot in occupied_spots:
                color = "red"
                display_value = occupied_spots[spot][-4:]
            elif value == 'P':
                color = "green"
                display_value = "P"
            elif value == 'X':
                color = "gray"

            x1, y1 = c * 50, r * 50
            x2, y2 = x1 + 50, y1 + 50
            canvas.create_rectangle(x1, y1, x2, y2, fill=color)
            if display_value:
                canvas.create_text(x1 + 25, y1 + 25, text=display_value)

def toggle_edit_mode():
    global edit_mode
    edit_mode = not edit_mode
    edit_button.config(text="Exit Edit Mode" if edit_mode else "Enter Edit Mode")
    if not edit_mode:
        update_layout()
        draw_parking_space(canvas)

def cell_click(event):
    if not edit_mode:
        return

    c = event.x // 50
    r = event.y // 50
    if r >= len(parking_layout) or c >= len(parking_layout[0]):
        return
    
    current_value = parking_layout[r][c]
    new_value = {' ': 'P', 'P': 'X', 'X': ' '}.get(current_value, ' ')
    
    if new_value in ['A', 'B', 'C', 'D']:
        entry_points[new_value] = (r, c)
    parking_layout[r][c] = new_value
    draw_parking_space(canvas)

def update_layout():
    global parking_spots
    parking_spots = {(r, c) for r in range(len(parking_layout)) for c in range(len(parking_layout[r])) if parking_layout[r][c] == 'P'}

def on_park():
    car_number = car_number_entry.get().strip()
    entry_gate = entry_gate_entry.get().strip().upper()
    
    if entry_gate in entry_points:
        spot, path, error = park_car(car_number, entry_gate)
        if spot:
            messagebox.showinfo("Parking Success", f"Car {car_number} parked at {spot}.")
            draw_parking_space(canvas, show_path=True, path=path)
        else:
            messagebox.showerror("Parking Failed", error)
    else:
        messagebox.showerror("Invalid Entry Gate", "Please enter a valid entry gate (A, B, C, or D).")

def on_unpark():
    car_number = car_number_entry.get().strip()
    
    freed_spot, nearest_exit, path = unpark_car(car_number)
    if freed_spot:
        nearest_exit_name = next(key for key, value in entry_points.items() if value == nearest_exit)
        messagebox.showinfo("Unparking Success", f"Car {car_number} has been unparked from {freed_spot} to exit at {nearest_exit_name}. Path: {path}")
        draw_parking_space(canvas, show_path=True, path=path)
    else:
        messagebox.showerror("Unparking Failed", "Car not found.")

def toggle_path():
    draw_parking_space(canvas)

# Create the main application window
root = tk.Tk()
root.title("Smart Parking System")

# Create a frame for the controls
control_frame = tk.Frame(root)
control_frame.pack()

tk.Label(control_frame, text="Car Number:").grid(row=0, column=0)
car_number_entry = tk.Entry(control_frame)
car_number_entry.grid(row=0, column=1)

tk.Label(control_frame, text="Entry Gate (A, B, C, or D):").grid(row=1, column=0)
entry_gate_entry = tk.Entry(control_frame)
entry_gate_entry.grid(row=1, column=1)

park_button = tk.Button(control_frame, text="Park", command=on_park)
park_button.grid(row=2, column=0)

unpark_button = tk.Button(control_frame, text="Unpark", command=on_unpark)
unpark_button.grid(row=2, column=1)

toggle_path_button = tk.Button(control_frame, text="Toggle Path", command=toggle_path)
toggle_path_button.grid(row=3, column=0)

edit_button = tk.Button(control_frame, text="Enter Edit Mode", command=toggle_edit_mode)
edit_button.grid(row=3, column=1)

# Create the canvas for drawing the parking layout
canvas = tk.Canvas(root, width=400, height=400, bg="white")
canvas.pack()
canvas.bind("<Button-1>", cell_click)

# Draw the initial parking layout
draw_parking_space(canvas)

# Start the application loop
root.mainloop()
