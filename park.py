import heapq
import math
import tkinter as tk
from tkinter import messagebox, simpledialog

# Initialize default parking layout
def create_parking_layout(rows, cols, gates):
    layout = [[' ' for _ in range(cols)] for _ in range(rows)]
    # Place entry points based on layout dimensions
    for gate in gates:
        layout[gate[0]][gate[1]] = gate[2]  # gate[2] is the identifier (A, B, C, D, etc.)
    
    # Fill parking spaces
    for r in range(rows):
        for c in range(cols):
            if layout[r][c] == ' ':
                layout[r][c] = 'P'  # Fill with parking spots
    return layout

# Prompt user for grid size and gate positions
def get_grid_size_and_gates():
    rows = simpledialog.askinteger("Input", "Enter number of rows (min 4):", minvalue=4)
    cols = simpledialog.askinteger("Input", "Enter number of columns (min 4):", minvalue=4)
    
    gates = []
    number_of_gates = simpledialog.askinteger("Input", "Enter number of entry gates (max 4):", minvalue=1, maxvalue=4)

    for i in range(number_of_gates):
        gate_row = simpledialog.askinteger("Gate Position", f"Enter row for Gate {chr(65 + i)} (0 to {rows - 1}):", minvalue=0, maxvalue=rows - 1)
        gate_col = simpledialog.askinteger("Gate Position", f"Enter column for Gate {chr(65 + i)} (0 to {cols - 1}):", minvalue=0, maxvalue=cols - 1)
        gates.append((gate_row, gate_col, chr(65 + i)))  # chr(65 + i) converts to A, B, C, ...

    return rows, cols, gates

# Get the grid size and gates from the user
rows, cols, gates = get_grid_size_and_gates()
parking_layout = create_parking_layout(rows, cols, gates)

# Define entry points based on the new layout
entry_points = {gate[2]: (gate[0], gate[1]) for gate in gates}
parking_spots = {(r, c) for r in range(len(parking_layout)) for c in range(len(parking_layout[r])) if parking_layout[r][c] == 'P'}
occupied_spots = {}
car_data_file = "car_data_astar.txt"  # File to store car data
edit_mode = False
showing_path = False
last_path = []

def heuristic(a, b):
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)  # Euclidean distance

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
                
                tentative_g_score = g_score[current] + 1  # Each move costs 1 unit
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
        return None, None  # Return a tuple when no spots are available

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

    if nearest_spot is None:
        return None, None, "Parking area Full."  # Display a clear message for full parking

    if path:
        total_distance = len(path)  # Total distance in grid units
        occupied_spots[nearest_spot] = car_number
        with open(car_data_file, "a") as file:
            file.write(f"{car_number}, {path}, {nearest_spot}, Distance: {total_distance} units\n")
        
        return nearest_spot, path, total_distance
    else:
        return None, None, "No available parking spots, try entering from other gateways."

def find_nearest_exit(car_spot):
    return min(entry_points.values(), key=lambda x: heuristic(car_spot, x))

def unpark_car(car_number):
    for spot, number in occupied_spots.items():
        if number == car_number:
            nearest_exit = find_nearest_exit(spot)
            path = astar(spot, nearest_exit, parking_layout)
            total_distance = len(path)  # Total distance in grid units
            del occupied_spots[spot]
            with open(car_data_file, "a") as file:
                file.write(f"{car_number} unparked from {spot} to exit at {nearest_exit}, Path: {path}, Total Distance: {total_distance} units\n")
            return spot, nearest_exit, path, total_distance
    return None, None, None, None

def draw_parking_space(canvas, show_path=False, path=[]):
    canvas.delete("all")
    for r in range(len(parking_layout)):
        for c in range(len(parking_layout[r])):
            value = parking_layout[r][c]
            spot = (r, c)
            color = "white"
            display_value = ""

            if spot in path and show_path:
                color = "cyan"  # Path color
            elif value in entry_points:
                color = "yellow"
                display_value = value
            elif spot in occupied_spots:
                color = "red"
                display_value = occupied_spots[spot][-4:]
            elif value == 'P':
                color = "darkgreen"
                display_value = "P"
            elif value == 'X':
                color = "gray"

            x1, y1 = c * 50, r * 50  # Simple square grid
            x2, y2 = x1 + 50, y1 + 50  # Each cell is 50x50 pixels
            canvas.create_rectangle(x1, y1, x2, y2, fill=color)
            if display_value:
                canvas.create_text(x1 + 25, y1 + 25, text=display_value)  # Center text in the rectangle

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
    
    # Prevent changing entry gates
    if current_value in entry_points:
        return
    
    new_value = {' ': 'P', 'P': 'X', 'X': ' '}.get(current_value, ' ')
    parking_layout[r][c] = new_value
    draw_parking_space(canvas)

def update_layout():
    global parking_spots
    parking_spots = {(r, c) for r in range(len(parking_layout)) for c in range(len(parking_layout[r])) if parking_layout[r][c] == 'P'}

def on_park():
    car_number = car_number_entry.get().strip()
    entry_gate = entry_gate_entry.get().strip().upper()
    
    if entry_gate in entry_points:
        spot, path, total_distance = park_car(car_number, entry_gate)
        if spot:
            messagebox.showinfo("Parking Success", f"Car {car_number} parked at {spot}. Distance: {total_distance} units.")
            draw_parking_space(canvas, show_path=True, path=path)
        else:
            messagebox.showerror("Parking Failed", total_distance)  # total_distance is the error message here
    else:
        messagebox.showerror("Invalid Entry Gate", "Please enter a valid entry gate (A, B, C, D, etc.).")

def on_unpark():
    car_number = car_number_entry.get().strip()
    
    freed_spot, nearest_exit, path, total_distance = unpark_car(car_number)
    if freed_spot:
        nearest_exit_name = next(key for key, value in entry_points.items() if value == nearest_exit)
        messagebox.showinfo("Unparking Success", f"Car {car_number} has been unparked from {freed_spot} to exit at {nearest_exit_name}. Path: {path}. Total Distance: {total_distance} units.")
        draw_parking_space(canvas, show_path=True, path=path)
    else:
        messagebox.showerror("Unparking Failed", "Car not found.")

def toggle_path():
    global showing_path
    showing_path = not showing_path
    if showing_path:
        draw_parking_space(canvas, show_path=True, path=last_path)
    else:
        draw_parking_space(canvas)

# Create the main application window
root = tk.Tk()
root.title("Smart Parking System")

# Create a frame for the controls
control_frame = tk.Frame(root)
control_frame.pack()

tk.Label(control_frame, text="Car Number: ").grid(row=0, column=0)
car_number_entry = tk.Entry(control_frame)
car_number_entry.grid(row=0, column=1)

tk.Label(control_frame, text="Entry Gate: ").grid(row=1, column=0)
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
