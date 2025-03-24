import tkinter as tk
from tkinter import ttk
import numpy as np

class MapEditor:
    def __init__(self, master, grid_size=1000):
        self.master = master
        self.grid_size = grid_size
        self.cell_size = 2
        self.grid = np.zeros((grid_size, grid_size), dtype=int)
        self.start_pos = None
        self.goal_pos = None
        self.current_mode = "obstacle"
        self.last_pos = None
        self.create_widgets()

    def create_widgets(self):
        # Control panel
        control_frame = ttk.Frame(self.master)
        control_frame.pack(side=tk.TOP, pady=10)

        # Mode buttons
        ttk.Button(control_frame, text="Draw Obstacles", 
                 command=lambda: self.set_mode("obstacle")).grid(row=0, column=0, padx=5)
        ttk.Button(control_frame, text="Set Start", 
                 command=lambda: self.set_mode("start")).grid(row=0, column=1, padx=5)
        ttk.Button(control_frame, text="Set Goal", 
                 command=lambda: self.set_mode("goal")).grid(row=0, column=2, padx=5)
        ttk.Button(control_frame, text="Clear Map", 
                 command=self.clear_map).grid(row=0, column=3, padx=5)
        ttk.Button(control_frame, text="Save & Run", 
                 command=self.save_and_run).grid(row=0, column=4, padx=5)

        # Canvas with scrollbars
        canvas_frame = ttk.Frame(self.master)
        canvas_frame.pack(pady=10, expand=True, fill=tk.BOTH)

        self.canvas = tk.Canvas(canvas_frame, width=800, height=600, bg="white", cursor="crosshair",
                              scrollregion=(0, 0, self.grid_size*self.cell_size, self.grid_size*self.cell_size))
        h_scroll = ttk.Scrollbar(canvas_frame, orient=tk.HORIZONTAL, command=self.canvas.xview)
        v_scroll = ttk.Scrollbar(canvas_frame, orient=tk.VERTICAL, command=self.canvas.yview)
        self.canvas.configure(xscrollcommand=h_scroll.set, yscrollcommand=v_scroll.set)

        # Layout components
        self.canvas.grid(row=0, column=0, sticky="nsew")
        h_scroll.grid(row=1, column=0, sticky="ew")
        v_scroll.grid(row=0, column=1, sticky="ns")

        # Event bindings
        self.canvas.bind("<B1-Motion>", self.draw)
        self.canvas.bind("<ButtonRelease-1>", self.reset_last_pos)
        self.canvas.bind("<Button-1>", self.set_special_point)

        # Status bar
        self.status = ttk.Label(self.master, text="Current Mode: Obstacle Drawing")
        self.status.pack(side=tk.BOTTOM, fill=tk.X)

    def set_mode(self, mode):
        self.current_mode = mode
        mode_labels = {"obstacle": "Obstacle", "start": "Start", "goal": "Goal"}
        self.status.config(text=f"Current Mode: {mode_labels[mode]} Setting")

    def clear_map(self):
        """Reset map and all states"""
        self.grid = np.zeros((self.grid_size, self.grid_size), dtype=int)
        self.start_pos = None
        self.goal_pos = None
        self.canvas.delete("all")
        self.status.config(text="Map cleared!")

    def get_cell_coords(self, event):
        """Convert event coordinates to grid coordinates (supports scrolling)"""
        x = self.canvas.canvasx(event.x)
        y = self.canvas.canvasy(event.y)
        col = int(x // self.cell_size)
        row = int(y // self.cell_size)
        return (max(0, min(col, self.grid_size-1)), 
                max(0, min(row, self.grid_size-1)))

    def draw(self, event):
        """Handle obstacle drawing with continuous drag"""
        if self.current_mode != "obstacle":
            return
          
        x, y = self.get_cell_coords(event)
        if self.last_pos and (x, y) != self.last_pos:
            # Draw line using Bresenham's algorithm
            for (cx, cy) in self.bresenham_line(*self.last_pos, x, y):
                self.grid[cy][cx] = 1
                self.draw_cell(cx, cy, "black")
        else:
            self.grid[y][x] = 1
            self.draw_cell(x, y, "black")
        self.last_pos = (x, y)

    def set_special_point(self, event):
        """Set start or goal position"""
        x, y = self.get_cell_coords(event)
        color = "green" if self.current_mode == "start" else "red"
      
        # Clear old position
        if self.current_mode == "start" and self.start_pos:
            old_x, old_y = self.start_pos
            self.grid[old_y][old_x] = 0
            self.draw_cell(old_x, old_y, "white")
        elif self.goal_pos:
            old_x, old_y = self.goal_pos
            self.grid[old_y][old_x] = 0
            self.draw_cell(old_x, old_y, "white")

        # Set new position
        if self.current_mode == "start":
            self.start_pos = (x, y)
        else:
            self.goal_pos = (x, y)
          
        self.grid[y][x] = 0  # Ensure special points are walkable
        self.draw_cell(x, y, color)

    def draw_cell(self, x, y, color):
        """Draw single cell on canvas"""
        x0 = x * self.cell_size
        y0 = y * self.cell_size
        x1 = x0 + self.cell_size
        y1 = y0 + self.cell_size
        self.canvas.create_rectangle(x0, y0, x1, y1, fill=color, outline="")

    def bresenham_line(self, x0, y0, x1, y1):
        """Bresenham's line algorithm implementation"""
        points = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
      
        while True:
            points.append((x0, y0))
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy
        return points

    def reset_last_pos(self, event):
        self.last_pos = None

    def save_and_run(self):
        """Save map and start simulation"""
        if not self.start_pos or not self.goal_pos:
            self.status.config(text="Error: Set both start and goal positions!")
            return

        # Convert coordinates (row, column format)
        start_col, start_row = self.start_pos
        goal_col, goal_row = self.goal_pos
      
        np.savez("custom_map.npz",
                 grid=self.grid,
                 start=np.array([start_row, start_col]),
                 goal=np.array([goal_row, goal_col]))

        self.status.config(text="Map saved! Starting simulation...")
        self.master.after(1000, self.run_simulation)

    def run_simulation(self):
        """Close editor and launch simulation"""
        self.master.destroy()
        import subprocess
        subprocess.run(["python", "simulation_with_astar.py"])

if __name__ == "__main__":
    root = tk.Tk()
    root.title("Maze Map Editor")
    editor = MapEditor(root)
    root.mainloop()