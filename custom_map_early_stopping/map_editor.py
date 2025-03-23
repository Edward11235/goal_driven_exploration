import tkinter as tk
from tkinter import ttk
import numpy as np

class MapEditor:
    def __init__(self, master, grid_size=500):
        self.master = master
        self.grid_size = grid_size
        self.cell_size = 4  # Each cell is 4x4 pixels
      
        # Initialize map data
        self.grid = np.zeros((grid_size, grid_size), dtype=int)
        self.start_pos = None
        self.goal_pos = None
        self.current_mode = "obstacle"  # 'obstacle', 'start', 'goal'
      
        # Setup GUI
        self.create_widgets()
        self.last_pos = None
      
    def create_widgets(self):
        # Control Frame
        control_frame = ttk.Frame(self.master)
        control_frame.pack(side=tk.TOP, pady=10)
      
        # Mode Buttons
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
      
        # Canvas
        canvas_size = self.grid_size * self.cell_size
        self.canvas = tk.Canvas(self.master, width=canvas_size, height=canvas_size,
                              bg="white", cursor="crosshair")
        self.canvas.pack(pady=10)
      
        # Event Bindings
        self.canvas.bind("<B1-Motion>", self.draw)
        self.canvas.bind("<ButtonRelease-1>", self.reset_last_pos)
        self.canvas.bind("<Button-1>", self.set_special_point)
      
        # Status Bar
        self.status = ttk.Label(self.master, text="Current Mode: Obstacle Drawing")
        self.status.pack(side=tk.BOTTOM, fill=tk.X)
  
    def set_mode(self, mode):
        self.current_mode = mode
        self.status.config(text=f"Current Mode: {mode.capitalize()} Setting")
  
    def clear_map(self):
        self.grid = np.zeros((self.grid_size, self.grid_size), dtype=int)
        self.start_pos = None
        self.goal_pos = None
        self.canvas.delete("all")
        self.status.config(text="Map Cleared!")
  
    def get_cell_coords(self, event):
        col = event.x // self.cell_size
        row = event.y // self.cell_size
        return max(0, min(col, self.grid_size-1)), max(0, min(row, self.grid_size-1))
  
    def draw(self, event):
        x, y = self.get_cell_coords(event)
        if self.current_mode == "obstacle":
            if self.last_pos and (x, y) != self.last_pos:
                # Draw line between last position and current position
                for (cx, cy) in self.bresenham_line(self.last_pos[0], self.last_pos[1], x, y):
                    self.grid[cy][cx] = 1
                    self.draw_cell(cx, cy, "black")
            else:
                self.grid[y][x] = 1
                self.draw_cell(x, y, "black")
            self.last_pos = (x, y)
  
    def set_special_point(self, event):
        x, y = self.get_cell_coords(event)
        if self.current_mode == "start":
            self.start_pos = (x, y)
            self.draw_cell(x, y, "green")
        elif self.current_mode == "goal":
            self.goal_pos = (x, y)
            self.draw_cell(x, y, "red")
  
    def draw_cell(self, x, y, color):
        x0 = x * self.cell_size
        y0 = y * self.cell_size
        x1 = x0 + self.cell_size
        y1 = y0 + self.cell_size
        self.canvas.create_rectangle(x0, y0, x1, y1, fill=color, outline="")
  
    def bresenham_line(self, x0, y0, x1, y1):
        """Bresenham's line algorithm for obstacle drawing"""
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
        if self.start_pos is None or self.goal_pos is None:
            self.status.config(text="Error: Set both start and goal positions!")
            return
      
        np.savez("custom_map.npz", 
                grid=self.grid,
                start=self.start_pos,
                goal=self.goal_pos)
      
        self.status.config(text="Map saved! Starting simulation...")
        self.master.after(1000, self.run_simulation)
  
    def run_simulation(self):
        self.master.destroy()
        import subprocess
        subprocess.run(["python", "simulation.py", "custom_map.npz"])

if __name__ == "__main__":
    root = tk.Tk()
    root.title("Map Editor")
    editor = MapEditor(root)
    root.mainloop()