
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
        self.moving_obstacles = []
        self.current_moving_obstacle_path = []
        self.create_widgets()

    def create_widgets(self):
        control_frame = ttk.Frame(self.master)
        control_frame.pack(side=tk.TOP, pady=10)

        ttk.Button(control_frame, text="Draw Obstacles", 
                 command=lambda: self.set_mode("obstacle")).grid(row=0, column=0, padx=5)
        ttk.Button(control_frame, text="Set Start", 
                 command=lambda: self.set_mode("start")).grid(row=0, column=1, padx=5)
        ttk.Button(control_frame, text="Set Goal", 
                 command=lambda: self.set_mode("goal")).grid(row=0, column=2, padx=5)
        ttk.Button(control_frame, text="Moving Obstacle", 
                 command=self.start_moving_obstacle).grid(row=0, column=3, padx=5)
        ttk.Button(control_frame, text="Clear Map", 
                 command=self.clear_map).grid(row=0, column=4, padx=5)
        ttk.Button(control_frame, text="Save & Run", 
                 command=self.save_and_run).grid(row=0, column=5, padx=5)

        canvas_frame = ttk.Frame(self.master)
        canvas_frame.pack(pady=10, expand=True, fill=tk.BOTH)

        self.canvas = tk.Canvas(canvas_frame, width=800, height=600, bg="white", cursor="crosshair",
                              scrollregion=(0, 0, self.grid_size*self.cell_size, self.grid_size*self.cell_size))
        h_scroll = ttk.Scrollbar(canvas_frame, orient=tk.HORIZONTAL, command=self.canvas.xview)
        v_scroll = ttk.Scrollbar(canvas_frame, orient=tk.VERTICAL, command=self.canvas.yview)
        self.canvas.configure(xscrollcommand=h_scroll.set, yscrollcommand=v_scroll.set)

        self.canvas.grid(row=0, column=0, sticky="nsew")
        h_scroll.grid(row=1, column=0, sticky="ew")
        v_scroll.grid(row=0, column=1, sticky="ns")

        self.canvas.bind("<B1-Motion>", self.draw)
        self.canvas.bind("<ButtonRelease-1>", self.reset_last_pos)
        self.canvas.bind("<Button-1>", self.set_special_point)
        self.canvas.bind("<Button-3>", self.on_canvas_right_click)

        self.status = ttk.Label(self.master, text="Current Mode: Obstacle Drawing")
        self.status.pack(side=tk.BOTTOM, fill=tk.X)

    def set_mode(self, mode):
        self.current_mode = mode
        self.status.config(text=f"Current Mode: {mode.capitalize()}")

    def clear_map(self):
        self.grid.fill(0)
        self.start_pos = None
        self.goal_pos = None
        self.moving_obstacles.clear()
        self.canvas.delete("all")

    def get_cell_coords(self, event):
        x = int(self.canvas.canvasx(event.x) // self.cell_size)
        y = int(self.canvas.canvasy(event.y) // self.cell_size)
        return (x, y)

    def draw(self, event):
        if self.current_mode == "obstacle":
            x, y = self.get_cell_coords(event)
            self.grid[y, x] = 1
            self.draw_cell(x, y, "black")

    def reset_last_pos(self, event):
        self.last_pos = None

    def set_special_point(self, event):
        x, y = self.get_cell_coords(event)
        if self.current_mode == "start":
            if self.start_pos:
                self.draw_cell(*self.start_pos, "white")
            self.start_pos = (x, y)
            self.draw_cell(x, y, "green")
        elif self.current_mode == "goal":
            if self.goal_pos:
                self.draw_cell(*self.goal_pos, "white")
            self.goal_pos = (x, y)
            self.draw_cell(x, y, "red")
        elif self.current_mode == "moving_obstacle":
            self.current_moving_obstacle_path.append((x, y))
            self.draw_cell(x, y, "orange")

    def on_canvas_right_click(self, event):
        if self.current_mode == "moving_obstacle":
            if len(self.current_moving_obstacle_path) >= 2:
                self.moving_obstacles.append(self.current_moving_obstacle_path.copy())
            self.current_moving_obstacle_path.clear()

    def draw_cell(self, x, y, color):
        self.canvas.create_rectangle(x*self.cell_size, y*self.cell_size,
                                     (x+1)*self.cell_size, (y+1)*self.cell_size,
                                     fill=color, outline="")

    def save_and_run(self):
        np.savez("custom_map.npz", grid=self.grid,
                 start=self.start_pos, goal=self.goal_pos,
                 moving_obstacles=np.array(self.moving_obstacles, dtype=object))
        self.master.destroy()
        import subprocess
        subprocess.run(["python", "simulation_with_astar.py"])

    def start_moving_obstacle(self):
        self.current_mode = "moving_obstacle"
        self.current_moving_obstacle_path = []
        self.status.config(text="Current Mode: Moving Obstacle - Left click to set path, Right click to finish")

if __name__ == "__main__":
    root = tk.Tk()
    root.title("Maze Map Editor")
    editor = MapEditor(root)
    root.mainloop()