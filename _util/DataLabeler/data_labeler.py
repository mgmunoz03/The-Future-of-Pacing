import os
import cv2
import tkinter as tk
from PIL import Image, ImageTk
import csv

# Constants for guideline positions
MID_HEIGHT_RATIO = 0.5
BOTTOM_HEIGHT_RATIO = 0.9
SUPPORTED_EXTENSIONS = ('.png', '.jpg', '.jpeg', '.bmp')

class LaneLabelingTool:
    def __init__(self, image_dir, output_csv="labels.csv"):
        self.image_dir = image_dir
        self.output_csv = output_csv
        self.image_files = [f for f in os.listdir(image_dir) if f.lower().endswith(SUPPORTED_EXTENSIONS)]
        self.index = 0
        self.clicks = []

        # Tkinter setup
        self.root = tk.Tk()
        self.root.title("ESP32-CAM Lane Labeling Tool")
        self.canvas = tk.Canvas(self.root)
        self.canvas.pack()

        self.canvas.bind("<Button-1>", self.on_click)
        self.load_image()

        self.root.mainloop()

    def load_image(self):
        if self.index >= len(self.image_files):
            self.root.quit()
            return
        file_path = os.path.join(self.image_dir, self.image_files[self.index])
        self.cv_image = cv2.imread(file_path, cv2.IMREAD_GRAYSCALE)
        self.height, self.width = self.cv_image.shape
        self.image = Image.fromarray(self.cv_image)
        self.tk_image = ImageTk.PhotoImage(image=self.image)
        self.canvas.config(width=self.width, height=self.width)
        self.canvas.create_image(0, 0, anchor=tk.NW, image=self.tk_image)
        
        # Draw guide lines
        self.y_mid = int(self.height * MID_HEIGHT_RATIO)
        self.y_bottom = int(self.height * BOTTOM_HEIGHT_RATIO)
        self.canvas.create_line(0, self.y_mid, self.width, self.y_mid, fill="red", dash=(4,2))
        self.canvas.create_line(0, self.y_bottom, self.width, self.y_bottom, fill="blue", dash=(4,2))

        self.clicks = []
    
    def on_click(self, event):
        # Expecting 4 clicks: mid-left, mid-right, bottom-left, bottom-right
        self.clicks.append((event.x, event.y))

        if len(self.clicks) == 4:
            filename = self.image_files[self.index]
            norm_coords = [round(p[0] / self.width, 4) for p in self.clicks] # x1, x2, x3, x4
            self.save_label(filename, norm_coords)
            self.index += 1
            self.load_image()

    def save_label(self, filename, norm_coords):
        file_exists = os.path.exists(self.output_csv)
        with open(self.output_csv, mode='a', newline='') as f:
            writer = csv.writer(f)
            if not file_exists:
                writer.writerow(['filename', 'mid_left_x', 'mid_right_x', 'bottom_left_x', 'bottom_right_x'])
            writer.writerow([filename] + norm_coords)
