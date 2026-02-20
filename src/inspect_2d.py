import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Button
from pathlib import Path
from rosbags.highlevel import AnyReader

# ---------------- CONFIGURATION ----------------
BAG_FOLDER = Path('depth') 
TOPIC_NAME = '/depth'
# Final robust parameters to fix 'black gaps'
MIN_DEPTH, MAX_DEPTH = 0.5, 2.0
Y_CROP_MIN, Y_CROP_MAX = -0.7, 0.5 # Widen to accommodate tumbling height
X_CROP_MIN, X_CROP_MAX = -0.8, 0.8
FX, FY, CX, CY = 525.0, 525.0, 319.5, 239.5

class ImageInspector:
    def __init__(self):
        self.frames = []
        self.current_idx = 0
        print("Loading frames...")
        with AnyReader([BAG_FOLDER]) as reader:
            conns = [x for x in reader.connections if x.topic == TOPIC_NAME]
            for conn, ts, raw in reader.messages(connections=conns):
                msg = reader.deserialize(raw, conn.msgtype)
                depth = np.ndarray(shape=(msg.height, msg.width), dtype=np.uint16, buffer=msg.data)
                depth_m = depth.astype(np.float32) / 1000.0 if np.max(depth) > 100 else depth.astype(np.float32)
                self.frames.append((ts, depth_m))
        print(f"Loaded {len(self.frames)} frames.")

    def update_plot(self, event=None):
        ts, depth = self.frames[self.current_idx]
        rows, cols = depth.shape
        v, u = np.indices((rows, cols))
        
        # Calculate 3D projected coordinates for mask logic
        x = (u - CX) * depth / FX
        y = (v - CY) * depth / FY
        
        # LOOSENED Mask Logic to fix Frame 3-6 gaps
        mask = (depth > MIN_DEPTH) & (depth < MAX_DEPTH) & \
               (y < Y_CROP_MAX) & (y > Y_CROP_MIN) & \
               (x < X_CROP_MAX) & (x > X_CROP_MIN)
        
        masked_view = depth.copy()
        masked_view[~mask] = 0 
        
        self.ax1.clear(); self.ax2.clear()
        self.ax1.imshow(depth, cmap='inferno', vmin=0, vmax=3.0)
        self.ax1.set_title(f"Raw Depth (Frame {self.current_idx})\nTime: {ts}"); self.ax1.axis('off')
        
        self.ax2.imshow(masked_view, cmap='inferno', vmin=0, vmax=3.0)
        self.ax2.set_title("Algorithm Mask (ROI - Loosened)"); self.ax2.axis('off')
        plt.draw()

    def next_frame(self, event): self.current_idx = (self.current_idx + 1) % len(self.frames); self.update_plot()
    def prev_frame(self, event): self.current_idx = (self.current_idx - 1) % len(self.frames); self.update_plot()

    def run(self):
        if not self.frames: return
        self.fig, (self.ax1, self.ax2) = plt.subplots(1, 2, figsize=(14, 7))
        axprev = plt.axes([0.3, 0.02, 0.1, 0.05]); axnext = plt.axes([0.6, 0.02, 0.1, 0.05])
        bnext = Button(axnext, 'Next >'); bnext.on_clicked(self.next_frame)
        bprev = Button(axprev, '< Prev'); bprev.on_clicked(self.prev_frame)
        self.update_plot(); plt.show()

if __name__ == "__main__":
    app = ImageInspector(); app.run()