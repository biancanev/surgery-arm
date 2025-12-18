import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Button

print("Minimal Animation Test")
print("======================")
print("This tests if animation works on your system.\n")

fig, ax = plt.subplots(figsize=(8, 6))
line, = ax.plot([], [], 'o-', linewidth=2, markersize=8)
ax.set_xlim(0, 1)
ax.set_ylim(0, 1)
ax.set_title('Click "Animate" to test')
ax.grid(True)

is_animating = False

def animate_line(event):
    global is_animating
    if is_animating:
        return
    
    is_animating = True
    print("Starting animation...")
    
    x_data = []
    y_data = []
    
    for i in range(50):
        t = i / 49
        x_data.append(t)
        y_data.append(np.sin(t * 2 * np.pi) * 0.5 + 0.5)
        
        line.set_data(x_data, y_data)
        ax.set_title(f'Animating... Frame {i+1}/50')
        
        fig.canvas.draw()
        fig.canvas.flush_events()
        plt.pause(0.02)
    
    ax.set_title('Animation Complete!')
    is_animating = False
    print("Animation complete!")
    print("\nIf you saw a sine wave being drawn, animation works!")
    print("The robot simulation should work too.")

ax_button = plt.axes([0.4, 0.02, 0.2, 0.05])
btn = Button(ax_button, 'Animate')
btn.on_clicked(animate_line)

plt.ion()
plt.show(block=True)