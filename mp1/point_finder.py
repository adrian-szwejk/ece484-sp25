# py to find the points in the image in terms of height and width

import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from PIL import Image
import numpy as np

# Load the image
image_path = "test_images/1.jpg"  
# image_path = "test_images/2.jpg"
# image_path = "test_images/3.jpg"  
# image_path = "test_images/4.jpg"
# image_path = "test_images/5.jpg"

img = Image.open(image_path)
width, height = img.size  # Get image dimensions

# Display the image
fig, ax = plt.subplots()
ax.imshow(img)
ax.set_xlim([0, width])  # Set X-axis limits based on width
ax.set_ylim([height, 0])  # Set Y-axis limits based on height (invert Y-axis)
ax.set_xlabel("Width (pixels)")
ax.set_ylabel("Height (pixels)")

# Function to capture mouse clicks
clicked_points = []

def onclick(event):
    if event.xdata is not None and event.ydata is not None:
        # Calculate fractional coordinates
        x_frac = round(event.xdata / width, 3)
        y_frac = round(event.ydata / height, 3)

        # Convert to readable fraction format
        x_fractional = f"{x_frac} * width"
        y_fractional = f"{y_frac} * height"

        clicked_points.append((x_fractional, y_fractional))
        print(f"Clicked at: ({x_fractional}, {y_fractional})")

        # Plot the selected point
        ax.scatter(event.xdata, event.ydata, c='red', marker='x')
        fig.canvas.draw()

# Connect the click event
fig.canvas.mpl_connect("button_press_event", onclick)

plt.show()

# After selecting points, print them
print("Selected points:", clicked_points)
