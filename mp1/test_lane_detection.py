import torch
import cv2
import numpy as np
import matplotlib.pyplot as plt
from utils.lane_detector import LaneDetector
from models.enet import ENet
import os

# Define dataset and checkpoint paths
DATASET_PATH = "/home/ap/Documents/UIUC/ece_484/MP/MP1/MP1_Code/mp-release-sp25/src/mp1/data/tusimple"  # Path to the TUSimple dataset
# CHECKPOINT_PATH = "checkpoints/enet_checkpoint_epoch_best.pth"  # Path to the trained model checkpoint
# CHECKPOINT_PATH = "checkpoints/enet_checkpoint_epoch_best_disc.pth"  # Path to the trained model checkpoint
CHECKPOINT_PATH = "checkpoints/enet_checkpoint_epoch_best_seg2.pth"  # Path to the trained model checkpoint

# Function to load the ENet model
def load_enet_model(checkpoint_path, device="cuda"):
    enet_model = ENet(binary_seg=2, embedding_dim=4).to(device)
    checkpoint = torch.load(checkpoint_path, map_location=device)
    enet_model.load_state_dict(checkpoint['model_state_dict'])
    enet_model.eval()
    return enet_model

def perspective_transform(image):
    """
    Transform an image into a bird's eye view.
        1. Calculate the image height and width.
        2. Define source points on the original image and corresponding destination points.
        3. Compute the perspective transform matrix using cv2.getPerspectiveTransform.
        4. Warp the original image using cv2.warpPerspective to get the transformed output.
    """
    
    ####################### TODO: Your code starts Here #######################
    
    # Calculate the image height and width
    height, width = image.shape[:2]

    src_points = np.float32([
        [0.215 * width, 0.835 * height], 
        [0.969 * width, 0.835 * height], 
        [0.371 * width, 0.486 * height], 
        [0.645 * width, 0.486 * height]
    ])
    dst_points = np.float32([[0, height], [width, height], [0, 0], [width, 0]])

    # Compute the perspective transform matrix using cv2.getPerspectiveTransform
    perspective_matrix = cv2.getPerspectiveTransform(src_points, dst_points)

    # Warp the original image using cv2.warpPerspective to get the transformed output
    transformed_image = cv2.warpPerspective(image, perspective_matrix, (width, height))


    ####################### TODO: Your code ends Here #######################
    
    return transformed_image


# Function to visualize lane predictions for multiple images in a single row
def visualize_lanes_row(images, instances_maps, alpha=0.7):
    """
    Visualize lane predictions for multiple images in a single row
    For each image:
        1. Resize it to 512 x 256 for consistent visualization.
        2. Apply perspective transform to both the original image and its instance map.
        3. Overlay the instance map to a plot with the corresponding original image using a specified alpha value.
    """
    
    num_images = len(images)
    fig, axes = plt.subplots(1, num_images, figsize=(15, 5))

    ####################### TODO: Your code starts Here #######################

    for i in range(num_images):
        # Resize the original image and its instance map to 512 x 256
        resized_image = cv2.resize(images[i], (512, 256), interpolation=cv2.INTER_NEAREST)
        resized_instances_map = cv2.resize(instances_maps[i], (512, 256), interpolation=cv2.INTER_NEAREST)

        # Apply perspective transform to both the original image and its instance map
        transformed_image = perspective_transform(resized_image)
        transformed_instances_map = perspective_transform(resized_instances_map)

        # Overlay the instance map to a plot with the corresponding original image using a specified alpha value 

        axes[i].imshow(transformed_image, alpha=alpha, cmap="gray")
        axes[i].imshow(transformed_instances_map, alpha=alpha, cmap="jet")
        axes[i].axis("off")

    # Set the title for the plot
    plt.suptitle("Lane Predictions")
    
    ####################### TODO: Your code ends Here #######################

    plt.tight_layout()
    plt.show()

def main():
    # Initialize device and model
    device = "cuda" if torch.cuda.is_available() else "cpu"
    enet_model = load_enet_model(CHECKPOINT_PATH, device)
    lane_predictor = LaneDetector(enet_model, device=device)

    # List of test image paths
    sub_paths = [
        "test_set/clips/0530/1492626047222176976_0/20.jpg",
        "test_set/clips/0530/1492626286076989589_0/20.jpg",
        "test_set/clips/0531/1492626674406553912/20.jpg",
        "test_set/clips/0601/1494452381594376146/20.jpg",
        "test_set/clips/0601/1494452431571697487/20.jpg"
    ]
    test_image_paths = [os.path.join(DATASET_PATH, sub_path) for sub_path in sub_paths]

    # Load and process images
    images = []
    instances_maps = []

    for path in test_image_paths:
        image = cv2.imread(path)
        if image is None:
            print(f"Error: Unable to load image at {path}")
            continue

        print(f"Processing image: {path}")
        instances_map = lane_predictor(image)
        images.append(image)
        instances_maps.append(instances_map)

    # Visualize all lane predictions in a single row
    if images and instances_maps:
        visualize_lanes_row(images, instances_maps)

if __name__ == "__main__":
    main()
