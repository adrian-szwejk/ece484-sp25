import torch
import cv2
import numpy as np
import matplotlib.pyplot as plt
from utils.lane_detector import LaneDetector
from models.enet import ENet
import os

# Define dataset and checkpoint paths
DATASET_PATH = "/home/ap/Documents/UIUC/ECE 484/MP/MP1/MP1_Code/mp-release-sp25/src/mp1/data/tusimple"  # Path to the TUSimple dataset
CHECKPOINT_PATH = "checkpoints/enet_checkpoint_epoch_best.pth"  # Path to the trained model checkpoint

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

    # Define source points on the original image and corresponding destination points
    # Define source points on the original image
    # Bottom-left corner
    # Bottom-right corner
    # Mid-right point (shifted up and right)
    # Mid-left point (shifted up and left)

    src_points = np.float32([
        [0, 0], 
        [width, 0], 
        [(3 * width )// 8, 0.6 * height], 
        [(3 * width )// 4, 0.6 * height]
    ])
    dst_points = np.float32([[0, 0], [width, 0], [0, height], [width, height]])

    # Compute the perspective transform matrix using cv2.getPerspectiveTransform
    perspective_matrix = cv2.getPerspectiveTransform(src_points, dst_points)

    # Warp the original image using cv2.warpPerspective to get the transformed output
    transformed_image = cv2.warpPerspective(image, perspective_matrix, (width, height))


    #alternate code
    # #1
    # height, width = image.shape[:2]
    # #2 https://theailearner.com/tag/cv2-getperspectivetransform/
    # # src = (height, width)
    # # dst = (0, 0)

    # pt_A = [41, 2001]
    # pt_B = [2438, 2986]
    # pt_C = [3266, 371]
    # pt_D = [1772, 136]

    # # Here, I have used L2 norm. You can use L1 also.
    # width_AD = np.sqrt(((pt_A[0] - pt_D[0]) ** 2) + ((pt_A[1] - pt_D[1]) ** 2))
    # width_BC = np.sqrt(((pt_B[0] - pt_C[0]) ** 2) + ((pt_B[1] - pt_C[1]) ** 2))
    # maxWidth = max(int(width_AD), int(width_BC))
    
    
    # height_AB = np.sqrt(((pt_A[0] - pt_B[0]) ** 2) + ((pt_A[1] - pt_B[1]) ** 2))
    # height_CD = np.sqrt(((pt_C[0] - pt_D[0]) ** 2) + ((pt_C[1] - pt_D[1]) ** 2))
    # maxHeight = max(int(height_AB), int(height_CD))

    # src = np.float32([pt_A, pt_B, pt_C, pt_D])
    # dst = np.float32([[0, 0],
    #                         [0, maxHeight - 1],
    #                         [maxWidth - 1, maxHeight - 1],
    #                         [maxWidth - 1, 0]])

    # M = cv2.getPerspectiveTransform(src, dst)

    # transformed_image = cv2.warpPerspective(image,M,(maxWidth, maxHeight),flags=cv2.INTER_LINEAR)

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
