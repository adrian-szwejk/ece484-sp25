---
# MP1: Model Training, Evaluation, and Testing Guide

This document provides detailed instructions for training, evaluating, and testing the model for lane detection. Follow the steps carefully to achieve optimal results.

## Training the Model

To train the model effectively, follow these steps:

1. **Run the Training Script**  
    Execute the `train.py` file with the optimal values for the following parameters:
    
    - **Epochs**: Number of training iterations.
        
    - **Learning Rate**: Step size for weight updates.
        
    - **Batch Size**: Number of samples processed in one forward/backward pass.  
        Ensure that the dataset is present locally and properly formatted.
        
2. **Adjust Loss Weights (Optional)**
    
    - If you need to train the model with a higher emphasis on segmentation or discriminative loss, adjust the weights in the total loss formula within the script. This allows you to fine-tune the model's focus on specific aspects of performance.
        
3. **Checkpoint Management**
    
    - During training, model checkpoints will be saved in the `/checkpoints` directory. Each checkpoint contains the weights and biases for a specific epoch.
        
    - After training, identify the optimal checkpoint file (e.g., `model_57.pth`) and rename it by replacing its epoch number (`_57`) with `_best`. For example: `model_best.pth`.
        
        - Checkpoints with `_disc` and `_seg` in their names refer to models trained with high discriminative and segmentation losses, respectively.
            
        - The `_best` checkpoint represents a balanced condition between losses.
            

## Model Evaluation

To evaluate the trained model:

1. Run the `eval.py` script.
    
2. Ensure that the dataset is present locally and correctly mapped in the script configuration.
    
3. The evaluation process will output metrics that help assess model performance.
    

## Model Testing for Lane Detection

To test lane detection using your trained model:

### 1. Source Point Selection for Perspective Transformation

- Use `point_finder.py` to identify source points in test images:
    
    - Test images should be located in the `/test_images` directory.
        
    - Load a test image into the script.
        
    - Click on points in the image as needed for perspective transformation.
        
    - The script will output coordinates (image height and width) for these points.
        
    - Update the `src` points in `test_lane_detection.py` based on this output.


### 2. Visualize Lane Detection

- Run `test_lane_detection.py` to visualize lane detection results.

## Additional Notes

- Always verify that your dataset is correctly formatted and accessible before running any scripts.
    
- Regularly monitor training logs to ensure there are no issues such as overfitting or underfitting.
    
- Keep backups of important checkpoints to avoid accidental data loss.

