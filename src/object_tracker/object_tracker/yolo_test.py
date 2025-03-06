from ultralytics import YOLO
import cv2

# Load the trained YOLOv10-small model
model = YOLO("best.pt")  # Replace with the correct path

# Load the image
image_path = "human.jpg"  # Replace with your image path
image = cv2.imread(image_path)

# Run inference
results = model(image)

# Display the results
for result in results:
    print(result.boxes)  # Bounding boxes
    print(result.probs)  # Class probabilities if available
    result.show()  # Show image with predictions

# Save the output image
output_path = "output.jpg"
results[0].save(filename=output_path)
print(f"Output saved to {output_path}")

