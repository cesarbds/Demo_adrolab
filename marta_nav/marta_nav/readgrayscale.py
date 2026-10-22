import cv2

# Replace 'your_image_path.png' with the actual path to your grayscale image
image_path = '/home/cesar/Downloads/depth.jpeg'

# Read the image in grayscale
gray_image = cv2.imread(image_path, cv2.IMREAD_UNCHANGED)

# Check if the image was successfully loaded
if gray_image is not None:
    # Print pixel values
    for row in gray_image:
        for pixel_value in row:
            print(pixel_value, end=' ')
        print()
else:
    print(f"Failed to read the image at path: {image_path}")
