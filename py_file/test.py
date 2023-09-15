import cv2

# Callback function for mouse events
def display_pixel_coordinates(event, x, y, flags, param):
    if event == cv2.EVENT_MOUSEMOVE:
        # Display the pixel coordinates on the image window
        cv2.imshow('Image', img_copy)
        cv2.putText(img_copy, f"X: {x}, Y: {y}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

# Load an image
img_path = "path_to_your_image.jpg"
img = cv2.imread(img_path)
img_copy = img.copy()

# Check if image is loaded
if img is None:
    print("Error: Couldn't read the image.")
else:
    # Create an OpenCV window
    cv2.namedWindow('Image')
    # Set the mouse callback function to display_pixel_coordinates
    cv2.setMouseCallback('Image', display_pixel_coordinates)

    # Display the image
    cv2.imshow('Image', img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

