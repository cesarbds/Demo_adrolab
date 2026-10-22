import requests
import cv2
import numpy as np
from io import BytesIO

url = 'http://143.106.11.151:4950/perception/camera/0/depth'
res = requests.get(url, stream=True)

if res.status_code == 200:
    # Check the content type
    content_type = res.headers.get('Content-Type', 'unknown')
    print("Content-Type:", content_type)

    # Assuming it's JPEG content
    if 'image/jpeg' in content_type:
        # Decode JPEG content using np.uint8
        image_data = cv2.imdecode(np.frombuffer(res.content, dtype=np.uint8), cv2.IMREAD_COLOR)

        # Display the image
        cv2.imshow('JPEG Image', image_data)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    else:
        print("Unsupported content type. Unable to decode.")

else:
    print(f"Failed to fetch the image. Status code: {res.status_code}")
