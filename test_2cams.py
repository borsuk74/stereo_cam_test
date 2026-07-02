#gst-launch-1.0 nvarguscamerasrc sensor-id=0 num-buffers=30 !  'video/x-raw(memory:NVMM),width=1920,height=1080,framerate=30/1' ! nvvidconv ! fakesink
import cv2                                                                                                                                                                                                                                                                                                                     
                                                                                                                                                                                                                                                                                                                               
CAM_LEFT = 0
CAM_RIGHT = 1


def gstreamer_pipeline(sensor_id, capture_width=1920, capture_height=1080, framerate=30):
    return (
        f"nvarguscamerasrc sensor-id={sensor_id} ! "
        f"video/x-raw(memory:NVMM), width=(int){capture_width}, height=(int){capture_height}, "
        f"format=(string)NV12, framerate=(fraction){framerate}/1 ! "
        f"nvvidconv ! video/x-raw, format=(string)BGRx ! "
        f"videoconvert ! video/x-raw, format=(string)BGR ! appsink"
    )


cap_left = cv2.VideoCapture(gstreamer_pipeline(CAM_LEFT), cv2.CAP_GSTREAMER)
cap_right = cv2.VideoCapture(gstreamer_pipeline(CAM_RIGHT), cv2.CAP_GSTREAMER)
if not cap_left.isOpened() or not cap_right.isOpened():
    raise RuntimeError("Could not open both cameras")
                                                                                                                                                                                                                                                                                                                              
while True:
    ok_left, frame_left = cap_left.read()                                                                                                                                                                                                                                                                                      
    ok_right, frame_right = cap_right.read()                                                                                                                                                                                                                                                                                   
    if not ok_left or not ok_right:
        break                                                                                                                                                                                                                                                                                                                  
                                                                                                                                                                                                                                                                                                                               
    if frame_left.shape != frame_right.shape:

        frame_right = cv2.resize(frame_right, (frame_left.shape[1], frame_left.shape[0]))                                                                                                                                                                                                                                      
    combined = cv2.hconcat([frame_left, frame_right])                                                                                                                                                                                                                                                                          
    cv2.imshow("Stereo Cameras", combined)                                                                                                                                                                                                                                                                                     
                                                                                                                                                                                                                                                                                                                              
    if cv2.waitKey(1) & 0xFF == ord('q'):                                                                                                                                                                                                                                                                                      
       break                                                                                                                                                                                                                                                                                                                  
                                                                                                                                                                                                                                                                                                                               
cap_left.release()                                                                                                                                                                                                                                                                                                             
cap_right.release()                                                                                                                                                                                                                                                                                                            
cv2.destroyAllWindows()   