# Need to install the following libraries
# In Pycharm, go to File-> Settings:->Project:->Python Interpreter:-> '+'
# opencv-python
# ultralytics
# cvzone
# after installing these need to check version of lapx. lapx>=0.5.2.
# It will auto update on first execution to lapx-0.5.11
# Then download a copy of the yolo11s.pt file
# Details about yolo11 can be found here https://github.com/ultralytics/ultralytics
import cv2
from ultralytics import YOLO
from collections import deque
import cvzone
import serial
import time
# define some variables that are going to be used later
count = 0
# Load the YOLO 11 Model
model = YOLO("yolo11s.pt")
# Load the list of detectable objects
names = model.model.names
# uncomment the following line if you want to see the list of objects that can be detected.
# print(names)
# The video to be processed can come from a file or an attached web camera
# For a camera feed un comment the following line, NB: you may need to adjust the number
# if more than one camera is installed
cap = cv2.VideoCapture(0)
# Serial Monitor
serialcomm = serial.Serial('COM18', 9600)
serialcomm.timeout = 1

# For processing a saved video file un comment the following line
# cap = cv2.VideoCapture('Video.mp4')
framesPerSecond = int(cap.get(cv2.CAP_PROP_FPS)) # or 30  # Get FPS (default to 30 if unknown)
# framesPerSecond = 60
print(f"framesPerSecond= {framesPerSecond}")
frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # Codec for the output video
# Frame buffer to store the last 5 seconds
past_buffer = deque(maxlen=10)
future_buffer = []  # Temporary buffer for the next 2 seconds
capturing_future = False
man_overboard = False
frames_to_capture = 10  # Number of frames for the next 2 seconds
message = "Triggered"
PicklePatrolActive = False
cv2.namedWindow("PicklePatrol",cv2.WINDOW_FULLSCREEN)


# Define a function that play a video on loop with a countdown timer
def play_video_with_timer(video_path, countdown_time):
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        print(f"Error: Cannot open video {video_path}")
        return
    # Get video properties
    fps = int(cap.get(cv2.CAP_PROP_FPS))
    frame_delay = int(1000 / fps)  # Delay between frames in milliseconds
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 1
    font_color = (0, 255, 0)  # Green
    thickness = 2
    position = (250, 50)  # Top-left corner
    start_time = time.time()
    while time.time() < start_time + countdown_time:
        # Reset video if it reaches the end
        cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                break
            # Calculate the remaining countdown time
            elapsed_time = time.time() - start_time
            remaining_time = max(0, countdown_time - int(elapsed_time))
            # Display the countdown timer on the video
            timer_text = f"EPIRB launch in: {remaining_time} sec"
            cv2.putText(frame, timer_text, position, font, font_scale, font_color, thickness)
            cvzone.putTextRect(frame, "If False Alarm, press Q to cancel", (150, 100), 1, 1, (0, 0, 0), (0, 255, 0))
            # Show the video frame
            biggerFrame = cv2.resize(frame, (960, 720))
            cv2.imshow("Overboard Footage with Countdown Timer", biggerFrame)
            # Break the loop if time runs out
            if remaining_time <= 0:
                print("Countdown completed!")
                serialcomm.write('y'.encode())
                global message
                message = "System must be reset. EPIRB has been launched."
                global PicklePatrolActive
                PicklePatrolActive = False
                break
            # Wait for a key press or frame delay
            if cv2.waitKey(frame_delay) & 0xFF == ord('q'):
                print("Playback interrupted by user.")
                cap.release()
                cv2.destroyAllWindows()
                return
        # Reset start time for looping
        # start_time = time.time()
# Usage
# video_path = "output_20241123-123456.mp4"  # Replace with your video path
# countdown_time = 20  # Countdown time in seconds
# play_video_with_timer(video_path, countdown_time)
# Setup a loop that runs continuously to process the video frames
while True:
    frame =[]
    ret, frame = cap.read()
    if not ret:
        break
    # flip the image so that if the person in front of camera moves to the right, the image moves right on the screen
    frame = cv2.flip(frame,1)
    count += 1
    if count % 3 != 0:
        continue
    height, width, _ = frame.shape
    results = model.track(frame, persist=True, classes=0)
    if results[0].boxes is not None and results[0].boxes.id is not None:
        # Get the boxes (x, y, w, h), class IDs, track IDs, and confidences
        boxes = results[0].boxes.xyxy.int().cpu().tolist()  # Bounding boxes
        class_ids = results[0].boxes.cls.int().cpu().tolist()  # Class IDs
        track_ids = results[0].boxes.id.int().cpu().tolist()  # Track IDs
        confidences = results[0].boxes.conf.cpu().tolist()  # Confidence score
        for box, class_id, track_id, conf in zip(boxes, class_ids, track_ids, confidences):
            c = names[class_id]
            x1, y1, x2, y2 = box
            cx = int(x1 + x2) // 2
            cy = int(y1 + y2) // 2
            # cv2.circle(frame, (cx, cy), 4, (255, 0, 0), -1)
            # If the person is on the left of the yellow line they are considered on the boat
            # If they are detected to the right of the line they have fallen overboard
            if cx > 320:  # look at the X coord of the center to see if its greater than the handrail
                # Draw Red Rectangle around detected people
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
                if PicklePatrolActive is True:
                    man_overboard = True
                    capturing_future = True
                    print("Capturing video after event...")
            else:
                # Draw Green Rectangle around detected people
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            # Print the tracker ID if you want a count of the people detected
            # cvzone.putTextRect(frame, f'{track_id}', (x1, y2), 1, 1)
            cv2.line(frame, (320, 40), (320, 480), (0, 255, 255), 1)
            cvzone.putTextRect(frame, "Ship Deck", (10, 460), 1, 1,(0,0,0))

            cvzone.putTextRect(frame, "Ocean", (550, 460), 1, 1,(0,0,0))
            cvzone.putTextRect(frame, "Hand Rail", (280, 460), 1, 1,(0,0,0))
            cleanFrame = frame.copy() # Get a copy of the frame with the Active State Text
            if man_overboard is True:
                cvzone.putTextRect(frame,message,(50,100),1,1,(0,0,0))
            if PicklePatrolActive:
                cvzone.putTextRect(frame, "Pickle Patrol Monitoring: Active",(10,20),1,1,(0,0,0))
            else:
                cvzone.putTextRect(frame, "Pickle Patrol Monitoring: De-Active",(10,20),1,1,(0,0,0))
            cvzone.putTextRect(frame, "press ""A"" to activate, ""D"" to de-activate",(10,40),1,1,(0,0,0))

            # cvzone.putTextRect(frame, f'{c}', (x1, y1), 1, 1)
            # cvzone.putTextRect(frame, f'{int(conf * 100)}%', (x1, y1), 1, 1)


    # If capturing future frames, add them to the future buffer
    if capturing_future:
        future_buffer.append(cleanFrame)
        print("Adding Frame to Future Buffer")
        if len(future_buffer) >= frames_to_capture:
            print("Future Buffer Now Full")
            # Stop capturing future frames once 2 seconds worth are captured
            capturing_future = False
            # Save the combined video
            # timestamp = time.strftime("%Y%m%d-%H%M%S")
            # output_filename = f'output_{timestamp}.mp4'
            output_filename = "OverboardEvent.mp4"
            out = cv2.VideoWriter(output_filename, fourcc, 10, (frame_width, frame_height))
            # Write past frames
            for buffered_frame in past_buffer:
                out.write(buffered_frame)
            # Write future frames
            for buffered_frame in future_buffer:
                out.write(buffered_frame)
            out.release()
            print(f"Saved the video (past 2 seconds and next 2 seconds) to {output_filename}")
            future_buffer = []  # Reset future buffer
            message = "Done Recording"
            play_video_with_timer(output_filename, 5)
    else:
        # Add the frame to the past buffer
        past_buffer.append(cleanFrame)
    bigger = cv2.resize(frame,(960,720))
    cv2.imshow("PicklePatrol", bigger)
    result = cv2.waitKey(1)
    if result & 0xFF == ord("q"):
        break
    if result & 0xFF == ord("a"):
        PicklePatrolActive = True
    if result & 0xFF == ord ("d"):
        PicklePatrolActive = False
# Release the video capture object and close the display window
cap.release()
cv2.destroyAllWindows()
