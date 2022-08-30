import cv2
import mediapipe as mp
import pyfirmata
from time import sleep

b=pyfirmata.Arduino('/dev/cu.usbserial-14210')

mp_drawing = mp.solutions.drawing_utils
mp_pose = mp.solutions.pose

count = 0

cap = cv2.VideoCapture(0)
pose= mp_pose.Pose(
    min_detection_confidence=0.8,
    min_tracking_confidence=0.8)
while True:
    success, image = cap.read()
    image = cv2.cvtColor(cv2.flip(image, 1), cv2.COLOR_BGR2RGB)
    image.flags.writeable = False
    results = pose.process(image)
    if results.pose_landmarks:
     landmarks = []
     for id, lm in enumerate(results.pose_landmarks.landmark):
            h, w, c = image.shape 
            cx, cy = int(lm.x * w), int(lm.y * h) 
            landmarks.append((cx,cy))    
    else: continue
    image.flags.writeable = True
    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    mp_drawing.draw_landmarks(
        image, results.pose_landmarks, mp_pose.POSE_CONNECTIONS)
    nose=landmarks[0]
    cv2.putText(image, "nose="+str(nose), (85, 125), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 5)
    cv2.putText(image, "sleep="+str(count)+"s", (85, 175), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 5)

    right_eye_out = landmarks[8]
    right_eye_in = landmarks[4]
    left_eye_out = landmarks[7]
    left_eye_in = landmarks[1]

    if right_eye_out[1] < right_eye_in[1] and left_eye_out[1] < left_eye_in[1]:
        b.digital[2].write(1)
        count += 1
        sleep(1)

    if count > 10:
        b.digital[3].write(1)

    else:
        b.digital[2].write(0)

    b.digital[2].write(0)

    cv2.imshow('MediaPipe Pose', image)
    if cv2.waitKey(1) == ord('q'):
      break
cap.release()

