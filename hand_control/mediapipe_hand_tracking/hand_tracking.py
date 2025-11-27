#!/usr/bin/env python3
"""
Real-time Hand Tracking using MediaPipe Hand Landmarker
Detects hands and tracks 21 landmarks per hand with high accuracy
"""
import cv2
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
import numpy as np
import time

# MediaPipe drawing utilities
mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles

print("Initializing MediaPipe Hand Landmarker...")

# Create Hand Landmarker with VIDEO mode for temporal tracking
base_options = python.BaseOptions(model_asset_path='models/hand_landmarker.task')
options = vision.HandLandmarkerOptions(
    base_options=base_options,
    running_mode=vision.RunningMode.VIDEO,  # VIDEO mode for temporal tracking
    num_hands=2,  # Track up to 2 hands
    min_hand_detection_confidence=0.1,  # Very low threshold for flat hands
    min_hand_presence_confidence=0.1,   # Very low threshold for better tracking
    min_tracking_confidence=0.01          # Very low threshold for fast motion
)
detector = vision.HandLandmarker.create_from_options(options)

print("✅ Hand Landmarker initialized successfully (VIDEO mode)!")

# Open webcam (same approach as Depth-Anything scripts)
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cap.set(cv2.CAP_PROP_FPS, 60)  # Increased to 60 FPS for fast motion

# Optimize camera settings for fast motion
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Reduce buffer to minimize lag
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3)  # Enable auto-exposure (3 = auto mode)

if not cap.isOpened():
    print("\n❌ Error: Could not open webcam!")
    print("\nTroubleshooting:")
    print("  1. Add yourself to video group: sudo usermod -aG video $USER")
    print("  2. Then logout and login again (or reboot)")
    print("  3. Check camera: ls -la /dev/video*")
    exit(1)

# Get actual FPS (camera may not support 60)
actual_fps = cap.get(cv2.CAP_PROP_FPS)
print(f"Webcam opened: 640x480 @ {actual_fps:.0f} FPS (requested 60 FPS)")
print("Press 'q' to quit, 's' to save frame, 'd' to toggle debug info")

frame_count = 0
fps_list = []
show_debug = True
timestamp_ms = 0  # Track timestamps for VIDEO mode

# Hand landmark connections for drawing
HAND_CONNECTIONS = mp_hands.HAND_CONNECTIONS

while True:
    ret, frame = cap.read()
    
    if not ret:
        print("Error: Could not read frame")
        break
    
    start_time = time.time()
    
    # Convert BGR to RGB (MediaPipe uses RGB)
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    
    # Create MediaPipe Image
    mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=frame_rgb)
    
    # Detect hands with timestamp for VIDEO mode
    timestamp_ms = int(time.time() * 1000)  # Convert to milliseconds
    detection_result = detector.detect_for_video(mp_image, timestamp_ms)
    
    process_time = time.time() - start_time
    fps = 1.0 / process_time if process_time > 0 else 0
    fps_list.append(fps)
    if len(fps_list) > 30:
        fps_list.pop(0)
    avg_fps = np.mean(fps_list)
    
    # Draw hand landmarks on the frame
    annotated_frame = frame.copy()
    
    if detection_result.hand_landmarks:
        for hand_idx, hand_landmarks in enumerate(detection_result.hand_landmarks):
            # Get handedness (Left/Right)
            handedness = detection_result.handedness[hand_idx][0]
            hand_label = handedness.category_name
            hand_score = handedness.score
            
            # Convert normalized landmarks to pixel coordinates
            h, w, _ = frame.shape
            landmark_points = []
            for landmark in hand_landmarks:
                x = int(landmark.x * w)
                y = int(landmark.y * h)
                landmark_points.append((x, y))
            
            # Draw landmarks
            for idx, (x, y) in enumerate(landmark_points):
                # Draw circles for landmarks
                color = (0, 255, 0) if hand_label == "Right" else (255, 0, 0)
                cv2.circle(annotated_frame, (x, y), 5, color, -1)
                
                # Draw landmark index for key points
                if show_debug and idx in [0, 4, 8, 12, 16, 20]:  # Wrist and fingertips
                    cv2.putText(annotated_frame, str(idx), (x+10, y), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)
            
            # Draw connections
            for connection in HAND_CONNECTIONS:
                start_idx = connection[0]
                end_idx = connection[1]
                cv2.line(annotated_frame, 
                         landmark_points[start_idx], 
                         landmark_points[end_idx], 
                         color, 2)
            
            # Draw hand label and bounding box
            # Calculate bounding box
            xs = [p[0] for p in landmark_points]
            ys = [p[1] for p in landmark_points]
            x_min, x_max = min(xs), max(xs)
            y_min, y_max = min(ys), max(ys)
            
            # Draw bounding box
            cv2.rectangle(annotated_frame, (x_min-10, y_min-10), (x_max+10, y_max+10), color, 2)
            
            # Draw label
            label_text = f"{hand_label} ({hand_score:.2f})"
            cv2.putText(annotated_frame, label_text, (x_min-10, y_min-25), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
            
            # Get wrist and fingertip positions for gestures
            wrist = landmark_points[0]
            thumb_tip = landmark_points[4]
            index_tip = landmark_points[8]
            middle_tip = landmark_points[12]
            ring_tip = landmark_points[16]
            pinky_tip = landmark_points[20]
            
            # Simple gesture detection: Thumbs up
            if show_debug:
                # Display key landmark coordinates
                info_y = 100 + hand_idx * 120
                cv2.putText(annotated_frame, f"{hand_label} Hand:", (10, info_y), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
                cv2.putText(annotated_frame, f"Wrist: {wrist}", (10, info_y+20), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
                cv2.putText(annotated_frame, f"Index: {index_tip}", (10, info_y+40), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
    
    # Display info
    num_hands = len(detection_result.hand_landmarks) if detection_result.hand_landmarks else 0
    cv2.putText(annotated_frame, f"FPS: {avg_fps:.1f} | Hands: {num_hands}", 
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    cv2.putText(annotated_frame, f"Frame: {frame_count} | Debug: {'ON' if show_debug else 'OFF'}", 
                (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    
    cv2.imshow('MediaPipe Hand Tracking', annotated_frame)
    
    key = cv2.waitKey(1) & 0xFF
    
    if key == ord('q'):
        break
    elif key == ord('s'):
        cv2.imwrite(f'hand_tracking_{frame_count}.jpg', annotated_frame)
        print(f"✅ Saved frame {frame_count}")
    elif key == ord('d'):
        show_debug = not show_debug
        print(f"Debug mode: {'ON' if show_debug else 'OFF'}")
    
    frame_count += 1

cap.release()
cv2.destroyAllWindows()

if fps_list:
    print(f"\nAverage FPS: {np.mean(fps_list):.1f}")
    print(f"Total frames processed: {frame_count}")
print("Done!")
