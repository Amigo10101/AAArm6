#!/usr/bin/env python3
"""
3D Hand Tracking with Position (XYZ) and Orientation (RPY) Calculation
Displays hand center position and orientation in real-time 3D viewer
"""
import cv2
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
import numpy as np
import time
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

# MediaPipe setup
mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles

print("Initializing MediaPipe Hand Landmarker...")

# Create Hand Landmarker with VIDEO mode
base_options = python.BaseOptions(model_asset_path='models/hand_landmarker.task')
options = vision.HandLandmarkerOptions(
    base_options=base_options,
    running_mode=vision.RunningMode.VIDEO,
    num_hands=2,
    min_hand_detection_confidence=0.1,
    min_hand_presence_confidence=0.1,
    min_tracking_confidence=0.01
)
detector = vision.HandLandmarker.create_from_options(options)

print("✅ Hand Landmarker initialized (VIDEO mode + 3D tracking)!")

# Open webcam
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

if not cap.isOpened():
    print("❌ Error: Could not open webcam!")
    exit(1)

print("Webcam opened: 640x480")
print("Press 'q' to quit, 's' to save data, '3' to toggle 3D view")

# 3D calculation functions
def calculate_hand_center(hand_landmarks):
    """Calculate hand center from wrist and palm landmarks"""
    wrist = np.array([hand_landmarks[0].x, hand_landmarks[0].y, hand_landmarks[0].z])
    palm_base = np.array([hand_landmarks[9].x, hand_landmarks[9].y, hand_landmarks[9].z])
    center = (wrist + palm_base) / 2
    return center

def calculate_hand_orientation(hand_landmarks):
    """Calculate hand orientation (RPY) from landmark vectors"""
    # Key landmarks for orientation
    wrist = np.array([hand_landmarks[0].x, hand_landmarks[0].y, hand_landmarks[0].z])
    middle_mcp = np.array([hand_landmarks[9].x, hand_landmarks[9].y, hand_landmarks[9].z])
    index_mcp = np.array([hand_landmarks[5].x, hand_landmarks[5].y, hand_landmarks[5].z])
    
    # Forward vector (wrist to middle finger base)
    forward = middle_mcp - wrist
    forward = forward / (np.linalg.norm(forward) + 1e-6)
    
    # Right vector (perpendicular to forward, towards index)
    to_index = index_mcp - wrist
    right = np.cross(forward, np.array([0, 0, 1]))
    right = right / (np.linalg.norm(right) + 1e-6)
    
    # Up vector (perpendicular to both)
    up = np.cross(right, forward)
    
    # Calculate Roll, Pitch, Yaw from rotation matrix
    # Roll (rotation around forward axis)
    roll = np.arctan2(up[1], up[2])
    
    # Pitch (rotation around right axis)
    pitch = np.arctan2(-forward[2], np.sqrt(forward[0]**2 + forward[1]**2))
    
    # Yaw (rotation around up axis)
    yaw = np.arctan2(forward[1], forward[0])
    
    return np.degrees([roll, pitch, yaw])

# Visualization setup
frame_count = 0
show_3d = False
hand_positions = []
hand_orientations = []

# Create figure for 3D plot (will be shown when needed)
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')
plt.ion()  # Interactive mode

while True:
    ret, frame = cap.read()
    if not ret:
        break
    
    start_time = time.time()
    
    # Convert to RGB
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=frame_rgb)
    
    # Detect hands with timestamp
    timestamp_ms = int(time.time() * 1000)
    detection_result = detector.detect_for_video(mp_image, timestamp_ms)
    
    # Process detected hands
    annotated_frame = frame.copy()
    
    if detection_result.hand_landmarks:
        for hand_idx, hand_landmarks in enumerate(detection_result.hand_landmarks):
            # Get handedness
            handedness = detection_result.handedness[hand_idx][0]
            hand_label = handedness.category_name
            
            # Calculate 3D position and orientation
            center_3d = calculate_hand_center(hand_landmarks)
            rpy = calculate_hand_orientation(hand_landmarks)
            
            # Store for 3D visualization
            if len(hand_positions) > 100:  # Keep last 100 points
                hand_positions.pop(0)
                hand_orientations.pop(0)
            hand_positions.append(center_3d)
            hand_orientations.append(rpy)
            
            # Draw landmarks on frame
            h, w, _ = frame.shape
            landmark_points = []
            for landmark in hand_landmarks:
                x = int(landmark.x * w)
                y = int(landmark.y * h)
                landmark_points.append((x, y))
                cv2.circle(annotated_frame, (x, y), 3, (0, 255, 0), -1)
            
            # Draw hand skeleton
            HAND_CONNECTIONS = mp_hands.HAND_CONNECTIONS
            for connection in HAND_CONNECTIONS:
                start_idx, end_idx = connection[0], connection[1]
                cv2.line(annotated_frame, landmark_points[start_idx], 
                        landmark_points[end_idx], (0, 255, 0), 2)
            
            # Calculate 2D center for display
            center_2d = (int(center_3d[0] * w), int(center_3d[1] * h))
            
            # Draw center point
            cv2.circle(annotated_frame, center_2d, 10, (255, 0, 0), -1)
            cv2.circle(annotated_frame, center_2d, 15, (255, 255, 0), 2)
            
            # Display 3D position and orientation
            info_y = 100 + hand_idx * 150
            cv2.putText(annotated_frame, f"{hand_label} Hand 3D Data:", 
                       (10, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            
            # XYZ Position (normalized 0-1)
            cv2.putText(annotated_frame, f"X: {center_3d[0]:.3f}  Y: {center_3d[1]:.3f}  Z: {center_3d[2]:.3f}", 
                       (10, info_y+25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
            # RPY Orientation (degrees)
            cv2.putText(annotated_frame, f"Roll: {rpy[0]:.1f}°  Pitch: {rpy[1]:.1f}°  Yaw: {rpy[2]:.1f}°", 
                       (10, info_y+50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    
    # Calculate FPS
    fps = 1.0 / (time.time() - start_time)
    
    # Display FPS and instructions
    cv2.putText(annotated_frame, f"FPS: {fps:.1f} | Press '3' for 3D view, 'q' to quit", 
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    
    cv2.imshow('Hand Tracking - 3D Position & Orientation', annotated_frame)
    
    # Update 3D plot if enabled
    if show_3d and len(hand_positions) > 0:
        ax.clear()
        
        # Plot hand positions
        positions = np.array(hand_positions)
        ax.scatter(positions[:, 0], positions[:, 1], positions[:, 2], 
                  c='blue', marker='o', s=50, alpha=0.6)
        
        # Plot current position larger
        if len(positions) > 0:
            current = positions[-1]
            ax.scatter(current[0], current[1], current[2], 
                      c='red', marker='o', s=200, alpha=1.0)
            
            # Draw orientation axes
            if len(hand_orientations) > 0:
                rpy = hand_orientations[-1]
                # Simplified orientation visualization
                ax.text(current[0], current[1], current[2] + 0.1, 
                       f"R:{rpy[0]:.0f}° P:{rpy[1]:.0f}° Y:{rpy[2]:.0f}°",
                       fontsize=10)
        
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z (Depth)')
        ax.set_title('Hand 3D Position Tracking')
        
        # Set consistent limits
        ax.set_xlim([0, 1])
        ax.set_ylim([0, 1])
        ax.set_zlim([-0.5, 0.5])
        
        plt.draw()
        plt.pause(0.001)
    
    # Handle key presses
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break
    elif key == ord('3'):
        show_3d = not show_3d
        if show_3d:
            plt.show()
            print("3D view enabled")
        else:
            plt.close()
            print("3D view disabled")
    elif key == ord('s') and len(hand_positions) > 0:
        # Save current data
        print(f"\nSaved hand data at frame {frame_count}:")
        print(f"  Position (XYZ): {hand_positions[-1]}")
        print(f"  Orientation (RPY): {hand_orientations[-1]}")
    
    frame_count += 1

cap.release()
cv2.destroyAllWindows()
plt.close('all')
print("Done!")
