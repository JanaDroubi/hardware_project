import depthai as dai
import cv2
import numpy as np
import serial
import time

# --- DepthAI pipeline ---
pipeline = dai.Pipeline()
cam = pipeline.createColorCamera()
cam.setPreviewSize(640, 480)
cam.setBoardSocket(dai.CameraBoardSocket.RGB)
cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)

xout = pipeline.createXLinkOut()
xout.setStreamName("color")
cam.preview.link(xout.input)

# --- Surface size ---
surface_size_cm = 35.0  # 35cm x 35cm
surface_size_mm = surface_size_cm * 10  # convert to mm for Teensy

# --- Open serial to Teensy ---
ser = serial.Serial('COM18', 115200)  # adjust COM port
time.sleep(2)  # wait for Teensy reset

# --- Helper function to map pixel to mm ---
def map_pixel_to_mm(px, py, surface_rect):
    x_s, y_s, w_s, h_s = surface_rect
    # Map pixels to mm (top-left = -size/2, bottom-right = size/2)
    rel_x_mm = (px - x_s) * surface_size_mm / w_s - surface_size_mm / 2
    rel_y_mm = surface_size_mm / 2 - (py - y_s) * surface_size_mm / h_s  # invert y-axis
    return rel_x_mm, rel_y_mm

with dai.Device(pipeline) as device:
    q = device.getOutputQueue("color", maxSize=1, blocking=False)
    print("Detecting red surface and orange ball... Press 'q' to quit")

    while True:
        try:
            frame = q.get().getCvFrame()
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            # --- Detect red surface ---
            lower_red1 = np.array([0, 150, 150])
            upper_red1 = np.array([10, 255, 255])
            lower_red2 = np.array([160, 150, 150])
            upper_red2 = np.array([180, 255, 255])
            mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
            mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
            mask_surface = cv2.bitwise_or(mask_red1, mask_red2)

            contours, _ = cv2.findContours(mask_surface, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if contours:
                c = max(contours, key=cv2.contourArea)
                x_s, y_s, w_s, h_s = cv2.boundingRect(c)
                surface_mask = np.zeros_like(mask_surface)
                cv2.drawContours(surface_mask, [c], -1, 255, -1)
                surface_rect = (x_s, y_s, w_s, h_s)

                # --- Draw thin square around surface ---
                cv2.rectangle(frame, (x_s, y_s), (x_s + w_s, y_s + h_s), (0, 255, 0), 1)

                # --- Draw center axes (0,0) ---
                center_x = x_s + w_s // 2
                center_y = y_s + h_s // 2
                cv2.line(frame, (center_x, y_s), (center_x, y_s + h_s), (255, 0, 0), 1)  # vertical
                cv2.line(frame, (x_s, center_y), (x_s + w_s, center_y), (255, 0, 0), 1)  # horizontal
                cv2.putText(frame, "(0,0)", (center_x + 5, center_y - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)

            else:
                surface_mask = np.zeros_like(mask_surface)
                continue  # no surface detected

            # --- Detect orange ball inside surface ---
            lower_orange = np.array([5, 150, 150])
            upper_orange = np.array([20, 255, 255])
            mask_ball = cv2.inRange(hsv, lower_orange, upper_orange)
            mask_ball = cv2.bitwise_and(mask_ball, mask_ball, mask=surface_mask)

            contours_ball, _ = cv2.findContours(mask_ball, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if contours_ball:
                c = max(contours_ball, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                if radius > 5:
                    # Draw ball
                    cv2.circle(frame, (int(x), int(y)), int(radius), (0, 165, 255), -1)

                    # Convert to mm relative to surface center
                    x_mm, y_mm = map_pixel_to_mm(x, y, surface_rect)

                    # Send to Teensy
                    ser.write(f"{x_mm:.2f},{y_mm:.2f}\n".encode())

                    # Show coordinates on frame
                    cv2.putText(frame, f"({x_mm:.1f},{y_mm:.1f})mm",
                                (int(x)+5, int(y)-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,165,255), 1)

            cv2.imshow("Ball on Surface", frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        except Exception as e:
            print("Error:", e)
            break

cv2.destroyAllWindows()
ser.close()
print("Program terminated.")
