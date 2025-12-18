import cv2

class LineDetector:
    def __init__(self):
        self.cam = cv2.VideoCapture(0)
    
    def detect_line(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blurred, 50, 150)
        
        lines = cv2.HoughLinesP(edges, 1, np.pi/180, 50, 
                                minLineLength=30, maxLineGap=10)
        
        return lines
    
    def lines_to_waypoints(self, lines, num_waypoints=20):
        if lines is None:
            return []
        
        all_points = []
        for line in lines:
            x1, y1, x2, y2 = line[0]
            for t in np.linspace(0, 1, num_waypoints):
                x = int((1-t)*x1 + t*x2)
                y = int((1-t)*y1 + t*y2)
                all_points.append((x, y))
        
        return all_points

class CameraCalibration:
    def __init__(self):
        self.transform_matrix = None
    
    def calibrate(self, pixel_points, robot_points):
        self.transform_matrix, _ = cv2.findHomography(
            np.array(pixel_points), 
            np.array(robot_points)
        )
    
    def pixel_to_robot(self, pixel_coord):
        px = np.array([pixel_coord[0], pixel_coord[1], 1])
        robot = self.transform_matrix @ px
        return robot[:2] / robot[2]