#!/usr/bin/env python3
"""
Path Planner module for the Selfie-Drawing Robot
Author: Amrith David
"""
import cv2
import numpy as np
from typing import List, Tuple, Dict

class PathPlanner:
    """Tool path planning class for converting edge images to robot paths"""
    
    def __init__(self):
        """Initialise the path planner"""
        self.paths = []
        self.original_image = None
    
    def extract_paths_from_image(self, image):
        """Extract vector paths from an edge image"""
        self.original_image = image.copy()
        
        # Convert to binary image
        _, binary_image = cv2.threshold(image, 127, 255, cv2.THRESH_BINARY)
        
        print(f"Image shape: {binary_image.shape}")
        print(f"Image min/max values: {np.min(binary_image)}/{np.max(binary_image)}")
        
        # Find contours in the image
        contours, _ = cv2.findContours(binary_image, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        
        print(f"Number of raw contours found: {len(contours)}")
        
        self.paths = []
        
        for contour in contours:
            # Skip small contours
            if len(contour) < 10:
                continue
                
            # Simplify contour using Douglas-Peucker algorithm
            epsilon = 0.002 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)
            
            if len(approx) < 3:
               continue
                
            points = []
            for point in approx:
                x = point[0][0]
                y = point[0][1]
                points.append((x, y))
            
            self.paths.append(points)
        
        print(f"Number of filtered paths: {len(self.paths)}")
        
        return self.paths
    
    def transform_coordinates_with_corners(self, canvas_corners):
        """Transform image coordinates to robot workspace coordinates using canvas corners"""
        if not self.paths or not canvas_corners:
            print("No paths or canvas corners to transform")
            return self.paths
            
        print(f"Transform: Canvas corners received with {len(canvas_corners)} points")
        
        if len(canvas_corners) < 4:
            print("Error: Need 4 canvas corners for transformation")
            return self.paths
            
        # Extract corner positions
        bl = canvas_corners[0]
        br = canvas_corners[1]
        tr = canvas_corners[2]
        tl = canvas_corners[3]
        
        # Margin adjustments (converting from mm to m)
        x_margin_m = 73.75 * 1.05 / 1000.0
        y_margin_m = 52.5 * 1.05 / 1000.0
        
        # Calculate edge vectors
        bottom_vector = [br['x'] - bl['x'], br['y'] - bl['y']]
        left_vector = [tl['x'] - bl['x'], tl['y'] - bl['y']]
        
        # Calculate unit vectors
        bottom_length = (bottom_vector[0]**2 + bottom_vector[1]**2)**0.5
        left_length = (left_vector[0]**2 + left_vector[1]**2)**0.5
        
        bottom_unit = [bottom_vector[0]/bottom_length, bottom_vector[1]/bottom_length]
        left_unit = [left_vector[0]/left_length, left_vector[1]/left_length]
        
        # Calculate adjusted corner positions with margins
        bl_adjusted = {
            'x': bl['x'] + x_margin_m * bottom_unit[0] + y_margin_m * left_unit[0],
            'y': bl['y'] + x_margin_m * bottom_unit[1] + y_margin_m * left_unit[1],
            'z': bl['z']
        }
        
        br_adjusted = {
            'x': br['x'] - x_margin_m * bottom_unit[0] + y_margin_m * left_unit[0],
            'y': br['y'] - x_margin_m * bottom_unit[1] + y_margin_m * left_unit[1],
            'z': br['z']
        }
        
        tr_adjusted = {
            'x': tr['x'] - x_margin_m * bottom_unit[0] - y_margin_m * left_unit[0],
            'y': tr['y'] - x_margin_m * bottom_unit[1] - y_margin_m * left_unit[1],
            'z': tr['z']
        }
        
        tl_adjusted = {
            'x': tl['x'] + x_margin_m * bottom_unit[0] - y_margin_m * left_unit[0],
            'y': tl['y'] + x_margin_m * bottom_unit[1] - y_margin_m * left_unit[1],
            'z': tl['z']
        }
        
        print(f"Adjusted canvas corners with margins: x={x_margin_m*1000}mm, y={y_margin_m*1000}mm")
        
        if not self.paths:
            return self.paths
        
        # Find bounding box of image paths
        min_x = float('inf')
        max_x = float('-inf')
        min_y = float('inf')
        max_y = float('-inf')

        for path in self.paths:
            for point in path:
                x, y = point[0], point[1]
                if x < min_x:
                    min_x = x
                if x > max_x:
                    max_x = x
                if y < min_y:
                    min_y = y
                if y > max_y:
                    max_y = y
                    
        img_width = max_x - min_x
        img_height = max_y - min_y
        
        if img_width <= 0 or img_height <= 0:
            print("Error: Invalid image dimensions")
            return self.paths
            
        print(f"Image bounds: X: {min_x} to {max_x}, Y: {min_y} to {max_y}")
        
        # Transform each path
        transformed_paths = []
        for path in self.paths:
            transformed_path = []
            for point in path:
                # Normalise coordinates to [0,1] range
                norm_x = (point[0] - min_x) / img_width
                norm_y = (point[1] - min_y) / img_height
                
                # Invert Y coordinate
                norm_y = 1.0 - norm_y
                
                # Bilinear interpolation to map to robot coordinates
                robot_x = (1 - norm_x) * (1 - norm_y) * bl_adjusted['x'] + \
                          norm_x * (1 - norm_y) * br_adjusted['x'] + \
                          (1 - norm_x) * norm_y * tl_adjusted['x'] + \
                          norm_x * norm_y * tr_adjusted['x']
                          
                robot_y = (1 - norm_x) * (1 - norm_y) * bl_adjusted['y'] + \
                          norm_x * (1 - norm_y) * br_adjusted['y'] + \
                          (1 - norm_x) * norm_y * tl_adjusted['y'] + \
                          norm_x * norm_y * tr_adjusted['y']
                
                robot_z = (bl_adjusted['z'] + br_adjusted['z'] + tr_adjusted['z'] + tl_adjusted['z']) / 4.0
                
                transformed_path.append((robot_x, robot_y))
            
            transformed_paths.append(transformed_path)
        
        self.paths = transformed_paths
        
        print(f"Transformation complete: {len(self.paths)} paths transformed")
        return self.paths
    
    def optimise_paths(self):
        """Optimise paths to minimise travel distance and drawing time"""
        if not self.paths:
            print("No paths to optimise")
            return self.paths
            
        print(f"Optimising {len(self.paths)} paths")
        
        # Import the optimiser
        from .path_optimiser import PathOptimiser
        optimiser = PathOptimiser()
        
        # Store original for metrics
        original_length = self._calculate_total_path_length(self.paths)
        
        # Optimise using best algorithm for complexity
        self.paths = optimiser.optimise_paths(self.paths)
        
        # Calculate improvement
        optimised_length = self._calculate_total_path_length(self.paths)
        reduction_percent = (original_length - optimised_length) / original_length * 100
        
        print(f"Original path length: {original_length:.3f} units")
        print(f"Optimised path length: {optimised_length:.3f} units")
        print(f"Optimisation reduced path length by {reduction_percent:.1f}%")
        
        return self.paths

    def _calculate_total_path_length(self, paths):
        """Calculate the total length of all paths including travel between paths"""
        if not paths:
            return 0
        
        total_length = 0
        current_point = (0, 0)
        
        for path in paths:
            if not path:
                continue
            
            # Add travel distance to start of path
            total_length += self._point_distance(current_point, path[0])
            
            # Add length of the path itself
            for i in range(1, len(path)):
                total_length += self._point_distance(path[i-1], path[i])
            
            current_point = path[-1]
        
        return total_length

    def _point_distance(self, p1, p2):
        """Calculate Euclidean distance between two points"""
        return ((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)**0.5