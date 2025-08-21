#!/usr/bin/env python3
"""
Path Optimisation for UR3 Drawing Robot
Author: Amrith David
"""

import numpy as np
from typing import List, Tuple, Dict
import random
import time

class PathOptimiser:
    """Path optimisation using 2-opt and simulated annealing algorithms"""
    
    def __init__(self):
        self.paths = []
        self.distance_matrix = None
        
    def optimise_paths(self, paths: List[List[Tuple]], method: str = 'auto') -> List[List[Tuple]]:
        """
        Optimise path order to minimise total travel distance
        
        Args:
            paths: List of paths (each path is a list of points)
            method: '2opt', 'simulated_annealing', or 'auto'
        
        Returns:
            Optimised list of paths
        """
        self.paths = paths
        
        if len(paths) <= 1:
            return paths
            
        self._build_distance_matrix()
        
        if method == 'auto':
            method = '2opt' if len(paths) <= 25 else 'simulated_annealing'
        
        if method == '2opt':
            return self._2opt_optimisation()
        elif method == 'simulated_annealing':
            return self._simulated_annealing_optimisation()
        else:
            return self._2opt_optimisation()
    
    def _build_distance_matrix(self):
        """Pre-compute distances between all path endpoints"""
        n = len(self.paths)
        self.distance_matrix = np.zeros((n * 2, n * 2))
        
        for i in range(n):
            for j in range(n):
                if i != j:
                    dist_end_to_start = self._point_distance(
                        self.paths[i][-1] if self.paths[i] else (0, 0),
                        self.paths[j][0] if self.paths[j] else (0, 0)
                    )
                    dist_end_to_end = self._point_distance(
                        self.paths[i][-1] if self.paths[i] else (0, 0),
                        self.paths[j][-1] if self.paths[j] else (0, 0)
                    )
                    
                    self.distance_matrix[i*2][j*2] = dist_end_to_start
                    self.distance_matrix[i*2][j*2+1] = dist_end_to_end
    
    def _2opt_optimisation(self) -> List[List[Tuple]]:
        """2-opt algorithm: iteratively swaps pairs of edges to minimise distance"""
        current_order = self._get_greedy_initial_order()
        current_distance = self._calculate_tour_distance(current_order)
        initial_distance = current_distance
        
        improved = True
        iterations = 0
        
        while improved and iterations < 100:
            improved = False
            iterations += 1
            
            for i in range(1, len(current_order) - 2):
                for j in range(i + 1, len(current_order)):
                    if j - i == 1:
                        continue
                        
                    new_order = current_order[:]
                    new_order[i:j] = reversed(new_order[i:j])
                    
                    new_distance = self._calculate_tour_distance(new_order)
                    
                    if new_distance < current_distance:
                        current_order = new_order
                        current_distance = new_distance
                        improved = True
                        break
                        
                if improved:
                    break
        
        return self._apply_order(current_order)
    
    def _simulated_annealing_optimisation(self) -> List[List[Tuple]]:
        """Simulated annealing: probabilistic optimisation for complex drawings"""
        temperature = 100.0
        cooling_rate = 0.995
        min_temperature = 0.1
        
        current_order = list(range(len(self.paths)))
        random.shuffle(current_order)
        current_distance = self._calculate_tour_distance(current_order)
        
        best_order = current_order[:]
        best_distance = current_distance
        
        iterations = 0
        
        while temperature > min_temperature and iterations < 10000:
            iterations += 1
            
            new_order = current_order[:]
            i, j = random.sample(range(len(new_order)), 2)
            new_order[i], new_order[j] = new_order[j], new_order[i]
            
            new_distance = self._calculate_tour_distance(new_order)
            delta = new_distance - current_distance
            
            if delta < 0 or random.random() < np.exp(-delta / temperature):
                current_order = new_order
                current_distance = new_distance
                
                if current_distance < best_distance:
                    best_order = current_order[:]
                    best_distance = current_distance
            
            temperature *= cooling_rate
        
        return self._apply_order(best_order)
    
    def _get_greedy_initial_order(self) -> List[Tuple]:
        """Get initial order using nearest neighbour heuristic"""
        if not self.paths:
            return []
            
        unvisited = list(range(len(self.paths)))
        order = []
        current_pos = (0, 0)
        
        while unvisited:
            min_dist = float('inf')
            next_path_idx = unvisited[0]
            reverse = False
            
            for idx in unvisited:
                path = self.paths[idx]
                if not path:
                    continue
                    
                dist_to_start = self._point_distance(current_pos, path[0])
                if dist_to_start < min_dist:
                    min_dist = dist_to_start
                    next_path_idx = idx
                    reverse = False
                
                dist_to_end = self._point_distance(current_pos, path[-1])
                if dist_to_end < min_dist:
                    min_dist = dist_to_end
                    next_path_idx = idx
                    reverse = True
            
            order.append((next_path_idx, reverse))
            unvisited.remove(next_path_idx)
            
            path = self.paths[next_path_idx]
            if path:
                current_pos = path[-1] if not reverse else path[0]
        
        return order
    
    def _calculate_tour_distance(self, order) -> float:
        """Calculate total distance for a given order"""
        if not order:
            return 0
            
        total = 0
        current_pos = (0, 0)
        
        for item in order:
            if isinstance(item, tuple):
                idx, reverse = item
            else:
                idx = item
                reverse = False
                
            path = self.paths[idx]
            if path:
                start_point = path[-1] if reverse else path[0]
                total += self._point_distance(current_pos, start_point)
                
                for i in range(1, len(path)):
                    if reverse:
                        total += self._point_distance(path[-(i+1)], path[-i])
                    else:
                        total += self._point_distance(path[i-1], path[i])
                
                current_pos = path[0] if reverse else path[-1]
        
        return total
    
    def _apply_order(self, order) -> List[List[Tuple]]:
        """Apply the ordering to create final path list"""
        result = []
        for item in order:
            if isinstance(item, tuple):
                idx, reverse = item
                path = self.paths[idx]
                if reverse and path:
                    result.append(path[::-1])
                else:
                    result.append(path)
            else:
                result.append(self.paths[item])
        return result
    
    def _point_distance(self, p1: Tuple, p2: Tuple) -> float:
        """Calculate Euclidean distance between two points"""
        return np.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)