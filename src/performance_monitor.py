"""
Performance monitoring for volumetric detection pipeline
"""

import numpy as np
from collections import deque
from typing import Dict, List
from dataclasses import dataclass

@dataclass
class PerformanceMetrics:
    """Performance tracking for academic evaluation"""
    processing_time: float
    memory_usage_mb: float
    grid_updates_per_second: float
    triangulation_accuracy: float
    false_positive_rate: float
    detection_latency: float
    computational_complexity: Dict[str, float]

class PerformanceMonitor:
    """Performance monitoring for academic evaluation"""
    
    def __init__(self):
        self.frame_metrics = []
        self.processing_times = deque(maxlen=100)
        self.memory_usage = deque(maxlen=100)
        
    def record_frame(self, processing_time: float, num_targets: int, 
                    num_observations: int, grid):
        """Record performance metrics for single frame"""
        grid_memory = (grid.occupancy_probs.nbytes + 
                      grid.confidence_scores.nbytes + 
                      grid.last_update_time.nbytes) / (1024 * 1024)
        
        metrics = {
            'processing_time': processing_time,
            'num_targets': num_targets,
            'num_observations': num_observations,
            'memory_usage_mb': grid_memory,
            'grid_updates_per_second': 1.0 / processing_time if processing_time > 0 else 0,
            'targets_per_second': num_targets / processing_time if processing_time > 0 else 0
        }
        
        self.frame_metrics.append(metrics)
        self.processing_times.append(processing_time)
        self.memory_usage.append(grid_memory)
        
        if len(self.frame_metrics) > 1000:
            self.frame_metrics = self.frame_metrics[-1000:]
    
    def get_summary(self) -> Dict:
        """Get performance summary statistics"""
        if not self.frame_metrics:
            return {}
        
        recent_metrics = self.frame_metrics[-50:]
        
        summary = {
            'avg_processing_time': np.mean([m['processing_time'] for m in recent_metrics]),
            'max_processing_time': np.max([m['processing_time'] for m in recent_metrics]),
            'avg_memory_usage_mb': np.mean([m['memory_usage_mb'] for m in recent_metrics]),
            'avg_targets_detected': np.mean([m['num_targets'] for m in recent_metrics]),
            'processing_rate_hz': 1.0 / np.mean([m['processing_time'] for m in recent_metrics]),
            'computational_efficiency': self._calculate_efficiency(recent_metrics)
        }
        
        return summary
    
    def _calculate_efficiency(self, metrics: List[Dict]) -> float:
        """Calculate computational efficiency metric"""
        total_targets = sum(m['num_targets'] for m in metrics)
        total_time = sum(m['processing_time'] for m in metrics)
        
        if total_time > 0:
            return total_targets / total_time
        return 0.0