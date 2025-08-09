"""
Debug version of space carving to identify why no targets are detected
"""

def _parallel_space_carving_debug(self, observations, timestamp, max_workers):
    """Debug version with detailed logging"""
    
    # Step 1: Get detection positions to focus search area
    detection_positions = []
    for obs in observations:
        for obj in obs.detected_objects:
            detection_positions.append(obj['world_position'])
    
    if not detection_positions:
        print("    DEBUG: No detection positions found")
        return []
    
    detection_positions = np.array(detection_positions)
    print(f"    DEBUG: Found {len(detection_positions)} detection positions")
    print(f"    DEBUG: Detection positions range: X[{detection_positions[:, 0].min():.1f}, {detection_positions[:, 0].max():.1f}]")
    print(f"    DEBUG:                           Y[{detection_positions[:, 1].min():.1f}, {detection_positions[:, 1].max():.1f}]")
    print(f"    DEBUG:                           Z[{detection_positions[:, 2].min():.1f}, {detection_positions[:, 2].max():.1f}]")
    
    # Step 2: Create focused voxel grid around detections only
    search_radius = self.voxel_resolution * 8  # Increased search radius
    focused_coords = self._get_focused_voxel_coordinates(detection_positions, search_radius)
    
    print(f"    DEBUG: Search radius: {search_radius:.1f}m, Voxel resolution: {self.voxel_resolution:.1f}m")
    print(f"    DEBUG: Focused search: {len(focused_coords):,} voxels")
    
    if len(focused_coords) == 0:
        print("    DEBUG: No focused coordinates generated")
        return []
    
    # Test a few sample voxels
    sample_results = []
    max_samples = min(100, len(focused_coords))
    sample_indices = np.linspace(0, len(focused_coords)-1, max_samples, dtype=int)
    
    for i in sample_indices[:10]:  # Test first 10 samples
        voxel_coord = focused_coords[i]
        
        # Check distance to detections
        distances = np.linalg.norm(detection_positions - voxel_coord, axis=1)
        min_dist = np.min(distances)
        
        # Check consistency
        is_consistent = self._is_voxel_consistent_fast_debug(voxel_coord, observations, detection_positions)
        
        # Calculate confidence
        confidence = self._calculate_voxel_confidence_fast(voxel_coord, observations)
        
        sample_results.append({
            'voxel': voxel_coord,
            'min_dist': min_dist,
            'is_consistent': is_consistent,
            'confidence': confidence,
            'passes_dist_check': min_dist < self.voxel_resolution * 4
        })
    
    print(f"    DEBUG: Sample voxel analysis:")
    for i, result in enumerate(sample_results):
        print(f"      Voxel {i}: dist={result['min_dist']:.1f}, consistent={result['is_consistent']}, conf={result['confidence']:.3f}, passes_dist={result['passes_dist_check']}")
    
    # Count how many voxels pass each filter
    passing_distance = 0
    passing_consistency = 0
    passing_confidence = 0
    
    for voxel_coord in focused_coords[:1000]:  # Check first 1000 to avoid slowdown
        distances = np.linalg.norm(detection_positions - voxel_coord, axis=1)
        min_dist = np.min(distances)
        
        if min_dist < self.voxel_resolution * 4:
            passing_distance += 1
            
            if self._is_voxel_consistent_fast_debug(voxel_coord, observations, detection_positions):
                passing_consistency += 1
                
                confidence = self._calculate_voxel_confidence_fast(voxel_coord, observations)
                if confidence >= 0.2:
                    passing_confidence += 1
    
    print(f"    DEBUG: Filter analysis (first 1000 voxels):")
    print(f"      Passing distance filter: {passing_distance}/1000")
    print(f"      Passing consistency filter: {passing_consistency}/1000") 
    print(f"      Passing confidence filter: {passing_confidence}/1000")
    
    # Original processing with reduced sample for speed
    chunk_size = max(1, len(focused_coords) // max_workers)
    voxel_chunks = [focused_coords[i:i + chunk_size] 
                   for i in range(0, len(focused_coords), chunk_size)]
    
    def process_voxel_chunk_debug(chunk):
        """Process a chunk of voxels with debug info"""
        chunk_results = []
        chunk_stats = {
            'total': len(chunk),
            'distance_pass': 0,
            'consistency_pass': 0,
            'confidence_pass': 0
        }
        
        for voxel_coord in chunk:
            distances = np.linalg.norm(detection_positions - voxel_coord, axis=1)
            min_dist = np.min(distances)
            
            if min_dist < self.voxel_resolution * 4:
                chunk_stats['distance_pass'] += 1
                
                if self._is_voxel_consistent_fast_debug(voxel_coord, observations, detection_positions):
                    chunk_stats['consistency_pass'] += 1
                    confidence = self._calculate_voxel_confidence_fast(voxel_coord, observations)
                    
                    if confidence >= 0.2:
                        chunk_stats['confidence_pass'] += 1
                        chunk_results.append({
                            'position': voxel_coord,
                            'confidence': confidence,
                            'method': 'space_carving'
                        })
        
        return chunk_results, chunk_stats
    
    # Process first chunk only for debugging
    if voxel_chunks:
        results, stats = process_voxel_chunk_debug(voxel_chunks[0])
        print(f"    DEBUG: First chunk stats: {stats}")
        print(f"    DEBUG: First chunk results: {len(results)} voxels passed all filters")
        
        if len(results) > 0:
            print(f"    DEBUG: Sample result confidence values: {[r['confidence'] for r in results[:5]]}")
    
    # For now, return empty to avoid full processing
    return []

def _is_voxel_consistent_fast_debug(self, voxel_coord, observations, detection_positions):
    """Debug version of consistency check"""
    # Pre-filter: must be close to at least one detection
    distances_to_detections = np.linalg.norm(detection_positions - voxel_coord, axis=1)
    min_detection_dist = np.min(distances_to_detections)
    
    if min_detection_dist > self.voxel_resolution * 5:
        return False
    
    # More lenient consistency check
    consistent_count = 0
    observation_details = []
    
    for obs in observations:
        if len(obs.detected_objects) > 0:
            obs_positions = np.array([det['world_position'] for det in obs.detected_objects])
            distances = np.linalg.norm(obs_positions - voxel_coord, axis=1)
            min_obs_dist = np.min(distances)
            
            is_close = min_obs_dist < self.voxel_resolution * 4
            observation_details.append({
                'camera': obs.camera_id,
                'num_detections': len(obs.detected_objects),
                'min_distance': min_obs_dist,
                'is_close': is_close
            })
            
            if is_close:
                consistent_count += 1
    
    # Debug print for first few voxels
    if len(observation_details) > 0 and min_detection_dist < self.voxel_resolution * 2:
        pass  # Could add detailed logging here if needed
    
    return consistent_count >= 1

# Add these debug methods to your VolumetricDetectionPipeline class
def add_debug_methods_to_pipeline(pipeline):
    """Add debug methods to existing pipeline instance"""
    import types
    pipeline._parallel_space_carving_debug = types.MethodType(_parallel_space_carving_debug, pipeline)
    pipeline._is_voxel_consistent_fast_debug = types.MethodType(_is_voxel_consistent_fast_debug, pipeline)