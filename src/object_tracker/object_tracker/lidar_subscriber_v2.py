import rclpy
import numpy as np
import threading
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from smrr_interfaces.msg import Entities
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from .include.transform import GeometricTransformations

# ====================== TUNING PARAMETERS ======================
# Detection parameters
MAX_DETECTION_RANGE = 4.5           # meters (max range to consider objects)
HUMAN_RADIUS = 0.75                 # meters (radius around humans to exclude from static obstacles)

# Tracking thresholds
NEW_OBJ_DISTANCE_THRESHOLD = 1.0   # meters (max distance to match visual to lidar detection)
TRACKING_THRESHOLD = 0.75           # meters (radius to associate lidar points with tracks)

# Confidence parameters
MIN_CONFIDENCE_FRAMES = 3           # frames needed to confirm a track
MAX_MISSED_FRAMES = 5               # frames before removing a lost track
CONFIDENCE_DECAY = 0.9              # factor for confidence decay per frame (0.8-0.95)

# Stationary object detection
MAX_NO_VISUAL_FRAMES = 5            # frames without visual update before checking stationarity
MIN_MOVEMENT_THRESHOLD = 0.1        # meters (movement below this is considered stationary)
MIN_MOVEMENT_FRAMES = 20             # frames needed to assess movement
POSITION_HISTORY_LENGTH = 5         # how many past positions to store
# ===============================================================

class PersonTracker:
    def __init__(self):
        self.position = np.zeros(2, dtype=np.float64)
        self.previous_positions = []  # Track position history
        self.class_id = ""
        self.confidence = 1.0         # Start with 1.0 confidence
        self.missed_detections = 0
        self.last_seen = 0
        self.last_visual_update = 0

class LidarSubscriber(Node):
    def __init__(self):
        super().__init__("lidar_subscriber")
        self.cb_group = ReentrantCallbackGroup()
        
        # Subscribers
        self.laser_sub = self.create_subscription(
            LaserScan, "/scan", self.lidar_callback, 10, 
            callback_group=self.cb_group)
        
        # Publishers
        self.laser_pub = self.create_publisher(
            Entities, "/object_tracker/laser_data_array_", 10)
        self.stat_obs_pub = self.create_publisher(
            Entities, "/object_tracker/static_data_array", 10)
        
        self.transform = GeometricTransformations(self)
        self.tracked_persons = []
        self.frame_count = 0
        self.visual_data = None
        self.visual_data_lock = threading.Lock()
        
        self.get_logger().info("Lidar subscriber node has been started")

    def process_lidar_data(self, msg):
        """Convert LaserScan to Cartesian coordinates with filtering"""
        ranges = np.array(msg.ranges, dtype=np.float64)
        valid_mask = (ranges <= MAX_DETECTION_RANGE) & \
                     (ranges >= msg.range_min) & \
                     np.isfinite(ranges)
        ranges = ranges[valid_mask]
        
        angles = np.linspace(
            msg.angle_min, 
            msg.angle_min + len(msg.ranges) * msg.angle_increment,
            len(msg.ranges),
            dtype=np.float64
        )[valid_mask]
        
        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)
        return np.column_stack((x, y))

    def update_visual_data(self, msg):
        with self.visual_data_lock:
            self.visual_data = msg

    def get_visual_data(self):
        with self.visual_data_lock:
            return self.visual_data if self.visual_data else Entities()

    def update_tracks(self, lidar_points, visual_data):
        self.frame_count += 1
        current_time = self.frame_count
        
        # Process visual data
        try:
            visual_points = np.column_stack((
                np.array(visual_data.x, dtype=np.float64),
                np.array(visual_data.y, dtype=np.float64),
                np.array(visual_data.classes)
            )) if len(visual_data.x) > 0 else np.empty((0, 3), dtype=np.float64)
        except (ValueError, TypeError):
            visual_points = np.empty((0, 3), dtype=np.float64)
        
        # Apply confidence decay
        for track in self.tracked_persons:
            track.confidence *= CONFIDENCE_DECAY
        
        # Update existing tracks with lidar data
        for track in self.tracked_persons:
            distances = np.linalg.norm(lidar_points - track.position, axis=1)
            nearby_points = lidar_points[distances <= TRACKING_THRESHOLD]
            
            if len(nearby_points) > 0:
                new_position = np.median(nearby_points, axis=0)
                track.previous_positions = [track.position] + track.previous_positions[:POSITION_HISTORY_LENGTH-1]
                track.position = new_position
                track.confidence = min(track.confidence + 1, 10)
                track.missed_detections = 0
                track.last_seen = current_time
        
        # Match visual detections
        for visual_point in visual_points:
            try:
                v_pos = np.array(visual_point[:2], dtype=np.float64)
                v_class = str(visual_point[2])
                
                if np.any(~np.isfinite(v_pos)) or np.linalg.norm(v_pos) > MAX_DETECTION_RANGE:
                    continue
                    
                # Find closest track
                min_dist = float('inf')
                closest_track = None
                for track in self.tracked_persons:
                    dist = np.linalg.norm(track.position - v_pos)
                    if dist < min_dist and dist <= NEW_OBJ_DISTANCE_THRESHOLD:
                        min_dist = dist
                        closest_track = track
                        
                if closest_track:
                    closest_track.class_id = v_class
                    closest_track.confidence = min(closest_track.confidence + 1, 10)
                    closest_track.missed_detections = 0
                    closest_track.last_seen = current_time
                    closest_track.last_visual_update = current_time
                    closest_track.previous_positions = [closest_track.position] + closest_track.previous_positions[:POSITION_HISTORY_LENGTH-1]
                else:
                    new_track = PersonTracker()
                    new_track.position = v_pos
                    new_track.class_id = v_class
                    new_track.last_seen = current_time
                    new_track.last_visual_update = current_time
                    self.tracked_persons.append(new_track)
            except Exception as e:
                self.get_logger().warning(f"Visual point error: {str(e)}", throttle_duration_sec=1)
                
        # Remove stale or stationary tracks
        self.tracked_persons = [t for t in self.tracked_persons if not self.should_remove_track(t, current_time)]
        
        # Identify static obstacles
        if len(self.tracked_persons) > 0:
            person_positions = np.array([t.position for t in self.tracked_persons], dtype=np.float64)
            distances = np.linalg.norm(lidar_points[:, np.newaxis] - person_positions, axis=2)
            static_obstacles = lidar_points[np.all(distances > HUMAN_RADIUS, axis=1)]
        else:
            static_obstacles = lidar_points
            
        return static_obstacles

    def should_remove_track(self, track, current_time):
        # Basic removal conditions
        if (current_time - track.last_seen > MAX_MISSED_FRAMES) or \
           (track.confidence < 0.1):
            return True
            
        # Stationary object removal
        if (current_time - track.last_visual_update > MAX_NO_VISUAL_FRAMES) and \
           self.is_stationary(track):
            return True
            
        return False

    def is_stationary(self, track):
        if len(track.previous_positions) < MIN_MOVEMENT_FRAMES:
            return False
            
        max_movement = max(
            np.linalg.norm(track.position - pos)
            for pos in track.previous_positions[:MIN_MOVEMENT_FRAMES]
        )
        return max_movement < MIN_MOVEMENT_THRESHOLD

    def lidar_callback(self, msg):
        try:
            lidar_points = self.process_lidar_data(msg)
            visual_data = self.get_visual_data()
            static_obstacles = self.update_tracks(lidar_points, visual_data)
            
            # Prepare and publish results
            person_data = []
            for track in self.tracked_persons:
                if track.confidence >= MIN_CONFIDENCE_FRAMES:
                    person_data.append([track.position[0], track.position[1], track.class_id])
            
            self.publish_results(person_data, static_obstacles)
        except Exception as e:
            self.get_logger().error(f"Lidar callback error: {str(e)}", throttle_duration_sec=1)

    def publish_results(self, person_data, static_data):
        # Publish person tracks
        person_msg = Entities()
        if len(person_data) > 0:
            person_data = np.array(person_data, dtype=object)
            person_msg.count = len(person_data)
            person_msg.classes = person_data[:, 2].tolist()
            person_msg.x = person_data[:, 0].astype(float).tolist()
            person_msg.y = person_data[:, 1].astype(float).tolist()
        self.laser_pub.publish(person_msg)
        
        # Publish static obstacles
        static_msg = Entities()
        if len(static_data) > 0:
            static_msg.x = static_data[:, 0].astype(float).tolist()
            static_msg.y = static_data[:, 1].astype(float).tolist()
        self.stat_obs_pub.publish(static_msg)

class VisualDataSubscriber(Node):
    def __init__(self, lidar_node):
        super().__init__("visual_data_subscriber")
        self.cb_group = ReentrantCallbackGroup()
        self.lidar_node = lidar_node
        self.visual_sub = self.create_subscription(
            Entities, "/object_tracker/visual_dynamic_obs_array", 
            self.visual_callback, 10, callback_group=self.cb_group)

    def visual_callback(self, msg):
        self.lidar_node.update_visual_data(msg)

def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    
    lidar_sub = LidarSubscriber()
    visual_sub = VisualDataSubscriber(lidar_sub)
    
    executor.add_node(lidar_sub)
    executor.add_node(visual_sub)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        lidar_sub.destroy_node()
        visual_sub.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()