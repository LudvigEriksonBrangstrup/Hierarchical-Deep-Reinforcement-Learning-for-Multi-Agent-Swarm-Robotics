import random
import math
import numpy as np
from pygame.locals import *
from score import Score

from .shape import Shape



# Constants
BOX_WIDTH = 800
BOX_HEIGHT = 600
SQUARE_SIZE = 50
TIME_STEP = 0.1
VELOCITY = 0.1
RESTITUTION = 0.2
WALL_RESTITUTION = 1 #0.9
DAMPING_FACTOR = 1 # This adds drag to the simulation




class Square(Shape):
    
    def __init__(self, id = None , pos = None , is_draggable=False, color=None, tgt_circ = None, tgt_circ_tgt_pos = None):
        pos = [random.uniform(BOX_WIDTH / 4, 3 * BOX_WIDTH / 4), random.uniform(BOX_HEIGHT / 4, 3 * BOX_HEIGHT / 4)] if pos is None else pos
        #pos = [random.uniform(SQUARE_SIZE, BOX_WIDTH - SQUARE_SIZE), random.uniform(SQUARE_SIZE, BOX_HEIGHT - SQUARE_SIZE)] if pos is None else pos
        velocity = [random.uniform(-200 * VELOCITY, 200 * VELOCITY), random.uniform(-200 * VELOCITY, 200 * VELOCITY)] #[0, 0]
        mass = 1
        moment_of_inertia = (mass * (SQUARE_SIZE ** 2)) / 6
        super().__init__(id,pos, velocity, mass, moment_of_inertia , is_draggable = is_draggable, color = color)
        self.size = SQUARE_SIZE
        self.angle = random.uniform(0, 360) 
        self.angular_velocity = 0  # degrees per second
        self.box_width = BOX_WIDTH
        self.box_height = BOX_HEIGHT
        if self.is_draggable:
            self.velocity = [0, 0]
            self.score = Score()
            self.tgt_pos = None
            self.tgt_circ = tgt_circ
            if tgt_circ: 
                if tgt_circ.pos:
                    while True:
                        tgt_circ_tgt_pos = [
                            random.randint(int(BOX_WIDTH * 1/4), int(BOX_WIDTH * 3/4)),
                            random.randint(int(BOX_HEIGHT * 1/4), int(BOX_HEIGHT * 3/4))
                        ]
                        distance = np.linalg.norm(np.subtract(tgt_circ_tgt_pos, tgt_circ.pos))
                        if distance >= 200:
                            break
            if tgt_circ_tgt_pos:
                self.tgt_circ_tgt_pos = tgt_circ_tgt_pos
                


            self.tgt_circ_tgt_pos = tgt_circ_tgt_pos

            self.approved_ids = {tgt_circ.id if tgt_circ else None} 

            self.left_wheel_speed = 0
            self.right_wheel_speed = 0

            self._rel_circ_pos = None
            self._rel_tgt_pos_to_circ = None
            self._rel_pos_nearest_bndry = None
            self._init_rel_circ_pos = None
            self._init_rel_tgt_pos_to_circ = None
            self._init_dist_to_circ = None
            self._init_tgt_dist_to_circ = None
            self.previous_pos = None
            self.previous_angle = None
            self.data_velocity = None
            self.data_angular_velocity = None


            self.calculate_initial_pos()

        else:
            self.score = None


    
    def update(self, steer_with_keys):
        if not self.is_draggable:
            # Move pos based on velocity
            self.pos[0] += self.velocity[0] * TIME_STEP
            self.pos[1] += self.velocity[1] * TIME_STEP
            # Angle update and damping
            self.angle += self.angular_velocity * TIME_STEP
            self.velocity[0] *= DAMPING_FACTOR
            self.velocity[1] *= DAMPING_FACTOR
            self.angular_velocity *= DAMPING_FACTOR
            self.check_wall_collision()
        else:
            self.velocity = [0, 0]
            self.score.update_score_based_on_pos_and_movement(self.pos , left_wheel_speed = self.left_wheel_speed, right_wheel_speed = self.right_wheel_speed)
            self.score.update_score_based_on_target_circle(self.tgt_circ, self._init_tgt_dist_to_circ, self.tgt_circ_tgt_pos)
            self.score.update_score_based_on_square_circle_distance(self.pos, self.tgt_circ.pos, self._init_dist_to_circ)
            
            self._rel_circ_pos = np.subtract(self.tgt_circ.pos, self.pos).tolist()
            self._rel_tgt_pos_to_circ = np.subtract(self.tgt_circ_tgt_pos, self.tgt_circ.pos).tolist()
    
            self.score.update_current_step_score(self._rel_circ_pos, self._rel_tgt_pos_to_circ, self._init_dist_to_circ, self._init_tgt_dist_to_circ, self.left_wheel_speed, self.right_wheel_speed)
            
            self.check_wall_collision()
    
            if steer_with_keys:
                self.handle_steering()

            if self.previous_pos:
                self.data_velocity = np.subtract(self.pos, self.previous_pos).tolist()
            if self.previous_angle:
                self.data_angular_velocity = self.angle - self.previous_angle
            self.previous_pos = self.pos.copy()
            self.previous_angle = self.angle

    def reset(self, id=None, pos=None, is_draggable=False, color=None, target_circle=None):
        self.pos = [random.uniform(BOX_WIDTH / 4, 3 * BOX_WIDTH / 4), random.uniform(BOX_HEIGHT / 4, 3 * BOX_HEIGHT / 4)] if pos is None else pos
        self.velocity = [random.uniform(-200 * VELOCITY, 200 * VELOCITY), random.uniform(-200 * VELOCITY, 200 * VELOCITY)]
        self.angle = random.uniform(0, 360)
        self.angular_velocity = 0  # degrees per second
        #print("target_circle pos: ", self.tgt_circ.pos)
        if self.is_draggable:
            self.velocity = [0, 0]
            self.score.reset()
            #print("target_circle_score after reset: ", self.score.target_circle_score)
            self.tgt_pos = None
            self.tgt_circ_tgt_pos = [random.randint(BOX_WIDTH * 1/4, BOX_WIDTH * 3/4), random.randint(BOX_HEIGHT * 1/4, BOX_HEIGHT * 3/4)]
            self.left_wheel_speed = 0
            self.right_wheel_speed = 0
            self.calculate_initial_pos()


    def calculate_initial_pos(self):
        if self.tgt_circ:
            self._rel_circ_pos = np.subtract(self.tgt_circ.pos, self.pos).tolist()
            self._init_rel_circ_pos = self._rel_circ_pos.copy()
            self._init_dist_to_circ = np.linalg.norm(self._init_rel_circ_pos)
    
        if self.tgt_circ_tgt_pos:
            self._rel_tgt_pos_to_circ = np.subtract(self.tgt_circ_tgt_pos, self.tgt_circ.pos).tolist()
            self._init_rel_tgt_pos_to_circ = self._rel_tgt_pos_to_circ.copy()
            self._init_tgt_dist_to_circ = np.linalg.norm(self._init_rel_tgt_pos_to_circ)
        else:
            print("NO NEW TARGET POS")



# =====================================================================================================
#                                      LIDAR SCANNING
# =====================================================================================================



    def _raycast_distance(self, direction):
        # Calculate ray in x-direction
        if direction[0] > 0:
            dist_x = (self.box_width - self.pos[0]) / direction[0]  # Right boundary
        elif direction[0] < 0:
            dist_x = (0 - self.pos[0]) / direction[0]  # Left boundary
        else:
            dist_x = float('inf')  # Ray is vertical, no intersection with x-boundaries

        # Calculate ray in  y-direction
        if direction[1] > 0:
            dist_y = (self.box_height - self.pos[1]) / direction[1]  # Top boundary
        elif direction[1] < 0:
            dist_y = (0 - self.pos[1]) / direction[1]  # Bottom boundary
        else:
            dist_y = float('inf')  # Ray is horizontal, no intersection with y-boundaries

        distance = min(dist_x, dist_y)

        return distance


    def lidar_scan(self):
        def _rotate_vector_to_robot_frame(vector, robot_angle):
            cos_theta = math.cos(math.radians(-robot_angle))
            sin_theta = math.sin(math.radians(-robot_angle))
            
            rotated_x = cos_theta * vector[0] - sin_theta * vector[1]
            rotated_y = sin_theta * vector[0] + cos_theta * vector[1]
            
            return [rotated_x, rotated_y]
        distances = []
        num_rays = 8
        angle_step = (2 * math.pi) / num_rays

        for i in range(num_rays):
            ray_angle = i * angle_step
            
            direction = [math.cos(ray_angle), math.sin(ray_angle)]
            
            rotated_direction = _rotate_vector_to_robot_frame(direction, self.angle)
            
            distance = self._raycast_distance(rotated_direction)
            
            # if i == 0:
            #     print("ray_angle: ", ray_angle)
            #     print("rotated_direction: ", rotated_direction)
            #     print("distance: ", distance)
            
            distances.append(distance)

        return distances



# =====================================================================================================
#                                     DATA EXTRACTION
# =====================================================================================================

    

    def get_data(self):
        def _rotate_vector_to_robot_frame(vector, robot_angle):
            cos_theta = math.cos(math.radians(-robot_angle))  
            sin_theta = math.sin(math.radians(-robot_angle))
            
            rotated_x = cos_theta * vector[0] - sin_theta * vector[1]
            rotated_y = sin_theta * vector[0] + cos_theta * vector[1]
            
            return [rotated_x, rotated_y]

        relative_pos_of_circle = np.subtract(self.tgt_circ.pos, self.pos).tolist()

        relative_vector_circle_to_target = np.subtract(self.tgt_circ_tgt_pos, self.tgt_circ.pos).tolist()
        
        def _nearest_boundary_vector():
            x_min, x_max = 0, BOX_WIDTH 
            y_min, y_max = 0, BOX_HEIGHT 
            
            dist_x_min = self.pos[0] - x_min
            dist_x_max = x_max - self.pos[0]
            dist_y_min = self.pos[1] - y_min
            dist_y_max = y_max - self.pos[1]

            min_dist = min(dist_x_min, dist_x_max, dist_y_min, dist_y_max)

            if min_dist == dist_x_min:
                return [-min_dist, 0]  
            elif min_dist == dist_x_max:
                return [min_dist, 0] 
            elif min_dist == dist_y_min:
                return [0, -min_dist] 
            else: 
                return [0, min_dist] 
        
        relative_pos_of_circle = _rotate_vector_to_robot_frame(relative_pos_of_circle, self.angle)
        relative_vector_circle_to_target = _rotate_vector_to_robot_frame(relative_vector_circle_to_target, self.angle)
        relative_pos_to_nearest_boundary = _rotate_vector_to_robot_frame(_nearest_boundary_vector(), self.angle)
        
        
        unit_direction_vector = [
            math.cos(math.radians(self.angle)), 
            math.sin(math.radians(self.angle))
        ]
        normalized_pos = [self.pos[0] / BOX_WIDTH, self.pos[1] / BOX_HEIGHT]
        normalized_tgt_circ_pos = [self.tgt_circ.pos[0] / BOX_WIDTH, self.tgt_circ.pos[1] / BOX_HEIGHT]

        normalized_tgt_circ_tgt_pos = [self.tgt_circ_tgt_pos[0] / BOX_WIDTH, self.tgt_circ_tgt_pos[1] / BOX_HEIGHT]
        data = {
            'pos': self.pos,
            'velocity': self.velocity,
            'unit_direction_vector': unit_direction_vector,
            'angular_velocity': self.angular_velocity,
            'relative_pos_of_circle': relative_pos_of_circle,
            'relative_vector_circle_to_target': relative_vector_circle_to_target,
            'relative_pos_to_nearest_boundary': relative_pos_to_nearest_boundary,
            'circle_velocity': self.tgt_circ.velocity, 
            'target_circle_pos': self.tgt_circ.pos,
            'normalized_pos': normalized_pos,
            'normalized_tgt_circ_pos': normalized_tgt_circ_pos,
            'normalized_tgt_circ_tgt_pos': normalized_tgt_circ_tgt_pos
        }
        
        return data



# =====================================================================================================
#                                       STEERING
# =====================================================================================================



    def handle_steering(self):
        """Handle steering dynamics based on the left and right wheel speeds."""
        linear_velocity = (self.left_wheel_speed + self.right_wheel_speed) / 2
        angular_velocity = (self.right_wheel_speed - self.left_wheel_speed) / self.size

        self.angle -= math.degrees(angular_velocity)
        self.pos[0] += linear_velocity * math.cos(math.radians(self.angle))
        self.pos[1] -= linear_velocity * math.sin(math.radians(self.angle))
            


# =====================================================================================================
#                                       GETTERS
# =====================================================================================================



    def get_moment_of_inertia(self):
        return self.moment_of_inertia

    def get_rotated_corners(self):
        """
        Calculate the corners of the square after applying rotation.
    
        This method computes the positions of the four corners of the square
        after rotating it by the square's current angle.
    
        Returns:
            list
            list of list of float: A list containing the coordinates of the 
            rotated corners of the square. Each corner is represented as a 
            list of two floats [x, y].
        """
        half_size = SQUARE_SIZE / 2
        angle_rad = math.radians(-self.angle)
        cos_theta = math.cos(angle_rad)
        sin_theta = math.sin(angle_rad)
        corners = [
            [-half_size, -half_size],
            [half_size, -half_size],
            [half_size, half_size],
            [-half_size, half_size]
        ]
        rotated_corners = []
        for corner in corners:
            rotated_x = self.pos[0] + (corner[0] * cos_theta - corner[1] * sin_theta)
            rotated_y = self.pos[1] + (corner[0] * sin_theta + corner[1] * cos_theta)
            rotated_corners.append([rotated_x, rotated_y])
        return rotated_corners


    def is_point_inside(self, point):
        """Check if a point is inside the square using its current rotation."""
        corners = self.get_rotated_corners()
        n = len(corners)
        inside = False

        p1x, p1y = corners[0]
        for i in range(n+1):
            p2x, p2y = corners[i % n]
            if point[1] > min(p1y, p2y):
                if point[1] <= max(p1y, p2y):
                    if point[0] <= max(p1x, p2x):
                        if p1y != p2y:
                            x_intersect = (point[1] - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                        if p1x == p2x or point[0] <= x_intersect:
                            inside = not inside
            p1x, p1y = p2x, p2y

        return inside
    


# =====================================================================================================
#                                       WALL COLLISONS
# =====================================================================================================



    def check_wall_collision(self):
        """Check for collision with the walls and handle it."""
        half_size = self.size * math.sqrt(2) / 2  # Use half diagonal as half size
        proximity = self.size * 0.8
        
        rotated_corners = self.get_rotated_corners()
        min_x = min(corner[0] for corner in rotated_corners)
        max_x = max(corner[0] for corner in rotated_corners)
        min_y = min(corner[1] for corner in rotated_corners)
        max_y = max(corner[1] for corner in rotated_corners)

        if min_x <= 0:
            self.velocity[0] *= -WALL_RESTITUTION
            self.pos[0] = half_size - min_x
        elif max_x >= BOX_WIDTH:
            self.velocity[0] *= -WALL_RESTITUTION
            self.pos[0] = (BOX_WIDTH - half_size)

        if min_y <= 0:
            self.velocity[1] *= -WALL_RESTITUTION
            self.pos[1] = half_size - min_y 
        elif max_y >= BOX_HEIGHT:
            self.velocity[1] *= -WALL_RESTITUTION
            self.pos[1] = BOX_HEIGHT - half_size


        if self.is_draggable and self.score:
            self.velocity = [0, 0]
            if (min_x <= proximity or max_x >= BOX_WIDTH - proximity or min_y <= proximity or max_y >= BOX_HEIGHT - proximity):
                self.score.update_score_based_on_wall_collision()



# =====================================================================================================
#                              SQUARE - SQUARE COLLISONS
# =====================================================================================================



    def handle_square_square_collision(self, other):
        """Handle collision between two squares."""
        if not self.aabb_collision_check(other):
            #print("No collision")
            return
        #print("collision check")
        if self.sat_collision_check(other):
            if self.score:
                self.score.update_score_based_on_collision(other.id, self.approved_ids)
            self.colliding_with.add(other)
            other.colliding_with.add(self)
            self.resolve_collision_with(other)
            #print(f"Square at pos {self.pos} is colliding with square at pos {other.pos}")
        else:
            if other in self.colliding_with:
                self.colliding_with.remove(other)
            if self in other.colliding_with:
                other.colliding_with.remove(self)


    def resolve_collision_with(self, other):
        """Resolve the collision between two squares."""
        min_overlap = float('inf')
        best_axis = None

        # Check all axes for both squares
        axes = self.get_normals(self.get_rotated_corners()) + other.get_normals(other.get_rotated_corners())
        for axis in axes:
            # Project both squares onto the axis
            p1_min, p1_max = self.project_onto_axis(self.get_rotated_corners(), axis)
            p2_min, p2_max = other.project_onto_axis(other.get_rotated_corners(), axis)

            # Calculate overlap on this axis
            overlap = min(p1_max, p2_max) - max(p1_min, p2_min)
            if overlap > 0 and overlap < min_overlap:
                min_overlap = overlap
                best_axis = axis
                # Determine the direction of the normal (should point from other to self)
                d = np.array(self.pos) - np.array(other.pos)
                if np.dot(d, axis) < 0:
                    best_axis = -np.array(axis)  # Reverse the normal direction

        if best_axis is None or min_overlap <= 0:
            return 

        # Use the smallest overlap axis as the collision normal
        nx, ny = best_axis

        # Apply position correction based on the overlap
        correction = min_overlap / 2
        correction_vector = [correction * nx, correction * ny]
        self.pos[0] += correction_vector[0]
        self.pos[1] += correction_vector[1]
        other.pos[0] -= correction_vector[0]
        other.pos[1] -= correction_vector[1]


    def aabb_collision_check(self, other):
        """Check for collision between two squares using AABB collision detection."""
        half_size = SQUARE_SIZE * math.sqrt(2) / 2
        self_min_x = self.pos[0] - half_size
        self_max_x = self.pos[0] + half_size
        self_min_y = self.pos[1] - half_size
        self_max_y = self.pos[1] + half_size
        other_min_x = other.pos[0] - half_size
        other_max_x = other.pos[0] + half_size
        other_min_y = other.pos[1] - half_size
        other_max_y = other.pos[1] + half_size
        return (self_min_x < other_max_x and self_max_x > other_min_x and
                self_min_y < other_max_y and self_max_y > other_min_y)


    def sat_collision_check(self, other):
        """Check for collision between two squares using the Separating Axis Theorem."""
        axes = self.get_normals(self.get_rotated_corners()) + other.get_normals(other.get_rotated_corners())

        for axis in axes:
            # Project both squares onto the axis
            proj1_min, proj1_max = self.project_onto_axis(self.get_rotated_corners(), axis)
            proj2_min, proj2_max = other.project_onto_axis(other.get_rotated_corners(), axis)

            if proj1_max < proj2_min or proj2_max < proj1_min:
                return False  # Found a separating axis, no collision

        return True  # No separating axis found --> collision occurs


    def get_normals(self, corners):
        """Generate normals for the SAT based on the current corners of the square."""
        normals = []
        n = len(corners)
        for i in range(n):
            # Take each edge and compute its perpendicular
            edge = (corners[(i + 1) % n][0] - corners[i][0], corners[(i + 1) % n][1] - corners[i][1])
            normal = (-edge[1], edge[0])  # Perpendicular to the edge
            # Normalize the normal
            norm_length = math.sqrt(normal[0] ** 2 + normal[1] ** 2)
            normalized_normal = (normal[0] / norm_length, normal[1] / norm_length)
            normals.append(normalized_normal)
        return normals


    def project_onto_axis(self, corners, axis):
        """Project the square onto the axis and return the min and max projections."""
        projections = [(corner[0] * axis[0] + corner[1] * axis[1]) for corner in corners]
        return min(projections), max(projections)

