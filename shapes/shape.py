import random
import math
import numpy as np
from pygame.locals import *



# Constants
BOX_WIDTH = 800
BOX_HEIGHT = 600
SQUARE_SIZE = 50
CIRCLE_RADIUS = 15
NUM_SQUARES = 2
NUM_CIRCLES = 120
TIME_STEP = 0.1
VELOCITY = 0.1
RESTITUTION = 0.2
WALL_RESTITUTION = 1 #0.9
DAMPING_FACTOR = 1 # This adds drag to the simulation
CIRCLE_DAMPING_FACTOR = 0.99
MOUSE_DRAGGED_SQUARE = NUM_SQUARES -1  # The index of the draggable square



class Shape:
    def __init__(self, id,pos, velocity, mass, moment_of_inertia, is_draggable=False , color = None):
        self.id = id
        self.pos = pos 
        self.velocity = velocity 
        self.mass = mass
        self.is_draggable = is_draggable
        self.color = (random.randint(0, 200), random.randint(80, 255), random.randint(80, 255)) if color is None else color
        self.colliding_with = set()  # Track which squares are in contact with this square
        self.last_pos = pos
        self.moment_of_inertia = moment_of_inertia
        self.point_of_impact = None
        self.angular_velocity = 0  # degrees per second

    def calculate_total_energy(self):
        """Calculate the total energy (kinetic + rotational)."""
        speed = math.hypot(self.velocity[0], self.velocity[1])
        kinetic_energy = 0.5 * self.mass * speed ** 2
        rotational_energy = 0.5 * self.get_moment_of_inertia() * self.angular_velocity ** 2
        return kinetic_energy + rotational_energy

    def update(self):
        """Update the pos and velocity of the shape."""
        pass 

    def reset(self):
        """Reset the shape to its initial state."""
        pass

    def check_wall_collision(self):
        """Method to handle wall collisions."""
        pass  

    def get_moment_of_inertia(self):
        """Return the moment of inertia, implemented in child classes."""
        raise NotImplementedError

    def handle_collision(self, other):
        from .square import Square # Avoid circular import
        from .circle import Circle
        """Handle collisions between two shapes."""
        if isinstance(self, Square) and isinstance(other, Square):
            self.handle_square_square_collision(other)
        elif isinstance(self, Circle) and isinstance(other, Circle):
            self.handle_circle_circle_collision(other)
        elif isinstance(self, Square) and isinstance(other, Circle):
            self.handle_square_circle_collision(other)
        elif isinstance(self, Circle) and isinstance(other, Square):
            other.handle_square_circle_collision(self)  

    def aabb_test(self, circle, square_corners):
        """Perform an Axis-Aligned Bounding Box (AABB) test."""
        # Calculate the AABB for the square
        min_x = min(corner[0] for corner in square_corners)
        max_x = max(corner[0] for corner in square_corners)
        min_y = min(corner[1] for corner in square_corners)
        max_y = max(corner[1] for corner in square_corners)

        # Calculate the AABB for the circle
        circle_min_x = circle.pos[0] - circle.radius
        circle_max_x = circle.pos[0] + circle.radius
        circle_min_y = circle.pos[1] - circle.radius
        circle_max_y = circle.pos[1] + circle.radius

        if max_x < circle_min_x or min_x > circle_max_x or max_y < circle_min_y or min_y > circle_max_y:
            return False 

        return True 

    def handle_square_circle_collision(self, circle):
        """Handle Square-Circle collision in the parent class."""
        
        square_corners = self.get_rotated_corners()

        if self.is_point_inside(circle.pos):
            if self.score:
                self.score.update_score_based_on_collision(circle.id, self.approved_ids)
            #return self.resolve_collision_with_circle_when_circle_is_inside(circle, square_corners)
            return circle.resolve_collision_with_circle_when_circle_is_inside(self,square_corners)
        
        closest_point = circle.find_closest_point_on_square(self)
        distance_to_circle = math.hypot(closest_point[0] - circle.pos[0], closest_point[1] - circle.pos[1])
        if distance_to_circle < circle.radius:
            if self.score:
                self.score.update_score_based_on_collision(circle.id, self.approved_ids)
            # There is a collision, resolve it
            self.resolve_collision_with_circle(circle, closest_point)
   

    def resolve_collision_with_circle(self, circle, closest_point):
        """Resolve the collision between a square and a circle."""
        collision_normal = np.array(circle.pos) - np.array(closest_point)
        distance_to_square = np.linalg.norm(collision_normal)

        collision_normal /= distance_to_square

        penetration_depth = circle.radius - distance_to_square
        # if distance_to_square < 0.5:
        #     print("distance_to_circle", distance_to_square)
        #     circle.pos[0] += 50
        #     circle.pos[1] += 50



        if penetration_depth > 0:
            #print("shape of")
            circle.pos += (penetration_depth) * collision_normal

            # target_pos = np.array(closest_point) + collision_normal * circle.radius
            # circle.pos = target_pos

            #velocity_dot_normal = np.dot(circle.velocity, collision_normal)
            #circle.velocity -= 2 * velocity_dot_normal * collision_normal

    def ensure_no_sticking(self, circle, closest_point, collision_normal):
        """Ensure the circle is fully moved out of the square and not sticking."""
        corrected_distance = np.linalg.norm(np.array(circle.pos) - np.array(closest_point))

        if corrected_distance < circle.radius:
            overlap = circle.radius - corrected_distance
            circle.pos += overlap * collision_normal  
        
    #     if self.sat_collision_check(other):
    #         self.resolve_collision_with(other)

    def get_moment_of_inertia(self):
        return self.moment_of_inertia
    
