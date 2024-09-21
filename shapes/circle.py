import random
import math
import numpy as np
from pygame.locals import *
from .shape import Shape


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




class Circle(Shape):
    def __init__(self, id = None, radius = None,pos = None, is_draggable=False, color=None):
        pos = [random.uniform(radius + BOX_WIDTH * 1/4 , BOX_WIDTH * 3/4 - radius), random.uniform(radius+ BOX_HEIGHT * 1/4, BOX_HEIGHT * 3/4 - radius)] if pos is None else pos
        velocity = [0,0] #[random.uniform(-100, 100), random.uniform(-100, 100)]
        mass = math.pi * (radius ** 2)
        moment_of_inertia = 0.5 * mass * (radius ** 2)
        super().__init__(id, pos, velocity, mass, moment_of_inertia, is_draggable= is_draggable, color=color)
        self.radius = radius
        self.previous_pos = pos

    def reset(self):
        self.pos = [random.uniform(BOX_WIDTH * 1/4 , BOX_WIDTH * 3/4), random.uniform(BOX_HEIGHT * 1/4, BOX_HEIGHT * 3/4)]
        self.velocity = [0, 0]
        self.previous_pos = self.pos

    def resolve_collision_with_circle_when_circle_is_inside(self, square, square_corners):
        min_distance = float('inf')
        best_normal = None



        # Loop over each edge of the square
        for i in range(len(square_corners)):
            edge_start = np.array(square_corners[i])
            edge_end = np.array(square_corners[(i + 1) % len(square_corners)])

            midpoint = (edge_start + edge_end) / 2

            edge_vector = edge_end - edge_start

            normal = np.array([-edge_vector[1], edge_vector[0]])
            normal = normal / np.linalg.norm(normal) 

            distance = np.linalg.norm(self.pos - midpoint)

            if distance < min_distance:
                min_distance = distance
                closest_midpoint = midpoint
                best_normal = normal

        overlap = self.radius - min_distance

        if overlap > 0:
            self.pos -= best_normal * overlap 

    def find_closest_point_on_square(self, square):
        # TODO route to find_closest_point_on_square_edges() 
        """
        Find the closest point on a square to the circle's center, considering both edges and corners.
        If the circle is inside the square, find the closest point on the nearest edge.
        """
        #square_corners = square.get_rotated_corners()

        # min_x = min(corner[0] for corner in square_corners)
        # max_x = max(corner[0] for corner in square_corners)
        # min_y = min(corner[1] for corner in square_corners)
        # max_y = max(corner[1] for corner in square_corners)



    
        return self.find_closest_point_on_square_edges(square) 

    def find_closest_point_on_square_edges(self, square):
        square_corners = square.get_rotated_corners()
        """
        Find the closest point on the edges of a square to the circle's center.
        This is used when the circle's center is inside the square.
        """
        min_distance = float('inf')
        closest_point = None

        for i in range(len(square_corners)):
            edge_start = np.array(square_corners[i])
            edge_end = np.array(square_corners[(i + 1) % len(square_corners)]) 

            point_on_edge = self.closest_point_on_edge(edge_start, edge_end)

            distance = np.linalg.norm(np.array(self.pos) - point_on_edge)

            if distance < min_distance:
                min_distance = distance
                closest_point = point_on_edge

        return closest_point

    import numpy as np
    
    def closest_point_on_edge(self, edge_start: np.ndarray, edge_end: np.ndarray) -> np.ndarray:
        """
        Find the closest point on an edge (line segment) to the circle's center.
    
        This method calculates the closest point on a given edge (defined by its start and end points)
        to the center of the circle. It uses vector projection to find the point on the infinite line
        defined by the edge, and then clamps this point to the segment defined by the edge.
    
        Parameters:
            edge_start (np.ndarray): The starting point of the edge as a NumPy array [x, y].
            edge_end (np.ndarray): The ending point of the edge as a NumPy array [x, y].
    
        Returns:
            np.ndarray: The closest point on the edge to the circle's center as a NumPy array [x, y].
        """
        edge_vector = edge_end - edge_start
        t = np.dot(np.array(self.pos) - edge_start, edge_vector) / np.dot(edge_vector, edge_vector)
        t = max(0, min(1, t))  # Clamp to [0, 1] to ensure the point lies on the edge
        closest_point = edge_start + t * edge_vector
        return closest_point

    def update(self):
        # check if NaN
        if math.isnan(self.pos[0]) or math.isnan(self.pos[1]):
            print(" FEEEEEEELLLLL posern är NaN")
            self.pos = [BOX_WIDTH/2, BOX_HEIGHT/2]
        self.pos[0] += self.velocity[0] * TIME_STEP
        self.pos[1] += self.velocity[1] * TIME_STEP
        self.check_wall_collision()

        self.velocity[0] *= CIRCLE_DAMPING_FACTOR
        self.velocity[1] *= CIRCLE_DAMPING_FACTOR



    def handle_circle_circle_collision(self, other):
        """Handle Circle-Circle collision."""
        distance_between_centers = math.hypot(self.pos[0] - other.pos[0], self.pos[1] - other.pos[1])

        if distance_between_centers < self.radius + other.radius:
            # Resolve the collision
            self.resolve_circle_collision(other)

    def check_wall_collision(self):
        if self.pos[0] - self.radius <= 0:
            self.velocity[0] *= -WALL_RESTITUTION
            self.pos[0] += self.radius - self.pos[0]
        elif self.pos[0] + self.radius >= BOX_WIDTH:
            self.velocity[0] *= -WALL_RESTITUTION
            self.pos[0] -= self.pos[0] + self.radius - BOX_WIDTH

        if self.pos[1] - self.radius <= 0:
            self.velocity[1] *= -WALL_RESTITUTION
            self.pos[1] += self.radius - self.pos[1]
        elif self.pos[1] + self.radius >= BOX_HEIGHT:
            self.velocity[1] *= -WALL_RESTITUTION
            self.pos[1] -= self.pos[1] + self.radius - BOX_HEIGHT

    def get_moment_of_inertia(self):
        return 0.5 * self.mass * (self.radius ** 2)

    def resolve_circle_collision(self, other):
        """Resolve the collision between two circles."""
        collision_vector = np.array(self.pos) - np.array(other.pos)
        collision_normal = collision_vector / np.linalg.norm(collision_vector)

        v1_normal = np.dot(self.velocity, collision_normal)
        v2_normal = np.dot(other.velocity, collision_normal)

        self.velocity = self.velocity - v1_normal * collision_normal + v2_normal * collision_normal
        other.velocity = other.velocity - v2_normal * collision_normal + v1_normal * collision_normal

        overlap = self.radius + other.radius - np.linalg.norm(collision_vector)
        correction_vector = 0.5 * overlap * collision_normal
        self.pos += correction_vector
        other.pos -= correction_vector