'''
BDA450
Assignment 1
Alex Natis
Ball Flight Simulator
'''


import matplotlib.pyplot as plt
import math
import astropy.coordinates

class BallFlightSim:

    def __init__(self):
# startingspeed, startingbearing, startingtrajectory, windspeed, windbearing,
                  #targetx, targety, timeinterval
        # Create state for object
        self.position = [0,0,2.2]
        self.startingspeed = 0
        self.startingbearing = 0
        self.startingtrajectory = 0
        self.speed = 0
        self.bearing = 0
        self.trajectory = 0
        self.windspeed = 0
        self.windbearing = 0
        self.targetx = 0
        self.targety = 0
        self.timeinterval = 0
        
        # Record Keeping
        self.xpos = []
        self.ypos = []
        self.zpos = []
        self.veloc = []

    def initialize(self, startingspeed, startingbearing, startingtrajectory, windspeed, windbearing,
                   targetx, targety, timeinterval):

        self.startingspeed = startingspeed
        self.startingbearing = startingbearing
        self.startingtrajectory = startingtrajectory
        self.windspeed = windspeed
        self.windbearing = windbearing
        self.targetx = targetx
        self.targety = targety
        self.timeinterval = timeinterval

        # Initialize time, and all data tracking lists to initial state
        self.t = 0.
        self.timesteps = [self.t]
        self.xpos = [self.position[0]]
        self.ypos = [self.position[1]]
        self.zpos = [self.position[2]]
        self.veloc = [self.startingspeed]

    def observe(self):
        self.xpos.append(self.position[0])
        self.ypos.append(self.position[1])
        self.zpos.append(self.position[2])
        self.veloc.append(self.speed)
        self.timesteps.append(self.t)

    def update(self):
        #Takes a 3D vector, and returns a tuple of the x, y, and z components
        def spherical_to_components(magnitude, bearing, trajectory):
            return astropy.coordinates.spherical_to_cartesian(magnitude, math.radians(trajectory), math.radians(bearing))

        #Takes the x, y, and z components of a 3D vector, and returns a tuple of magnitude, bearing, and trajectory
        def components_to_spherical(x, y, z):
            magnitude, trajectory, bearing = astropy.coordinates.cartesian_to_spherical(x, y, z)
            return magnitude, math.degrees(bearing.to_value()), math.degrees(trajectory.to_value())

        #Takes two 3D vectors (each specified by magnitude, bearing, and trajectory) and returns a
        #tuple representing the sum of the two vectors
        def add_spherical_vectors(magnitude1, bearing1, trajectory1, magnitude2, bearing2, trajectory2):
            x1, y1, z1 = spherical_to_components(magnitude1, bearing1, trajectory1)
            x2, y2, z2 = spherical_to_components(magnitude2, bearing2, trajectory2)
            return components_to_spherical(x1 + x2, y1 + y2, z1 + z2)

        if self.t == 0:
            headwind_vector = add_spherical_vectors(-1*(self.startingspeed), self.startingbearing, self.startingtrajectory, self.windspeed, self.windbearing, 0)
            drag_force = 0.003 * (headwind_vector[0] * headwind_vector[0])
            drag_acceleration_magnitude = drag_force / 0.2
            drag_velocity_vector = (drag_acceleration_magnitude * self.timeinterval, headwind_vector[1], headwind_vector[2])

            gravity_velocity_vector = [9.8 * self.timeinterval, 0, -90]
            
            ball_velocity_vector = add_spherical_vectors(self.startingspeed, self.startingbearing, self.startingtrajectory, drag_velocity_vector[0], drag_velocity_vector[1], drag_velocity_vector[2])
            ball_velocity_vector = add_spherical_vectors(ball_velocity_vector[0], ball_velocity_vector[1], ball_velocity_vector[2], gravity_velocity_vector[0], gravity_velocity_vector[1], gravity_velocity_vector[2])
            ball_velocity_dir = spherical_to_components(ball_velocity_vector[0], ball_velocity_vector[1], ball_velocity_vector[2])

            xpos = self.position[0] + (ball_velocity_dir[0] * self.timeinterval)
            ypos = self.position[1] + (ball_velocity_dir[1] * self.timeinterval)
            zpos = self.position[2] + (ball_velocity_dir[2] * self.timeinterval)

        elif self.t > 0:
            headwind_vector = add_spherical_vectors(-1*(self.speed), self.bearing, self.trajectory, self.windspeed, self.windbearing, 0)
            drag_force = 0.003 * (headwind_vector[0] * headwind_vector[0])
            drag_acceleration_magnitude = drag_force / 0.2
            drag_velocity_vector = ((drag_acceleration_magnitude * self.timeinterval), headwind_vector[1], headwind_vector[2])

            gravity_velocity_vector = [9.8 * self.timeinterval, 0, -90]
            
            ball_velocity_vector = add_spherical_vectors(self.speed, self.bearing, self.trajectory, drag_velocity_vector[0], drag_velocity_vector[1], drag_velocity_vector[2])
            ball_velocity_vector = add_spherical_vectors(ball_velocity_vector[0], ball_velocity_vector[1], ball_velocity_vector[2], gravity_velocity_vector[0], gravity_velocity_vector[1], gravity_velocity_vector[2])
            ball_velocity_dir = spherical_to_components(ball_velocity_vector[0], ball_velocity_vector[1], ball_velocity_vector[2])

            xpos = self.position[0] + (ball_velocity_dir[0] * self.timeinterval)
            ypos = self.position[1] + (ball_velocity_dir[1] * self.timeinterval)
            zpos = self.position[2] + (ball_velocity_dir[2] * self.timeinterval)

        #Update positions
        self.speed = ball_velocity_vector[0]
        self.bearing = ball_velocity_vector[1]
        self.trajectory = ball_velocity_vector[2]
        self.position = [xpos, ypos, zpos]
        self.t = self.t + self.timeinterval

    def runsim(self, startingspeed, startingbearing, startingtrajectory, windspeed, windbearing,
                   targetx, targety, timeinterval):
        self.initialize(startingspeed, startingbearing, startingtrajectory, windspeed, windbearing,
                   targetx, targety, timeinterval)

        # Run until ball hits ground
        while self.position[2] > 0:
            self.update()
            self.observe()

        distanceToTarget = math.sqrt((self.position[0] - self.targetx)**2 + (self.position[1] - self.targety)**2)
        print(f'Final distance from target: {distanceToTarget:4.2f}')

        # plot x vs time
        plt.title("X Position vs. Time")
        plt.ylabel("x (Distance) (m)")
        plt.xlabel("Time (s)")
        plt.plot(self.timesteps, self.xpos)
        plt.show()
        # plot x vs z
        plt.title("X Position vs. Z Position")
        plt.xlabel("x (Distance) (m)")
        plt.ylabel("z (Height) (m)")
        plt.plot(self.xpos, self.zpos)
        plt.show()
        # plot x vs y
        plt.title("X Position vs. Y Position")
        plt.xlabel("x (Distance) (m)")
        plt.ylabel("y (Distance) (m)")
        plt.plot(self.xpos, self.ypos)
        plt.plot(targetx, targety, 'ro')
        plt.axis('square')
        plt.show()
        # plot velocity vs time
        plt.title("Velocity vs. Time")
        plt.ylabel("Velocity (m/s)")
        plt.xlabel("Time (s)")
        plt.plot(self.timesteps, self.veloc)
        plt.show()
        # 3D plot of path
        ax = plt.axes(projection='3d')
        plt.title("3D Path of Ball")
        ax.set_xlabel("x (m)")
        ax.set_ylabel("y (m)")
        ax.set_zlabel("z (m)")
        plt.plot(self.xpos, self.ypos, self.zpos)
        plt.plot(targetx, targety, 0, 'ro')
        plt.show()


if __name__ == '__main__':
    sim = BallFlightSim()
    sim.runsim(27.2, 48.6, 10, 20, 170, 10, 20, .01)
    #sim.runsim(40, -35, 50, 25, 90, 30, 30, .01)