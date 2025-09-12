import datetime
import numpy as np
import matplotlib.pyplot as plt
from params import CarParams, FrameParams, IC, PIDParams

class CarSim:

    def __init__(self):
        self.car_params = CarParams()
        self.frame_params = FrameParams()

        xmin = self.frame_params.xmin
        xmax = self.frame_params.xmax
        ymin = self.frame_params.ymin
        ymax = self.frame_params.ymax


        self.fig, self.ax = plt.subplots()
        self.ax.set_aspect('equal')
        self.ax.set_xlim(xmin, xmax)
        self.ax.set_ylim(ymin, ymax)
        self.ax.grid(True)


    def R_yaw(self, yaw):
        return np.array([[np.cos(yaw), -np.sin(yaw), 0],
                         [np.sin(yaw), np.cos(yaw), 0],
                         [0, 0, 1]])

    def create_car(self, x, y, yaw):

        pos = np.array([x, y, 0])

        lf = self.car_params.L / 2      # distace from COM to front
        lr = self.car_params.L / 2      # distance from COM to rear
        wl = self.car_params.W / 2      # distance from COM to left
        wr = self.car_params.W / 2      # distance from COM to right


        rot = self.R_yaw(yaw)

        fr = rot @ np.array([lf, -wr, 0]) + pos
        fl = rot @ np.array([lf, wl, 0]) + pos
        rl = rot @ np.array([-lr, wl, 0]) + pos
        rr = rot @ np.array([-lr, -wr, 0]) + pos

        points = np.array([fr, fl, rl, rr, fr])

        carx = points[:, 0]
        cary = points[:, 1]

        return carx, cary
    
    def create_random_target(self):
        xmin = self.frame_params.xmin
        xmax = self.frame_params.xmax
        ymin = self.frame_params.ymin
        ymax = self.frame_params.ymax

        target_x = np.random.uniform(xmin, xmax)
        target_y = np.random.uniform(ymin, ymax)

        return target_x, target_y
    
    def target_found(self, carx, cary, target_x, target_y):
        if target_x > min(carx) and target_x < max(carx) and target_y > min(cary) and target_y < max(cary):
            return True
        else:
            return False

    def create_lidar(self, x, y, yaw):
    
        r = self.car_params.R_lidar

        theta_scan = np.linspace(yaw -np.pi/3, yaw + np.pi/3, 50)

        lidarx = r * np.cos(theta_scan) + x
        lidary = r * np.sin(theta_scan) + y

        return lidarx, lidary

    def create_random_obstacle(self):
        xmin = self.frame_params.xmin
        xmax = self.frame_params.xmax
        ymin = self.frame_params.ymin
        ymax = self.frame_params.ymax

        x = np.random.uniform(xmin, xmax)
        y = np.random.uniform(ymin, ymax)

        theta = np.random.uniform(0, 2*np.pi)

        R = 5
        a = R*0.4
        b = 5

        r = np.linspace(theta - np.pi, theta + np.pi, 100)

        obstaclex = (R + a*np.sin(b*r))*np.sin(r) + x
        obstacley = (R + a*np.sin(b*r))*np.cos(r) + y

        return obstaclex, obstacley
    
    def lidar_scan(self, lidar_x, lidar_y, obstacle_x, obstacle_y):
        lidar_radius = self.car_params.R_lidar
        obstacle_r = np.sqrt((obstacle_x - lidar_x)**2 + (obstacle_y - lidar_y)**2)
        x = np.empty(0)
        y = np.empty(0)
        for i, r in enumerate(obstacle_r):
            if r < lidar_radius:
                x = np.append(x, obstacle_x[i])
                y = np.append(y, obstacle_y[i])

        return x, y
    
    def draw_car(self, carx, cary):
        line, = plt.plot(carx, cary, color='blue')
        return line

    def draw_target(self, target_x, target_y):
        line = plt.scatter(target_x, target_y, color='green', s=50)
        return line
    
    def draw_lidar(self, lidarx, lidary):
        line, = plt.plot(lidarx, lidary, color='pink')
        return line

    def draw_obstacle(self, obstaclex, obstacley):
        line, = plt.plot(obstaclex, obstacley, color='black')
        return line

    def draw_danger(self, x, y):
        line = plt.scatter(x, y, color='red', s=50)
        return line

class Dynamics:

    def __init__(self):
        self.car_params = CarParams()
        self.frame_params = FrameParams()

    def R_yaw(self, yaw):
        return np.array([[np.cos(yaw), -np.sin(yaw)],
                         [np.sin(yaw), np.cos(yaw)],
                        ])

    def global_kinematic_model(self, dt, x, y, yaw, vr, vl):

        W = self.car_params.W

        v_long = (vr + vl) * 0.5
        yaw_dot = (vr - vl) / W
        x_dot = v_long * np.cos(yaw)
        y_dot = v_long * np.sin(yaw)

        x += x_dot * dt
        y += y_dot * dt
        yaw += yaw_dot * dt

        return v_long, x, y, yaw
    
class Controller():

    def __init__(self):
        self.car_params = CarParams()
        self.frame_params = FrameParams()
        self.PID_params = PIDParams()

        self.sum_error_speed = 0.0
        self.prev_error_speed = 0.0

        self.sum_error_steer = 0.0
        self.prev_error_steer = 0.0

    def speed_pid(self, error, dt):
        Kp = self.PID_params.Kp_speed
        Ki = self.PID_params.Ki_speed
        Kd = self.PID_params.Kd_speed

        u = Kp*error + Ki*self.sum_error_speed*dt + Kd*(error - self.prev_error_speed)/dt
        self.sum_error_speed += error
        self.prev_error_speed = error
        return u

    def steer_pid(self, error, dt):
        Kp = self.PID_params.Kp_steer
        Ki = self.PID_params.Ki_steer
        Kd = self.PID_params.Kd_steer

        u = Kp*error + Ki*self.sum_error_steer*dt + Kd*(error - self.prev_error_steer)/dt
        self.sum_error_steer += error
        self.prev_error_steer = error
        return u
    
    def step_pid(self, dt, v, x, y, yaw, v_ref, x_ref, y_ref):

        W = self.car_params.W

        yaw_ref = np.arctan2(y_ref - y, x_ref - x)
        yaw_error = yaw_ref - yaw
        yaw_error = (yaw_error + np.pi) % (2 * np.pi) - np.pi

        K = self.PID_params.K_scalling
        scaling_factor = np.exp(-K * abs(yaw_error))
        v_ref *= scaling_factor
        v_error = v_ref - v

        u_speed = self.speed_pid(v_error, dt)
        u_steer = self.steer_pid(yaw_error, dt)

        vr = u_speed + 0.5 * W * u_steer
        vl = u_speed - 0.5 * W * u_steer

        return vr, vl

def main():
    carsim = CarSim()
    dynamics = Dynamics()
    controller = Controller()

    car_params = CarParams()
    ic = IC()

    dt = car_params.dt
    v, x, y, yaw = ic.v_long, ic.x, ic.y, ic.yaw
    v_ref = IC.v_ref

    carx, cary = carsim.create_car(x, y, yaw)
    target_x, target_y = carsim.create_random_target()
    
    car = carsim.draw_car(carx, cary)
    target = carsim.draw_target(target_x, target_y)

    while True:
        while not carsim.target_found(carx, cary, target_x, target_y):
            vr, vl = controller.step_pid(dt, v, x, y, yaw, v_ref, target_x, target_y)
            v, x, y, yaw = dynamics.global_kinematic_model(dt, x, y, yaw, vr, vl)

            carx, cary = carsim.create_car(x, y, yaw)
            car.remove()
            car = carsim.draw_car(carx, cary)
            plt.pause(dt)
        
        target.remove()
        target_x, target_y = carsim.create_random_target()
        target = carsim.draw_target(target_x, target_y)

if __name__ == "__main__":
    main()