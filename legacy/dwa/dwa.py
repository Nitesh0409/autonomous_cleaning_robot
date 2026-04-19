#! /usr/bin/python3
# -*- coding: utf-8 -*-
import sys, math, time

from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.Qt import *
from PyQt5.QtWidgets import QApplication, QGraphicsLineItem, QGraphicsScene, QGraphicsView

from dwa_window import Ui_DWA_Simulator

import numpy as np
np.seterr(divide='ignore', invalid='ignore')
import matplotlib.pyplot as plt
import pandas as pd
import math
import sys, time

#global parameters
goal_from_picture_x = 10
goal_from_picture_y = 10
sampling_interval = 0.1

def min_max_normalize(data):

    data = np.array(data)

    max_data = max(data)
    min_data = min(data)

    if max_data - min_data == 0:
        data = [0.0 for i in range(len(data))]
    else:
        data = (data - min_data) / (max_data - min_data)

    return data

def angle_range_corrector(angle):

    if angle > math.pi:
        while angle > math.pi:
            angle -=  2 * math.pi
    elif angle < -math.pi:
        while angle < -math.pi:
            angle += 2 * math.pi

    return angle

def write_circle(center_x, center_y, angle, circle_size=0.2):
    
    circle_x = [] 
    circle_y = []

    steps = 100
    for i in range(steps):
        circle_x.append(center_x + circle_size*math.cos(i*2*math.pi/steps))
        circle_y.append(center_y + circle_size*math.sin(i*2*math.pi/steps))

    circle_line_x = [center_x, center_x + math.cos(angle) * circle_size]
    circle_line_y = [center_y, center_y + math.sin(angle) * circle_size]

    return circle_x, circle_y, circle_line_x, circle_line_y

class Path():
    def __init__(self, u_th, u_v): 
        self.x = None
        self.y = None
        self.th = None
        self.u_v = u_v
        self.u_th = u_th

class Obstacle():
    def __init__(self, x, y, size):
        self.x = x
        self.y = y
        self.size = size

class Two_wheeled_robot():
    def __init__(self, init_x, init_y, init_th):
        
        #initialization
        self.x = init_x
        self.y = init_y
        self.th = init_th
        self.u_v = 0.0
        self.u_th = 0.0

        self.traj_x = [init_x]
        self.traj_y = [init_y]
        self.traj_th = [init_th]
        self.traj_u_v = [0.0]
        self.traj_u_th = [0.0]

    def update_state(self, u_th, u_v, dt):

        self.u_th = u_th
        self.u_v = u_v

        next_x = self.u_v * math.cos(self.th) * dt + self.x
        next_y = self.u_v * math.sin(self.th) * dt + self.y
        next_th = self.u_th * dt + self.th

        self.traj_x.append(next_x)
        self.traj_y.append(next_y)
        self.traj_th.append(next_th)

        self.x = next_x
        self.y = next_y
        self.th = next_th

        return self.x, self.y, self.th

class Simulator_DWA_robot():
    def __init__(self):

        # two wheeled robot
        self.max_accelation = 1
        self.max_ang_accelation = 100 * math.pi /180
        self.lim_max_velo = 1.6 # m/s
        self.lim_min_velo = 0.0 # m/s
        self.lim_max_ang_velo = math.pi
        self.lim_min_ang_velo = -math.pi

    def predict_state(self, ang_velo, velo, x, y, th, dt, pre_step):
        next_xs = []
        next_ys = []
        next_ths = []

        for i in range(pre_step):
            temp_x = velo * math.cos(th) * dt + x
            temp_y = velo * math.sin(th) * dt + y
            temp_th = ang_velo * dt + th

            next_xs.append(temp_x)
            next_ys.append(temp_y)
            next_ths.append(temp_th)

            x = temp_x
            y = temp_y
            th = temp_th

        return next_xs, next_ys, next_ths

# DWA
class DWA():
    def __init__(self):
        global sampling_interval

        self.simu_robot = Simulator_DWA_robot()

        self.pre_time = 3
        self.pre_step = 30
        self.delta_velo = 0.02
        self.delta_ang_velo = 0.02

        # define weight
        self.weight_angle = 0.04
        self.weight_velo = 0.2
        self.weight_obs = 0.1

        self.area_dis_to_obs = 5

        self.traj_paths = []
        self.traj_opt = []

    def calc_input(self, g_x, g_y, state, obstacles):
        # create path
        paths = self._make_path(state)
        # evaluate path
        opt_path = self._eval_path(paths, g_x, g_y, state, obstacles)

        self.traj_opt.append(opt_path)

        return paths, opt_path

    def _make_path(self, state): 
        # calculate range of velocity and angular velocity
        min_ang_velo, max_ang_velo, min_velo, max_velo = self._calc_range_velos(state)

        # list of all paths
        paths = []

        # evaluate all velocities and angular velocities combinations
        for ang_velo in np.arange(min_ang_velo, max_ang_velo, self.delta_ang_velo):
            for velo in np.arange(min_velo, max_velo, self.delta_velo):

                path = Path(ang_velo, velo)

                next_x, next_y, next_th \
                    = self.simu_robot.predict_state(ang_velo, velo, state.x, state.y, state.th, sampling_interval, self.pre_step)

                path.x = next_x
                path.y = next_y
                path.th = next_th

                # add path
                paths.append(path)

        # save tha path
        self.traj_paths.append(paths)

        return paths

    def _calc_range_velos(self, state): # calculate range of velocity and angular velocity
        # angular velocity
        range_ang_velo = sampling_interval * self.simu_robot.max_ang_accelation
        min_ang_velo = state.u_th - range_ang_velo
        max_ang_velo = state.u_th + range_ang_velo
        # minimum
        if min_ang_velo < self.simu_robot.lim_min_ang_velo:
            min_ang_velo = self.simu_robot.lim_min_ang_velo
        # maximum
        if max_ang_velo > self.simu_robot.lim_max_ang_velo:
            max_ang_velo = self.simu_robot.lim_max_ang_velo

        # velocity
        range_velo = sampling_interval * self.simu_robot.max_accelation
        min_velo = state.u_v - range_velo
        max_velo = state.u_v + range_velo
        # minimum
        if min_velo < self.simu_robot.lim_min_velo:
            min_velo = self.simu_robot.lim_min_velo
        # maximum
        if max_velo > self.simu_robot.lim_max_velo:
            max_velo = self.simu_robot.lim_max_velo

        return min_ang_velo, max_ang_velo, min_velo, max_velo

    def _eval_path(self, paths, g_x, g_y, state, obastacles):
        # detect nearest obstacle
        nearest_obs = self._calc_nearest_obs(state, obastacles)

        score_heading_angles = []
        score_heading_velos = []
        score_obstacles = []

        # evaluate all pathes
        for path in paths:
            # (1) heading_angle
            score_heading_angles.append(self._heading_angle(path, g_x, g_y))
            # (2) heading_vel
            score_heading_velos.append(self._heading_velo(path))
            # (3) obstacle
            score_obstacles.append(self._obstacle(path, nearest_obs))

        # normalization
        for scores in [score_heading_angles, score_heading_velos, score_obstacles]:
            scores = min_max_normalize(scores)

        score = 0.0

        for k in range(len(paths)):
            temp_score = 0.0

            temp_score = self.weight_angle * score_heading_angles[k] + \
                         self.weight_velo * score_heading_velos[k] + \
                         self.weight_obs * score_obstacles[k]

            if temp_score > score:
                opt_path = paths[k]
                score = temp_score

        try:
            return opt_path
        except:
            ret = QMessageBox.information(None, "message", "Can not calculate optimal path!", QMessageBox.Ok)

    def _heading_angle(self, path, g_x, g_y):

        last_x = path.x[-1]
        last_y = path.y[-1]
        last_th = path.th[-1]

        # calculate angle
        angle_to_goal = math.atan2(g_y-last_y, g_x-last_x)

        # calculate score
        score_angle = angle_to_goal - last_th
        score_angle = abs(angle_range_corrector(score_angle))
        score_angle = math.pi - score_angle

        return score_angle

    def _heading_velo(self, path):

        score_heading_velo = path.u_v

        return score_heading_velo

    def _calc_nearest_obs(self, state, obstacles):
        nearest_obs = []

        for obs in obstacles:
            temp_dis_to_obs = math.sqrt((state.x - obs.x) ** 2 + (state.y - obs.y) ** 2)

            if temp_dis_to_obs < self.area_dis_to_obs :
                nearest_obs.append(obs)

        return nearest_obs

    def _obstacle(self, path, nearest_obs):
        # obstacle avoidance
        score_obstacle = 2
        temp_dis_to_obs = 0.0

        for i in range(len(path.x)):
            for obs in nearest_obs: 
                temp_dis_to_obs = math.sqrt((path.x[i] - obs.x) * (path.x[i] - obs.x) + (path.y[i] - obs.y) *  (path.y[i] - obs.y))

                if temp_dis_to_obs < score_obstacle:
                    score_obstacle = temp_dis_to_obs #the nearest obstacle

                # collision with obstacl
                if temp_dis_to_obs < obs.size + 0.75: #0.75 is the margin
                    score_obstacle = -float('inf')
                    break
            else:
                continue

            break

        return score_obstacle

class Const_goal():
    def __init__(self):
        self.traj_g_x = []
        self.traj_g_y = []

    def calc_goal(self, time_step):
        global goal_from_picture_x, goal_from_picture_y
        g_x  = goal_from_picture_x
        g_y = goal_from_picture_y

        self.traj_g_x.append(g_x)
        self.traj_g_y.append(g_y)

        return g_x, g_y

class Main_controller():

    new_goal_flag = False

    def __init__(self):

        self.robot = Two_wheeled_robot(0.0, 0.0, 0.0)
        self.goal_maker = Const_goal()
        self.controller = DWA()
        self.obstacles =[Obstacle(4, 1, 0.25), Obstacle(0, 4.5, 0.25),  Obstacle(3, 4.5, 0.25), Obstacle(5, 3.5, 0.25),  Obstacle(7.5, 9.0, 0.25)]

    def run_to_goal(self, time_step, goal_flag):
        global sampling_interval

        if(goal_flag == False):
            g_x, g_y = self.goal_maker.calc_goal(time_step)

            # input from controller
            paths, opt_path = self.controller.calc_input(g_x, g_y, self.robot, self.obstacles)
            
            if(opt_path is not None):
                u_th = opt_path.u_th
                u_v = opt_path.u_v
            else:
                window.close()

            # update robot state
            self.robot.update_state(u_th, u_v, sampling_interval)

            # reach the goal?
            dis_to_goal = np.sqrt((g_x-self.robot.x)**2 + (g_y-self.robot.y)**2)
            if dis_to_goal < 0.5:
                self.new_goal_flag = True

            time_step += 1
            
            window.draw(self.robot.traj_x, self.robot.traj_y, self.robot.traj_th, self.goal_maker.traj_g_x, self.goal_maker.traj_g_y, self.controller.traj_paths, self.controller.traj_opt, self.obstacles) #draw simulation result

        return self.new_goal_flag

class Simulation_Window(QDialog):

    time_step = 0
    goal_flag = False

    controller = Main_controller()
    timer = QTimer()

    def __init__(self,parent=None):
        super(Simulation_Window, self).__init__(parent)
        self.ui = Ui_DWA_Simulator()
        self.ui.setupUi(self)

        #DWA parameters initial values
        self.ui.pre_time_spinBox.setValue(3)
        self.ui.pre_step_spinBox.setValue(30)
        self.ui.vel_delta_SpinBox.setValue(0.02)
        self.ui.ang_vel_delta_SpinBox.setValue(0.02)
        self.ui.sampling_interval_SpinBox.setValue(0.1)
        self.ui.angle_weight_SpinBox.setValue(0.04)
        self.ui.velocity_weight_SpinBox.setValue(0.2)
        self.ui.obstacle_weight_SpinBox.setValue(0.1)
        self.ui.area_dis_to_obs_SpinBox.setValue(5)

        #robot parameters initial values
        self.ui.Max_Acc_SpinBox.setValue(1)
        self.ui.Max_Ang_Acc_SpinBox.setValue(100*math.pi/180)
        self.ui.Max_Vel_SpinBox.setValue(1.6)
        self.ui.Min_Vel_SpinBox.setValue(0.0)
        self.ui.Max_Ang_Vel_SpinBox.setValue(math.pi)
        self.ui.Min_Ang_Vel_SpinBox.setValue(-math.pi)

        self.ui.Possible_passes_spinBox.setValue(8)

    def do_calculations(self):
        global sampling_interval

        if(self.goal_flag == False):
            self.goal_flag = self.controller.run_to_goal(self.time_step, self.goal_flag)
        self.time_step += 1

        if(self.goal_flag == True):
            self.timer.stop()

        #controller parameters
        self.controller.controller.pre_time = self.ui.pre_time_spinBox.value()
        self.controller.controller.pre_step = self.ui.pre_step_spinBox.value()
        self.controller.controller.delta_velo = self.ui.vel_delta_SpinBox.value()
        self.controller.controller.delta_ang_velo = self.ui.ang_vel_delta_SpinBox.value()
        sampling_interval = self.ui.sampling_interval_SpinBox.value()
        self.controller.controller.weight_angle = self.ui.angle_weight_SpinBox.value()
        self.controller.controller.weight_velo = self.ui.velocity_weight_SpinBox.value()
        self.controller.controller.weight_obs = self.ui.obstacle_weight_SpinBox.value()
        self.controller.controller.area_dis_to_obs = self.ui.area_dis_to_obs_SpinBox.value()

        #robot parameters
        self.controller.controller.simu_robot.max_accelation = self.ui.Max_Acc_SpinBox.value()
        self.controller.controller.simu_robot.max_ang_accelation = self.ui.Max_Ang_Acc_SpinBox.value()
        self.controller.controller.simu_robot.lim_max_velo = self.ui.Max_Vel_SpinBox.value()
        self.controller.controller.simu_robot.lim_min_velo = self.ui.Min_Vel_SpinBox.value()
        self.controller.controller.simu_robot.lim_max_ang_velo = self.ui.Max_Ang_Vel_SpinBox.value()
        self.controller.controller.simu_robot.lim_min_ang_velo = self.ui.Min_Ang_Vel_SpinBox.value()

    def pause(self):
        self.timer.stop()

    def reset(self):
        self.controller.robot.traj_x.clear()
        self.controller.robot.traj_y.clear()
        self.controller.robot.traj_th.clear()
        self.controller.goal_maker.traj_g_x.clear()
        self.controller.goal_maker.traj_g_y.clear()
        self.controller.controller.traj_paths.clear()
        self.controller.controller.traj_opt.clear()
        self.controller.new_goal_flag = False
        self.controller.goal_flag = False
        self.goal_flag = False
        self.time_step = 0

        self.controller.robot.x = 0.0
        self.controller.robot.y = 0.0
        self.controller.robot.th = 0.0
        self.controller.robot.u_v = 0.0
        self.controller.robot.u_th = 0.0
        self.controller.robot.traj_x = [0.0]
        self.controller.robot.traj_y = [0.0]
        self.controller.robot.traj_th = [0.0]
        self.controller.robot.traj_u_v = [0.0]
        self.controller.robot.traj_u_th = [0.0]

    def Start_simulation(self):
        self.timer.timeout.connect(self.do_calculations)
        self.timer.start(50)

    def draw(self, traj_x, traj_y, traj_th, goal_x, goal_y, traj_paths, traj_opt, obstacles):

        scale = 0.025 # 1pixel = 0.025m
        C = 1/scale
        X_offset = 100
        Y_offset = 500
        
        self.scene = GraphicsScene()
        self.ui.graphicsView.setScene(self.scene)

        pen_axis = QPen(Qt.black)
        pen_axis.setWidth(3)
        self.scene.addLine(QLineF(X_offset, Y_offset, X_offset, X_offset), pen_axis)
        self.scene.addLine(QLineF(X_offset, Y_offset, Y_offset, Y_offset), pen_axis)
        pen_axis.setWidth(1)
        pen_axis.setStyle(Qt.DashLine)
        self.scene.addLine(QLineF(Y_offset, Y_offset, Y_offset, X_offset), pen_axis)
        self.scene.addLine(QLineF(X_offset, X_offset, Y_offset, X_offset), pen_axis)
        self.scene.addLine(QLineF(X_offset, 300, Y_offset, 300), pen_axis)
        self.scene.addLine(QLineF(300, X_offset, 300, Y_offset), pen_axis)

        item_10 = self.scene.addText("10", QFont('Arial Black', 15, QFont.Light))
        item_10.setPos(60,90)
        item_10_2 = self.scene.addText("10", QFont('Arial Black', 15, QFont.Light))
        item_10_2.setPos(490,500)
        item_0 = self.scene.addText("0", QFont('Arial Black', 15, QFont.Light))
        item_0.setPos(70,500)

        ###draw trajectory###
        pen_traj=QPen(Qt.blue)
        pen_traj.setStyle(Qt.DashLine)
        for i in range(len(traj_x) - 1):
            self.scene.addLine(QLineF(40*traj_x[i] + 100, -40*traj_y[i] + 500, 40*traj_x[i + 1] + 100, -40*traj_y[i + 1] + 500), pen_traj)

        ###draw possible paths###
        pen_path = QPen()
        pen_path.setColor(QColor(200, 150, 0))
        a = len(traj_paths)
        b = len(traj_paths[a-1])

        N = self.ui.Possible_passes_spinBox.value()
        interval = int(b / N)
        for k in range(0, len(traj_paths[a-1])-1, interval):
            for j in range(len(traj_paths[a-1][b-1].x)-1):
                self.scene.addLine(QLineF(C*traj_paths[a-1][k].x[j] + X_offset, -C*traj_paths[a-1][k].y[j] + Y_offset, C*traj_paths[a-1][k].x[j+1] + X_offset, -C*traj_paths[a-1][k].y[j+1] + Y_offset), pen_path)

        #draw optimal path
        pen_opt = QPen(Qt.red)
        c = len(traj_opt)
        for n in range(len(traj_opt[c-1].x) - 1):
            self.scene.addLine(QLineF(C*traj_opt[c-1].x[n] + X_offset, -C*traj_opt[c-1].y[n] + Y_offset, C*traj_opt[c-1].x[n + 1] + X_offset, -C*traj_opt[c-1].y[n + 1] + Y_offset), pen_opt)

        #draw obstacles
        pen_obstacle = QPen()
        pen_obstacle.setColor(QColor(200, 0, 150))
        for p in range(len(obstacles)):
            obstacle_diameter = C*obstacles[p].size
            self.scene.addEllipse(C*obstacles[p].x + X_offset - obstacle_diameter/2, -C*obstacles[p].y + Y_offset - obstacle_diameter/2, obstacle_diameter, obstacle_diameter, pen_obstacle, QBrush(QColor(200, 0, 150)))
                
        ###draw the robot #40 = 1/0.025###
        X = traj_x[self.time_step]
        Y = traj_y[self.time_step]
        th = traj_th[self.time_step] - math.pi/2
        
        robot_vertices = [[C*(X + (- 0.25*math.sin(th))) + X_offset,                      -C*(Y + (0.25*math.cos(th))) + Y_offset], 
                          [C*(X + (-0.2*math.cos(th) - 0.15*math.sin(th))) + X_offset,    -C*(Y + (-0.2*math.sin(th) + 0.15*math.cos(th))) + Y_offset], 
                          [C*(X + (-0.2*math.cos(th) - (-0.15)*math.sin(th))) + X_offset, -C*(Y + (-0.2*math.sin(th) - 0.15*math.cos(th))) + Y_offset],
                          [C*(X + (0.2*math.cos(th) - (-0.15)*math.sin(th))) + X_offset,  -C*(Y + (0.2*math.sin(th) - 0.15*math.cos(th))) + Y_offset], 
                          [C*(X + (0.2*math.cos(th) - 0.15*math.sin(th))) + X_offset,     -C*(Y + (0.2*math.sin(th) + 0.15*math.cos(th))) + Y_offset]]
        
        qpoly_robot = QPolygonF([QPointF(p[0], p[1]) for p in robot_vertices])
        pen_robot = QPen(Qt.red)
        pen_robot.setWidth(2)
        self.scene.addPolygon(qpoly_robot, pen_robot)

        ###draw goal###
        pen_goal = QPen(Qt.blue)
        diameter = 16
        self.scene.addEllipse(C*goal_x[self.time_step] + X_offset - diameter/2, -C*goal_y[self.time_step] + Y_offset - diameter/2, diameter, diameter, pen_goal, QBrush(Qt.green))

class GraphicsScene(QGraphicsScene):

    def __init__(self, parent=None):
        QGraphicsScene.__init__(self, 0, 0, 600, 600, parent = None)
        self.opt = ""
        
    def mousePressEvent(self, event):
        global goal_from_picture_x, goal_from_picture_y
                
        goal_from_picture_x = (event.scenePos().x() - 100)*0.025
        goal_from_picture_y = -(event.scenePos().y() - 500)*0.025

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = Simulation_Window()
    window.show()
    sys.exit(app.exec_())

