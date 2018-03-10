#!/usr/bin/env python

import sys
import pygame
from utils import *
import config
import constant
from pygame.locals import *


class rapid_expand_random_tree:

    def __init__(self):
        self.state = 'init'
        self.init_pose_set = False
        self.goal_pose_set = False
        self.point_init = None
        self.point_goal = None
        self.nodes = []
        self.node_goal = Node(None, None)
        self.obstacles = []

        self.fpsClock = pygame.time.Clock()
        self.screen = pygame.display.set_mode(constant.WINSIZE)
        self.count = 0

        self.reset()
        # self.handle_event()

    def build_tree(self, nodes, state):
        node_goal = Node(None, None)
        found_next = False
        point_rand, node_nearest = self.get_next_node(nodes, found_next)
        point_new = self.step_from_to(node_nearest.point, point_rand)
        node_new = Node(point_new, node_nearest)
        nodes.append(node_new)

        if dist(nodes[-1].point, Point2D(self.point_goal)) <= constant.RADIUS:
            state = 'complete'
            node_goal = nodes[-1]
        return node_goal, state, nodes

    def get_next_node(self, nodes, found_next):
        while found_next == False:
            # self.fpsClock.tick(100)
            point_rand = self.get_random_clear()
            node_temp_nearest = nodes[0]
            for node in nodes:
                if node is not None and node.point is not None:
                    if dist(node.point, point_rand) <= dist(node_temp_nearest.point, point_rand):
                        if self.collides(self.step_from_to(node.point, point_rand)) == False:
                            # pygame.draw.line(self.screen, constant.RED, [node.point.x, node.point.y],[point_rand.x, point_rand.y],1)
                            node_temp_nearest = Node(
                                self.step_from_to(node.point, point_rand), node)
                            found_next = True
            pygame.draw.circle(self.screen, constant.GREEN, [int(
                node_temp_nearest.point.x), int(node_temp_nearest.point.y)], 2)
        return point_rand, node_temp_nearest

    def update(self):
        while True:
            self.handle_event()
            if self.state == 'init':
                print("Goal Point not set yet")
                self.fpsClock.tick(10)

            elif self.state == 'complete':
                node_current = self.node_goal.parent
                while node_current.parent != None:
                    pygame.draw.line(self.screen, constant.CYAN,
                                     [node_current.point.x, node_current.point.y], [node_current.parent.point.x, node_current.parent.point.y])
                    node_current = node_current.parent

            elif self.state == 'build tree':
                self.count += 1
                if self.count < constant.NUMMODES:
                    self.node_goal, self.state, self.nodes = self.build_tree(
                        self.nodes, self.state)
                    if self.count % 100 == 0:
                        print("node: {}".format(self.count))
                else:
                    print("Ran out of nodes")
                    return

    def get_random(self):
        return Point2D((random.random()*constant.XDIM, random.random()*constant.YDIM))

    def get_random_clear(self):
        p = self.get_random()
        while self.collides(p) == True:
            p = self.get_random()
        return p
        # if self.collides(p) == False:
        #    return p

    def step_from_to(self, p1, p2):
        if dist(p1, p2) < constant.EPSILON:
            return p2
        else:
            theta = atan2(p2.y - p1.y, p2.x - p1.x)
            return Point2D((p1.x + constant.EPSILON*cos(theta), p1.y + constant.EPSILON*sin(theta)))

    def collides(self, p):
        #print("collllllll   :: ", len(self.obstacles))
        for obs in self.obstacles:
            if obs.is_in_obstacle(p) == True:
                return True
        return False

    def init_obstacles(self, config_num):
        obstacles = []
        print("config {}".format(config_num))
        if config_num > 1:
            obstacles.append(
                MyPolygon([(random.random()*constant.XDIM, random.random()*constant.YDIM), (random.random()*constant.XDIM, random.random()*constant.YDIM), (random.random()*constant.XDIM, random.random()*constant.YDIM), (random.random()*constant.XDIM, random.random()*constant.YDIM), (random.random()*constant.XDIM, random.random()*constant.YDIM), (random.random()*constant.XDIM, random.random()*constant.YDIM)]))
        if config_num > 2:
            obstacles.append(MyPolygon([(200, 100), (100, 200), (105, 105)]))
        if config_num > 3:
            obstacles.append(MyPolygon([(500, 200), (450, 210), (300, 100)]))
        if config_num > 4:
            obstacles.append(MyPolygon([(random.random()*constant.XDIM, random.random()*constant.YDIM), (random.random(
            )*constant.XDIM, random.random()*constant.YDIM), (random.random()*constant.XDIM, random.random()*constant.YDIM)]))

        for obs in obstacles:
            pygame.draw.polygon(
                self.screen, constant.RED, obs.vertex_list, 2)
            print(obs.vertex_list)
        self.obstacles = obstacles

    def reset(self):
        self.count = 0
        self.screen.fill(constant.WHITE)
        self.init_obstacles(constant.GAME_LEVEL)

    def handle_event(self):
        for e in pygame.event.get():
            if e.type == QUIT or (e.type == KEYUP and e.key == K_ESCAPE):
                sys.exit("Existing")
            if e.type == MOUSEBUTTONDOWN:
                print("mouse down")
                if self.state == 'init':
                    if self.init_pose_set == False:
                        self.nodes = []
                        if self.collides(e.pos) == False:
                            print("initial pose set: {}".format(e.pos))
                            self.point_init = e.pos
                            self.nodes.append(
                                Node(Point2D(self.point_init), None))
                            self.init_pose_set = True
                            pygame.draw.circle(
                                self.screen, constant.BLUE, self.point_init, constant.RADIUS)
                    elif self.goal_pose_set == False:
                        if self.collides(e.pos) == False:
                            print("goal pose set: {}".format(e.pos))
                            self.point_goal = e.pos
                            self.goal_pose_set = True
                            pygame.draw.circle(
                                self.screen, constant.GREEN, self.point_goal, constant.RADIUS)
                            self.state = 'build tree'
                else:
                    self.state = 'init'
                    self.init_pose_set = False
                    self.goal_pose_set = False
                    self.reset()

        pygame.display.update()
        self.fpsClock.tick(100)


if __name__ == "__main__":
    rrt = rapid_expand_random_tree()
    rrt.update()
