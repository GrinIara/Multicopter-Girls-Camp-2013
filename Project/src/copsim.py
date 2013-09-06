#!/usr/bin/python

import time, math, pygame, pygame.locals, pygame.gfxdraw, random
from numpy import array
from numpy.linalg import norm
from threading import Thread
from sets import ImmutableSet
from sympy import *

SCREEN_SIZE = (1280, 720)
RESOLUTION = 0.5 # 1 pixel is 2 meters
RANGE_X = SCREEN_SIZE[0] * RESOLUTION / 4.0
RANGE_Y = SCREEN_SIZE[1] * RESOLUTION / 4.0
SCREEN_CENTER = (SCREEN_SIZE[0] / 2.0, SCREEN_SIZE[1] / 2.0)
NUM_NODES = 3
COM_RANGE = 100
COM_FREQ = 2
COM_JITTER = COM_FREQ/2
TIMEOUT = COM_FREQ * 2
MAX_FPS = 25
SIMULATION_RESOLUTION = 10

OBSTACLE_SEP = 20
OBSTACLE_P = 0.1
SEPARATION = 80
P = 1.0
CUTOFF = 3.0

running = True
ts = 0

nodes = []
obstacles = []

def vlen(a):
    return math.sqrt(a[0] * a[0] + a[1] * a[1]);

def vsub(a, b):
    return (a[0] - b[0], a[1] - b[1])

def vadd(a, b):
    return (a[0] + b[0], a[1] + b[1])

def vscale(a, s):
    return (a[0] * s, a[1] * s)

def vnorm(a):
    try:
        il = 1.0/vlen(a)
    except ZeroDivisionError:
        il = 1.0

    return (a[0] * il, a[1] * il)

def timestamp():
    return ts / 1000.0

class Obstacle:
    MODE_IN, MODE_OUT = range(2)

    def __init__(self, pos = (0.0, 0.0), radius = 0.0, mode = MODE_OUT):
        self.pos = pos
        self.radius = radius
        self.mode = mode

class TSNode:
    def __init__(self, ts, node):
        self.ts = ts
        self.node = node
        self.pos = node.pos

class Node:
    def __init__(self, pos = (0.0, 0.0), rng = 500.0, trg = (0.0, 0.0), spd = 1.0, obstacles = []):
        self.pos = pos
        self.trg = trg
        self.spd = spd
        self.rng = rng
        self.tsnodes = []
        self.ts_prev = timestamp()
        self.ts_com = ts
        self.obstacles = obstacles
        self.manual = False

    def move(self):
        dt = timestamp() - self.ts_prev
        self.pos = vadd(vscale(self.trg, dt), self.pos)
        self.ts_prev = timestamp()

    def _limit(self, vec):
        if vlen(vec) > CUTOFF:
            vec = vscale(vnorm(vec), CUTOFF)
        return vec


    def _step(self, tv):
        while True:
            dist = norm(tv)
            if dist == 0.0:
                tv = array([random.random(), random.random()])
            else:
                break

        tv_norm = tv / dist
        ctrl = P * math.log(1.0/SEPARATION * dist, 0.5)
        return tv_norm * ctrl

    def swarm(self):
        trg = self.trg
        for n in self.tsnodes:
            if vlen(vsub(n.pos, self.pos)) <= 1.2 * SEPARATION:
                vec = vsub(self.pos, n.pos)
                trg = vadd(self._step(array(vec)), trg)

        self.trg = self._limit(trg)

    def avoid(self):
        for o in self.obstacles:
            if o.mode == Obstacle.MODE_OUT:
                ov = vsub(self.pos, o.pos)
                dist = vlen(ov)
                test = dist < o.radius + OBSTACLE_SEP
            elif o.mode == Obstacle.MODE_IN:
                ov = vsub(o.pos, self.pos)
                dist = vlen(ov)
                test = dist > o.radius - OBSTACLE_SEP

            if test:
                tv = vnorm(ov)
                x = 1.0/OBSTACLE_SEP * dist - o.radius
                if x < 0.05: x = 0.05
                ctrl = OBSTACLE_P * math.log(x, 0.5)
                trg = vadd(self.trg, vscale(tv, ctrl))
                self.trg = self._limit(trg)

    def update(self, ts, node):
        found = False
        for n in self.tsnodes:
            if n.node == node:
                n.ts = ts
                n.pos = node.pos
                found = True
                break

        if not found:
            self.tsnodes.append(TSNode(ts, node))

    def timeout(self):
        for n in self.tsnodes:
            if ts - n.ts > TIMEOUT:
                self.tsnodes.remove(n)

    def send(self, nodes):
        if ts - self.ts_com >= COM_FREQ + random.randrange(-COM_JITTER, COM_JITTER, 1):
            nds = set(nodes) - set([self])
            for n in nds:
                if vlen(vsub(n.pos, self.pos)) <= self.rng:
                    n.update(ts, self)
            self.ts_com = ts

class EnclosingCircle:
    def __init__(self, center = (0.0, 0.0), radius = 0.0):
        self.center = center
        self.radius = radius


def reset():
    global nodes, ts

    del nodes[:]
    del obstacles[:]

    ts = 0
    for _ in range(NUM_NODES):
        tx = random.randrange(-RANGE_X, RANGE_X, 1)
        ty = random.randrange(-RANGE_Y, RANGE_Y, 1)
        spd = random.randrange(10, 30, 1) / 10.0
        nodes.append(Node((tx, ty), COM_RANGE, (0.0, 0.0), spd, obstacles))

def transform(pos):
    return (int(SCREEN_CENTER[0] + pos[0] / RESOLUTION), int(SCREEN_CENTER[1] - pos[1] / RESOLUTION))

def rtransform(pos):
    return ((pos[0] - SCREEN_CENTER[0]) * RESOLUTION, -(pos[1] - SCREEN_CENTER[1]) * RESOLUTION)

class Painter(Thread):
    _COL_ROTORS = (127, 127, 127)
    _COL_ROTORS_MANUAL = (255, 255, 255)
    _COL_FRAME = (255, 255, 255)
    _COL_RANGE = (255, 255, 255)
    _COL_LINK = (255, 0, 0)
    _COL_BG = (0, 0, 0)
    _COL_SPEED = (0, 255, 0)
    _COL_SHADOW = (0, 0, 255)
    _COL_TEXT = (255, 255, 255)
    _COL_OBSTACLES_OUT = (255, 255, 0)
    _COL_OBSTACLES_IN = (0, 255, 255)

    def __init__(self, nodes, obstacles):
        Thread.__init__(self)
        self.setDaemon(True)

        pygame.font.init()
        self.nodes = nodes
        self.obstacles = obstacles
        SCREEN_CENTER = (SCREEN_SIZE[0] / 2.0, SCREEN_SIZE[1] / 2.0)
        self.screen_flags = pygame.HWSURFACE | pygame.DOUBLEBUF # | pygame.FULLSCREEN
        self.screen = pygame.display.set_mode(SCREEN_SIZE, self.screen_flags, 32)
        self.font_hud = pygame.font.Font(None, 24)
        self.font_dist = pygame.font.Font(None, 16)

    def _draw_nodes(self):
        for n in self.nodes:
            if not n.manual:
                col = self._COL_ROTORS
            else:
                col = self._COL_ROTORS_MANUAL
            pos = transform(n.pos)
            pygame.gfxdraw.line(self.screen, pos[0] - 16, pos[1], pos[0] + 16, pos[1], self._COL_FRAME)
            pygame.gfxdraw.line(self.screen, pos[0], pos[1] + 16, pos[0], pos[1] - 16, self._COL_FRAME)
            pygame.gfxdraw.filled_circle(self.screen, pos[0], pos[1] + 16, 8, col)
            pygame.gfxdraw.filled_circle(self.screen, pos[0] + 16, pos[1], 8, col)
            pygame.gfxdraw.filled_circle(self.screen, pos[0], pos[1] - 16, 8, col)
            pygame.gfxdraw.filled_circle(self.screen, pos[0] - 16, pos[1], 8, col)

    def _draw_shadows(self):
        for a in self.nodes:
            for b in a.tsnodes:
                pa, pb = transform(a.pos), transform(b.pos)
                pygame.gfxdraw.line(self.screen, pa[0], pa[1], pb[0], pb[1], self._COL_SHADOW)
                pygame.gfxdraw.filled_circle(self.screen, pb[0], pb[1], 4, self._COL_SHADOW)

    def _draw_ranges(self):
        for n in self.nodes:
            pos = transform(n.pos)
            pygame.gfxdraw.circle(self.screen, pos[0], pos[1], int(n.rng / RESOLUTION), self._COL_RANGE)

    def _draw_links(self):
        todraw = set([])
        for a in self.nodes:
            for b in a.tsnodes:
                todraw.add(ImmutableSet([a, b.node]))

        for a, b in todraw:
            pa, pb = transform(a.pos), transform(b.pos)
            pygame.gfxdraw.line(self.screen, pa[0], pa[1], pb[0], pb[1], self._COL_LINK)
            dist = self.font_dist.render('%.1fm' % vlen(vsub(b.pos, a.pos)), True, self._COL_TEXT)
            v = vsub(pb, pa)
            mid = vadd(pa, vscale(vnorm(v), vlen(v) / 2.0))
            self.screen.blit(dist, map(int, mid))


    def _draw_speeds(self):
        for n in nodes:
            trg = vadd(n.pos, vscale(n.trg, 10.0))
            a, b = transform(n.pos), transform(trg)
            pygame.gfxdraw.line(self.screen, a[0], a[1], b[0], b[1], self._COL_SPEED)

    def _draw_obstacles(self):
        for o in self.obstacles:
            pos = transform(o.pos)
            rad = int(o.radius / RESOLUTION)
            if o.mode == Obstacle.MODE_OUT:
                col = self._COL_OBSTACLES_OUT
            elif o.mode == Obstacle.MODE_IN:
                col = self._COL_OBSTACLES_IN
            pygame.gfxdraw.circle(self.screen, pos[0], pos[1], rad, col)

    def _draw_hud(self):
        time = self.font_hud.render('Time: %.2fs' % timestamp(), True, self._COL_TEXT)
        self.screen.blit(time, (0, 0))

    def run(self):
        last = time.time()
        while running:
            self.screen.fill(self._COL_BG)

            # self._draw_ranges()
            self._draw_obstacles()
            self._draw_shadows()
            self._draw_links()
            self._draw_nodes()
            self._draw_speeds()
            self._draw_hud()

            pygame.display.flip()

            now = time.time()
            dt = now - last
            last = now

            delay = 1.0/MAX_FPS - dt
            if delay > 0.0:
                time.sleep(delay)

class Input():
    def __init__(self, nodes, obstacles):
        self.nodes = nodes
        self.obstacles = obstacles
        self.grabbed = False
        self.dobstacle = False

    def input(self):
        global running, RESOLUTION

        for evt in pygame.event.get():
            if evt.type == pygame.locals.MOUSEBUTTONDOWN:
                mpos = rtransform(evt.pos)
                if pygame.mouse.get_pressed()[0]:
                    for n in self.nodes:
                        if vlen(vsub(mpos, n.pos)) < 16:
                            self.grabbed = n
                            n.manual = True
                if pygame.mouse.get_pressed()[1]:
                    obs = Obstacle(mpos, mode = Obstacle.MODE_IN)
                    self.obstacles.append(obs)
                    self.dobstacle = obs
                if pygame.mouse.get_pressed()[2]:
                    obs = Obstacle(mpos, mode = Obstacle.MODE_OUT)
                    self.obstacles.append(obs)
                    self.dobstacle = obs
            elif evt.type == pygame.locals.MOUSEBUTTONUP:
                if self.grabbed:
                    self.grabbed.manual = False
                    self.grabbed = False
                if self.dobstacle:
                    self.dobstacle = False
            elif evt.type == pygame.locals.MOUSEMOTION:
                if self.grabbed:
                    mpos = rtransform(evt.pos)
                    node = self.grabbed
                    trg = vscale(vsub(mpos, node.pos), 0.05)
                    if vlen(trg) > CUTOFF:
                        trg = vscale(vnorm(trg), CUTOFF)
                    node.trg = trg
                if self.dobstacle:
                    mpos = rtransform(evt.pos)
                    self.dobstacle.radius = vlen(vsub(mpos, self.dobstacle.pos))
            elif evt.type == pygame.locals.KEYDOWN:
                if evt.key == ord('r'):
                    reset()
                elif evt.key == ord('q'):
                    running = False
                elif evt.key == ord('-'):
                    RESOLUTION = 1.5 * RESOLUTION
                elif evt.key == ord('+'):
                    RESOLUTION = 2.0/3.0 * RESOLUTION

def send():
    for n in nodes:
        n.timeout()
        n.send(nodes)

def motion():
    for n in nodes:
        if not n.manual:
            n.swarm()
            n.avoid()
        n.move()

painter = Painter(nodes, obstacles)
painter.start()
input = Input(nodes, obstacles)

reset()

while running:
    send()
    motion()
    input.input()
    ts += SIMULATION_RESOLUTION
    #time.sleep(1)
