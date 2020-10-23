import pygame
import math
from queue import PriorityQueue, Queue, LifoQueue


WIDTH = 800
pygame.init()
display = pygame.display.set_mode((WIDTH, WIDTH))
pygame.display.set_caption("Path Finding Algorithms Visualization")

RED = (255, 0, 0) # closed
GREEN = (0, 255, 0) # open
BLUE = (0, 255, 0)
YELLOW = (255, 255, 0)
WHITE = (255, 255, 255)
BLACK = (0, 0, 0) # barrier
PURPLE = (128, 0, 128) # end
ORANGE = (255, 165 ,0) # start
GREY = (128, 128, 128)
TURQUOISE = (64, 224, 208) # path


class Node:
    def __init__(self, row, col, width, total_rows):
        self.row = row
        self.col = col
        self.x = row * width
        self.y = col * width
        self.total_rows = total_rows
        self.color = WHITE
        self.width = width
        self.neighbours = []

    def get_pos(self):
        return self.row, self.col

    def is_visited(self):
        return self.color == RED

    def is_white(self):
        return self.color == WHITE

    def is_end(self):
        return self.color == PURPLE

    def is_start(self):
        return self.color == ORANGE

    def is_open(self):
        return self.color == GREEN

    def get_neighbours(self):
        return self.neighbours

    def is_barrier(self):
        return self.color == BLACK

    def reset(self):
        self.color = WHITE

    def make_start(self):
        self.color = ORANGE

    def make_end(self):
        self.color = PURPLE

    def make_barrier(self):
        self.color = BLACK

    def make_path(self):
        self.color = TURQUOISE

    def make_visited(self):
        self.color = RED

    def make_open(self):
        self.color = GREEN

    def draw(self, display):
        pygame.draw.rect(display, self.color, (self.x, self.y, self.width, self.width))

    def update_neighbours(self, grid):
        self.neighbours = []

        if self.row + 1 < self.total_rows and not grid[self.row + 1][self.col].is_barrier():
            self.neighbours.append(grid[self.row + 1][self.col])

        if self.row - 1 > 0 and not grid[self.row - 1][self.col].is_barrier():
            self.neighbours.append(grid[self.row - 1][self.col])

        if self.col + 1 < self.total_rows and not grid[self.row][self.col + 1].is_barrier():
            self.neighbours.append(grid[self.row][self.col + 1])

        if self.col - 1 > 0 and not grid[self.row][self.col - 1].is_barrier():
            self.neighbours.append(grid[self.row][self.col - 1])


def h(p1, p2):
    x1, y1 = p1
    x2, y2 = p2
    return abs(x1 - x2) + abs(y1 - y2)


def reconstruct_path(path, current, draw):
    while current in path:
        current = path[current]
        current.make_path()
        draw()


def reconstruct_path_BD(path, current, draw):
    while current in path:
        current.make_path()
        current = path[current]
        draw()


def BIDIRECTIONAL_BFS(draw, start, end, grid):
    path_start = {}
    path_end = {}
    open_set_start = Queue()
    open_set_start_hash = { start }
    open_set_start.put(start)
    open_set_end = Queue()
    open_set_end_hash = { end }
    open_set_end.put(end)

    while not open_set_end.empty() and not open_set_start.empty():
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit

        if not open_set_start.empty():
            node_start = open_set_start.get()

            if node_start == end or node_start in open_set_end_hash:
                reconstruct_path_BD(path_start, node_start, draw)
                reconstruct_path_BD(path_end, node_start, draw)
                start.make_start()
                end.make_end()
                return True

            for neighbour in node_start.get_neighbours():
                if neighbour not in open_set_start_hash:
                    path_start[neighbour] = node_start
                    open_set_start_hash.add(neighbour)
                    open_set_start.put(neighbour)
                    neighbour.make_open()

        if not open_set_end.empty():
            node_end = open_set_end.get()

            if node_end == start or node_end in open_set_start_hash:
                reconstruct_path_BD(path_end, node_end, draw)
                reconstruct_path_BD(path_start, node_end, draw)
                start.make_start()
                end.make_end()
                return True

            for neighbour in node_end.get_neighbours():
                if neighbour not in open_set_end_hash:
                    path_end[neighbour] = node_end
                    open_set_end.put(neighbour)
                    open_set_end_hash.add(neighbour)
                    neighbour.make_open()

        draw()

        if start != node_start:
            node_start.make_visited()

        if end != node_end:
            node_end.make_visited()

    return False


def BIDIRECTIONAL_DIJKSTRA(draw, start, end, grid):
    path_start = {}
    path_end = {}
    open_set_start = PriorityQueue()
    open_set_end = PriorityQueue()
    open_set_start_hash = { start }
    open_set_end_hash = { end }
    count_start = 0
    count_end = 0
    distance_start = {node: float('inf') for row in grid for node in row}
    distance_end = {node: float('inf') for row in grid for node in row}
    distance_start[start] = 0
    distance_end[end] = 0
    open_set_start.put((0, count_start, start))
    open_set_end.put((0, count_end, end))

    while not open_set_start.empty() and not open_set_end.empty():
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()

        if not open_set_start.empty():
            node_start = open_set_start.get()[2]
            open_set_start_hash.remove(node_start)

            if node_start == end or node_start in open_set_end_hash:
                reconstruct_path_BD(path_start, node_start, draw)
                reconstruct_path_BD(path_end, node_start, draw)
                start.make_start()
                end.make_end()
                return True

            for neighbour in node_start.get_neighbours():
                temp_distance = distance_start[node_start] + 1
                if temp_distance < distance_start[neighbour]:
                    distance_start[neighbour] = temp_distance
                    path_start[neighbour] = node_start
                    if node_start not in open_set_start_hash:
                        count_start += 1
                        open_set_start.put((distance_start[neighbour], count_start, neighbour))
                        open_set_start_hash.add(neighbour)
                        neighbour.make_open()

            if node_start != start:
                node_start.make_visited()

        if not open_set_end.empty():
            node_end = open_set_end.get()[2]
            open_set_end_hash.remove(node_end)

            if node_end == start or node_end in open_set_start_hash:
                reconstruct_path_BD(path_end, node_end, draw)
                reconstruct_path_BD(path_start, node_end, draw)
                start.make_start()
                end.make_end()
                return True

            for neighbour in node_end.get_neighbours():
                temp_distance = distance_end[node_end] + 1
                if temp_distance < distance_end[neighbour]:
                    distance_end[neighbour] = temp_distance
                    path_end[neighbour] = node_end
                    if neighbour not in open_set_end_hash:
                        count_end += 1
                        open_set_end.put((distance_end[neighbour], count_end, neighbour))
                        open_set_end_hash.add(neighbour)
                        neighbour.make_open()

            if node_end != end:
                node_end.make_visited()

        draw()

    return False


def BIDIRECTIONAL_A_STAR(draw, start, end, grid):
    path_start = {}
    path_end = {}
    open_set_start = PriorityQueue()
    open_set_end = PriorityQueue()
    open_set_start_hash = { start }
    open_set_end_hash = { end }
    count_start = 0
    count_end = 0
    g_score_start = {node: float('inf') for row in grid for node in row}
    g_score_end = {node: float('inf') for row in grid for node in row}
    f_score_start = {node: float('inf') for row in grid for node in row}
    f_score_end = {node: float('inf') for row in grid for node in row}
    g_score_start[start] = 0
    g_score_end[end] = 0
    f_score_start[start] = h(start.get_pos(), end.get_pos())
    f_score_end[end] = h(start.get_pos(), end.get_pos())
    open_set_start.put((f_score_start[start], count_start, start))
    open_set_end.put((f_score_end[end], count_end, end))

    while not open_set_start.empty() and not open_set_end.empty():
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()

        if not open_set_start.empty():
            node_start = open_set_start.get()[2]
            open_set_start_hash.remove(node_start)

            if node_start == end or node_start in open_set_end_hash:
                reconstruct_path_BD(path_start, node_start, draw)
                reconstruct_path_BD(path_end, node_start, draw)
                start.make_start()
                end.make_end()
                return True

            for neighbour in node_start.get_neighbours():
                temp_g_score = g_score_start[node_start] + 1
                if temp_g_score < g_score_start[neighbour]:
                    g_score_start[neighbour] = temp_g_score
                    path_start[neighbour] = node_start
                    f_score_start[neighbour] = temp_g_score + h(node_start.get_pos(), end.get_pos())
                    if neighbour not in open_set_start_hash:
                        count_start += 1
                        open_set_start.put((f_score_start[neighbour], count_start, neighbour))
                        open_set_start_hash.add(neighbour)
                        neighbour.make_open()


            if node_start != start:
                node_start.make_visited()

        if not open_set_end.empty():
            node_end = open_set_end.get()[2]
            open_set_end_hash.remove(node_end)

            if node_end == start or node_end in open_set_start_hash:
                reconstruct_path_BD(path_end, node_end, draw)
                reconstruct_path_BD(path_start, node_end, draw)
                start.make_start()
                end.make_end()
                return True

            for neighbour in node_end.get_neighbours():
                temp_g_score = g_score_end[node_end] + 1
                if temp_g_score < g_score_end[neighbour]:
                    g_score_end[neighbour] = temp_g_score
                    f_score_end[neighbour] = temp_g_score + h(node_end.get_pos(), start.get_pos())
                    path_end[neighbour] = node_end
                    if neighbour not in open_set_end_hash:
                        count_end += 1
                        open_set_end.put((f_score_end[neighbour], count_end, neighbour))
                        open_set_end_hash.add(neighbour)
                        neighbour.make_open()

            if node_end != end:
                node_end.make_visited()

        draw()

    return False


def BFS(draw, start, end, grid):
    path = {}
    open_set = Queue()
    open_set.put(start)

    while not open_set.empty():
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()

        node = open_set.get()

        if node == end:
            reconstruct_path(path, end, draw)
            start.make_start()
            end.make_end()
            return True

        for neighbour in node.get_neighbours():
            if neighbour.is_white() or neighbour == end:
                path[neighbour] = node
                open_set.put(neighbour)
                neighbour.make_open()

        draw()

        if node != start:
            node.make_visited()

    return False


def DFS(draw, start, end, grid):
    path = {}
    open_set = LifoQueue()
    open_set.put(start)

    while not open_set.empty():
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()

        node = open_set.get()

        if node == end:
            reconstruct_path(path, end, draw)
            end.make_end()
            start.make_start()
            return True

        for neighbour in node.get_neighbours():
            if neighbour.is_white() or neighbour == end:
                path[neighbour] = node
                open_set.put(neighbour)
                neighbour.make_open()

        draw()

        if node != start:
            node.make_visited()

    return False


def GREEDY_BEST_FIRST_SEARCH(draw, start, end, grid):
    open_set = PriorityQueue()
    f_score = {node: float('inf') for row in grid for node in row}
    f_score[start] = h(start.get_pos(), end.get_pos())
    count = 0
    open_set.put((f_score[start], count, start))
    open_set_hash = {start}
    path = {}
    visited_hash = set()

    while not open_set.empty():
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()

        node = open_set.get()[2]
        open_set_hash.remove(node)
        visited_hash.add(node)

        if node == end:
            reconstruct_path(path, end, draw)
            end.make_end()
            start.make_start()
            return True

        for neighbour in node.get_neighbours():
            f_score[neighbour] =h(neighbour.get_pos(), end.get_pos())
            if neighbour not in open_set_hash and neighbour not in visited_hash:
                path[neighbour] = node
                count += 1
                open_set.put((f_score[neighbour], count, neighbour))
                open_set_hash.add(neighbour)
                neighbour.make_open()

        draw()

        if node != start:
            node.make_visited()

    return False


def DIJKSTRA(draw, start, end, grid):
    open_set = PriorityQueue()
    open_set_hash = { start }
    count = 0
    open_set.put((0, count, start))
    path = {}
    distance = {node: float('inf') for row in grid for node in row}
    distance[start] = 0

    while not open_set.empty():
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()

        node = open_set.get()[2]
        open_set_hash.remove(node)

        if node == end:
            reconstruct_path(path, end, draw)
            end.make_end()
            start.make_start()
            return True

        for neighbour in node.get_neighbours():
            temp_dist = distance[node] + 1
            if temp_dist < distance[neighbour]:
                distance[neighbour] = temp_dist
                path[neighbour] = node
                if neighbour not in open_set_hash:
                    neighbour.make_open()
                    count += 1
                    open_set.put((distance[neighbour], count, neighbour))
                    open_set_hash.add(neighbour)

        draw()

        if node != start:
            node.make_visited()

    return False


def A_STAR(draw, start, end, grid):
    open_set = PriorityQueue()
    g_score = {node: float('inf') for row in grid for node in row}
    g_score[start] = 0
    f_score = {node: float('inf') for row in grid for node in row}
    f_score[start] = h(start.get_pos(), end.get_pos())
    count = 0
    open_set.put((f_score[start], count, start))
    open_set_hash = { start }
    path = {}

    while not open_set.empty():
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()

        node = open_set.get()[2]
        open_set_hash.remove(node)

        if node == end:
            reconstruct_path(path, end, draw)
            end.make_end()
            start.make_start()
            return True

        for neighbour in node.get_neighbours():
            temp_g_score = g_score[node] + 1
            if temp_g_score < g_score[neighbour]:
                g_score[neighbour] = temp_g_score
                f_score[neighbour] = temp_g_score + h(neighbour.get_pos(), end.get_pos())
                path[neighbour] = node
                if neighbour not in open_set_hash:
                    count += 1
                    open_set.put((f_score[neighbour], count, neighbour))
                    open_set_hash.add(neighbour)
                    neighbour.make_open()

        draw()

        if node != start:
            node.make_visited()

    return False

def A_STAR_OPTIMIZED(draw, start, end, grid):
    open_set = set([start])
    closed_set = set()
    g_score = {node: float('inf') for row in grid for node in row}
    g_score[start] = 0
    f_score = {node: float('inf') for row in grid for node in row}
    f_score[start] = h(start.get_pos(), end.get_pos())
    path = {}

    while open_set:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()

        node = min(open_set, key=lambda x: f_score[x])
        open_set.discard(node)
        closed_set.add(node)

        if node == end:
            reconstruct_path(path, end, draw)
            end.make_end()
            start.make_start()
            return True

        for neighbour in node.get_neighbours():
            temp_g_score = g_score[node] + 1
            if temp_g_score < g_score[neighbour] and neighbour not in closed_set:
                open_set.add(neighbour)
                g_score[neighbour] = temp_g_score
                f_score[neighbour] = temp_g_score + h(neighbour.get_pos(), end.get_pos())
                path[neighbour] = node
                neighbour.make_open()

        draw()

        if node != start:
            node.make_visited()

    return False


def make_grid(rows, width):
    grid = []
    node_width = width // rows
    for i in range(rows):
        grid.append([])
        for j in range(rows):
            node = Node(i, j, node_width, rows)
            grid[i].append(node)

    return grid

def draw_grid(display, grid, rows, width):
    node_width = width // rows
    for i in range(rows):
        pygame.draw.line(display, GREY, (0, i * node_width), (width, i * node_width))

    for i in range(rows):
        pygame.draw.line(display, GREY, (i * node_width, 0), (i * node_width, width))


def draw(display, grid, rows, width):
    display.fill(WHITE)

    for row in grid:
        for node in row:
            node.draw(display)

    draw_grid(display, grid, rows, width)
    pygame.display.update()


def get_pos_clicked(pos, rows, width):
    node_width = width // rows
    y, x = pos

    row = y // node_width
    col = x // node_width

    return row, col


def main(win, width):
    ROWS = 50
    node_width = width // ROWS

    start, end = None, None
    grid = make_grid(ROWS, width)

    run = True
    flag = False
    solution = True

    while run:
        draw(display, grid, ROWS, width)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False

            if pygame.mouse.get_pressed()[0]:
                pos = pygame.mouse.get_pos()
                row, col = get_pos_clicked(pos, ROWS, width)
                node = grid[row][col]
                if not start and node != end:
                    start = node
                    start.make_start()

                elif not end and node != start:
                    end = node
                    end.make_end()

                elif node != start and node != end:
                    node.make_barrier()

            elif pygame.mouse.get_pressed()[2]:
                pos = pygame.mouse.get_pos()
                row, col = get_pos_clicked(pos, ROWS, width)
                node = grid[row][col]
                node.reset()

                if node == start:
                    start = None

                elif node == end:
                    end = None

            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE and start and end:
                    flag = True
                    for row in grid:
                        for node in row:
                            node.update_neighbours(grid)

                if event.key == pygame.K_KP1 and start and end:
                     pygame.display.set_caption("BIDIRECTIONAL_BFS PATH VISUALIZATION")
                     solution = BIDIRECTIONAL_BFS(lambda: draw(win, grid, ROWS, width), start, end, grid)

                elif event.key == pygame.K_KP2 and flag and start and end:
                     pygame.display.set_caption("BIDIRECTIONAL_DIJKSTRA PATH VISUALIZATION")
                     solution = BIDIRECTIONAL_DIJKSTRA(lambda: draw(win, grid, ROWS, width), start, end, grid)

                elif event.key == pygame.K_KP3 and flag and start and end:
                    pygame.display.set_caption("BIDIRECTIONAL_A_STAR PATH VISUALIZATION")
                    solution = BIDIRECTIONAL_A_STAR(lambda: draw(win, grid, ROWS, width), start, end, grid)

                elif event.key == pygame.K_KP4 and flag and start and end:
                     pygame.display.set_caption("BFS PATH VISUALIZATION")
                     solution = BFS(lambda: draw(win, grid, ROWS, width), start, end, grid)

                elif event.key == pygame.K_KP5 and flag and start and end:
                     pygame.display.set_caption("DFS PATH VISUALIZATION")
                     solution = DFS(lambda: draw(win, grid, ROWS, width), start, end, grid)

                elif event.key == pygame.K_KP6 and flag and start and end:
                     pygame.display.set_caption("GREEDY_BEST_FIRST_SEARCH PATH VISUALIZATION")
                     solution = GREEDY_BEST_FIRST_SEARCH(lambda: draw(win, grid, ROWS, width), start, end, grid)

                elif event.key == pygame.K_KP7 and flag and start and end:
                     pygame.display.set_caption("DIJKSTRA PATH VISUALIZATION")
                     solution = DIJKSTRA(lambda: draw(win, grid, ROWS, width), start, end, grid)

                elif event.key == pygame.K_KP8 and flag and start and end:
                     pygame.display.set_caption("A_STAR PATH VISUALIZATION")
                     solution = A_STAR(lambda: draw(win, grid, ROWS, width), start, end, grid)

                elif event.key == pygame.K_KP9 and flag and start and end:
                     pygame.display.set_caption("OPTIMIZED A_STAR PATH VISUALIZATION")
                     solution = A_STAR_OPTIMIZED(lambda: draw(win, grid, ROWS, width), start, end, grid)

                if solution == False:
                    pygame.display.set_caption("NO PATH FOUND :(")

                if event.key == pygame.K_c:
                    start = None
                    end = None
                    grid = make_grid(ROWS, width)

    pygame.quit()

main(display, WIDTH)
