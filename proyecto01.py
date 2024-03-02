from collections import defaultdict, deque
import heapq
import time

# Leer el laberinto desde un archivo
def readMaze(filename):
    maze = []

    try:
        with open(filename, 'r') as archivo:
            for linea in archivo:
                fila = [int(c) for c in linea.strip()]
                maze.append(fila)
    except FileNotFoundError:
        print(f"El archivo {filename} no fue encontrado.")
    except Exception as e:
        print(f"Ocurrió un error al leer el archivo: {e}")

    return maze

maze = readMaze('test_maze.txt')

# Distancia euclidiana
def euclidean_distance(node, goal):
    x1, y1 = node
    x2, y2 = goal
    return ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5

# Distancia de Manhattan
def manhattan_distance(node, goal):
    x1, y1 = node
    x2, y2 = goal
    return abs(x1 - x2) + abs(y1 - y2)

# Encontrar la posición inicial y final del laberinto
def start_goal(maze):
    rows, cols = len(maze), len(maze[0])
    start, goal = None, None

    for i in range(rows):
        for j in range(cols):
            if maze[i][j] == 2:
                start = (i, j)
            elif maze[i][j] == 3:
                goal = (i, j)

    return start, goal

# Convertir el laberinto a un grafo
def toGraph(maze):
    graph = defaultdict(dict)
    rows, cols = len(maze), len(maze[0])

    for i in range(rows):
        for j in range(cols):
            if maze[i][j] != 1:
                neighbors = []
                if i > 0 and maze[i - 1][j] != 1:
                    neighbors.append((i - 1, j))
                if i < rows - 1 and maze[i + 1][j] != 1:
                    neighbors.append((i + 1, j))
                if j > 0 and maze[i][j - 1] != 1:
                    neighbors.append((i, j - 1))
                if j < cols - 1 and maze[i][j + 1] != 1:
                    neighbors.append((i, j + 1))

                graph[(i, j)] = {neighbor: 1 for neighbor in neighbors}

    return graph

# Algoritmo Breadth-First Search
def bfs(graph, start, goal, queue_structure):
    visited = set()
    queue = queue_structure()
    queue.append((start, [start]))

    while queue:
        node, path = queue.popleft()
        if node not in visited:
            visited.add(node)
            if node == goal:
                return path
            nodes = graph.get(node, [])
            for nextNode in nodes:
                if nextNode not in visited:
                    queue.append((nextNode, path + [nextNode]))

    return None

# Algoritmo Depth-First Search
def dfs(graph, node, goal, path=[], queue_structure=None):
    visited = set()
    stack = queue_structure()
    stack.append((node, path + [node]))

    while stack:
        node, path = stack.pop()
        if node not in visited:
            visited.add(node)
            if node == goal:
                return path
            nodes = graph.get(node, [])
            for nextNode in nodes:
                if nextNode not in visited:
                    stack.append((nextNode, path + [nextNode]))

    return None

# Algoritmo Depth-Delimited Search
def dds(graph, start, goal, depth_limit, queue_structure=None):
    visited = set()
    stack = queue_structure()
    stack.append((start, [start], 0))

    while stack:
        node, path, depth = stack.pop()
        if node not in visited and depth <= depth_limit:
            visited.add(node)
            if node == goal:
                return path
            nodes = graph.get(node, [])
            for nextNode in nodes:
                if nextNode not in visited:
                    stack.append((nextNode, path + [nextNode], depth + 1))

    return None


# Algoritmo Greedy Best-First Search
def gbfs(graph, start, goal, heuristic, queue_structure):
    visited = set()
    priority_queue = []
    heapq.heappush(priority_queue, (heuristic(start, goal), start, []))

    while priority_queue:
        _, node, path = heapq.heappop(priority_queue)
        if node not in visited:
            visited.add(node)
            if node == goal:
                return path
            neighbors = graph.get(node, [])
            for nextNode in neighbors:
                if nextNode not in visited:
                    heapq.heappush(priority_queue, (heuristic(nextNode, goal), nextNode, path + [nextNode]))

    return None

# Algoritmo A* Search
def astar(graph, start, goal, heuristic, queue_structure):
    visited = set()
    priority_queue = []
    heapq.heappush(priority_queue, (heuristic(start, goal), 0, start, []))

    while priority_queue:
        _, cost, node, path = heapq.heappop(priority_queue)
        if node not in visited:
            visited.add(node)
            if node == goal:
                return path
            neighbors = graph.get(node, {})
            for nextNode, edge_cost in neighbors.items():
                if nextNode not in visited:
                    heapq.heappush(priority_queue, (heuristic(nextNode, goal) + cost + edge_cost, cost + edge_cost, nextNode, path + [nextNode]))

    return None


graph = toGraph(maze)
start, goal = start_goal(maze)

fifo = deque
lifo = list
priority = heapq

# BFS
start_time = time.time()
resultBFS = bfs(graph, start, goal, fifo)
end_time = time.time()
print('Breadth-first search:')
print('Path:', resultBFS)
print('Time:', end_time - start_time, 'seconds')
print('Steps:', len(resultBFS) - 1 if resultBFS else 0)

if resultBFS:
    for step in resultBFS:
        x, y = step
        maze[x][y] = 9

    '''
    print('Laberinto resuelto por BFS:')
    for row in maze:
        print(row)
    '''
else:
    print('No se encontró solución para el laberinto.')


# DFS
start_time = time.time()
resultDFS = dfs(graph, start, goal, queue_structure=lifo)
end_time = time.time()
print('\nDepth-first search:')
print('Path:', resultDFS)
print('Time:', end_time - start_time, 'seconds')
print('Steps:', len(resultDFS) - 1 if resultDFS else 0)

if resultDFS:
    for step in resultDFS:
        x, y = step
        maze[x][y] = 9

    '''
    print('Laberinto resuelto por DFS:')
    for row in maze:
        print(row)
    '''
else:
    print('No se encontró solución para el laberinto.')


# DDS
start_time = time.time()
resultDDS = dds(graph, start, goal, 1000, queue_structure=lifo)
end_time = time.time()
print('\nDepth-delimited search:')
print('Path:', resultDDS)
print('Time:', end_time - start_time, 'seconds')
print('Steps:', len(resultDDS) - 1 if resultDDS else 0)

if resultDDS:
    for step in resultDDS:
        x, y = step
        maze[x][y] = 9

    '''
    print('Laberinto resuelto por DDS:')
    for row in maze:
        print(row)
    '''
else:
    print('No se encontró solución para el laberinto.')


# GBFS
start_time = time.time()
resultGBFS = gbfs(graph, start, goal, euclidean_distance, priority)
end_time = time.time()
print('\nGreedy best-first search:')
print('Path:', resultGBFS)
print('Time:', end_time - start_time, 'seconds')
print('Steps:', len(resultGBFS) - 1 if resultGBFS else 0)

if resultGBFS:
    for step in resultGBFS:
        x, y = step
        maze[x][y] = 9

    '''
    print('Laberinto resuelto por GBFS:')
    for row in maze:
        print(row)
    '''
else:
    print('No se encontró solución para el laberinto.')


# A*
start_time = time.time()
resultAStar = astar(graph, start, goal, manhattan_distance, priority)
end_time = time.time()
print('\nA* search:')
print('Path:', resultAStar)
print('Time:', end_time - start_time, 'seconds')
print('Steps:', len(resultAStar) - 1 if resultAStar else 0)

if resultAStar:
    for step in resultAStar:
        x, y = step
        maze[x][y] = 9

    '''
    print('Laberinto resuelto por A*:')
    for row in maze:
        print(row)
    '''
else:
    print('No se encontró solución para el laberinto.')

