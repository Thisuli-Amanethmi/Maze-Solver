import copy
import heapq  # priority queue


def create_maze(columns, rows):
    maze = [[' - ' for _ in range(columns)] for _ in range(rows)]

    # start
    start_column = 1
    start_row = 2
    maze[start_column][start_row] = ' S '

    # goal
    goal_column = 4
    goal_row = 3
    maze[goal_column][goal_row] = ' G '

    # 4 barriers in remaining cells
    maze[1][0] = ' | '  # barrier 1
    maze[3][1] = ' | '  # barrier 2
    maze[3][4] = ' | '  # barrier 3
    maze[5][1] = ' | '  # barrier 4

    print("Maze")
    display_maze(maze)

    return maze


def display_maze(maze):
    for i in range(len(maze)):  # no of rows
        for j in range(len(maze[0])):  # no of columns
            print(maze[j][i], end="")
        print()


def neighbours_sort(current, maze):
    # order: left-up, left, left-down, up, down, right-up, right, right-down
    directions = [(-1, -1), (-1,0), (-1, 1), (0,-1), (0,1), (1, -1), (1,0), (1, 1)] # (col, row)
    neighbours = []

    for dx, dy in directions:
        new_x, new_y = current[0] + dx, current[1] + dy
        if 0 <= new_x < len(maze) and 0 <= new_y < len(maze[0]) and maze[new_x][new_y] != ' | ':
            neighbours.append((new_x, new_y))
            # print(new_x, new_y)

    # sorting neighbours in increasing order
    neighbours.sort(key=lambda x: (x[0], x[1]))
    # print("Neighbours: ", neighbours)
    return neighbours


def dfs_search(maze):
    maze_dfs = copy.deepcopy(maze)

    print("DFS search")
    stack = []
    visited_nodes = []
    # print("v:", visited_nodes)
    parent = {}

    start = None
    goal = None
    goal_found = False

    for i in range(len(maze_dfs)):
        for j in range(len(maze_dfs[0])):
            if maze_dfs[i][j] == ' S ':
                start = (i, j)
            elif maze_dfs[i][j] == ' G ':
                goal = (i, j)

    while not goal_found:
        stack.append(start)
        parent[start] = None

        while stack:
            # print("Stack: ", stack)
            current = stack.pop()  # visiting to the top element in the stack
            if current not in visited_nodes:
                visited_nodes.append(current)  # adding it to the visited list
            # print("v2: ", visited_nodes)

            if current == goal:
                goal_found = True
                print("Goal found !!!")
                break

            # getting current's neighbours
            neighbours = neighbours_sort(current, maze_dfs)
            # adding un-visited neighbours of the current, to the stack
            for neighbour in neighbours:
                if neighbour not in visited_nodes and neighbour not in stack:
                    stack.append(neighbour)
                    parent[neighbour] = current
            # print("stack: ", stack)

        if goal not in visited_nodes:
            print("Goal is not reachable :(")
            return

        # backtrack to find the path
        path = []
        while current is not None:
            path.append(current)
            current = parent[current]
        path.reverse()

        for cell in path:
            if cell != start and cell != goal:
                maze_dfs[cell[0]][cell[1]] = ' * '

        display_maze(maze_dfs)

        print("Visited nodes: ", visited_nodes)
        print("Time to find the goal in minutes: ", len(visited_nodes))
        print("Final Path: ", path)
        print()

    return path


def calculate_heuristic_cost(current_node, goal_node):
    heuristic_cost = abs(current_node[0] - goal_node[0]) + abs(current_node[1] - goal_node[1])
    # print("Heuristic cost: ", heuristic_cost)
    return heuristic_cost


def valid_successors(current_node, maze, visited_nodes):
    # order: left-up, left, left-down, up, down, right-up, right, right-down
    directions = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]  # (col, row)
    successors = []

    for dx, dy in directions:
        new_x, new_y = current_node[0] + dx, current_node[1] + dy
        if 0 <= new_x < len(maze) and 0 <= new_y < len(maze[0]) and maze[new_x][new_y] != ' | ':
            if maze[new_x][new_y] not in visited_nodes:
                successors.append((new_x, new_y))
                # print(new_x, new_y)

    # sorting neighbours in increasing order
    successors.sort(key=lambda x: (x[0], x[1]))
    # print("Successors: ", successors)
    return successors


def a_star_search(maze):
    print("A* Search")
    maze_a_star = copy.deepcopy(maze)

    start = None
    goal = None
    # getting start and goal nodes
    for i in range(len(maze_a_star)):
        for j in range(len(maze_a_star[0])):
            if maze_a_star[j][i] == ' S ':
                start = (j, i)
            elif maze_a_star[j][i] == ' G ':
                goal = (j, i)

    heap = []
    # adding start node into the heap
    heapq.heappush(heap, (calculate_heuristic_cost(start, goal), 0, start, None))  # (cost, node, path)
    visited_nodes = set()
    parents = {start: None}

    while heap:
        fn, gn, current_node, parent = heapq.heappop(heap)  # popping the cell with the lowest fn value

        if current_node == goal:
            print("Goal found !!!")

            path = []
            while current_node is not None:
                path.append(current_node)
                current_node = parents[current_node]
            path.reverse()

            for cell in path:
                if cell != start and cell != goal:
                    maze_a_star[cell[0]][cell[1]] = ' * '
            display_maze(maze_a_star)

            print("Visited nodes: ", visited_nodes)
            print("Time to find the goal in minutes: ", len(visited_nodes))
            print("Final Path: ", path)
            return path

        # otherwise
        visited_nodes.add(current_node)

        # getting successors
        successors = valid_successors(current_node, maze_a_star, visited_nodes)
        for successor in successors:
            new_fn = gn + 1 + calculate_heuristic_cost(successor, goal)
            new_gn = gn + 1

            if successor not in parents or new_gn < gn:
                parents[successor] = current_node
                heapq.heappush(heap, (new_fn, new_gn, successor, current_node))

    print("Goal is not reachable :(")
    return None  # no path found


columns = 6
rows = 6


def main():
    maze = create_maze(columns, rows)
    print()
    dfs_search(maze)
    print()
    a_star_search(maze)


if __name__ == "__main__":
    main()

