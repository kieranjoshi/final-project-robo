from queue import PriorityQueue
from dataclasses import dataclass, field
from typing import Any
import numpy as np
# Prio queue compare prio only
@dataclass(order=True) 
class PrioritizedItem:
    priority: int
    item: Any=field(compare=False)

# 3D hash table ReservationTable to keep track of Agent state
class ReservationTable:
    def __init__(self):
        self.table = {}

    def reserve(self, x, y, time, agent_id):
        self.table[(x, y, time)] = agent_id
    
    def is_reserved(self, x, y, time):
        return (x, y, time) in self.table
    
# The agent start & goal positions in 2d space
class Agent:
    def __init__(self, id, start, goal):
        self.id = id
        self.start = start
        self.goal = goal

class Position:
    def __init__(self, x ,y):
        self.x = x
        self.y = y
    
    # Equality operator (==) overide comparison between position objects 
    def __eq__(self, other):
        return self.x == other.x and self.y == other.y

# Available actions 
# Movement 
class Action:
    MOVE_UP = (0, 1)
    MOVE_RIGHT = (1, 0)
    MOVE_LEFT = (-1, 0)
    MOVE_DOWN = (0, -1)
    WAIT = (0, 0)

    @classmethod
    def get_actions(cls):
        return [cls.MOVE_UP, cls.MOVE_DOWN, cls.MOVE_LEFT, cls.MOVE_RIGHT, cls.WAIT]


# Node object to keep track of visited cells to be reconstructed later
class Node:
    def __init__(self, priority, position, time, cost, parent=None):
        priority = priority
        self.x = position.x
        self.y = position.y
        self.time = time
        self.cost = cost
        self.parent = parent #Node pointer to parent node

    def calculate_f_cost(self):
        self.f_cost = self.g_cost + self.h_cost


def reconstruct_path(node):
    path = []
    current = node
    while current:
        path.append((Position(current.x, current.y), current.time))
        current = current.parent
    return list(reversed(path))

# Manhattan distance heuristic
def manhattan_distance(pos1, pos2):
    return abs(pos1.x - pos2.x) + abs(pos1.y - pos2.y)

# TODO: Implimented this heuristic function
# Source - https://github.com/balzer82/3D-OccupancyGrid-Python/blob/master/3D-AStar-PathPlanning.ipynb
# Heuristic berechnen
# def calcheuristic(grid,goal):

#     for z in range(zdim):
#         for y in range(ydim):
#             for x in range(xdim):
                 
#                 # Euklidische Distanz für jede Zelle zum Ziel berechnen
#                 dist=((x-goal[0])**2+(y-goal[1])**2+(z-goal[2])**2)**(1/2.0)
            
#                 # Höhe
#                 zheu = -6.0*float(z)
                
#                 # Horizontale von Soll
#                 yheu = np.abs(float(y) - goal[1])
                
#                 # und Höhe und Abweichung von y=0
#                 heuristic[x,y,z]= dist + yheu #+ zheu
#     '''     
#     for i in range(len(heuristic)):
#         print(heuristic[i])
#     '''
#     return heuristic


# TODO: Impliment robot state standstill to fill future reservation slots
# Source- https://www.davidsilver.uk/wp-content/uploads/2020/03/coop-path-AIIDE.pdf
# Cooperative A* - A decoupled approach with each agent's path calculated one at a time
# Each path's route passed into a 3D hash table containing a route's position 
# & at what time <X,Y,time>. Entries in the table are considered impassable by subsequent
# agent searches.
# Encountered issues:
# - When robot reaches goal, subquent time intervals lack the information of this end state (fixed)
# - With end state persistentence, there are certain agent-goal configurations where later agents are blocked off from their goal
#       - Leads to a "lost" agent
# - If two adjacent agents attempt to travel to eachother's cell at the same time interval, this allows them to "phase through eachother"
                                     
def cooperative_astar(agent_list, grid):
    print("In CA* \n")
    reservationTable = ReservationTable()
    paths = {}
    

    # class Agent:
    #   def __init__(self, id, start, goal):
    #     self.id = id
    #     self.start = start
    #     self.goal = goal
    for agent in agent_list:
        print("Agent ID [" + str(agent.id) + "]\n")
        start = agent.start
        goal = agent.goal
        openlist = PriorityQueue(maxsize = 0)
        closedlist = set()

        # --Node members--
        # priority = priority
        # self.x = position.x
        # self.y = position.y
        # self.time = time
        # self.cost = cost
        # self.parent = parent

        # Initial node
        start_node = Node(
            priority = 0,
            position = start,
            time = 0,
            cost = 0
        )
        # Wrap the start node in PrioritizedItem
        openlist.put(PrioritizedItem(priority=0, item=start_node))

        # Node initializer
        node_count = 0
        while not openlist.empty():
            #Current type - Node item
            current_item = openlist.get()
            current = current_item.item

            node_count += 1

            print("t= " + str(current.time) + " | Current Node #: " + str(node_count) + " at [" + str(current.x) + " , " + str(current.y) 
                  + "] - Cost: " + str(current.cost))

            #print(current)


            # Reconstruct path and store after path calculation
            if Position(current.x, current.y) == goal:
                paths[agent] = reconstruct_path(current)
                for pos, time in paths[agent]:
                    reservationTable.reserve(pos.x, pos.y, time, agent.id)
                
                last_pos = paths[agent][-1]

                for i in range(100):
                    reservationTable.reserve(last_pos[0].x,last_pos[0].y,last_pos[1]+i,agent.id)

                
                break
            
            # Skip if already visited
            current_pos = (current.x, current.y, current.time)
            if current_pos in closedlist:
                continue
            # And add to closed list if not
            closedlist.add(current_pos)

            # Consider all movement options including wait()
            for action in Action.get_actions():
                next_x = current.x + action[0]
                next_y = current.y + action[1]
                next_time = current.time + 1

                reserve_check = reservationTable.is_reserved(next_x, next_y, next_time)
                if (reserve_check):
                    print("Encountered reserved cell at: <%s, %s, time=%s>" % (str(next_x), str(next_y), str(next_time)))

                if (0 <= next_x < len(grid) and 
                    0 <= next_y < len(grid[0]) and 
                    grid[next_x][next_y] == 0 and # Check if within valid bounds
                    not reservationTable.is_reserved(next_x, next_y, next_time)):

                    # g-cost (cost from start)
                    new_cost = current.cost + 1

                    # h-cost (heuristic estimate to goal)
                    heuristic = manhattan_distance(
                        Position(next_x, next_y),
                        agent.goal
                    )
                    # f-cost (total estimated cost: f = g + h)
                    priority = new_cost + heuristic
                

                    next_node = Node(
                        priority = priority,
                        position = Position(next_x, next_y),
                        time = next_time,
                        cost = new_cost,
                        parent = current #
                    )

                    openlist.put(PrioritizedItem(priority = priority, item = next_node))

    return paths


agents = [
    Agent(1, Position(3, 0), Position(3, 6)),
    Agent(2, Position(0, 3), Position(6, 3))
]
grid = np.zeros([7,7])
grid[3][3] = 1
complex_grid = np.zeros([8,8])
complex_agents = [
        Agent(1, Position(3, 0), Position(3, 7)),
        Agent(2, Position(0, 0), Position(7, 7)),
        Agent(3, Position(7, 0), Position(0, 7)),
        Agent(4, Position(2, 2), Position(5, 5)),
        Agent(5, Position(5, 3), Position(1, 6))
    ]



def sortfunc(agent):
    return manhattan_distance(agent.start,agent.goal)

complex_agents = sorted(complex_agents,key=sortfunc)
# Find paths
# Identify prohibited positions (start and goal positions of agents)
prohibited_positions = set()
for agent in complex_agents:
    prohibited_positions.add((agent.start.x, agent.start.y))
    prohibited_positions.add((agent.goal.x, agent.goal.y))

# Add 1s to the complex grid without placing on prohibited positions
positions_added = 0  # Counter for added positions
for _ in range(10):
    x = np.random.randint(0, 8)
    y = np.random.randint(0, 8)
    if (x, y) not in prohibited_positions and complex_grid[x, y] == 0:
        complex_grid[x, y] = 1
        positions_added += 1
paths = cooperative_astar(agents, grid)

complex_paths = cooperative_astar(complex_agents, complex_grid)



# Print results
for agent, path in paths.items():
    print(f"Agent {agent.id} path:")
    for pos, time in path:
        print(f"t={time}: ({pos.x}, {pos.y})")

import matplotlib.pyplot as plt
import matplotlib.animation as animation

def visualize(grid, paths):
    # Create figure with extra space for text
    fig = plt.figure(figsize=(8, 9))
    
    # Create a gridspec to manage subplot layout
    gs = plt.GridSpec(2, 1, height_ratios=[1, 8])
    
    # Create separate axes for text and grid
    text_ax = fig.add_subplot(gs[0])
    grid_ax = fig.add_subplot(gs[1])
    
    # Define a set of colors for agents - Move this up before plotting paths
    colors = plt.cm.get_cmap("tab10", len(paths))
    
    # Plot planned paths
    for idx, (agent, path) in enumerate(paths.items()):
        path_x = [pos.y for pos, _ in path]
        path_y = [pos.x for pos, _ in path]
        
    
        color = colors(idx)
        grid_ax.plot(path_x, path_y, color=color, alpha=0.5, linewidth=1)
        
        # Add arrows to show direction
        for i in range(len(path_x)-1):
            dx = path_x[i+1] - path_x[i]
            dy = path_y[i+1] - path_y[i]
            grid_ax.arrow(path_x[i], path_y[i], dx*0.3, dy*0.3,
                        head_width=0.1, head_length=0.1, 
                        color=color, alpha=0.5)
    
    text_ax.axis('off')
    time_text = text_ax.text(0.5, 0.5, '', ha='center', va='center', fontsize=14, transform=text_ax.transAxes)

    grid_ax.set_xticks(range(grid.shape[1]))
    grid_ax.set_yticks(range(grid.shape[0]))
    grid_ax.set_xticks(np.arange(-0.5, grid.shape[1], 1), minor=True)
    grid_ax.set_yticks(np.arange(-0.5, grid.shape[0], 1), minor=True)
    grid_ax.grid(which="minor", color="black", linestyle='-', linewidth=0.5)
    grid_ax.set_xticklabels([])
    grid_ax.set_yticklabels([])
    grid_ax.set_xlim(-0.5, grid.shape[1] - 0.5)
    grid_ax.set_ylim(-0.5, grid.shape[0] - 0.5)

    # Visualize obstacles
    for x in range(grid.shape[0]):
        for y in range(grid.shape[1]):
            if grid[x, y] == 1:  # Obstacle
                rect = plt.Rectangle((y - 0.5, x - 0.5), 1, 1, color="black")
                grid_ax.add_patch(rect)
    # Define a set of colors for agents
    colors = plt.cm.get_cmap("tab10", len(paths))

    # Mark initial positions and goals
    for idx, (agent, path) in enumerate(paths.items()):
        start = path[0][0]  # Unpack Position object for start
        goal = path[-1][0]  # Unpack Position object for goal
        color = colors(idx)
        grid_ax.text(start.y, start.x, 'A', color=color, ha='center', va='center', fontsize=14)
        grid_ax.text(goal.y, goal.x, 'G', color=color, ha='center', va='center', fontsize=14)

    # Create the animation
    frames = max(len(path) for path in paths.values())
    patches = []

    for idx, path in enumerate(paths.values()):
        pos, _ = path[0]  # Unpack first Position object and time
        color = colors(idx)
        patch = plt.Circle((pos.y, pos.x), 0.3, color=color, animated=True)
        grid_ax.add_patch(patch)
        patches.append((patch, path))

    # Add time display text
    # time_text = grid_ax.text(0.02, 0.98, '', transform=grid_ax.transAxes, fontsize=12)

    def update(frame):
        # Update agent positions
        for patch, path in patches:
            if frame < len(path):
                pos, time = path[frame]  # Now we use both position and time
                patch.center = (pos.y, pos.x)
        
        # Update time display
        time_text.set_text(f'Time Step: {frame}')
        
        return [patch for patch, _ in patches] + [time_text]
    ani = animation.FuncAnimation(fig, update, frames=frames, blit=True, interval=500, repeat=False)
    plt.show()
# Visualize the paths

# if paths:
#     visualize(grid, paths)


visualize(complex_grid, complex_paths)
