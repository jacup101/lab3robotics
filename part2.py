from pyamaze import maze,COLOR,agent
import pyhop
# Goal is always top left unless we specify otherwise




goal_state = pyhop.Goal('maze')
goal_state.loc = {'me': {'x': 20, 'y': 20}}

# Note that this is in y, x (backwards :( )
m=maze(20,20)
m.CreateMaze(loopPercent=10, x=goal_state.loc['me']['x'], y=goal_state.loc['me']['y'])




# Declare state and variable bindings
maze_state = pyhop.State('maze')
maze_state2 = pyhop.State('maze')

maze_state.loc = {'me': {'x': 1, 'y': 1}}
maze_state.last_direction = {'me': {'direction': 'null'}}

maze_state.visited = {'me': []}

# N, S, E, W --> North, South, East, West
maze_state.direction = {'me': 'N'}

# 0 is open, 1 is wall
maze_state.maze = {'maze': m.maze_map, 'rows': m.rows, 'cols': m.cols#[
    # North
    #[0,1,0,0,0],
    #[0,0,0,1,0],
    #[0,1,1,1,0],
    #[0,1,0,0,0]
}


maze_state2.loc = {'me': {'x': 1, 'y': 20}}
maze_state2.last_direction = {'me': {'direction': 'null'}}

maze_state2.visited = {'me': []}

# N, S, E, W --> North, South, East, West
maze_state2.direction = {'me': 'N'}

# 0 is open, 1 is wall
maze_state2.maze = {'maze': m.maze_map, 'rows': m.rows, 'cols': m.cols#[
    # North
    #[0,1,0,0,0],
    #[0,0,0,1,0],
    #[0,1,1,1,0],
    #[0,1,0,0,0]
}
# Operators
# Move
# a = agent = 'me'

def move(state, a, x, y):
    if x <= state.maze['cols'] and y <= state.maze['rows']  and x > 0 and y > 0:
        if (y,x) not in state.visited[a]:
            # print("({x},{y} not in {vis})".format(x=x,y=y,vis=state.visited[a]))
            state.loc[a]['x'] = x
            state.loc[a]['y'] = y
            state.visited[a].append((y,x))
            # state.maze['maze'][y][x] = 2
            return state
        else:
            return False
            # print("{x},{y} in {vis}".format(x=x,y=y,vis=state.visited[a]))
    return False





# Method
def move_east(state, a, goal):
    # If there's no wall to right and we're not at right edge of maze, we can move right
    x = state.loc[a]['x']
    y = state.loc[a]['y']
    if x == goal.loc[a]['x'] and y == goal.loc[a]['y']:
        return []
    if state.maze['rows'] >= (x+1) and state.maze['maze'][(y,x)]['E'] == 1:
        # print("Moving east from {x}, {y}".format(x=x,y=y))
        return [('move', a, x+1, y), ('move_toward_goal', a, goal_state)]
    return False

def move_west(state, a, goal):
    # if there is no wall to the left and we're not at left edge of maze, we can move west
    x = state.loc[a]['x']
    y = state.loc[a]['y']
    if x == goal.loc[a]['x'] and y == goal.loc[a]['y']:
        return []
    if 0 < x and state.maze['maze'][(y,x)]['W'] == 1: #
        # print("Moving west from {x}, {y}".format(x=x,y=y))
        return [('move', a, x-1, y), ('move_toward_goal', a, goal_state)]
    return False

def move_north(state, a, goal):
    #if there is no wall above we can move up
    x = state.loc[a]['x']
    y = state.loc[a]['y']
    if x == goal.loc[a]['x'] and y == goal.loc[a]['y']:
        return []
    if 0 < y - 1 and state.maze['maze'][(y,x)]['N'] == 1:
        # print("Moving north from {x}, {y}".format(x=x,y=y))
        return [('move', a, x, y-1), ('move_toward_goal', a, goal_state)]
    return False

def move_south(state, a, goal):
    # if there is no wall below
    x = state.loc[a]['x']
    y = state.loc[a]['y']
    if  x == goal.loc[a]['x'] and y == goal.loc[a]['y']:
        return []
    if state.maze['cols'] >= (y + 1) and state.maze['maze'][(y,x)]['S'] == 1:
        # print("Moving south from {x}, {y}".format(x=x,y=y))
        return [('move', a, x, y+1), ('move_toward_goal', a, goal_state)]
    return False



def plan_to_path(startX, startY,path):
    pathString = getDirection(startX, startY, path[0][2], path[0][3])
    for i in range(1,len(path)):
        pathString += getDirection(path[i-1][2],path[i-1][3],path[i][2],path[i][3])
        # print(path[i], path[i-1])
    return pathString


def getDirection(oldX, oldY, newX, newY):
    if oldY == newY:
        if oldX < newX:
            return "E"
        else:
            return "W"
    else:
        if oldY < newY:
            return "S"
        else:
            return "N"

# Methods
# move_towards_goal, rotate




a=agent(m,shape='square',footprints=True, x=maze_state.loc['me']['y'], y=maze_state.loc['me']['x'])
a2=agent(m,shape='arrow',footprints=True, x=maze_state2.loc['me']['y'], y=maze_state2.loc['me']['x'])


pyhop.declare_operators(move)
pyhop.declare_methods('move_toward_goal', move_west, move_north, move_east, move_south)


# Use planner to get a plan
pyhop.print_state(maze_state)
pyhop.print_goal(goal_state)

plan = pyhop.pyhop(maze_state,tasks=[('move_toward_goal', 'me', goal_state)], verbose=1)
path = plan_to_path(maze_state.loc['me']['x'], maze_state.loc['me']['y'], plan)
print(path)

plan2 = pyhop.pyhop(maze_state2,tasks=[('move_toward_goal', 'me', goal_state)], verbose=1)
path2 = plan_to_path(maze_state2.loc['me']['x'], maze_state2.loc['me']['y'], plan2)
print(path2)

m.tracePath({a:path})
m.tracePath({a2:path2})
m.run()




#'maze': {(1, 1): {'E': 1, 'W': 0, 'N': 0, 'S': 1}, (2, 1): {'E': 1, 'W': 0, 'N': 1, 'S': 1}, (3, 1): {'E': 0, 'W': 0, 'N': 1, 'S': 1}, (4, 1): {'E': 1, 'W': 0, 'N': 1, 'S': 0}, (5, 1): {'E': 1, 'W': 0, 'N': 0, 'S': 0}, (1, 2): {'E': 1, 'W': 1, 'N': 0, 'S': 0}, (2, 2): {'E': 0, 'W': 1, 'N': 0, 'S': 1}, (3, 2): {'E': 1, 'W': 0, 'N': 1, 'S': 0}, (4, 2): {'E': 0, 'W': 1, 'N': 0, 'S': 1}, (5, 2): {'E': 1, 'W': 1, 'N': 1, 'S': 0}, (1, 3): {'E': 0, 'W': 1, 'N': 0, 'S': 1}, (2, 3): {'E': 0, 'W': 0, 'N': 1, 'S': 0}, (3, 3): {'E': 1, 'W': 1, 'N': 0, 'S': 0}, (4, 3): {'E': 1, 'W': 0, 'N': 0, 'S': 1}, (5, 3): {'E': 0, 'W': 1, 'N': 1, 'S': 0}, (1, 4): {'E': 1, 'W': 0, 'N': 0, 'S': 1}, (2, 4): {'E': 0, 'W': 0, 'N': 1, 'S': 1}, (3, 4): {'E': 1, 'W': 1, 'N': 1, 'S': 0}, (4, 4): {'E': 0, 'W': 1, 'N': 0, 'S': 1}, (5, 4): {'E': 1, 'W': 0, 'N': 1, 'S': 0}, (1, 5): {'E': 0, 'W': 1, 'N': 0, 'S': 1}, (2, 5): {'E': 0, 'W': 0, 'N': 1, 'S': 1}, (3, 5): {'E': 0, 'W': 1, 'N': 1, 'S': 1}, (4, 5): {'E': 0, 'W': 0, 'N': 1, 'S': 1}, (5, 5): {'E': 0, 'W': 1, 'N': 1, 'S': 0}}, 'rows': 5, 'cols': 5}
#    maze.loc = {'me': {'x': 5, 'y': 5}