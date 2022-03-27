from  pycreate2 import Create2
import time
from pyamaze import maze,COLOR,agent
import pyhop

# Goal is always top left unless we specify otherwise

irl_maze = {
    # Important
    (1, 5): {'W': 0, 'E': 0, 'S': 1, 'N': 1}, #
    (2, 5): {'W': 0, 'E': 0, 'S': 1, 'N': 1}, #
    (3, 5): {'W': 1, 'E': 0, 'S': 1, 'N': 1}, #
    (4, 5): {'W': 0, 'E': 0, 'S': 0, 'N': 1}, #
    (3, 4): {'W': 1, 'E': 1, 'S': 0, 'N': 0}, #
    (3, 3): {'W': 1, 'E': 1, 'S': 1, 'N': 0}, #
    (4, 3): {'W': 0, 'E': 0, 'S': 1, 'N': 1}, #
    (5, 3): {'W': 0, 'E': 0, 'S': 1, 'N': 1}, #
    (3, 2): {'W': 1, 'E': 1, 'S': 0, 'N': 0}, #
    (3, 1): {'W': 0, 'E': 1, 'S': 0, 'N': 0},
    # Unimportant
    (1, 1): {'W': 0, 'E': 0, 'S': 0, 'N': 0}, 
    (1, 2): {'W': 0, 'E': 0, 'S': 0, 'N': 0}, 
    (1, 3): {'W': 0, 'E': 0, 'S': 0, 'N': 0}, 
    (1, 4): {'W': 0, 'E': 0, 'S': 0, 'N': 0}, 
    (2, 1): {'W': 0, 'E': 0, 'S': 0, 'N': 0},
    (2, 2): {'W': 0, 'E': 0, 'S': 0, 'N': 0},
    (2, 3): {'W': 0, 'E': 0, 'S': 0, 'N': 0},
    (2, 4): {'W': 0, 'E': 0, 'S': 0, 'N': 0},
    (4, 1): {'W': 0, 'E': 0, 'S': 0, 'N': 0},
    (5, 1): {'W': 0, 'E': 0, 'S': 0, 'N': 0},
    (4, 2): {'W': 0, 'E': 0, 'S': 0, 'N': 0},
    (5, 2): {'W': 0, 'E': 0, 'S': 0, 'N': 0},
    (4, 4): {'W': 0, 'E': 0, 'S': 0, 'N': 0},
    (5, 4): {'W': 0, 'E': 0, 'S': 0, 'N': 0}, 
    (5, 5): {'W': 0, 'E': 0, 'S': 0, 'N': 0}
}

#irl_maze = {
#    (3, 1): {'W': 0, 'E': 0, 'S': 1, 'N': 0},
#    (3, 2): {'W': 0, 'E': 0, 'S': 1, 'N': 1},
#    (3, 3): {'W': 0, 'E': 0, 'S': 0, 'N': 1},
#    (2, 1): {'W': 0, 'E': 1, 'S': 0, 'N': 0},
#    (2, 2): {'W': 1, 'E': 1, 'S': 1, 'N': 0},
#    (2, 3): {'W': 1, 'E': 0, 'S': 1, 'N': 1},
#    (1, 1): {'W': 0, 'E': 0, 'S': 1, 'N': 1},
#    (1, 2): {'W': 1, 'E': 0, 'S': 0, 'N': 1},
#    (1, 3): {'W': 0, 'E': 0, 'S': 1, 'N': 1}
#}



goal_state = pyhop.Goal('maze')
goal_state.loc = {'me': {'x': 5, 'y': 1}}

# Note that this is in y, x (backwards :( )



# Declare state and variable bindings
maze_state = pyhop.State('maze')

maze_state.loc = {'me': {'x': 3, 'y': 5}}
maze_state.last_direction = {'me': {'direction': 'null'}}

maze_state.visited = {'me': []}

# N, S, E, W --> North, South, East, West
maze_state.direction = {'me': 'N'}

# 0 is open, 1 is wall
maze_state.maze = {'maze': irl_maze, 'rows': 5, 'cols': 5#[
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


def turn_bot_right(bot, angle, speed):
    curr_angle = 0
    # Grab angle
    bot.drive_direct(-speed, speed)    
    while -angle < curr_angle < angle:
        # Grab how much we've turned
        # Add to curr_angle
        sensors = bot.get_sensors()
        angle_change = sensors.angle
        curr_angle += angle_change
    bot.drive_stop()
    return curr_angle

def turn_bot_left(bot, angle, speed):
    curr_angle = 0
    # Grab angle
    bot.drive_direct(speed, -speed)    
    while -angle < curr_angle < angle:
        # Grab how much we've turned
        # Add to curr_angle
        sensors = bot.get_sensors()
        angle_change = sensors.angle
        curr_angle += angle_change
    bot.drive_stop()
    return curr_angle

def turnRightUntilNoLeft(speed):
    bot.drive_direct(-1 * speed, speed)
    noTrue= True
    while noTrue:
        sensors = bot.get_sensors()
        bump = sensors.light_bumper
        if not bump.left and not bump.center_left:
            bot.drive_stop()
            return

def turnLeftUntilNoRight(speed):
    bot.drive_direct(speed, -1 * speed)
    noTrue= True
    while noTrue:
        sensors = bot.get_sensors()
        bump = sensors.light_bumper
        if not bump.right and not bump.center_right:
            bot.drive_stop()
            return


def move_bot_forward(bot, distance, speed):
    dist = 0
    # Grab angle
    bot.drive_direct(speed, speed)    
    while dist < distance:
        time.sleep(.1)
        # Grab how much we've turned
        # Add to curr_angle
        sensors = bot.get_sensors()
        angle_change = sensors.distance
        dist += angle_change

        bump = sensors.light_bumper
        if bump.front_left and bump.front_right:
            print("hit a wall")
            return dist
        # Adjustment
        if bump.left and bump.right:
            print("Did not adjust")
        elif bump.center_left and bump.left:
            print("Adjusting Right...")
            bot.drive_stop()
            turnRightUntilNoLeft(200)
            bot.drive_direct(speed, speed)
        elif bump.center_right and bump.right:
            print("Adjusting Left...")
            bot.drive_stop()
            turnLeftUntilNoRight(200)
            bot.drive_direct(speed, speed)
        
    bot.drive_stop()
    return dist


def turnLeft(bot):
    print("left")
    turn_bot_left(bot, 95, 100)

def turnRight(bot):
    print("right")
    turn_bot_right(bot, 95, 100)

def doNothing(bot):
    print("nothing")

def turnBackwards(bot):
    turn_bot_right(bot, 180, 100)


def handleTurn(currentDir, newDir, bot):
    print("Current Dir: {currentDir}, New Dir: {newDir}".format(currentDir=currentDir, newDir=newDir))
    mapDirections = {
        ('N','N'): doNothing,
        ('N','W'): turnLeft,
        ('N', 'S'): turnBackwards,
        ('N', 'E'): turnRight,
        ('S','S'): doNothing,
        ('S','W'): turnRight,
        ('S', 'N'): turnBackwards,
        ('S', 'E'): turnLeft,
        ('E', 'E'): doNothing,
        ('E', 'N'): turnLeft,
        ('E','W'): turnBackwards,
        ('E', 'S'): turnRight,
        ('W', 'W'): doNothing,
        ('W', 'S'): turnLeft,
        ('W', 'E'): turnBackwards,
        ('W', 'N') :turnRight
    }
    mapDirections[(currentDir, newDir)](bot)

def moveForward(bot):
    print("going forward")
    move_bot_forward(bot, 350, 150)


def executePlan(plan, startX, startY, bot):
    currentDir = 'N'
    for i in range(len(plan)):
        nextDir = plan[i:i+1]
        handleTurn(currentDir, nextDir, bot)
        moveForward(bot)
        currentDir = nextDir

# Methods
# move_towards_goal, rotate

port = "/dev/tty.usbserial-DN02693O"  # where is your serial port?
bot = Create2(port)

# Start the Create 2
bot.start()
bot.full()


pyhop.declare_operators(move)
pyhop.declare_methods('move_toward_goal', move_west, move_north, move_east, move_south)

# Use planner to get a plan
pyhop.print_state(maze_state)
pyhop.print_goal(goal_state)

plan = pyhop.pyhop(maze_state,tasks=[('move_toward_goal', 'me', goal_state)], verbose=3)
path = plan_to_path(maze_state.loc['me']['x'], maze_state.loc['me']['y'], plan)
print(path)


executePlan(path, 2, 3, bot)







#'maze': {(1, 1): {'E': 1, 'W': 0, 'N': 0, 'S': 1}, (2, 1): {'E': 1, 'W': 0, 'N': 1, 'S': 1}, (3, 1): {'E': 0, 'W': 0, 'N': 1, 'S': 1}, (4, 1): {'E': 1, 'W': 0, 'N': 1, 'S': 0}, (5, 1): {'E': 1, 'W': 0, 'N': 0, 'S': 0}, (1, 2): {'E': 1, 'W': 1, 'N': 0, 'S': 0}, (2, 2): {'E': 0, 'W': 1, 'N': 0, 'S': 1}, (3, 2): {'E': 1, 'W': 0, 'N': 1, 'S': 0}, (4, 2): {'E': 0, 'W': 1, 'N': 0, 'S': 1}, (5, 2): {'E': 1, 'W': 1, 'N': 1, 'S': 0}, (1, 3): {'E': 0, 'W': 1, 'N': 0, 'S': 1}, (2, 3): {'E': 0, 'W': 0, 'N': 1, 'S': 0}, (3, 3): {'E': 1, 'W': 1, 'N': 0, 'S': 0}, (4, 3): {'E': 1, 'W': 0, 'N': 0, 'S': 1}, (5, 3): {'E': 0, 'W': 1, 'N': 1, 'S': 0}, (1, 4): {'E': 1, 'W': 0, 'N': 0, 'S': 1}, (2, 4): {'E': 0, 'W': 0, 'N': 1, 'S': 1}, (3, 4): {'E': 1, 'W': 1, 'N': 1, 'S': 0}, (4, 4): {'E': 0, 'W': 1, 'N': 0, 'S': 1}, (5, 4): {'E': 1, 'W': 0, 'N': 1, 'S': 0}, (1, 5): {'E': 0, 'W': 1, 'N': 0, 'S': 1}, (2, 5): {'E': 0, 'W': 0, 'N': 1, 'S': 1}, (3, 5): {'E': 0, 'W': 1, 'N': 1, 'S': 1}, (4, 5): {'E': 0, 'W': 0, 'N': 1, 'S': 1}, (5, 5): {'E': 0, 'W': 1, 'N': 1, 'S': 0}}, 'rows': 5, 'cols': 5}
#    maze.loc = {'me': {'x': 5, 'y': 5}