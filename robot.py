# -*- coding: utf-8 -*-
"""

@author: Aditya Karlekar
"""


#Defined in tester.py
dir_sensors = {'u': ['l', 'u', 'r'], 'r': ['u', 'r', 'd'],
               'd': ['r', 'd', 'l'], 'l': ['d', 'l', 'u'],
               'up': ['l', 'u', 'r'], 'right': ['u', 'r', 'd'],
               'down': ['r', 'd', 'l'], 'left': ['d', 'l', 'u']}

#dir_move has been augmented with symbols '<', '>', '^' and 'v'
dir_move = {'u': [0, 1], 'r': [1, 0], 'd': [0, -1], 'l': [-1, 0],
            'up': [0, 1], 'right': [1, 0], 'down': [0, -1], 'left': [-1, 0],
            '^': [0, 1], '>': [1, 0], 'v': [0, -1], '<': [-1, 0],}

dir_reverse = {'u': 'd', 'r': 'l', 'd': 'u', 'l': 'r',
               'up': 'd', 'right': 'l', 'down': 'u', 'left': 'r'}

#the rotation values in degrees 
degrees = {'up': 0,'left': -90,'right':90, 'down':180,'u': 0,'l': -90,'r':90, 'd':180, 
				 '^': 0, '<': -90, '>': 90, 'v': 180}


#Following files were used for testing
# python tester.py test_maze_01.txt
# python tester.py test_maze_02.txt
# python tester.py test_maze_03.txt


#Robot class with initializer and required functions
class Robot(object):
    def __init__(self, maze_dim):
        
        self.location = [0, 0]
        self.heading = 'up'
        self.maze_dim = maze_dim    # variable having maze dimensions. supplied from tester.py file
        self.maze_area = maze_dim ** 2.  #maze area
        self.maze_grid = [[0 for row in range(maze_dim)] for col in range(maze_dim)] # Grid for wall locations for each maze.
        self.goal_found = False
        self.backmoves = 0  # Initial value of Down direction.
        self.moves = 0 # numberber of steps taken in each trial
        self.run = 0 # Exploration or Optimization Trial
        self.path_grid = [[0 for row in range(maze_dim)] for col in range(maze_dim)] # Grid for mapping path
        self.path_value = [[99]*maze_dim for col in range(maze_dim)] # Value score grid
        self.policy = [[' ']*maze_dim for col in range(maze_dim)] # Grid to map optimal route.
        self.goal_discovered = [maze_dim/2 - 1, maze_dim/2] or [maze_dim/2, maze_dim/2] or [maze_dim/2, maze_dim/2 - 1] or [maze_dim/2 - 1, maze_dim/2 - 1]  # Goal area in maze center
        self.heuristics_grid = [[-1]*maze_dim for col in range(maze_dim)] # heuristic grid for A* algorithm
        
        
    def next_move(self, sensors):
        
        #roatation = 0
        #movement = 0
        
        if self.run == 0 :
             rotation, movement = self.first_run(sensors)          
        elif self.run == 1:
            rotation, movement = self.second_run(sensors)
        
        return rotation, movement
        
    #Code for first run: In the first run, we explore the maze and map it
    def first_run(self,sensors): 
        
        
        #Counting the steps taken in first run
        print "Exploration Step Count: ", self.moves, sensors
        self.moves +=1

        #Finding robot's location         
        x = self.location[0]
        y = self.location[1]
        
        print "Location: ", self.location
        # Add 1 to path_grid
        self.path_grid[x][y] += 1
        
        # Calculate the percentage of the maze the robot has visited
        area_explored = self.area_explored()

        print "%.3f%% area of the maze has been explored" % area_explored
        
        #Extrapolating the wall positions from sensor readings
        number = self.wall_positions(sensors, self.backmoves)
        self.maze_grid[x][y] = number
        
        #Finding robot's next move
        rotation, movement = self.calculate_next_move(x, y, sensors)
        
        #Updating the backmoves values
        if movement == 0:
            self.backmoves = 0
        elif movement == 1:
            self.backmoves = 1
        elif movement == -1 or movement == -2: #-1 and -2 values for deadends. If the robot hits a deadend
            for i in range(2):
                if self.get_direction(x, y, i):
                    self.backmoves = 0
                else:
                    self.backmoves = 1

        #updating robot's direction
        self.update_direction(rotation, movement)
        
        #finding new location
        x_new = self.location[0]
        y_new = self.location[1]
        
        area_to_explore = 70 #the area_to_explore values are changed to , 50, 55, 60, 65, 70, 75, 80, 85, 90, 95 and 100 for tabulating results

       #Check if the new location is the goal state
        if x_new in self.goal_discovered and y_new in self.goal_discovered:
            if self.path_grid[x_new][y_new] == 0:
                print"#### Goal has been discovered after " + str(self.moves) + " moves. #### \n"
                print'Exploration continues till ' + str(area_to_explore) + '% has been covered.\n'
                self.goal_found = True
            
        elif self.goal_found == True and area_explored >= area_to_explore:  
            print'The exploration has ended. Starting next trial.'
            goal_bound = self.goal_discovered
            self.value_calculation(goal_bound) # Computing value function for optimal path
            print"\n #### Maze Grid #### \n", self.maze_grid
            print"\n #### Policy Grid #### \n", self.policy
            print"\n #### Path Grid #### \n", self.path_grid
            print"\n #### Path value #### \n", self.path_value
            print"\n #### Heuristic Grid ####\n", self.heuristics_grid
            
                  
            # Changing variables to default for second run
            
            rotation = 'Reset'
            movement = 'Reset'
            self.run = 1
            self.heading = 'up'
            self.goal_found = False
            self.location = [0,0]
            self.moves = 0
            
        return rotation, movement
    
    def get_direction(self, x, y, i):
        '''
        Updating backmoves values based on robot current direction
        '''
        if self.heading == 'l' or self.heading == 'left': #robot facing left and wall present on right 9, 12, 13 show wall on right side
            if self.maze_grid[x + i][y] == 9 or self.maze_grid[x + i][y] == 12 or self.maze_grid[x + i][y] == 13:
                return True
                        
        elif self.heading == 'd' or self.heading == 'down': #robot facing down and wall present on top 6, 12, 14 shows wall on top side
            if self.maze_grid[x][y+i] == 6 or self.maze_grid[x][y + i] == 12 or self.maze_grid[x][y + i] == 14:
                return True
            
        elif self.heading == 'u' or self.heading == 'up': #robot facing up and wall present on down side. 3, 9, 11 shows wall on top side
            if self.maze_grid[x][y - i] == 3 or self.maze_grid[x][y - i] == 9 or self.maze_grid[x][y - i] == 11:
                return True
            
        elif self.heading == 'r' or self.heading == 'right': # robot facing right and wall present on left side. 3, 6, 7 show wall on left side
            if self.maze_grid[x - i][y] == 3 or self.maze_grid[x - i][y] == 6 or self.maze_grid[x - i][y] == 7:
                return True
        
        return False

    
    #This is the optimization stwp. We use the information learned during first run
    #to find an optimal path to the goal position.
    def second_run(self,sensors):
        '''
        The second_run function generates a policy
        for the explored maze to reach the goal in 
        shortest time
        '''
                              
        print('The Optimization Run Step #: ', self.moves, sensors, self.location)
        self.moves += 1
        
        movement = 1 #Movement is 1 as robot's first step is always in front
        
        #Finding current location
        x = self.location[0]
        y = self.location[1]
        
        #Rotation
        angle_head = degrees[self.heading]
        optimal_angle_head = degrees[self.policy[x][y]]
        rotation = optimal_angle_head - angle_head
        
        #angle correction as the robot could rotate only 180 degrees
        if rotation == -270:
            rotation = 90
        elif rotation == 270:
            rotation = -90
            
        #Changing angle to index
        index = rotation/90 + 1
        direction = dir_sensors[self.heading][index] #direction change
        
        #Movement: maximum three consecutive steps allowed
        while movement < 3:
            loc = self.policy[x][y]
            x += dir_move[direction][0]
            y += dir_move[direction][1]
            
            if self.policy[x][y] == loc:
                movement += 1
            else:
                break
        
        # Updating direction and location
        self.update_direction(rotation, movement)

        return rotation, movement
    
    
    def update_sensors(self, sensors):
        for i in range(len(sensors)):
            if sensors[i] > 0:
                sensors[i] = 1
        return sensors
    
    
    def wall_positions(self, sensors, backmoves):
        '''
        Generates binary value to represent wall surrounding robot
        '''
        #Setting sensor reading to open or closed
        #1 for open, 0 for closed
        sensors = self.update_sensors(sensors)
                
        # 1 = North, 2 = East, 4 = South, 8 = West. Each sensor will give a reading of 1 (open) or 0 (closed)       
        if self.heading == 'd' or self.heading == 'down':
            k = (sensors[2]*8) + (sensors[1]*4) + (sensors[0]*2) + backmoves
            
        elif self.heading == 'u' or self.heading == 'up':
            k = (sensors[0]*8) + (backmoves*4) + (sensors[2]*2) + sensors[1]
            
        elif self.heading == 'r' or self.heading == 'right':
            k = (backmoves*8) + (sensors[2]*4) + (sensors[1]*2) + sensors[0]
            
        elif self.heading == 'l' or self.heading == 'left':
            k = (sensors[1]*8) + (sensors[0]*4) + (backmoves*2) + sensors[2]
            
        return k
    
    def area_explored(self):
        '''
        function to calculate the area covered
        '''
        explored = 0
        for x in range(self.maze_dim):
            for y in range(self.maze_dim):
                if self.path_grid[x][y] > 0:
                    explored += 1
        area_covered = (explored / (self.maze_area) ) * 100
        
        return area_covered

    
    def update_direction(self, rotation, movement):
        '''
         Updates the direction of the robot
        '''
        
        #Rotation angles to indexs [0,1,2]
        if rotation == -90 or rotation == 270:
            rotation = 0
            
        elif rotation == 0 or rotation == 360:
            rotation = 1
        
        elif rotation == 90 or rotation == -270:
            rotation = 2
        
        #Calculating new heading
        self.heading = dir_sensors[self.heading][rotation]
        
        #Updating location
        
        self.location[0] += dir_move[self.heading][0]*movement
        self.location[1] += dir_move[self.heading][1]*movement
        
    def reverse(self, x, y, sensors):
        '''
        Reverse check for deep deadends
        '''
        if sensors == [0,0,0] and self.heading == 'u' and self.maze_grid[x][y-1] == 5: #deep end in front
            return True
        
        elif  sensors == [0,0,0] and self.heading == 'l' and self.maze_grid[x + 1][y] == 10: #deep end at left
            return True
        
        elif   sensors == [0,0,0] and self.heading == 'd' and self.maze_grid[x][y+1] == 5: #deep end at back
            return True
        
        elif sensors == [0,0,0] and self.heading == 'r' and self.maze_grid[x-1][y] == 10: #deep end at right
            return True
        
        return False
    

    def calculate_next_move(self, x, y, sensors):
        '''
        Calculates the next move for robot to make
        '''
        
        prompt = 'Robot out of bounds. Reversing Course'
        
        
        if self.reverse(x, y, sensors): #if deep deadend is present, then reverse by moving two steps backward
            movement = -2
            rotation = 0
            print prompt
            
        elif sensors == [0,0,0]:    #for shallow deadends
            movement = -1
            rotation = 0
            print prompt

        else: #Calling A*
            
            rotate = [-90,0,90]
            next_move = self.a_star(x, y, sensors)
            
            (t,h,x_1,y_1,sensor) = next_move #Creating a tuple by breaking list into separate variables
            rotation = rotate[sensor]   #rotate towards open cell
            movement = 1 #move 1 cell
            
        return rotation, movement   

    def a_star(self, x, y, sensors): #A* algorithm
        '''
        A* algorithm
        '''
        #This code uses elements from Udacity
        #Artificial Intelligence for Robotics Course
        # https://www.udacity.com/course/artificial-intelligence-for-robotics--cs373
        
        self.heuristics_grid = self.generate_heuristics() #generate heuristic for A*
        actions = []
        
            
        for i in range(len(sensors)): #checkling all sensor readings
            if sensors[i] > 0: #If numbers is present, then it's open
                sensors[i] = 1
            
                 #Moving to next open cell
                x_new = x + dir_move[dir_sensors[self.heading][i]][0]
                y_new = y + dir_move[dir_sensors[self.heading][i]][1]
                    
                if x_new >= 0 and x_new < self.maze_dim and y_new >= 0 and y_new < self.maze_dim:
                    times = self.path_grid[x_new][y_new] #The number of times this cell has been visited
                    heuristics = self.heuristics_grid[x_new][y_new] #Heuristic value
                    actions.append([times, heuristics, x_new, y_new, i]) # List of possible actions
                        
        actions.sort(reverse = True) #Sorting the list in reverse order 
        next_move = actions.pop() #Move to cell closest to center or/and not explored
                         
        return next_move 
    
    def generate_heuristics(self):
        '''
        Function to generate heuristic values for A* algorithm
        '''
        for x in range(self.maze_dim):
            for y in range(self.maze_dim):
                # positive minimum distance from a goal cell to any other cell
                x_1 = min(abs(x - int(self.maze_dim/2 - 1)), abs(x - int(self.maze_dim/2)))
                y_1 = min(abs(y - int(self.maze_dim/2 - 1)), abs(y - int(self.maze_dim/2)))
                self.heuristics_grid[x][y] = x_1 + y_1 # heuristic value for the cell
                
        return self.heuristics_grid

    def value_calculation(self, goal_position):
        '''
        Dynamic programming method using a Value Function for calculating minimum number 
        of steps to the goal from each cell. 
        '''
        #This code uses elements from Udacity
        #Artificial Intelligence for Robotics Course
        # https://www.udacity.com/course/artificial-intelligence-for-robotics--cs373
        
        change = True #boolean for while loop
        
        while change:
            change = False #Setting change to false for stopping condition
            cost = 1
            
            for x in range(self.maze_dim):
                for y in range(self.maze_dim):
                    if x in goal_position and y in goal_position:
                        if self.path_value[x][y] > 0: #Preventing endless looping
                            self.path_value[x][y] = 0 #set goal position to 0
                            self.policy[x][y] = '*' #Assign * to goal position
                            print 'Goal position: ' + str(goal_position) + '\n'
                            
                            change = True
                            
                    else:
                        actions = self.valid_actions([x,y]) # check for valid actions
                        
                        #Starting from goal state and increasing incrementally by 1 to open spaces
                        for i in range(len(actions)):
                                
                                x_new = x + dir_move[actions[i]][0]
                                y_new = y + dir_move[actions[i]][1]
                                
                                if x_new >= 0 and x_new < self.maze_dim and y_new >= 0 and y_new < self.maze_dim: #inside the maze
                                    r = self.path_value[x_new][y_new] + cost  #Adding cost to path value
                                    
                                    if r < self.path_value[x][y]:
                                        change = True
                                        self.path_value[x][y] = r #Updating path_value with new count number
                                        self.policy[x][y] = actions[i]  #Updating policy grid
                
                
                                        
    def valid_actions(self, point):
        '''
        function to return a list of valid actions based on walls in a cell
        '''

        valid_actions = []

        walls = self.maze_grid[point[0]][point[1]]

        # select valid actions based on wall value
        
        if walls in [2, 3, 6, 7, 10, 11, 14, 15] :
            valid_actions.append('>')
            
        if walls in [8, 9, 10, 11, 12, 13, 14, 15] :
            valid_actions.append('<')
        
        if walls in [1, 3, 5, 7, 9, 11, 13, 15] :
            valid_actions.append('^')
            
        if walls in [4, 5, 6, 7, 12, 13, 14, 15]:
            valid_actions.append('v')

        return valid_actions

    
        
    
              