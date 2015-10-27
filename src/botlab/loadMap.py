import string
import csv
import numpy as np
import matplotlib.pyplot as plt
import time
import A_star
from operator import itemgetter
import udsp
import maps
MAP_RES_PIX_PER_M = 100
class MotionPlanner:
    
    def __init__(self):
        self.map_data = []
        self.disc_data = 0
        self.disc_size = 10
        self.nogo_threshold = 180
        self.open_states = []
        self.closed_states = []
        self.state_coords = []
        self.obstacle_list = []
        self.fringe_obstacles = set()
        self.x_size = 0
        self.y_size = 0
        self.curr_state = 0
        self.goal_state = 0
        self.path = []
        self.wildfire_grid_vals = 0
        self.wavefront_grid_vals = 0
        
        return
    
    def loadMapFromTxt(self,fname):
        f = open(fname, 'r')
        file_text = f.read();
        text_rows = file_text.split('\n')
        text_rows = text_rows[0:-1]
        n_rows = len(text_rows)
        n_cols = len(text_rows[0].split(','))
        pix_data = np.zeros((n_rows,n_cols))
        for row_num in range(0,n_rows):
            for col_num in range(0,n_cols):
                split_row = text_rows[row_num].split(',')            
                pix_data[row_num, col_num] = int(split_row[col_num])
        self.map_data = pix_data

    def loadMapFromBot(self):
        p = udsp.UDSP()
        p.serveAT('tmp')
        di = p.dataIter()
        for obj in di:
            print "-"*40
            print repr(obj)
        print "...done"
        p.close()
            
    def writeMapToFile(self):
        a = open('map.csv','w')
        np.savetxt(a, self.map_data)
        a.close()
        print 'writted'
    
    def plotMap(self):
        x = np.arange(0,len(self.map_data))
        norm_data = -self.map_data
        c_map = plt.get_cmap('Greys')
        plt.pcolor(x,x,norm_data,cmap = 'Greys')
        plt.draw()

    def discretizeMap(self):
        if(round(self.map_data.shape[0]/self.disc_size) != self.map_data.shape[0]/self.disc_size):
            a = 1
            #print "Warning: disretization size does not perfectly divide space."
        self.x_size = int(np.floor(self.map_data.shape[0]/self.disc_size))
        self.y_size = int(np.floor(self.map_data.shape[1]/self.disc_size))
        self.disc_data = np.zeros((self.x_size, self.y_size))
        # Take average of pixels to determine which cells are available and which aren't
        state_counter = 0;
        for y_state in range(0,self.y_size):
            y_slice = slice(y_state*self.disc_size, (y_state+1)*self.disc_size, 1)
            for x_state in range(0,self.x_size):
                x_slice = slice(x_state*self.disc_size, (x_state+1)*self.disc_size, 1)
                avg_pixel_val = np.average(self.map_data[x_slice,y_slice])
                if(avg_pixel_val > self.nogo_threshold):
                    self.disc_data[x_state, y_state] = 0
                    self.open_states.append(state_counter)
                else:
                    self.disc_data[x_state, y_state] = 1
                    self.closed_states.append(state_counter)
                    self.obstacle_list.append((x_state, y_state))
                coords_tuple = ((x_state+1/2)*self.disc_size, (y_state+1/2)*self.disc_size)
                self.state_coords.append(coords_tuple)
                state_counter+=1

    def plotDiscretize(self):        
        #fig = plt.figure()
        x = np.arange(0,self.x_size*self.disc_size)
        y = np.arange(0,self.y_size*self.disc_size)
        data = np.zeros((self.x_size*self.disc_size, self.y_size*self.disc_size))
        for y_state in range(0,self.y_size):
            y_slice = slice(y_state*self.disc_size, (y_state+1)*self.disc_size, 1)
            for x_state in range(0,self.x_size):
                x_slice = slice(x_state*self.disc_size, (x_state+1)*self.disc_size, 1)
                data[x_slice,y_slice] = self.disc_data[x_state,y_state]
        c_map = plt.get_cmap('Greys')
        plt.pcolor(x,y,data, alpha = 0.7, cmap = 'Greys')
	fig = plt.gcf()
	ax = fig.gca()
	ax.set_xticks(np.arange(0,self.x_size*self.disc_size,self.disc_size))
	ax.set_yticks(np.arange(0,self.y_size*self.disc_size,self.disc_size))
	plt.grid()
        plt.draw()

    def plotObstacle(self):        
        
        x = np.arange(0,self.x_size*self.disc_size)
        y = np.arange(0,self.y_size*self.disc_size)
        data = np.zeros((self.x_size*self.disc_size, self.y_size*self.disc_size))
        for obs in self.obstacle_list:
            y_slice = slice(obs[1]*self.disc_size, (obs[1]+1)*self.disc_size, 1)
            x_slice = slice(obs[0]*self.disc_size, (obs[0]+1)*self.disc_size, 1)
            data[x_slice,y_slice] = 1

        c_map = plt.get_cmap('Reds')
        plt.pcolor(x,y,data, alpha = 0.7, cmap = 'Reds')
	fig = plt.gcf()
	ax = fig.gca()
	ax.set_xticks(np.arange(0,self.x_size*self.disc_size,self.disc_size))
	ax.set_yticks(np.arange(0,self.y_size*self.disc_size,self.disc_size))
	plt.xticks(range(0,self.x_size))
	plt.yticks(range(0,self.x_size))
	plt.grid()
        plt.draw()

    def plotPath(self, tuple_list):
        x = np.arange(0,self.x_size*self.disc_size)
        y = np.arange(0,self.y_size*self.disc_size)
        map_mat = np.zeros(self.map_data.shape)
        for tup in tuple_list:
            x_slice = slice(tup[0]*self.disc_size, (tup[0]+1)*self.disc_size, 1)
            y_slice = slice(tup[1]*self.disc_size, (tup[1]+1)*self.disc_size, 1)
            map_mat[x_slice, y_slice] = 1
        plt.pcolor(x,y,map_mat, alpha = 0.1, cmap = 'Blues')    

    def plotValues(self):
        x = np.arange(0,self.x_size*self.disc_size)
        y = np.arange(0,self.y_size*self.disc_size)
        map_mat = np.zeros(self.map_data.shape)
        for y_state in range(0,self.y_size):
	    y_slice = slice(y_state*self.disc_size, (y_state+1)*self.disc_size, 1)
	    for x_state in range(0,self.x_size):
	        x_slice = slice(x_state*self.disc_size, (x_state+1)*self.disc_size, 1)
	        map_mat[x_slice,y_slice] = self.wildfire_grid_vals[x_state,y_state]
        plt.pcolor(x,y,map_mat, alpha = 0.2, cmap = 'Greens')    
    '''
	def get_neighbors(self, curr_node):
		
		dirs = ([1, 0], [0, 1], [-1 ,0], [0 -1])
		neighbors = []
		neighbor_num = 0

		for dir in dirs:
			map(add, curr_node)
			tmp = [curr_node + dir[0], curr_node + dir[1]]
			if (0 <= tmp[0] < self.y_size) and (0 <= tmp[1] < self.x_size and (tmp not in self.obstacle_list):
				neighbors.append(tmp)
				neighbor_num += 1

		return neighbors, neighbor_num
    '''
    def astar(self, curr_pos, goal_pos):
        curr_node = self.getGrid(curr_pos)
        goal_node = self.getGrid(goal_pos)
	    # if((curr_node in self.obstacle_list) or (goal_node in self.obstacle_list)):
	   # raise StandardError('Neither the goal nor the Current node can be obstacle nodes')
        print 'start in obstacle_list?', curr_node in self.obstacle_list
        print 'goal in obstacle_list?', curr_node in self.obstacle_list
        self.Astar = A_star.A_star(curr_node, goal_node, self.y_size, self.x_size, self.obstacle_list)
        if self.Astar.search():
            self.path = self.Astar.path
    
    def waveFront(self, curr_node, goal_node):
	if((curr_node in self.obstacle_list) or (goal_node in self.obstacle_list)):
	    raise StandardError('Neither the goal nor the Current node can be obstacle nodes')

	self.Astar = A_star.A_star(curr_node, goal_node, self.y_size, self.x_size, self.obstacle_list)
	value_mat = np.zeros((self.x_size, self.y_size))
        reached_curr_flag = 0
        explored_list = []
        curr_node_list = [goal_node]
	curr_value = 2
	increment = 1
        while(reached_curr_flag == 0):
	    next_node_list = set()
	    for node in curr_node_list:
		if(value_mat[node[0],node[1]]==0):
		    if(node==curr_node):
			reached_curr_flag = 1
		    value_mat[node[0],node[1]] = curr_value
		    neighbors, num_neighbors, obstacles = self.Astar.get_neighbors(node)
   		    next_node_list = next_node_list.union(neighbors)
		    self.fringe_obstacles = self.fringe_obstacles.union(obstacles)
	    curr_node_list = next_node_list	
	    curr_value += increment	
	for node in self.obstacle_list:
	    value_mat[node[0],node[1]] = 1

	self.wavefront_grid_vals = value_mat
    
    def wildFire(self):
	search_depth = 100
	increment = 1
	value_mat = np.zeros((self.x_size, self.y_size))
	current_neighbors = self.fringe_obstacles
	for curr_value in range(search_depth*increment,0,-increment):
	    next_node_list = set()
	    for node in current_neighbors:
		if(value_mat[node[0],node[1]]==0):
	            value_mat[node[0],node[1]] = curr_value
		    neighbors, num_neighbors, obstacles = self.Astar.get_neighbors(node)
		    next_node_list = next_node_list.union(neighbors)
	    current_neighbors = next_node_list
	self.wildfire_grid_vals = value_mat

    def firePath(self, curr_node, goal_node):
        path = [curr_node]
        previous_neighbors = [curr_node]
        while(path[-1]!=goal_node):
            next_node = []
            curr_node = path[-1]
            curr_wave_val = self.wavefront_grid_vals[curr_node[0],curr_node[1]]
            neighbors, num_neighbors, obstacles = self.Astar.get_neighbors(curr_node)
            node_tupes = []
            for node in neighbors:
		fire_val = self.wildfire_grid_vals[node[0],node[1]]
		wave_val = self.wavefront_grid_vals[node[0],node[1]]
                heur = self.Astar.get_heuristic(node)
                if((node not in path) and wave_val<=curr_wave_val):
		        node_tupes.append((node, fire_val, wave_val, heur))
            previous_neighbors.extend(neighbors)
              
            sorted_tupes = sorted(node_tupes, key = lambda t: (t[1], t[2]))

	    next_node = sorted_tupes[0][0]
            path.append(next_node) 
            
	    self.path = path
	    if(len(path)>200):    
                break        	
    
    def getGrid(self, pos_tup):
        x_pos = 6000.0 - pos_tup[1]
        y_pos = pos_tup[0]    
        x_val = int(((x_pos/1000.0)*MAP_RES_PIX_PER_M)/self.disc_size)
        y_val = int(((y_pos/1000.0)*MAP_RES_PIX_PER_M)/self.disc_size) 
        return (x_val, y_val)


    def getGrid(self, pos_tup):
        y_pos = pos_tup[0]
        x_pos = 6000.0-pos_tup[1] 
        print 'xpos', x_pos
        print 'y_pos', y_pos   
        x_val = int(((x_pos/1000.0)*MAP_RES_PIX_PER_M)/self.disc_size)
        y_val = int(((y_pos/1000.0)*MAP_RES_PIX_PER_M)/self.disc_size) 
        return (x_val, y_val)

    def getCoords(self, grid_tup):
        x_val = grid_tup[0]
        y_val = grid_tup[1]
        x_val = ((float(x_val*self.disc_size) + (self.disc_size/2.0))/MAP_RES_PIX_PER_M)*1000.0
        y_val = ((float(y_val*self.disc_size) + (self.disc_size/2.0))/MAP_RES_PIX_PER_M)*1000.0
        x_pos = y_val
        y_pos = 6000 - x_val
        return(x_pos, y_pos)

    def getPathCoords(self):
        coords_path = map(self.getCoords, self.path)
        return coords_path


