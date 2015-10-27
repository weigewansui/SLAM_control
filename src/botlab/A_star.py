import sys
class A_star:
  def __init__(self,init_node, goal, grid_n_rows, grid_n_cols, obstacle_list):
    init_cost = -1
    self.goal_node = goal
    self.closed_set = []
    self.open_set = [self.set_node(init_node, init_cost, 1)]
    self.path = []
    self.obstacles = obstacle_list;
    self.curr_node = init_node
    self.g_score = 0
    self.init_node = init_node
    self.n_rows = grid_n_rows
    self.n_cols = grid_n_cols
    self.obstacle_list = obstacle_list
  
  def set_node(self, pos_tup, cost, parent):
    heuristic = self.get_heuristic(pos_tup)
    return (pos_tup, cost+heuristic, cost, heuristic, parent)

  def get_neighbors(self, curr_node_tup):
    cost = curr_node_tup[2]
    curr_node = curr_node_tup[0]
    dirs = [(1, 0), (0, 1), (-1 ,0), (0,-1), (-1, -1), (1, -1), (-1, 1), (1, 1)]
    neighbors = []
    obstacles = []
    neighbor_num = 0

    for dir in dirs:
      tmp = (curr_node[0] + dir[0], curr_node[1] + dir[1])

      if (0 <= tmp[0] < self.n_rows) and (0 <= tmp[1] < self.n_cols) and (tmp not in self.obstacle_list):
        path_cost = 1

        # increase the path cost if it's nearby an obstacle
        for nearby in [(1, 0), (0, 1), (-1 ,0), (0,-1)]:
          if ((tmp[0] + nearby[0], tmp[1] + nearby[1]) in self.obstacle_list):
            path_cost += 2
        tmp_tup_node = self.set_node(tmp, curr_node_tup[2] + path_cost, curr_node_tup)
        neighbors.append(tmp_tup_node)
        neighbor_num += 1

      elif(tmp in self.obstacle_list):
        obstacles.append(tmp)
    return neighbors, neighbor_num, obstacles

  def update_init_node(self, new_init_node):
    self.init_node = new_init_node

  def get_heuristic(self, next_node):
    heuristic = abs(self.goal_node[0] - next_node[0]) + abs(self.goal_node[1] - next_node[1])
    return heuristic

  def get_path(self, goal_node):
    curr_node = goal_node
    path = []
    while(curr_node[4] != 1):
      path.insert(0,curr_node[0])
      curr_node = curr_node[4]
      self.path = path
    return path


  def search(self):
    if((self.goal_node in self.obstacle_list) or (self.init_node in self.obstacle_list)):
        raise StandardError("Fuck you, you're trying to plan into an obstacle")
    while self.open_set:
      s_list = sorted(self.open_set,key=lambda x: x[1])
      q = s_list[0] # choose node with minimum f to expand
      q_index = self.open_set.index(q)
      q = self.open_set.pop(q_index)
      neighbors, neighbor_num, obstacles = self.get_neighbors(q) 
      for neighbor in neighbors:
        if(neighbor[0]==self.goal_node):
          path = self.get_path(neighbor)
          return path

        pos_match_list_open = [item for item in self.open_set if item[0] == neighbor[0]]
        pos_match_list_closed = [item for item in self.closed_set if item[0] == neighbor[0]]
        unzipped_list_open = zip(*pos_match_list_open)
        unzipped_list_closed = zip(*pos_match_list_closed)
        if(unzipped_list_open):
          open_max = max(unzipped_list_open[1])
        else:
          open_max = 0
        if(unzipped_list_closed):
          closed_max = max(unzipped_list_closed[1])
        else:
          closed_max = 0

        if((neighbor[1] < open_max) or (neighbor[1] < closed_max)):
          continue

        else:
          self.open_set.append(neighbor)

        self.closed_set.append(q)
                            
    return False

  def print_path(self):
    print self.path
