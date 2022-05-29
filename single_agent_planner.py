import heapq


def move(loc, dir):
    directions = [(0, -1), (1, 0), (0, 1), (-1, 0), (0, 0)]
    return loc[0] + directions[dir][0], loc[1] + directions[dir][1]


def get_sum_of_cost(paths):
    rst = 0
    for path in paths:
        rst += len(path) - 1
    return rst


def compute_heuristics(my_map, goal):
    # Use Dijkstra to build a shortest-path tree rooted at the goal location
    open_list = []
    closed_list = dict()
    root = {'loc': goal, 'cost': 0}
    heapq.heappush(open_list, (root['cost'], goal, root))
    closed_list[goal] = root
    while len(open_list) > 0:
        (cost, loc, curr) = heapq.heappop(open_list)
        for dir in range(4):
            child_loc = move(loc, dir)
            child_cost = cost + 1
            if child_loc[0] < 0 or child_loc[0] >= len(my_map) \
               or child_loc[1] < 0 or child_loc[1] >= len(my_map[0]):
               continue
            if my_map[child_loc[0]][child_loc[1]]:
                continue
            child = {'loc': child_loc, 'cost': child_cost}
            if child_loc in closed_list:
                existing_node = closed_list[child_loc]
                if existing_node['cost'] > child_cost:
                    closed_list[child_loc] = child
                    # open_list.delete((existing_node['cost'], existing_node['loc'], existing_node))
                    heapq.heappush(open_list, (child_cost, child_loc, child))
            else:
                closed_list[child_loc] = child
                heapq.heappush(open_list, (child_cost, child_loc, child))

    # build the heuristics table
    h_values = dict()
    for loc, node in closed_list.items():
        h_values[loc] = node['cost']
    return h_values


def build_constraint_table(constraints, agent) -> dict:
    # Task 1.2/1.3: Return a table that contains the list of constraints of
    #               the given agent for each time step. The table can be used
    #               for a more efficient constraint violation check in the
    #               is_constrained function.
    time_step_dict = {}
    for constraint in constraints:
        if constraint['agent'] == agent:
            time = constraint['time_step']
            if time in time_step_dict.keys():
                time_step_dict[time].append(constraint)
            else:
                time_step_dict[time] = [constraint]
    return time_step_dict


def get_location(path, time):
    if time < 0:
        return path[0]
    elif time < len(path):
        return path[time]
    else:
        return path[-1]  # wait at the goal location


def get_path(goal_node):
    path = []
    curr = goal_node
    while curr is not None:
        path.append(curr['loc'])
        curr = curr['parent']
    path.reverse()
    return path


def is_future_constraints_on_goal(goal_loc, time_step, constraint_table):
    for key in constraint_table.keys():
        if key > time_step:
            for constraint in constraint_table[key]:
                location = constraint['loc']
                if len(location) == 2:
                    if location[0] == goal_loc and location[1] == goal_loc:
                        return True
                elif location[0] == goal_loc:
                    return True
    return False


def is_constrained(curr_loc, next_loc, next_time, constraint_table: dict):
    ##############################
    if next_time not in constraint_table.keys():
        return False
    constraints = constraint_table[next_time]
    for constraint in constraints:
        location = constraint['loc']
        if len(location) == 2:
            if location[0] == curr_loc and location[1] == next_loc or \
                    location[0] == next_loc and location[1] == curr_loc:
                return True
        elif location[0] == next_loc:
            return True
    return False

    # Task 1.2/1.3: Check if a move from curr_loc to next_loc at time step next_time violates
    #               any given constraint. For efficiency the constraints are indexed in a constraint_table
    #               by time step, see build_constraint_table.


def is_location_out_of_boundaries(child_loc, my_map: list):
    if child_loc[0] < 0 or child_loc[1] < 0 or child_loc[0] >= len(my_map) or child_loc[1] >= len(my_map[0]):
        return True
    return False


def push_node(open_list, node):
    heapq.heappush(open_list, (node['g_val'] + node['h_val'], node['h_val'], node['loc'], node))


def pop_node(open_list):
    _, _, _, curr = heapq.heappop(open_list)
    return curr


def compare_nodes(n1, n2):
    """Return true is n1 is better than n2."""
    return n1['g_val'] + n1['h_val'] < n2['g_val'] + n2['h_val']


def a_star(my_map, start_loc, goal_loc, h_values, agent, constraints):
    """ my_map      - binary obstacle map
        start_loc   - start position
        goal_loc    - goal position
        agent       - the agent that is being re-planned
        constraints - constraints defining where robot should or cannot go at each time step
    """
    ##############################
    # Task 1.1: Extend the A* search to search in the space-time domain
    #           rather than space domain, only.

    open_list = []
    closed_list = dict()
    h_value = h_values[start_loc]

    c_table = dict(build_constraint_table(constraints, agent))
    root = {'loc': start_loc, 'g_val': 0, 'h_val': h_value, 'parent': None, 'time_step': 0}
    push_node(open_list, root)
    closed_list[(root['loc'], root['time_step'])] = root
    while len(open_list) > 0:
        curr = pop_node(open_list)
        #############################
        # Task 1.4: Adjust the goal test condition to handle goal constraints
        if curr['loc'] == goal_loc and not is_future_constraints_on_goal(goal_loc, curr['time_step'], c_table):
            return get_path(curr)
        for direction in range(5):
            child_loc = move(curr['loc'], direction)
            if is_location_out_of_boundaries(child_loc, my_map) or my_map[child_loc[0]][child_loc[1]] or \
                    is_constrained(curr['loc'], child_loc, curr['time_step'] + 1, c_table):
                # prone if found a constraint
                continue
            child = {'loc': child_loc,
                     'g_val': curr['g_val'] + 1,
                     'h_val': h_values[child_loc],
                     'parent': curr,
                     'time_step': curr['time_step'] + 1}
            if (child['loc'], child['time_step']) in closed_list:
                existing_node = closed_list[(child['loc'], child['time_step'])]
                if compare_nodes(child, existing_node):
                    closed_list[(child['loc']), child['time_step']] = child
                    push_node(open_list, child)
            else:
                closed_list[(child['loc']), child['time_step']] = child
                push_node(open_list, child)

    return None  # Failed to find solutions

