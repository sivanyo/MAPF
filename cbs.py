import time as timer
import heapq
import random
from single_agent_planner import compute_heuristics, a_star, get_location, get_sum_of_cost


def detect_collision(path1, path2):
    ##############################
    # Task 3.1: Return the first collision that occurs between two robot paths (or None if there is no collision)
    #           There are two types of collisions: vertex collision and edge collision.
    #           A vertex collision occurs if both robots occupy the same location at the same timestep
    #           An edge collision occurs if the robots swap their location at the same timestep.
    #           You should use "get_location(path, t)" to get the location of a robot at time t.
    num_ts = len(path1) if len(path1) >= len(path2) else len(path2)
    for ts in range(num_ts):
        path1_loc = get_location(path1, ts)
        path2_loc = get_location(path2, ts)
        # vertex collision
        if path1_loc == path2_loc:
            return [path1_loc, ts]
        prev_path1_loc = get_location(path1, ts - 1)
        prev_path2_loc = get_location(path2, ts - 1)
        # edge collision
        if path1_loc == prev_path2_loc and path2_loc == prev_path1_loc:
            return [prev_path1_loc, path1_loc, ts]

    # there are no collisions
    return None


def detect_collisions(paths):
    ##############################
    # Task 3.1: Return a list of first collisions between all robot pairs.
    #           A collision can be represented as dictionary that contains the id of the two robots, the vertex or edge
    #           causing the collision, and the timestep at which the collision occurred.
    #           You should use your detect_collision function to find a collision between two robots.
    collisions = []
    for first_path in range(len(paths) - 1):
        for second_path in range(first_path + 1, len(paths)):
            possible_collision = detect_collision(paths[first_path], paths[second_path])
            if possible_collision is not None:
                collisions.append({'a1': first_path,
                                   'a2': second_path,
                                   'loc': possible_collision[:-1],
                                   'time_step': possible_collision[-1]})
    return collisions


def standard_splitting(collision):
    ##############################
    # Task 3.2: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint prevents the first agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the second agent to be at the
    #                            specified location at the specified timestep.
    #           Edge collision: the first constraint prevents the first agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the second agent to traverse the
    #                          specified edge at the specified timestep
    first_constraint = {'agent': collision['a1'],
                        'loc': collision['loc'],
                        'time_step': collision['time_step']}
    loc = list(collision['loc'])
    loc.reverse()
    second_constraint = {'agent': collision['a2'],
                         'loc': loc,
                         'time_step': collision['time_step']}
    return [first_constraint, second_constraint]

    pass


def disjoint_splitting(collision):
    ##############################
    # Task 4.1: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint enforces one agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the same agent to be at the
    #                            same location at the timestep.
    #           Edge collision: the first constraint enforces one agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the same agent to traverse the
    #                          specified edge at the specified timestep
    #           Choose the agent randomly

    pass


class CBSSolver(object):
    """The high-level search of CBS."""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.num_of_generated = 0
        self.num_of_expanded = 0
        self.CPU_time = 0

        self.open_list = []

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def push_node(self, node):
        heapq.heappush(self.open_list, (node['cost'], len(node['collisions']), self.num_of_generated, node))
        print("Generate node {}".format(self.num_of_generated))
        self.num_of_generated += 1

    def pop_node(self):
        _, _, id, node = heapq.heappop(self.open_list)
        print("Expand node {}".format(id))
        self.num_of_expanded += 1
        return node

    def find_solution(self, disjoint=True):
        """ Finds paths for all agents from their start locations to their goal locations

        disjoint    - use disjoint splitting or not
        """

        self.start_time = timer.time()

        # Generate the root node
        # constraints   - list of constraints
        # paths         - list of paths, one for each agent
        #               [[(x11, y11), (x12, y12), ...], [(x21, y21), (x22, y22), ...], ...]
        # collisions     - list of collisions in paths
        root = {'cost': 0,
                'constraints': [],
                'paths': [],
                'collisions': []}
        for i in range(self.num_of_agents):  # Find initial path for each agent
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, root['constraints'])
            if path is None:
                raise BaseException('No solutions')
            root['paths'].append(path)

        root['cost'] = get_sum_of_cost(root['paths'])
        root['collisions'] = detect_collisions(root['paths'])
        self.push_node(root)

        # Task 3.1: Testing
        print(root['collisions'])

        # Task 3.2: Testing
        for first_collision in root['collisions']:
            print(standard_splitting(first_collision))

        ##############################
        # Task 3.3: High-Level Search
        #           Repeat the following as long as the open list is not empty:
        #             1. Get the next node from the open list (you can use self.pop_node()
        #             2. If this node has no collision, return solution
        #             3. Otherwise, choose the first collision and convert to a list of constraints (using your
        #                standard_splitting function). Add a new child node to your open list for each constraint
        #           Ensure to create a copy of any objects that your child nodes might inherit

        while len(self.open_list) > 0:
            curr_node = self.pop_node()
            if not curr_node['collisions']:
                self.print_results(curr_node)
                return curr_node['paths']
            first_collision = curr_node['collisions'][0]
            new_constraints = standard_splitting(first_collision)
            for constraint in new_constraints:
                agent = constraint['agent']
                curr_agent_constraints = list(curr_node['constraints']).copy()
                curr_agent_constraints.append(constraint)
                list_paths = list(curr_node['paths']).copy()

                # perform a star searching
                list_paths[agent] = a_star(self.my_map, self.starts[agent], self.goals[agent], self.heuristics[agent],
                                           agent, curr_agent_constraints)

                if list_paths[agent] is not None:
                    child_node = {'cost': get_sum_of_cost(list_paths),
                                  'constraints': curr_agent_constraints,
                                  'paths': list_paths,
                                  'collisions': detect_collisions(list_paths)}
                    self.push_node(child_node)
        raise BaseException('No solutions')

        self.print_results(root)
        return root['paths']


    def print_results(self, node):
        print("\n Found a solution! \n")
        CPU_time = timer.time() - self.start_time
        print("CPU time (s):    {:.2f}".format(CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(node['paths'])))
        print("Expanded nodes:  {}".format(self.num_of_expanded))
        print("Generated nodes: {}".format(self.num_of_generated))
