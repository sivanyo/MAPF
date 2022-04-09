import time as timer
from single_agent_planner import compute_heuristics, a_star, get_sum_of_cost


class PrioritizedPlanningSolver(object):
    """A planner that plans for each robot sequentially."""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.CPU_time = 0

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def find_solution(self):
        """ Finds paths for all agents from their start locations to their goal locations."""

        start_time = timer.time()
        result = []
        constraints = []
        """
        constrains for part 1.3 :
        """
        # constraints.append({'agent': 0, 'loc': [(1,5)], 'ts': 4})
        # constraints.append({'agent': 1, 'loc': [(1, 2), (1, 3)], 'ts': 0})
        """
        constrains for part 1.4 + 1.5 :
        """
        # constraints.append({'agent': 1, 'loc': [(1, 2)], 'ts': 2})
        # constraints.append({'agent': 1, 'loc': [(1, 4)], 'ts': 2})
        max_path_len = 0

        # first loop
        for i in range(self.num_of_agents):  # Find path for each agent
            max_num_of_iterations = max_path_len + len(self.my_map) * len(self.my_map[0])
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, constraints)
            if path is None or len(path) >= max_num_of_iterations:
                raise BaseException('No solutions')
            result.append(path)
            max_num_of_iterations = max_num_of_iterations if max_num_of_iterations > len(path) else len(path)

            # second loop
            ##############################
            # Task 2: Add constraints here
            #         Useful variables:
            #            * path contains the solution path of the current (i'th) agent, e.g., [(1,1),(1,2),(1,3)]
            #            * self.num_of_agents has the number of total agents
            #            * constraints: array of constraints to consider for future A* searches

            ##############################
            for ts in range(max_num_of_iterations + 1):
                for agent in range(i, self.num_of_agents):
                    if ts < len(path):
                        constraints.append({'agent': agent, 'loc': [path[ts]], 'time_step': ts})
                        # 2.2 addition
                        if ts != 0:
                            constraints.append({'agent': agent, 'loc': [path[ts], path[ts - 1]], 'time_step': ts})
                    else:
                        constraints.append({'agent': agent, 'loc': [path[-1]], 'time_step': ts})

        self.CPU_time = timer.time() - start_time

        print("\n Found a solution! \n")
        print("CPU time (s):    {:.2f}".format(self.CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(result)))
        print(result)
        return result
