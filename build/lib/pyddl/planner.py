from __future__ import print_function
from time import time
from pyddl import State
import heapq

def planner(problem, heuristic=None, state0=None, goal=None,
            monotone=False, verbose=True):
    """
    Implements A* search to find a plan for the given problem.
    Arguments:
    problem   - a pyddl Problem
    heuristic - a heuristic to use (h(state) = 0 by default)
    state0    - initial state (problem.initial_state by default)
    goal      - tuple containing goal predicates and numerical conditions
                (default is (problem.goals, problem.num_goals))
    monotone  - if True, only applies actions by ignoring delete lists
    verbose   - if True, prints statistics before returning
    """
    if heuristic is None:
        heuristic = null_heuristic
    if state0 is None:
        state0 = problem.initial_state
    if goal is None:
        goal = (problem.goals, problem.num_goals)

    states_explored = 0
    closed = set()
    fringe = [(heuristic(state0), -state0.cost, state0)]
    heapq.heapify(fringe)
    start = time()
    while True:
        if len(fringe) == 0:
            if verbose: print('States Explored: %d' % states_explored)
            return None

        # Get node with minimum evaluation function from heap
        h, _, node = heapq.heappop(fringe)
        states_explored += 1

        # Goal test
        if node.is_true(*goal):
            plan = node.plan()
            dur = time() - start
            if verbose:
                print('States Explored: %d' % states_explored)
                print('Time per state: %.3f ms' % (1000*dur / states_explored))
                print('Plan length: %d' % node.cost)
            return plan

        # Expand node if we haven't seen it before
        if node not in closed:
            closed.add(node)

            # Apply all applicable actions to get successors
            successors = set(node.apply(action, monotone)
                             for action in problem.grounded_actions
                             if node.is_true(action.preconditions,
                                             action.num_preconditions))

            # Compute heuristic and add to fringe
            for successor in successors:
                if successor not in closed:
                    f = successor.cost + heuristic(successor)
                    heapq.heappush(fringe, (f, -successor.cost, successor))


def backwards_planner(problem, heuristic=None, state0=None, goal=None,
            monotone=False, verbose=True):
    if heuristic is None:
        heuristic = null_heuristic
    if state0 is None:
        state0 = problem.initial_state
    if goal is None:
        goal = (problem.goals, problem.num_goals)

    states_explored = 0
    start = time()

    plan = []
    goals = [(State(goal, functions={}), plan)]

    init = state0

    def get_subgoal(g,  action_grounded):

        # check if action is relevant
        add_effects = set(action_grounded.add_effects)
        del_effects = set(action_grounded.del_effects)
        intersection_add = g.intersection(add_effects)
        intersection_del = g.intersection(del_effects)
        if len(intersection_add) != 0 and len(intersection_del) == 0:
            # print(action, intersection_add, intersection_del)
            subgoal = g.difference(add_effects).union(action_grounded.preconditions)
            return subgoal


    depth = 0
    while True:

        new_subgoals = []
        if len(goals) == 0:
            print("didnt find start :(")
            return



        for subgoal, plan in goals:
            if subgoal.is_true(init.predicates, set()):
                print(f"found start with {depth} actions in plan!")
                print(subgoal.predicates)
                for a in plan:
                    print(a)
                is_realisable = True

                state = state0
                for action in plan:
                    if state.is_true(action.preconditions, action.num_preconditions):
                        state = state.apply(action)
                        print(f"{action} IS realisable!!")
                    else:
                        print(f"{action} not realisable")
                        is_realisable = False
                        break
                if is_realisable:
                    #check is goal is last state is in goal
                    if state.is_true(goal, set()):
                        print("Goal is also realisible!")
                        return plan
                    else:
                        print("Goal is not realisible")
                        continue

            for action in problem.grounded_actions:
                new_subgoal = get_subgoal(subgoal.predicates, action)
                if new_subgoal:
                    state = State(new_subgoal, functions={}, predecessor=goal)
                    new_plan = list(plan)
                    new_plan.insert(0, action)
                    new_subgoals.append((state, new_plan))
        goals = new_subgoals


        if depth > 4:
            print("After depth of 4 didnt find plan :(")
            return

        # debugging
        print(depth)
        for temp, temp_plan in goals:
            print("----current----")
            for a in temp_plan:
                print(a)

        depth = depth + 1


########## HEURISTICS ##########

def null_heuristic(state):
    """Admissible, but trivial heuristic"""
    return 0

def plan_cost(plan):
    """Convert a plan to a cost, handling nonexistent plans"""
    if plan is None:
        return float('inf')
    else:
        return len(plan)

def monotone_heuristic(problem):
    """Heuristic that finds plans using only add lists of actions"""
    def h(state):
        monotone_plan = planner(problem, null_heuristic, state, monotone=True, verbose=False)
        return plan_cost(monotone_plan)
    return h

def subgoal_heuristic(problem):
    """Heuristic that computes the max cost of plans across all subgoals"""
    def h(state):
        costs = []
        for g in problem.goals:
            subgoal_plan = planner(problem, null_heuristic, state, ((g,), ()))
            costs.append(plan_cost(subgoal_plan))
        for g in problem.num_goals:
            subgoal_plan = planner(problem, null_heuristic, state, ((), (g,)))
            costs.append(plan_cost(subgoal_plan))
        return max(costs)
    return h
