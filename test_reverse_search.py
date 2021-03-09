from pyddl import Domain, Problem, State, Action, neg, backwards_planner

"""
Logic space for planner
"""
# Object types
type_gripper = "gripper"
type_block = "block"

# logic states predicates
focus = "focus"  # set focus on a block block
in_hand = "in_hand"  # a block is in hand (block, gripper)
block_free = "block_free"  # condition a block is graspable
hand_empty = "hand_empty"  # condition a hand is free
b_on_b = "block_on_block"  # condition block is on block


actions = [Action("Grab_Block",
              parameters=(
                  (type_gripper, "g"),
                  (type_block, "b"),
              ),
              preconditions=(
                  (hand_empty, "g"),
                  (block_free, "b")
              ),
              effects=(
                  neg((hand_empty, "g")),
                  (in_hand, "b", "g"),
              ),
              unique=True),
           Action("Place_Block",
              parameters=(
                  (type_gripper, "g"),
                  (type_block, "b"),
                  (type_block, "b_placed")
              ),
              preconditions=(
                  (in_hand, "b", "g"),
                  (block_free, "b_placed")
              ),
              effects=(
                  (b_on_b, "b", "b_placed"),
                  neg((block_free, "b_placed")),
                  neg((in_hand, "b", "g")),
                  (hand_empty, "g"),
              ),
              unique=True)
    ]

# setup config and get frame names
block_names = [f"b{x}" for x in range(1, 4)]
gripper_name = "R_gripper"

# put all objects into one dictionary with types
scene_obj = {
    type_block: block_names,
    type_gripper: (gripper_name,)
}

# get simple action from all controllers
domain = Domain(actions)

# goal is for now numerical order of block placed on each other, unless specified otherwise
goal = [(b_on_b, scene_obj[type_block][i], scene_obj[type_block][i + 1])\
        for i in range(len(scene_obj[type_block]) - 1)]
# also append free hand
goal.append((hand_empty, scene_obj[type_gripper][0]))

# normal initial conditions
init_free_hand = (hand_empty, scene_obj[type_gripper][0])
init_free_blocks = [(block_free, block) for block in scene_obj[type_block]]

# extend all initial conditions
init = [init_free_hand]
init.extend(init_free_blocks)

# define problem here
prob = Problem(
    domain,
    scene_obj,
    init=init,
    goal=goal
)

plan = backwards_planner(prob, goal=goal)

print("\nFinal Plan:")
for a in plan:
    print(a)

