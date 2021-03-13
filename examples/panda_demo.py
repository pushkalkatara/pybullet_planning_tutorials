import os
import sys
import numpy as np
import argparse
from termcolor import cprint

from pybullet_planning import BASE_LINK, RED, BLUE, GREEN
from pybullet_planning import load_pybullet, connect, wait_for_user, LockRenderer, has_gui, WorldSaver, HideOutput, \
    reset_simulation, disconnect, set_camera_pose, has_gui, set_camera, wait_for_duration, wait_if_gui, apply_alpha
from pybullet_planning import Pose, Point, Euler
from pybullet_planning import multiply, invert, get_distance
from pybullet_planning import create_obj, create_attachment, Attachment
from pybullet_planning import link_from_name, get_link_pose, get_moving_links, get_link_name, get_disabled_collisions, \
    get_body_body_disabled_collisions, has_link, are_links_adjacent
from pybullet_planning import get_num_joints, get_joint_names, get_movable_joints, set_joint_positions, joint_from_name, \
    joints_from_names, get_sample_fn, plan_joint_motion
from pybullet_planning import dump_world, set_pose
from pybullet_planning import get_collision_fn, get_floating_body_collision_fn, expand_links, create_box
from pybullet_planning import pairwise_collision, pairwise_collision_info, draw_collision_diagnosis, body_collision_info

HERE = os.path.dirname(__file__)
UR_ROBOT_URDF = os.path.join(HERE, 'data', 'universal_robot', 'ur_description', 'urdf', 'ur5.urdf')
RFL_ROBOT_URDF = os.path.join(HERE, 'data', 'eth_rfl_robot', 'eth_rfl_description', 'urdf', 'eth_rfl.urdf')

MIT_WORKSPACE_PATH = os.path.join(HERE, 'data', 'mit_3-412_workspace', 'urdf', 'mit_3-412_workspace.urdf')
EE_PATH = os.path.join(HERE, 'data', 'dms_bar_gripper.obj')
ATTACH_OBJ_PATH = os.path.join(HERE, 'data', 'bar_attachment.obj')
OBSTACLE_OBJ_PATH = os.path.join(HERE, 'data', 'box_obstacle.obj')
DUCK_OBJ_PATH = os.path.join(HERE, 'data', 'duck.obj')
ASSEMBLY_OBJ_DIR = os.path.join(HERE, 'data', 'kathrin_assembly')

TUTORIALS = {'DUCK', 'UR', 'RFL', 'Assembly', 'PANDA'}

def panda_demo(viewer=True):
    arm='right'

    connect(use_gui=viewer)
    robot = load_pybullet("franka_panda/panda.urdf", fixed_base=True)
    tableUid = load_pybullet("table/table.urdf", fixed_base=True, basePosition=[0.5,0,-0.65])
    #workspace = load_pybullet("plane.urdf", fixed_base=True)
    set_camera(yaw=0, pitch=-40, distance=1.5, target_position=(0.55, -0.35, 0.2))

    cprint('hello Panda! <ctrl+left mouse> to pan', 'green')
    wait_for_user()

    # create a box to be picked up
    # see: https://pybullet-planning.readthedocs.io/en/latest/reference/generated/pybullet_planning.interfaces.env_manager.create_box.html#pybullet_planning.interfaces.env_manager.create_box
    block = create_box(0.2, 0.2, 0.2)
    block_x = 0.2
    block_y = 0.3
    block_z = 0.0
    set_pose(block, Pose(Point(x=block_x, y=block_y, z=block_z), Euler(yaw=np.pi/2)))

    ik_joints = get_movable_joints(robot)
    ik_joint_names = get_joint_names(robot, ik_joints)
    
    # * if a subset of joints is used, use:
    #panda_joints = joints_from_names(robot, ik_joint_names[:7]) # this will disable the gantry-x joint
    cprint('Used joints: {}'.format(get_joint_names(robot, ik_joints)), 'yellow')

    # * get a joint configuration sample function:
    # it separately sample each joint value within the feasible range
    sample_fn = get_sample_fn(robot, ik_joints)

    cprint('Randomly sample robot configuration and set them! (no collision check is performed)', 'blue')
    wait_for_user()
    for i in range(5):
        # randomly sample a joint conf
        sampled_conf = sample_fn()
        set_joint_positions(robot, ik_joints, sampled_conf)
        cprint('#{} | Conf sampeld: {}'.format(i, sampled_conf), 'green')
        wait_for_user()

    # * now let's plan a trajectory
    # we use y-z-arm 6 joint all together here
    cprint('Randomly sample robot start/end configuration and comptue a motion plan! (no self-collision check is performed)', 'blue')
    print('Disabled collision links needs to be given (can be parsed from a SRDF via compas_fab)')
    for _ in range(5):
        print('='*10)

        q1 = list(sample_fn())
        # intentionly make the robot needs to cross the collision object
        # let it start from the right side
        q1[0] = 0.
        q1[1] = 0

        set_joint_positions(robot, ik_joints, q1)
        cprint('Sampled start conf: {}'.format(q1), 'cyan')
        wait_for_user()

        # let it ends at the left side
        q2 = list(sample_fn())
        q2[0] = 0.5
        q2[1] = 0.5
        cprint('Sampled end conf: {}'.format(q2), 'cyan')

        path = plan_joint_motion(robot, ik_joints, q2, obstacles=[block], self_collisions=False,
            custom_limits={ik_joints[0]:[0.0, 1.2]})
        if path is None:
            cprint('no plan found', 'red')
            continue
        else:
            wait_for_user('a motion plan is found! Press enter to start simulating!')

        # adjusting this number will adjust the simulation speed
        time_step = 0.03
        for conf in path:
            set_joint_positions(robot, ik_joints, conf)
            wait_for_duration(time_step)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-nv', '--noviewer', action='store_true', help='Enables the viewer during planning, default True')
    parser.add_argument('-d', '--demo', default='UR', choices=TUTORIALS, \
        help='The name of the demo')
    parser.add_argument('-db', '--debug', action='store_true', help='Debug mode')
    args = parser.parse_args()
    print('Arguments:', args)

    if args.demo == 'PANDA':
        panda_demo()
    else:
        raise NotImplementedError()


if __name__ == '__main__':
    main()
