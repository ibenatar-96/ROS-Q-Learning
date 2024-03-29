#!/usr/bin/env python3


"""

"""

import rospy
import subprocess
import time
from itertools import permutations, product
from task4_env.srv import navigate, info, place, pick
import csv
import random
import math

CSV_FILE = "../q_table.csv"
MAX_NAV = 8
MAX_PICK = 6
NUM_OF_ITERATIONS = 2000

nav_count = 0
pick_count = 0

prev_info = None

Q = {}
TERMINATE_STATE = [(0,4,4,4,4),(1,4,4,4,4),(2,4,4,4,4),(3,4,4,4,4),(4,4,4,4,4)]

def handle_csv(q_learn):
    global Q
    if q_learn == 0:
        Q = load_q_table()
    elif q_learn == 1:
        Q = create_q_table()
    elif q_learn == 2:
        Q = load_q_table()
    else:
        print("ERROR, Q Learn arg can be 0, 1, or 2.")
        terminate()
        exit(1)
    print(f"Enable Q Learning: {q_learn}")

def save_q_table(q_table, file=None):
    actions = {navigate_to_loc_0:'navigate_to_loc_0', 
        navigate_to_loc_1:'navigate_to_loc_1', 
        navigate_to_loc_2:'navigate_to_loc_2', 
        navigate_to_loc_3:'navigate_to_loc_3', 
        navigate_to_loc_4:'navigate_to_loc_4', 
        pick_toy:'pick_toy', 
        place_toy:'place_toy'}
    global CSV_FILE
    csv_file = CSV_FILE if file is None else file
    _states = set([state for (state, _) in q_table.keys()])
    # _actions = set([actions[action] for (_, action) in q_table.keys()])
    print(f"Saving Q Table to: {csv_file}")
    with open(csv_file, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)

        # Write header row with actions as column names
        list_actions = list(actions.values())
        writer.writerow(['State'] + list_actions)

        # Write each state as a row with corresponding Q-values for each action
        for _state in _states:
            row = [_state]
            for i,action in enumerate(actions.keys()):
                if actions[action] != list_actions[i]:
                    print(f"ERROR: \n\tSAVE_Q_TABLE ACTIONS[{action}] not eq. list_actions[{i}]: {actions[action]} != {list_actions[i]}")
                    terminate()
                    exit(1)
                q_value = q_table[_state, action]
                row.append(q_value)
            writer.writerow(row)

def load_q_table():
    Q = {}
    actions = {'navigate_to_loc_0': navigate_to_loc_0, 
            'navigate_to_loc_1': navigate_to_loc_1, 
            'navigate_to_loc_2': navigate_to_loc_2, 
            'navigate_to_loc_3': navigate_to_loc_3, 
            'navigate_to_loc_4': navigate_to_loc_4, 
            'pick_toy': pick_toy, 
            'place_toy': place_toy}

    with open(CSV_FILE, "r") as f:
        csvFile = csv.DictReader(f)
        for line in csvFile:
            state = tuple(map(int, line['State'].strip()[1:-1].split(", ")))
            for (key,action),val in { (state, key): eval(line[key]) for key in line.keys() if key != 'State'}.items():
                action_func = actions[action]
                Q[(key,action_func)] = val
    return Q


def create_q_table():
    Q = {}
    actions = [navigate_to_loc_0, navigate_to_loc_1, navigate_to_loc_2, navigate_to_loc_3, navigate_to_loc_4, pick_toy, place_toy]

    permutations_list = []
    for permutation in [element for element in product([0,1,2,3,4,5], repeat=4)]:
        for first_element in range(5):
            state = tuple([first_element]) + tuple(permutation)
            permutations_list.append(state)
    for opt in list(product(permutations_list, actions)):
        Q[opt] = 0
    save_q_table(Q)
    return Q

def navigate_to_loc_0():
    print(f"Navigating to location 0")
    resp = False
    try:
        global nav_count
        while nav_count < MAX_NAV and not resp:
            navigate_srv = rospy.ServiceProxy('/navigate', navigate)
            resp = navigate_srv(0)
            info = get_info()
            resp = bool(info['state']['robot_location'] == 0)
            nav_count += 1
            if not resp:
                print("Navigation to location 0 failed")
        print(f"Finished navigating to location 0, success: {resp}, nav_count: {nav_count}")
        return resp
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def navigate_to_loc_1():
    print(f"Navigating to location 1")
    resp = False
    try:
        global nav_count
        while nav_count < MAX_NAV and not resp:
            navigate_srv = rospy.ServiceProxy('/navigate', navigate)
            resp = navigate_srv(1)
            info = get_info()
            resp = bool(info['state']['robot_location'] == 1)
            nav_count += 1
            if not resp:
                print("Navigation to location 1 failed")
        print(f"Finished navigating to location 1, success: {resp}, nav_count: {nav_count}")
        return resp
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def navigate_to_loc_2():
    print(f"Navigating to location 2")
    resp = False
    try:
        global nav_count
        while nav_count < MAX_NAV and not resp:
            navigate_srv = rospy.ServiceProxy('/navigate', navigate)
            resp = navigate_srv(2)
            info = get_info()
            resp = bool(info['state']['robot_location'] == 2)
            nav_count += 1
            if not resp:
                print("Navigation to location 2 failed")
        print(f"Finished navigating to location 2, success: {resp}, nav_count: {nav_count}")
        return resp
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def navigate_to_loc_3():
    print(f"Navigating to location 3")
    resp = False
    try:
        global nav_count
        while nav_count < MAX_NAV and not resp:
            navigate_srv = rospy.ServiceProxy('/navigate', navigate)
            resp = navigate_srv(3)
            info = get_info()
            resp = bool(info['state']['robot_location'] == 3)
            nav_count += 1
            if not resp:
                print("Navigation to location 3 failed")
        print(f"Finished navigating to location 3, success: {resp}, nav_count: {nav_count}")
        return resp
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def navigate_to_loc_4():
    print(f"Navigating to location 4")
    resp = False
    try:
        global nav_count
        while nav_count < MAX_NAV and not resp:
            navigate_srv = rospy.ServiceProxy('/navigate', navigate)
            resp = navigate_srv(4)
            info = get_info()
            resp = bool(info['state']['robot_location'] == 4)
            nav_count += 1
            if not resp:
                print("Navigation to location 4 failed")
        print(f"Finished navigating to location 4, success: {resp}, nav_count: {nav_count}")
        return resp
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def place_toy():
    print(f"Placing toy")
    try:
        place_srv = rospy.ServiceProxy('/place', place)
        resp = place_srv()
        return resp.success
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def pick_toy():
    resp = False
    try:
        info = get_info()
        location = info['state']['robot_location']
        toy_type = info['state']['locations_toy'].get(location, 'None')
        print(f"Pick up toy type {toy_type} at location {location}")
        global pick_count
        if pick_count < MAX_PICK:
            pick_srv = rospy.ServiceProxy('/pick', pick)
            resp = pick_srv(toy_type)
            resp = resp.success
            pick_count += 1
        print(f"Finished picking up at {location}, success: {resp}, pick_count: {pick_count}")
        return resp
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def get_info(print_info=True):
    print(f"Getting /info")
    try:
        info_srv = rospy.ServiceProxy('/info', info)
        resp = info_srv()
        return parse_info(resp.internal_info, print_info=print_info)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def parse_info(info_str, print_info=True):
    info_dict = {}
    lines = info_str.splitlines()  # Split the string by lines

  # Extract state information
    for line in lines:
        if line.startswith("state:"):
            state = {}
            state_str = line[line.find(":")+1:]
            renamed_keys = {
                "robot location": "robot_location",
                "toys_location": "toys_location",
                "locations_toy": "locations_toy",
                "toys_reward": "toys_reward",
                "holding_toy": "holding_toy"
            }
            renamed_keys_index = []
            for key in renamed_keys:
                renamed_keys_index.append(state_str.find(key))
            renamed_keys_index.append(len(state_str)-1)
            renamed_keys_index.sort()
            for i in range(len(renamed_keys_index)-1):
                start = renamed_keys_index[i]
                end = renamed_keys_index[i+1]
                tmp = state_str[start:end].strip()
                tmp = tmp.split(":", 1)
                key = renamed_keys[tmp[0]]
                val = eval(tmp[1])
                state[key] = val
            info_dict['state'] = state

        if line.startswith("total rewards:"):
            global prev_info
            total_rewards = eval(line[line.find(":")+1:])
            info_dict['total_rewards'] = total_rewards
            if prev_info is None:
                info_dict['immediate_reward'] = total_rewards
            else:
                info_dict['immediate_reward'] = total_rewards - prev_info['total_rewards']
            if print_info:
                print(f"DEBUG: \n\ttotal_rewards = {info_dict['total_rewards']}, immediate_reward = {info_dict['immediate_reward']}")
    return info_dict

def setup_env():
    global nav_count, pick_count, prev_info
    prev_info = None
    navigate_to_loc_4()
    navigate_to_loc_4()
    nav_count = pick_count = 0
    prev_info = None

    subprocess.run('rosnode kill skills_server_node', shell=True)
    time.sleep(2)
    subprocess.Popen('rosrun task4_env skills_server.py', shell=True)
    time.sleep(2)

def bring_down_env():
    subprocess.run('rosnode kill skills_server_node', shell=True)
    time.sleep(2)
    subprocess.Popen('rosrun task4_env skills_server.py', shell=True)
    time.sleep(2)
    global nav_count, pick_count
    nav_count = pick_count = 0

def terminate():
    subprocess.run('rosnode kill -a', shell=True)
    subprocess.run('killall -9 rosmaster', shell=True)
    subprocess.run('killall -9 roscore', shell=True)

def encode_state(state):
    return (state['robot_location'],state['toys_location']['green'],state['toys_location']['blue'],state['toys_location']['black'],state['toys_location']['red'])

def find_max_action(q_table, encoded_state):
    matching_items = {key: value for key, value in q_table.items() if key[0] == encoded_state}
    if len(matching_items) == 0:
        print(f"ERROR: matching_items in find_max_action is empty, encoded_state: {encoded_state}")
    max_value = max(matching_items.values())
    action = [key for key, value in matching_items.items() if value == max_value][0][1]
    return action

def find_rand_action(q_table, encoded_state):
    matching_items = {key: value for key, value in q_table.items() if key[0] == encoded_state}
    num_options = len(matching_items)
    if len(matching_items) == 0:
        print(f"ERROR: matching_items in find_rand_action is empty, encoded_state: {encoded_state}")
    random_index = random.randrange(num_options)
    action = list(matching_items.keys())[random_index][1]
    return action

def find_max_value(q_table, encoded_state):
    matching_items = {key: value for key, value in q_table.items() if key[0] == encoded_state}
    if len(matching_items) == 0:
        print(f"ERROR: matching_items in find_max_value is empty, encoded_state: {encoded_state}")
    max_value = max(matching_items.values())
    return max_value

def get_decaying_epsilon(iteration, num_of_iterations):
  decay_rate = (1.0 - 0.01) / num_of_iterations
  clamped_iteration = min(max(iteration, 0), num_of_iterations)
  epsilon = 1.0 - (decay_rate * clamped_iteration)
  return epsilon

def run_episode(iteration, q_learn):
    global Q, TERMINATE_STATE, nav_count, pick_count, prev_info, NUM_OF_ITERATIONS
    # epsilon = get_decaying_epsilon(iteration, NUM_OF_ITERATIONS)
    epsilon = max(math.pow(0.998, iteration), 0.05) if q_learn > 0 else 0
    alpha = 0.01
    gamma = 0.95
    prev_info = None
    print(f"DEBUG: \n\tepsilon: {epsilon}")
    info = get_info(print_info=False)
    state = info['state']
    encoded_state = encode_state(state)
    while encoded_state not in TERMINATE_STATE and nav_count < MAX_NAV and pick_count < MAX_PICK:
        rnd_choice = bool(random.random() < epsilon)
        if rnd_choice:
            action = find_rand_action(Q, encoded_state)
        else:
            action = find_max_action(Q, encoded_state)
        print(f"Random choice: {rnd_choice}, Action chosen: {action}")
        action()
        info = get_info()
        next_state = info['state']
        next_encoded_state = encode_state(next_state)
        next_max_value = find_max_value(Q, next_encoded_state)
        print(f"DEBUG: BEFORE UPDATE:\n\tQ[{(encoded_state,action.__name__)}] = {Q[(encoded_state,action)]}\n\timmediate_reward: {info['immediate_reward']}")
        print(f"\n\tNext state: {next_encoded_state},\n\tNext max action: {find_max_action(Q,next_encoded_state)}\n\tNext max value: {next_max_value}")
        if q_learn > 0:
            Q[(encoded_state,action)] = (1 - alpha)*Q[(encoded_state,action)] + alpha*(info['immediate_reward'] + gamma * next_max_value)
            print(f"\nDEBUG: AFTER UPDATE:\n\tQ[{(encoded_state,action.__name__)}] = {Q[(encoded_state,action)]}")
        prev_info = info
        state = next_state
        encoded_state = next_encoded_state
        
        

def main(q_learn):
    handle_csv(q_learn)
    sample_cntr = 0
    average_rewards = 0
    global NUM_OF_ITERATIONS
    max_iterations = NUM_OF_ITERATIONS if q_learn > 0 else 10
    while sample_cntr < max_iterations:
        print(f"---- iteration {sample_cntr} ----\n")
        setup_env()
        print(f"+++++ STARTING EPISODE {sample_cntr} +++++")
        run_episode(sample_cntr, q_learn)
        print(f"+++++ FINISHED EPISODE {sample_cntr} +++++")
        if q_learn  > 0:
            global Q
            save_q_table(Q)
        else:
            info = get_info()
            episode_rewards = info['total_rewards']
            print(f"Episode {sample_cntr} total rewards: {episode_rewards}")
            average_rewards += episode_rewards
        bring_down_env()
        sample_cntr += 1
    if q_learn == 0:
        average_rewards /= 10
        print(f"Average rewards for 10 episodes: {average_rewards}")
    save_q_table(Q, file="../final_q_table.csv")
    terminate()

if __name__ == "__main__":
        args = rospy.myargv()
        if len(args) != 2:
            print("Please provide 1 arg only for q learning: 1 - True, 0 - False, 2 - Cont. learning on existing q_table")
        subprocess.Popen(['roslaunch', 'task4_env', 'task4_env.launch'])
        time.sleep(2)
        rospy.init_node("q_learner")
        rospy.wait_for_service('/navigate')
        rospy.wait_for_service('/info')
        rospy.wait_for_service('/pick')
        rospy.wait_for_service('/place')
        main(eval(args[1]))
        