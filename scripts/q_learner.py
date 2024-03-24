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

CSV_FILE = "../q_table.csv"
MAX_NAV = 8
MAX_PICK = 6

nav_count = 0
pick_count = 0

Q = {}
TERMINATE_STATE = [(0,4,4,4,4),(1,4,4,4,4),(2,4,4,4,4),(3,4,4,4,4),(4,4,4,4,4)]

def handle_csv(q_learn):
    print(f"Enable Q Learning: {q_learn}")
    global Q
    if q_learn:
        Q = create_q_table()
    if not q_learn:
        Q = load_q_table()

def save_q_table(q_table):
    actions = {'navigate_to_loc_0': navigate_to_loc_0, 
        'navigate_to_loc_1': navigate_to_loc_1, 
        'navigate_to_loc_2': navigate_to_loc_2, 
        'navigate_to_loc_3': navigate_to_loc_3, 
        'navigate_to_loc_4': navigate_to_loc_4, 
        'pick_toy': pick_toy, 
        'place_toy': place_toy}

    states = set([state for (state, _) in q_table.keys()])
    actions = set([action for (_, action) in q_table.keys()])

    with open(CSV_FILE, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)

        # Write header row with actions as column names
        writer.writerow(['State'] + list(actions))

        # Write each state as a row with corresponding Q-values for each action
        for state in states:
            row = [state]
            for action in actions:
                # Access Q-value from the dictionary, handle missing entries (e.g., 0)
                q_value = q_table.get((state, action), 0)
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
    for permutation in permutations(range(4)):
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
        while not resp and pick_count < MAX_PICK:
            pick_srv = rospy.ServiceProxy('/pick', pick)
            resp = pick_srv(toy_type)
            resp = resp.success
            pick_count += 1
        print(f"Finished picking up at {location}, success: {resp}, pick_count: {pick_count}")
        return resp
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def get_info():
    print(f"Getting /info")
    try:
        info_srv = rospy.ServiceProxy('/info', info)
        resp = info_srv()
        return parse_info(resp.internal_info)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def parse_info(info_str):
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
            tr_str = line[line.find(":")+1:]
            info_dict['total_rewards'] = eval(tr_str)

    return info_dict

def setup_env():
    navigate_to_loc_4()
    navigate_to_loc_4()
    global nav_count, pick_count
    nav_count = pick_count = 0

    subprocess.run('rosnode kill skills_server_node', shell=True)
    time.sleep(2)
    subprocess.Popen('rosrun task4_env skills_server.py', shell=True)
    time.sleep(2)
    

def bring_down_env():
    subprocess.run('rosnode kill skills_server_node', shell=True)
    time.sleep(2)
    subprocess.Popen('rosrun task4_env skills_server.py', shell=True)
    time.sleep(2)

def terminate():
    subprocess.run('rosnode kill -a', shell=True)
    subprocess.run('killall -9 rosmaster', shell=True)
    subprocess.run('killall -9 roscore', shell=True)

def run_episode():
    global Q, TERMINATE_STATE, nav_count, pick_count
    epsilon = 0.3
    alpa = 0.01
    gamma = 0.95
    state = info['state']
    encoded_state = (state['robot_location'],state['toys_location']['green'],state['toys_location']['blue'],state['toys_location']['black'],state['toys_location']['red'])
    while encoded_state not in TERMINATE_STATE and nav_count < MAX_NAV and pick_count < MAX_PICK:
        rnd_choice = bool(random.random() < epsilon)
        info = get_info()
        matching_items = {key: value for key, value in Q.items() if key[0] == encoded_state}
        if rnd_choice:
            num_options = len(matching_items)
            random_index = random.randrange(num_options)
            action = list(matching_items.keys())[random_index][1]
        else:
            max_value = max(matching_items.values())
            action = [key for key, value in matching_items.items() if value == max_value][0][1]
        print(f"Random choice: {rnd_choice}, Action chosen: {action}")
        action()


def main(q_learn):
    handle_csv(q_learn)
    sample_cntr = 0
    while sample_cntr < 1000:
        print(f"---- iteration {sample_cntr} ----\n")
        setup_env()
        run_episode()
        bring_down_env()
        sample_cntr += 1
    if q_learn:
        save_q_table()

if __name__ == "__main__":
        args = rospy.myargv()
        if len(args) != 2:
            print("Please provide 1 arg only for q learning: 1 - True, 0 - False")
        subprocess.Popen(['roslaunch', 'task4_env', 'task4_env.launch'])
        time.sleep(2)
        rospy.init_node("q_learner")
        rospy.wait_for_service('/navigate')
        rospy.wait_for_service('/info')
        rospy.wait_for_service('/pick')
        rospy.wait_for_service('/place')
        main(bool(eval(args[1])))
        terminate()