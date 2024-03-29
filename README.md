# Task 4 Submission - Q Learning in ROS

## How to run:
1. copy task4_env to ~/catkin\_ws/src
2. cd ~/catkin\_ws/src/task4\_env/scripts
3. chmod +x q\_learner.py
4. rosrun task4\_env q\_learner.py arg1 -- arg1 could be 0: Execute current policy (that is saved in q_table.csv), 1 - Run Q Learning and save Q table in q\_table.csv, 2 - Load Q table from q\_table.csv and continue training.

The current policy did not converge very well for me (collects about 2 toys each episode), and I think this happened because a couple of reasons:
1. My epsilon decay function is decreasing to rapidly (equation to calculate epsilon - max(0.998^(iteration number), 0.05))
2. Max number of picks in a episode is 8, this can lead to a number of issues - the agent can fail to go to certain locations in a single episode, and if just before getting to location 4, he will fail because he has exceeded the max number of navigations, then he will "learn" that picking and object is not a good choice (picking and object costs more than just navigating)
3. I ran "only" 2000 episodes, and there are some values in the Q table that were not referenced / not referenced enough, this lead to non optimal behaviour.

* Running 2000 episodes took me about 20 hours.
