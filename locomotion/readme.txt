Astar_Simple: Global Planner. Given current state and 2d map(in the height of 0.2m), 2.5d map,  generate 4d waypoint path(x,y,theta,is_hop) in real-time with the update of map. 

Local_Planner_Hop: local planner. given the current state and refence path, generate control command and send it to cheetah by using LCM. Stop in the front of the barriar(scripts/cheetah_planner.py 132-144) and send a LCM command to cheetah(scripts/control.py 173-180).

current state need to be send to the topic (\curr_state) [x,y,theta]
there might be lot's of bugs since I can't debug in the real robot.

roslaunch astar_simple astar2.launch
rosrun local_planner_hop cheetah_planner.py
rosrun local_planner_hop control.py

Main code: 
Astar_Simple:Astar.cpp 282 mian.cpp 107-140 
Local_Planner_Hop: cheetah_planner.py control.py
