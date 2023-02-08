import sys
import numpy as np
import os
import math

fname = "/home/zx/workspace/DSTO-Swarm/src/Utils/random_goals/launch/random_goals.launch"
def main(argv):
    goal_num_in = int(argv[1])
    height = 1.0
    theta = np.linspace(-3.14159, 3.14159, goal_num_in+1)
    r = 3.0
    file = open(fname, "w")

    str_head = "<launch>\n\
    <node pkg=\"random_goals\" name=\"random_goals_node\" type=\"random_goals_node\" output=\"screen\">\n\
        <param name=\"drone_num\" value=\"10\"/>\n\
        <param name=\"goal_num\" value=\"{goal_num}\"/>\n".format(goal_num=goal_num_in)
    file.write(str_head)

    for i in range(goal_num_in):
        str_goals = "        <rosparam param=\"goal{goal_id}\">[{p0},{p1},{p2}]</rosparam>\n".format(goal_id=i, p0=r * math.sin(theta[i]), p1=r * math.cos(theta[i]), p2=height, \
        target0_x=-r * math.sin(theta[i]), target0_y=-r * math.cos(theta[i]), target0_z=height) 
        file.write(str_goals)

    str_tail = "    </node>\n</launch>"
    file.write(str_tail)
    
    file.close()

if __name__ == '__main__':
    main(sys.argv)

