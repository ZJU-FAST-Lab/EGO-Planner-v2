# roslaunch ego_planner rviz.launch & sleep 5;

# roslaunch ego_planner swarm.launch & sleep 5;

source ../devel/setup.bash
i=0
while test $i -lt 10000000; do

    echo "launch code, i is $i"
    roslaunch ego_planner rviz.launch &
    roslaunch ego_planner single_drone_waypoints.launch &

    sleep 10

    j=0
    while test $j -lt 150; do
        ts=$(pidof traj_server)
        if [ $ts ]; then
            echo "traj_server Alive"
        else
            while test 1 -lt 160; do
                echo "traj_server get stuck get stuck get stuck get stuck get stuck get stuck get stuck get stuck get stuck get stuck get stuck"
                sleep 1
            done
        fi

        ego=$(pidof ego_planner_node)
        if [ $ego ]; then

            echo "ego_planner_node Alive"
        else
            while test 1 -lt 160; do
                echo "ego_planner_node get stuck get stuck get stuck get stuck get stuck get stuck get stuck get stuck get stuck get stuck get stuck"
                sleep 1
            done
        fi

        sleep 1
        j=$(expr $j + 1)
    done

    rosnode kill -a

    sleep 5

    i=$(expr $i + 1)

done

wait
