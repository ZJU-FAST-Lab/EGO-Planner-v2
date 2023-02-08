bool=1

for k in $( seq 1 1000 )
do
	roslaunch ego_planner rviz.launch & sleep 1; 
	roslaunch ego_planner swarm.launch & sleep 50;
	
	
	if [ 10 -ne `pgrep -c ego_` ]; then
		echo `pgrep -c ego_`
		echo "AAAAAAAAAAAAAAAAAAAAAA"
		bool=0
		break;
	fi
	
	if [ $bool -eq 0 ]; then
		echo "BBBBBBBBB"
		break;
	fi

	rosnode kill -a & sleep 3;
done


rosnode kill -a & sleep 5;
