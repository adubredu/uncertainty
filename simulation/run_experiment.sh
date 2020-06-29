#! /bin/bash

# for i in {1..5}
# do 
# 	python3 belief_3D.py conveyor-belt
# 	sleep 15
# done
python belief_3D.py pick-n-roll 1
sleep 30
python belief_3D.py bag-sort 1
sleep 30
python belief_3D.py pick-n-roll 2
sleep 30
python belief_3D.py bag-sort 2
sleep 30
python belief_3D.py pick-n-roll 3
sleep 30
python belief_3D.py bag-sort 3
sleep 30
python belief_3D.py pick-n-roll 4
sleep 30
python belief_3D.py bag-sort 4
sleep 30
python belief_3D.py pick-n-roll 5
sleep 30
python belief_3D.py bag-sort 5
sleep 30
# for i in {1..5}
# do 
# 	python3 belief_3D.py pick-n-roll
# 	sleep 15
# done

# for i in {1..5}
# do 
# 	python3 belief_3D.py bag-sort
# 	sleep 15
# done

# for i in {1..5}
# do 
# 	python3 belief_3D.py sbp
# 	sleep 15
# done

# for i in {1..5}
# do 
# 	python3 belief_3D.py optimistic
# 	sleep 15
# done


# for i in {1..5}
# do 
# 	python3 belief_3D.py mc-dynamic
# 	sleep 15
# done

# for i in {1..5}
# do 
# 	python3 belief_3D.py weighted-dynamic
# 	sleep 15
# done

# for i in {1..5}
# do 
# 	python3 belief_3D.py divergent-dynamic
# 	sleep 15
# done

# for i in {1..5}
# do 
# 	python3 belief_3D.py declutter
# 	sleep 15
# done