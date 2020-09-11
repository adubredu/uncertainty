#! /bin/bash

#python3 fdreplan.py algo difficulty arrangement_number run_number
for ((j=1; j<=5; j++))
do
	for ((i=1; i<=5; i++))
	do 
		python3 fdreplan.py fdreplan sas $j $i
		sleep 15
	done
done

sleep 60

for ((j=1; j<=5; j++))
do
	for ((i=1; i<=5; i++))
	do 
		python3 fdreplan.py classical-replanner sas $j $i
		sleep 15
	done
done

sleep 60

for ((j=1; j<=5; j++))
do
	for ((i=1; i<=5; i++))
	do 
		python3 fdreplan.py fdreplan las $j $i
		sleep 15
	done
done

sleep 60

for ((j=1; j<=5; j++))
do
	for ((i=1; i<=5; i++))
	do 
		python3 fdreplan.py classical-replanner las $j $i
		sleep 15
	done
done

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