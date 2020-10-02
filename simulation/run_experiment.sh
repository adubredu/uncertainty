#! /bin/bash

# python3 fdreplan.py algo difficulty arrangement_number run_number
# for ((j=1; j<=5; j++))
# do
# 	for ((i=1; i<=5; i++))
# 	do 
# 		python3 fdreplan.py fdreplan sas $j $i
# 		sleep 15
# 	done
# done

# sleep 60

# for ((j=1; j<=5; j++))
# do
# 	for ((i=1; i<=5; i++))
# 	do 
# 		python3 fdreplan.py classical-replanner sas $j $i
# 		sleep 15
# 	done
# done

# sleep 60

# for ((j=1; j<=5; j++))
# do
# 	for ((i=1; i<=5; i++))
# 	do 
# 		python3 fdreplan.py fdreplan las $j $i
# 		sleep 15
# 	done
# done

# sleep 60

# for ((j=1; j<=5; j++))
# do
# 	for ((i=1; i<=5; i++))
# 	do 
# 		python3 fdreplan.py classical-replanner las $j $i
# 		sleep 15
# 	done
# done

# POMCPS

for ((j=1; j<=5; j++))
do
	for ((i=1; i<=5; i++))
	do 
		python3 fdreplan.py pomcp_er sas $j $i
		sleep 15
	done
done

sleep 60

for ((j=1; j<=5; j++))
do
	for ((i=1; i<=5; i++))
	do 
		python3 fdreplan.py pomcp sas $j $i
		sleep 15
	done
done

sleep 60

for ((j=1; j<=5; j++))
do
	for ((i=1; i<=5; i++))
	do 
		python3 fdreplan.py pomcp_er las $j $i
		sleep 15
	done
done

sleep 60

for ((j=1; j<=5; j++))
do
	for ((i=1; i<=5; i++))
	do 
		python3 fdreplan.py pomcp las $j $i
		sleep 15
	done
done