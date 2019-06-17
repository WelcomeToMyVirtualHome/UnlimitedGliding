#!/bin/bash
make
file1="outputClustering.dat"
file2="outputVelocity.dat"
for (( i=1; $i <= 5; i++ )); do
	cd $i
	cp "../main" "main"
	if [ -f "$file1" ]
	then
		rm $file1
	fi
	if [ -f "$file2" ]
	then
		rm $file2
	fi
	./main params.dat &
	cd ..
done