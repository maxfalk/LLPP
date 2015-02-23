#!/bin/bash

EXE="./demo hugeScenario.xml"
SEQ=SEQ
OMP=OMP
PTHREADS=PTHREAD
THREADS=-threads
CUDA=CUDA
VECTOR=VECTOR
FLAGS=-mode
TIMING=--timing-mode
TIMES=4
GET=^Time.*$
FILE=report.txt
NUMTHREADS=4

echo -n "$SEQ;$VECTOR;$CUDA;" > $FILE;
for ((i=2; i <= NUMTHREADS; i++));
do
    echo -n "$OMP $i;$PTHREADS $i;" >> $FILE;

done

echo -e -n "\n" >> $FILE

for ((i=0; i < TIMES; i++))
do 
    echo "starting";
    ($EXE $FLAGS $SEQ $TIMING ) | grep $GET >> $FILE
    ($EXE $FLAGS $VECTOR $TIMING ) | grep $GET >> $FILE
    ($EXE $FLAGS $CUDA $TIMING ) | grep $GET >> $FILE
  
    for ((k=2; k <= NUMTHREADS; k++))
    do   
	($EXE $FLAGS $OMP $THREADS $k $TIMING ) | grep $GET >> $FILE;
	($EXE $FLAGS $PTHREADS $THREADS $k $TIMING )| grep $GET >> $FILE;   
    done
    

    echo -e -n "\n" >> $FILE

done

perl -pi -e 's/Time: //g' $FILE
perl -pi -e 's/ seconds.\n/;/g' $FILE

