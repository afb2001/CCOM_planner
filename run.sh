
map=""
goal="" 
args="$#"
for (( i=1; i <= "$#"; i++ )); do
    if [ ${!i} == "-map" ]; then
        if [ $((i+1)) -le "$#" ]; then
            var=$((i+1))
            map=${!var}
        fi
    fi

    if [ ${!i} == "-goal" ]; then
        if [ $((i+1)) -le "$#" ]; then
            var=$((i+1))
            goal=${!var}
        fi
    fi
done

cd executive
./shim.py $map $goal
cd -


