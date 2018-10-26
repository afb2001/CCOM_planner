
map=""
goal="" 
map1=""
goal1=""
nbos=""
args="$#"
path="../"
for (( i=1; i <= "$#"; i++ )); do
    if [ ${!i} == "-map" ]; then
        if [ $((i+1)) -le "$#" ]; then
            var=$((i+1))
            map=${!var}
            map1="-m $path$map"
        fi
    fi

    if [ ${!i} == "-goal" ]; then
        if [ $((i+1)) -le "$#" ]; then
            var=$((i+1))
            goal=${!var}
            goal1="-g $path$goal"
        fi
    fi

    if [ ${!i} == "-nobs" ]; then
        if [ $((i+1)) -le "$#" ]; then
            var=$((i+1))
            var1=${!var}
            nbos="-nobs $var1"
        fi
    fi
done

cd simulator

if [[ $map1 ]] && [[ $goal1 ]] && [[ $nbos ]]; then
    ./dynamic_obs_sim_3.py "-p TRUE" $map1 $goal1 $nbos &
    cd -
    cd executive
    
    ./shim.py $map1 $goal1
elif [[ $map1 ]] && [[ $goal1 ]]; then
    ./dynamic_obs_sim_3.py "-p TRUE" $map1 $goal1 &
    cd -
    cd executive
    ./shim.py $map1 $goal1
elif [[ $map1 ]] && [[ $nbos ]]; then
    ./dynamic_obs_sim_3.py "-p TRUE" $map1 $nbos &
    cd -
    cd executive
    ./shim.py $map1
elif [[ $nbos ]] && [[ $goal1 ]]; then
    ./dynamic_obs_sim_3.py "-p TRUE" $goal1 $nbos &
    cd -
    cd executive
    ./shim.py $goal1
elif [[ $nbos ]]; then
    ./dynamic_obs_sim_3.py "-p TRUE" $nbos &
    cd -
    cd executive
    ./shim.py
elif [[ $goal1 ]]; then
    ./dynamic_obs_sim_3.py "-p TRUE" $goal1 &
    cd -
    cd executive
    ./shim.py $goal1
elif [[ $map1 ]]; then
    ./dynamic_obs_sim_3.py "-p TRUE" $map1 &
    cd -
    cd executive
    ./shim.py $map1
else
    ./dynamic_obs_sim_3.py "-p TRUE" &
    cd -
    cd executive
    ./shim.py
fi

cd -


