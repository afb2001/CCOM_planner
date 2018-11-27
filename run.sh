
planner=()
simulator=("-p TRUE")
v=""
vis=()
path="../"

for (( i=1; i <= "$#"; i++ )); do

    if [ ${!i} == "-v" ]; then
        v="TRUE"
    fi
    #test
    if [ ${!i} == "-test" ]; then
        if [ $((i+1)) -le "$#" ]; then
            var=$((i+1))
            test=${!var}

            simulator+=("-m")
            simulator+=("$path$test.map")
            planner+=("-m")
            planner+=("$path$test.map")
            simulator+=("-g")
            simulator+=("$path$test.goal")
            planner+=("-g")
            planner+=("$path$test.goal")
            simulator+=("-af")
            simulator+=("$path$test.asv")
            simulator+=("-dynamic")
            simulator+=("$path$test.dynamic")
            vis+=("$path$test.map")
            vis+=("$path$test.goal")
            istest="TRUE"
        fi
    fi


    #both
    if [ ${!i} == "-map" ]; then
        if [ $((i+1)) -le "$#" ]; then
            var=$((i+1))
            map=${!var}
            simulator+=("-m")
            simulator+=("$path$map")
            planner+=("-m")
            planner+=("$path$map")
            vis+=("$path$map")
        fi
    fi

    if [ ${!i} == "-goal" ]; then
        if [ $((i+1)) -le "$#" ]; then
            var=$((i+1))
            goal=${!var}
            simulator+=("-g")
            simulator+=("$path$goal")
            planner+=("-g")
            planner+=("$path$goal")
            vis+=("$path$goal")
        fi
    fi

    if [ ${!i} == "-model" ]; then
        if [ $((i+1)) -le "$#" ]; then
            var=$((i+1))
            model=${!var}
            simulator+=("-model")
            simulator+=("$path$model")
            planner+=("-model")
            planner+=("$path$model")
        fi
    fi

    #executive only
    if [ ${!i} == "-debug" ]; then
        if [ $((i+1)) -le "$#" ]; then
            var=$((i+1))
            debug=${!var}
            planner+=("-debug")
            planner+=("$debug")
        fi
    fi

    #simulator only

    if [ ${!i} == "-dynamic" ]; then
        if [ $((i+1)) -le "$#" ]; then
            var=$((i+1))
            dynamic=${!var}
            simulator+=("-dynamic")
            simulator+=("$path$dynamic")
        fi
    fi

    if [ ${!i} == "-tiff" ]; then
        if [ $((i+1)) -le "$#" ]; then
            var=$((i+1))
            tiff=${!var}
            simulator+=("-tiff")
            simulator+=("$path$tiff")
        fi
    fi

    if [ ${!i} == "-af" ]; then
        if [ $((i+1)) -le "$#" ]; then
            var=$((i+1))
            asvfile=${!var}
            simulator+=("-af")
            simulator+=("$path$asvfile")
        fi
    fi

    if [ ${!i} == "-e" ]; then
        if [ $((i+1)) -le "$#" ]; then
            var=$((i+1))
            env=${!var}
            simulator+=("-e")
            simulator+=("$env")
        fi
    fi

    if [ ${!i} == "-nobs" ]; then
        if [ $((i+1)) -le "$#" ]; then
            var=$((i+1))
            nobs=${!var}
            simulator+=("-nobs")
            simulator+=("$nobs")
        fi
    fi

done
if [ "$v" = "TRUE" ]; then
    cd simulator
    ./dynamic_obs_sim_3.py "${simulator[@]}" &
    cd -
    cd executive
    ./shim.py "${planner[@]}" 2>&1>/dev/null | ./../visualizer/test.py  "${vis[@]}"
    cd -
else
    cd simulator
    ./dynamic_obs_sim_3.py "${simulator[@]}" &
    cd -
    cd executive
    ./shim.py "${planner[@]}"
    cd -
fi



# if [[ $map1 ]] && [[ $goal1 ]] && [[ $nbos ]]; then
#     ./dynamic_obs_sim_3.py "-p TRUE" $map1 $goal1 $nbos &
#     cd -
#     cd executive
    
#     ./shim.py $map1 $goal1
# elif [[ $map1 ]] && [[ $goal1 ]] && [[ $dynamic ]]; then
#     ./dynamic_obs_sim_3.py "-p TRUE" $map1 $goal1 $dynamic &
#     cd -
#     cd executive
#     ./shim.py $map1 $goal1
# elif [[ $map1 ]] && [[ $goal1 ]]; then
#     ./dynamic_obs_sim_3.py "-p TRUE" $map1 $goal1 &
#     cd -
#     cd executive
#     ./shim.py $map1 $goal1
# elif [[ $map1 ]] && [[ $nbos ]]; then
#     ./dynamic_obs_sim_3.py "-p TRUE" $map1 $nbos &
#     cd -
#     cd executive
#     ./shim.py $map1
# elif [[ $nbos ]] && [[ $goal1 ]]; then
#     ./dynamic_obs_sim_3.py "-p TRUE" $goal1 $nbos &
#     cd -
#     cd executive
#     ./shim.py $goal1
# elif [[ $nbos ]]; then
#     ./dynamic_obs_sim_3.py "-p TRUE" $nbos &
#     cd -
#     cd executive
#     ./shim.py
# elif [[ $goal1 ]]; then
#     ./dynamic_obs_sim_3.py "-p TRUE" $goal1 &
#     cd -
#     cd executive
#     ./shim.py $goal1
# elif [[ $map1 ]]; then
#     ./dynamic_obs_sim_3.py "-p TRUE" $map1 &
#     cd -
#     cd executive
#     ./shim.py $map1
# else
#     ./dynamic_obs_sim_3.py "-p TRUE" &
#     cd -
#     cd executive
#     ./shim.py
# fi

# cd -


