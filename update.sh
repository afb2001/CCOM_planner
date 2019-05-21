names=(
asv_sim
asv_helm
project11
project11_transformations
udp_bridge
marine_msgs
mission_plan
mbes_sim
remote_control
command_bridge
mission_manager
dubins_curves
path_follower
kongsberg_em_control
geographic_visualization_msgs
joy_to_helm
hover
)

for i in "${names[@]}"
do
	cd $i
	git fetch upstream
	git merge upstream/master
	cd ..
done
