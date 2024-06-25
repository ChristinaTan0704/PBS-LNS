# PBS for LNS Replan


This project is based on the [PBS official implementaion](https://github.com/Jiaoyang-Li/PBS) and allows users to replan paths for specified agents using Priority-Based Search (PBS). The input to the executable is a JSON file containing the complete feasible solution and a list of agents to replan. The algorithm maintains the paths of the remaining agents unchanged, treating them as space-time obstacles, and only replans the paths for the agents in the removal set using the PBS algorithm.


## Installation 
The code requires the external library [boost](https://www.boost.org/).
If you are using Ubantu, you can install it simply by
```shell script
sudo apt install libboost-all-dev
``` 
Another easy way of installing the boost library is to install anaconda/miniconda and then
```shell script
conda install -c anaconda libboost
```
which works for a variety of [systems](https://anaconda.org/anaconda/libboost)
(including linux, osx, and win).

If neither of the above method works, you can also follow the instructions
on the [boost](https://www.boost.org/) website and install it manually.


After you installed boost and downloaded the source code, go into the directory of the source code and compile it with CMake:
```shell script
cmake -DCMAKE_BUILD_TYPE=RELEASE .
make
```

## Usage



```shell
./pbs-replan \
--map random-32-32-20.map \
--state map-random-32-32-20-scene-1-agent-150.json \
--agentNum 150 \
--replanAgents 3 25 62 117 134 
```

- map (required): the .map file downloaded from the MAPF benchmark
- state (required): path to the current state JSON file, key: agent id, value: list of agent location in 2D x, y coordinate, check [map-random-32-32-20-scene-1-agent-150.json](map-random-32-32-20-scene-1-agent-150.json) as an example
- agentNum (required): number of agents in the current map
- replanAgents (required): list of agents to replan
- cutoffTime (optional): run time limit for running the removal and replan of LNS

You can find more details and explanations for all parameters with:
```
./pbs-replan --help
```
If the cost of replanned paths improves (improvement > 0), the program will output the newly planned paths for the replanned agents.
 
## References
[1] Hang Ma, Daniel Harabor, Peter J. Stuckey, Jiaoyahng Li and S. Koenig. 
Searching with Consistent Prioritization for Multi-Agent Path Finding. 
In Proceedings of the AAAI Conference on Artificial Intelligence (AAAI), 7643-7650, 2019.

[2] Jiaoyang Li, Zhe Chen, Daniel Harabor, Peter J. Stuckey and Sven Koenig.
MAPF-LNS2: Fast Repairing for Multi-Agent Path Finding via Large Neighborhood Search.
In Proceedings of the AAAI Conference on Artificial Intelligence, (in print), 2022.



<!-- 
python script/ml_implementation_new.py \
--option 6 \
--nns_collect_data \
--initial_state data/nns_random_scene/val_init_state_json/map-Paris_1_256-scene-10000-agent-550.json \
--output_folder data/uncheck_nns_val_suggested_data/training_data/Paris_1_256 \
--pbs_replan_exe pre_work/baseline/PBS/pbs \
--map_folder pre_work/baseline/MAPF-LNS/map \
--log_path data/uncheck_nns_val_suggested_data/log/map-Paris_1_256-scene-10000-agent-550-method-PBSRandomWalkLarge-nb-25.log \
--gen_subset_exe pre_work/baseline/RR_V2/rrv2 \
--destroyStrategy  RandomWalkLarge \
--neighborSize 25 \
--num_subset 100 \
--infer_time 1 \
--num_cores 8 \
--max_iter 50  \
--uniformNB 0


pre_work/baseline/PBS/pbs --map pre_work/baseline/MAPF-LNS/map/ost003d.map --agentNum 400 --state waste/nns_infer_debug/ost003d/nns_running_state_temp/2024-06-25_11-22-40/map-ost003d-scene-10-agent-400-state.json --cutoffTime 20 --replanAgents 62 117 134 163 168 182 208 209 249 344  -->