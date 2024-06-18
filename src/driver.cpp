/* Copyright (C) Jiaoyang Li
* Unauthorized copying of this file, via any medium is strictly prohibited
* Confidential
* Written by Jiaoyang Li <jiaoyanl@usc.edu>, May 2020
*/

/*driver.cpp
* Solve a MAPF instance on 2D grids.
*/
#include <boost/program_options.hpp>
#include <boost/tokenizer.hpp>
#include "PBS.h"
#include <chrono>
using namespace std::chrono;
typedef std::chrono::high_resolution_clock Time;
typedef std::chrono::duration<float> fsec;


// --map pre_work/baseline/MAPF-LNS/map/random-32-32-20.map --agentNum 150 --state data/initial_state_json_10s/LNS2/map-random-32-32-20-scene-1-agent-150.json --cutoffTime 20 --replanAgents 3 11 23 26 28 42 45 49 57 62 94 100 119 140 148 
// --map pre_work/baseline/MAPF-LNS/map/random-32-32-20.map --agentNum 150 --state data/initial_state_json_10s/LNS2/map-random-32-32-20-scene-1-agent-150.json --replanAgents 5 21 22 29 42 55 66 69 82 101 103 105 119 120 146 --cutoffTime 20
// --map pre_work/baseline/MAPF-LNS/map/random-32-32-20.map --agentNum 150 --state data/initial_state_json_10s/LNS2/map-random-32-32-20-scene-1-agent-150.json --replanAgents 1 2 3
// --map pre_work/baseline/MAPF-LNS/map/empty-8-8.map --agentNum 16 --state data/initial_state_json_30s/LACAM/map-empty-8-8-scene-4-agent-16.json --replanAgents 1 2 3
// --map pre_work/baseline/MAPF-LNS/map/Paris_1_256.map --agentNum 650 --state data/initial_state_json_10s/LNS2/map-Paris_1_256-scene-1-agent-650.json   --replanAgents 20 50 30 69 84 73
// --adaptive_weight 1 1 --pprun 1 --num_subset 20 --uniform_neighbor 2 --replanTime 0.6

/* Main function */
int main(int argc, char** argv)
{
	namespace po = boost::program_options;
	// Declare the supported options.
	po::options_description desc("Allowed options");
	std::vector<int> replan_agent;
	desc.add_options()
		("help", "produce help message")
		// pbs as a replan solver
		("state", po::value<string>()->default_value(""), "json file that stores the state")
		("replanAgents,ra", po::value<std::vector<int>>(&replan_agent)->multitoken(), "existing tabu list")

		// params for the input instance and experiment settings
		("map,m", po::value<string>()->default_value(""), "input file for map")
		("agents,a", po::value<string>()->default_value(""), "input file for agents")
		("output,o", po::value<string>(), "output file for statistics")
		("outputPaths", po::value<string>(), "output file for paths")
		("agentNum,k", po::value<int>()->default_value(0), "number of agents")
		("cutoffTime,t", po::value<double>()->default_value(20), "cutoff time (seconds)")
		("screen,s", po::value<int>()->default_value(1), "screen option (0: none; 1: results; 2:all)")
		("stats", po::value<bool>()->default_value(false), "write to files some detailed statistics")

		("sipp", po::value<bool>()->default_value(0), "using SIPP as the low-level solver")
		;
	po::variables_map vm;
	po::store(po::parse_command_line(argc, argv, desc), vm);

	if (vm.count("help")) {
		cout << desc << endl;
		return 1;
	}

	po::notify(vm);

	srand((int)time(0));
    auto start_time = Time::now();
	///////////////////////////////////////////////////////////////////////////
	// load the instance
    string input_state = vm["state"].as<string>();
    int agent_num = vm["agentNum"].as<int>();
    if (input_state != ""){
        agent_num = replan_agent.size();
    }
	Instance instance(vm["map"].as<string>(), vm["agents"].as<string>(),input_state, replan_agent, agent_num);
//    cout << "program run time 1 : " << ((fsec)(Time::now() - start_time)).count() << endl;
	srand(0);
    PBS pbs(instance, vm["sipp"].as<bool>(), vm["screen"].as<int>());
    pbs.replan_agents = replan_agent;
    pbs.state_json = input_state;
    // run
    double runtime = 0;
//    cout << "program run time 2 : " << ((fsec)(Time::now() - start_time)).count() << endl;
    pbs.solve(vm["cutoffTime"].as<double>());
    if (vm.count("output"))
        pbs.saveResults(vm["output"].as<string>(), vm["agents"].as<string>());
    if (pbs.solution_found && vm.count("outputPaths"))
        pbs.savePaths(vm["outputPaths"].as<string>());
    pbs.clearSearchEngines();
    cout << "program run time : " << ((fsec)(Time::now() - start_time)).count() << endl;
	return 0;

}