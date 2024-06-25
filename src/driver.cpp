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
		("state", po::value<string>()->required(), "json file that stores the state")
		("replanAgents,ra", po::value<std::vector<int>>(&replan_agent)->multitoken(), "existing tabu list")

		// params for the input instance and experiment settings
		("map,m", po::value<string>()->required(), "input file for map")
		("agentNum,k", po::value<int>()->default_value(0), "number of agents")
		("cutoffTime,t", po::value<double>()->default_value(20), "cutoff time (seconds)")
		("screen,s", po::value<int>()->default_value(1), "screen option (0: none; 1: results; 2:all)")
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
	Instance instance(vm["map"].as<string>(), input_state, replan_agent, agent_num);
	srand(0);
    PBS pbs(instance, vm["sipp"].as<bool>(), vm["screen"].as<int>());
    pbs.replan_agents = replan_agent;
    pbs.state_json = input_state;
    // run
    double runtime = 0;
    pbs.solve(vm["cutoffTime"].as<double>());

    pbs.clearSearchEngines();
    cout << "program run time : " << ((fsec)(Time::now() - start_time)).count() << endl;
	return 0;

}