#include "SpaceTimeAStar.h"
#include <chrono>
using namespace std::chrono;
typedef std::chrono::high_resolution_clock Time;
typedef std::chrono::duration<float> fsec;

// find path by time-space A* search
// Returns a shortest path that does not collide with paths in the path table


void SpaceTimeAStar::updatePath(const LLNode* goal, vector<PathEntry> &path)
{
    const LLNode* curr = goal;
    if (curr->is_goal)
        curr = curr->parent;
    path.reserve(curr->g_val + 1);
    while (curr != nullptr)
    {
        path.emplace_back(curr->location);
        curr = curr->parent;
    }
    std::reverse(path.begin(),path.end());
}

// Path SpaceTimeAStar::findOptimalPath(const PathTable &path_table)
// {
//     Path path;
//     double runtime_limit = 0.5;
//     num_expanded = 0;
//     num_generated = 0;

//     // build constraint table
//     auto t = clock();

//     int holding_time = -1; // the earliest timestep when the agent can hold its goal location.
//     if(!path_table.table.empty())
//     {
//         holding_time = path_table.table[goal_location].size();
//         while (holding_time > 0 && path_table.table[goal_location][holding_time - 1] == NO_AGENT)
//             holding_time--;
//     }

//     int lowerbound =  holding_time;

//     // generate start and add it to the OPEN & FOCAL list
//     auto start = new AStarNode(start_location, 0,
//                                max(lowerbound, my_heuristic[start_location]), nullptr, 0, 0, false);

//     num_generated++;
//     start->open_handle = open_list.push(start);
//     start->focal_handle = focal_list.push(start);
//     start->in_openlist = true;
//     allNodes_table.insert(start);
//     min_f_val = (int) start->getFVal();
//     // lower_bound = int(w * min_f_val));
//     auto start_time = Time::now();
//     while (!open_list.empty())
//     {
//         if (((fsec)(Time::now() - start_time)).count() >= runtime_limit){
//             releaseNodes();
//             return path;
//         }

//         updateFocalList(); // update FOCAL if min f-val increased
//         auto* curr = popNode();
//         assert(curr->location >= 0);
//         // check if the popped node is a goal
//         if (curr->location == goal_location && // arrive at the goal location
//             !curr->wait_at_goal && // not wait at the goal location
//             curr->timestep >= holding_time) // the agent can hold the goal location afterward
//         {
//             updatePath(curr, path);
//             break;
//         }

//         auto next_locations = instance.getNeighbors(curr->location);
//         next_locations.emplace_back(curr->location);
//         for (int next_location : next_locations)
//         {
//             int next_timestep = curr->timestep + 1;
//             if (path_table.makespan < next_timestep)
//             { // now everything is static, so switch to space A* where we always use the same timestep
//                 if (next_location == curr->location)
//                 {
//                     continue;
//                 }
//                 next_timestep--;
//             }

//             if (path_table.constrained(curr->location, next_location, next_timestep))
//                 continue;

//             // compute cost to next_id via curr node
//             int next_g_val = curr->g_val + 1;
//             int next_h_val = max(lowerbound - next_g_val, my_heuristic[next_location]);

//             // generate (maybe temporary) node
//             auto next = new AStarNode(next_location, next_g_val, next_h_val,
//                                       curr, next_timestep, 0, false);
//             if (next_location == goal_location && curr->location == goal_location)
//                 next->wait_at_goal = true;

//             // try to retrieve it from the hash table
//             auto it = allNodes_table.find(next);
//             if (it == allNodes_table.end())
//             {
//                 pushNode(next);
//                 allNodes_table.insert(next);
//                 continue;
//             }
//             // update existing node's if needed (only in the open_list)

//             auto existing_next = *it;
//             if (existing_next->getFVal() > next->getFVal() || // if f-val decreased through this new path
//                 (existing_next->getFVal() == next->getFVal() &&
//                  existing_next->num_of_conflicts > next->num_of_conflicts)) // or it remains the same but there's fewer conflicts
//             {
//                 if (!existing_next->in_openlist) // if its in the closed list (reopen)
//                 {
//                     existing_next->copy(*next);
//                     pushNode(existing_next);
//                 }
//                 else
//                 {
//                     bool add_to_focal = false;  // check if it was above the focal bound before and now below (thus need to be inserted)
//                     bool update_in_focal = false;  // check if it was inside the focal and needs to be updated (because f-val changed)
//                     bool update_open = false;
//                     if ((next_g_val + next_h_val) <= w * min_f_val)
//                     {  // if the new f-val qualify to be in FOCAL
//                         if (existing_next->getFVal() > w * min_f_val)
//                             add_to_focal = true;  // and the previous f-val did not qualify to be in FOCAL then add
//                         else
//                             update_in_focal = true;  // and the previous f-val did qualify to be in FOCAL then update
//                     }
//                     if (existing_next->getFVal() > next_g_val + next_h_val)
//                         update_open = true;

//                     existing_next->copy(*next);	// update existing node

//                     if (update_open)
//                         open_list.increase(existing_next->open_handle);  // increase because f-val improved
//                     if (add_to_focal)
//                         existing_next->focal_handle = focal_list.push(existing_next);
//                     if (update_in_focal)
//                         focal_list.update(existing_next->focal_handle);  // should we do update? yes, because number of conflicts may go up or down
//                 }
//             }

//             delete(next);  // not needed anymore -- we already generated it before
//         }  // end for loop that generates successors
//     }  // end while loop

//     releaseNodes();
//     return path;
// }



// Path SpaceTimeAStar::findOptimalPath(const set<int>& higher_agents, const vector<Path*>& paths, int agent, const PathTable &path_table)
Path SpaceTimeAStar::findOptimalPath(const set<int>& higher_agents, const vector<Path*>& paths, int agent, const vector<Path>& planned_paths)
{
   optimal = true;
   Path path;
   num_expanded = 0;
   num_generated = 0;

   // build constraint table
   auto t = clock();
   ConstraintTable constraint_table(instance.num_of_cols, instance.map_size);  // TODO change this to init with path table is all good , add a path table to
//    ConstraintTable constraint_table(path_table, instance.num_of_cols, instance.map_size);  // TODO change this to init with path table is all good , add a path table to
   for (auto planned_path : planned_paths)
   {
       constraint_table.insert2CT(planned_path);
   }
   
   for (int a : higher_agents)
   {
       constraint_table.insert2CT(*paths[a]);
   }
   runtime_build_CT = (double)(clock() - t) / CLOCKS_PER_SEC;

   if (constraint_table.constrained(start_location, 0))
   {
       return path;
   }

   t = clock();
   constraint_table.insert2CAT(agent, paths);
   runtime_build_CAT = (double)(clock() - t) / CLOCKS_PER_SEC;

   // the earliest timestep that the agent can hold its goal location. The length_min is considered here.
   auto holding_time = constraint_table.getHoldingTime(goal_location, constraint_table.length_min);
   auto static_timestep = constraint_table.getMaxTimestep() + 1; // everything is static after this timestep

   // generate start and add it to the OPEN & FOCAL list
   auto start = new AStarNode(start_location, 0, max(holding_time, my_heuristic[start_location]), nullptr, 0, 0);

   num_generated++;
   start->open_handle = open_list.push(start);
   start->in_openlist = true;
   allNodes_table.insert(start);

   while (!open_list.empty())
   {
       auto* curr = popNode();
       assert(curr->location >= 0);
       // check if the popped node is a goal
       if (curr->location == goal_location && // arrive at the goal location
           !curr->wait_at_goal && // not wait at the goal location
           curr->timestep >= holding_time) // the agent can hold the goal location afterward
       {
           updatePath(curr, path);
           break;
       }

       if (curr->timestep >= constraint_table.length_max)
           continue;

       auto next_locations = instance.getNeighbors(curr->location);
       next_locations.emplace_back(curr->location);
       for (int next_location : next_locations)
       {
           int next_timestep = curr->timestep + 1;
           if (static_timestep < next_timestep)
           { // now everything is static, so switch to space A* where we always use the same timestep
               if (next_location == curr->location)
               {
                   continue;
               }
               next_timestep--;
           }

           if (constraint_table.constrained(next_location, next_timestep) ||
               constraint_table.constrained(curr->location, next_location, next_timestep))
               continue;

           // compute cost to next_id via curr node
           int next_g_val = curr->g_val + 1;
           int next_h_val = max(holding_time - next_g_val, my_heuristic[next_location]);
           if (next_g_val + next_h_val > constraint_table.length_max)
               continue;
           int next_internal_conflicts = curr->num_of_conflicts +
                                         constraint_table.getNumOfConflictsForStep(curr->location, next_location, next_timestep);

           // generate (maybe temporary) node
           auto next = new AStarNode(next_location, next_g_val, next_h_val,
                                     curr, next_timestep, next_internal_conflicts);
           //if (next_location == goal_location && curr->location == goal_location)
           //    next->wait_at_goal = true;

           // try to retrieve it from the hash table
           auto it = allNodes_table.find(next);
           if (it == allNodes_table.end())
           {
               pushNode(next);
               allNodes_table.insert(next);
               continue;
           }
           // update existing node's if needed (only in the open_list)

           auto existing_next = *it;
           if (existing_next->getFVal() > next->getFVal() or  // it has smaller f value
                   (existing_next->getFVal() == next->getFVal() and existing_next->num_of_conflicts > next->num_of_conflicts)) // or it remains the same but there's fewer conflicts
           {
               existing_next->copy(*next);	// update existing node
               if (!existing_next->in_openlist) // if its in the closed list (reopen)
               {
                   pushNode(existing_next);
               }
               else
               {
                  open_list.increase(existing_next->open_handle);  // increase because #conflicts improved
               }
           }
           delete(next);  // not needed anymore -- we already generated it before
       }  // end for loop that generates successors
   }  // end while loop

   releaseNodes();
   return path;
}

inline AStarNode* SpaceTimeAStar::popNode()
{
    AStarNode* node;
    if (optimal)
    {
        node = open_list.top(); open_list.pop();
    }
    else
    {
        node = focal_list.top(); focal_list.pop();
        assert(node->in_openlist);
        open_list.erase(node->open_handle);
    }
    node->in_openlist = false;
    num_expanded++;
    return node;
}


inline void SpaceTimeAStar::pushNode(AStarNode* node)
{
    node->open_handle = open_list.push(node);
    node->in_openlist = true;
    num_generated++;
    if (!optimal and node->getFVal() <= w * min_f_val)
        node->focal_handle = focal_list.push(node);
}


void SpaceTimeAStar::updateFocalList()
{
    auto open_head = open_list.top();
    if (open_head->getFVal() > min_f_val)
    {
        int new_min_f_val = open_head->getFVal();
        for (auto n : open_list)
        {
            if (open_head->getFVal() > w * min_f_val && open_head->getFVal() <= w * new_min_f_val)
                n->focal_handle = focal_list.push(n);
        }
        min_f_val = new_min_f_val;
    }
}


void SpaceTimeAStar::releaseNodes()
{
    open_list.clear();
    focal_list.clear();
    for (auto node: allNodes_table)
        delete node;
    allNodes_table.clear();
}
