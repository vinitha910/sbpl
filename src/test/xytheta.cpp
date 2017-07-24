#include <cstring>
#include <iostream>
#include <string>
 
using namespace std;
 
#include <sbpl/headers.h>
 
// creating the footprint
void createFootprint(vector<sbpl_2Dpt_t>& perimeter){
    sbpl_2Dpt_t pt_m;
    double halfwidth = 0.01;
    double halflength = 0.01;
    pt_m.x = -halflength;
    pt_m.y = -halfwidth;
    perimeter.push_back(pt_m);
    pt_m.x = halflength;
    pt_m.y = -halfwidth;
    perimeter.push_back(pt_m);
    pt_m.x = halflength;
    pt_m.y = halfwidth;
    perimeter.push_back(pt_m);
    pt_m.x = -halflength;
    pt_m.y = halfwidth;
    perimeter.push_back(pt_m);
}
 
void initializeEnv(EnvironmentNAVXYTHETALAT& env, 
                   vector<sbpl_2Dpt_t>& perimeter, 
                   char* envCfgFilename, char* motPrimFilename){
    if (!env.InitializeEnv(envCfgFilename, perimeter, motPrimFilename)) {
        printf("ERROR: InitializeEnv failed\n");
        throw SBPL_Exception();
    }
}
 
void setEnvStartGoal(EnvironmentNAVXYTHETALAT& env, 
                     double start_x, double start_y, double start_theta,
                     double goal_x, double goal_y, double goal_theta, 
                     int& start_id, int& goal_id){
 
    start_id = env.SetStart(start_x, start_y, start_theta);
    goal_id = env.SetGoal(goal_x, goal_y, goal_theta);
}
 
void initializePlanner(SBPLPlanner*& planner, 
                       EnvironmentNAVXYTHETALAT& env,
                       int start_id, int goal_id,
                       double initialEpsilon, 
                       bool bsearchuntilfirstsolution){
    // work this out later, what is bforwardsearch?
    bool bsearch = false;
    planner = new ARAPlanner(&env, bsearch);
 
    // set planner properties
    if (planner->set_start(start_id) == 0) {
        printf("ERROR: failed to set start state\n");
        throw new SBPL_Exception();
    }
    if (planner->set_goal(goal_id) == 0) {
        printf("ERROR: failed to set goal state\n");
        throw new SBPL_Exception();
    }
    planner->set_initialsolution_eps(initialEpsilon);
    planner->set_search_mode(bsearchuntilfirstsolution);
}
 
int runPlanner(SBPLPlanner* planner, int allocated_time_secs, 
               vector<int>&solution_stateIDs){
    int bRet = planner->replan(allocated_time_secs, &solution_stateIDs);
 
    if (bRet) 
        printf("Solution is found\n");
    else 
        printf("Solution does not exist\n");
    return bRet;
}
 
void writeSolution(EnvironmentNAVXYTHETALAT& env, vector<int> solution_stateIDs,
                   const char* filename){
    std::string discrete_filename(std::string(filename) + std::string(".discrete"));
    FILE* fSol_discrete = fopen(discrete_filename.c_str(), "w");
    FILE* fSol = fopen(filename, "w");
    if (fSol == NULL) {
        printf("ERROR: could not open solution file\n");
        throw SBPL_Exception();
    }
 
    // write the discrete solution to file
    for (size_t i = 0; i < solution_stateIDs.size(); i++) {
        int x, y, theta;
        env.GetCoordFromState(solution_stateIDs[i], x, y, theta);
        double cont_x, cont_y, cont_theta;
        cont_x = DISCXY2CONT(x, 0.1);
        cont_y = DISCXY2CONT(y, 0.1);
        cont_theta = DiscTheta2Cont(theta, 16);
        fprintf(fSol_discrete, "%d %d %d\n", x, y, theta);
    }
    fclose(fSol_discrete);
 
    // write the continuous solution to file
    vector<sbpl_xy_theta_pt_t> xythetaPath;
    env.ConvertStateIDPathintoXYThetaPath(&solution_stateIDs, &xythetaPath);
    for (unsigned int i = 0; i < xythetaPath.size(); i++) {
        fprintf(fSol, "%.3f %.3f %.3f\n", xythetaPath.at(i).x, 
                                          xythetaPath.at(i).y, 
                                          xythetaPath.at(i).theta);
    }
    fclose(fSol);
}
 
void planxythetalat(char* envCfgFilename, char* motPrimFilename){
    // set the perimeter of the robot   
    vector<sbpl_2Dpt_t> perimeter;
    createFootprint(perimeter);
 
    // initialize an environment
    EnvironmentNAVXYTHETALAT env;
    initializeEnv(env, perimeter, envCfgFilename, motPrimFilename);
 
    // specify a start and goal state
    int start_id, goal_id;
    //setEnvStartGoal(env, .11, .11, 0, 35, 47.5, 0, start_id, goal_id);
    //setEnvStartGoal(env, 0.1, 0.125, 0, 0.325, 0.125, 0, start_id, goal_id);
    setEnvStartGoal(env, 0.325, 0.125, 0, 0, 0, 0, start_id, goal_id);
    //setEnvStartGoal(env, .15, .15, 0, 0, 0., 0, start_id, goal_id);
    std::cout << start_id << " " << goal_id << std::endl;
    int x, y, th;
    env.GetCoordFromState(goal_id, x, y, th);
    std::cout << "goal x:  " << x << " y: " << y << std::endl;
    env.GetCoordFromState(start_id, x, y, th);
    std::cout << "start x:  " << x << " y: " << y << std::endl;
  
    // initialize a planner with start and goal state
    //SBPLPlanner* planner = NULL;
    //double initialEpsilon = 3.0;
    //bool bsearchuntilfirstsolution = false;
    //initializePlanner(planner, env, start_id, goal_id, initialEpsilon, 
    //                  bsearchuntilfirstsolution);
 
    // plan
    //vector<int> solution_stateIDs;
    //double allocated_time_secs = 10.0; // in seconds
    //runPlanner(planner, allocated_time_secs, solution_stateIDs);
 
    // print stats
    //env.PrintTimeStat(stdout);
 
    // write out solutions
    //std::string filename("sol.txt");
    //writeSolution(env, solution_stateIDs, filename.c_str());
 
    //delete planner;

    std::unordered_map<std::pair<int,int>, int, EnvironmentNAVXYTHETALAT::pair_hash > obs_map;
    int obs_num = 0;
    env.CreateObsMap(obs_map, obs_num);

    // for (auto& x: obs_map)
    //  std::cout << "(" << x.first.first << ", " << x.first.second << "): " << x.second << std::endl;

    std::unordered_map<int, std::pair<int,int> > centroids;
    env.FindCentroids(obs_map, centroids, obs_num);

    for (auto& x: centroids)
       std::cout << x.first << ": (" << x.second.first << ", " << x.second.second << ")" << std::endl;

    //std::vector<std::vector<int> > S = {{1,3},{3}};
    std::vector<std::vector<int> > S = {{-3},{-3,-1}};
    //std::vector<std::vector<int> > S = {{-2,-1}};
    std::unordered_set<std::vector<int>, EnvironmentNAVXYTHETALAT::vector_hash> suffixes;
    env.Suffixes(S, suffixes);
    for (auto& x: suffixes) {
      for (int i = 0; i < x.size(); ++i) {
    	std::cout << x[i] << " ";
      }
      std::cout << std::endl;
    }

    //std::vector<int> beams(obs_num);
    //std::iota(test.begin(), test.end(), 1);
    
    std::priority_queue<vertex_sig, vertex_sig_vec, comparator> Q;
    std::unordered_map<std::pair<int, std::vector<int> >, std::pair<int, std::vector<int> >, hash_vertex_sig>  prev_;
    std::unordered_set<std::pair<int, std::vector<int> >, hash_vertex_sig> goals;
    env.HBSP(Q, prev_, goals, true, centroids, S, suffixes, env, start_id, goal_id);

    std::unordered_map<std::pair<int, std::vector<int> >, std::vector<std::pair<int, std::vector<int> > >, hash_vertex_sig> paths_;
    env.CreateGoalSet(goal_id, S, goals);
    env.GetHBSPPaths(goals, prev_, paths_);

    vector<int> solution_stateIDs;
    std::cout << goals.size() << std::endl;
    int cx, cy, cth;
    for(auto& p: paths_) {
      for(auto& c: p.second) {
    	env.GetCoordFromState(c.first, cx, cy, cth);
    	std::cout << "(" << cx << ", " << cy << ") ";
    	solution_stateIDs.push_back(c.first);
    	for(auto& sig: c.second)
    	  std::cout << sig << " ";
    	std::cout << std::endl;
      }
      std::cout << std::endl;
    }
    std::string filename("backwards_dijkstras.txt");
    writeSolution(env, solution_stateIDs, filename.c_str());
}
 
 
int main(int argc, char *argv[])
{
    planxythetalat(argv[1], argv[2]);
}
