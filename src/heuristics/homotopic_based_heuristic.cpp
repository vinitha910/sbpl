/*
 * Copyright (c) 2017, Vinitha Ranganeni
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Carnegie Mellon University nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
 
#include <sbpl/heuristics/homotopic_based_heuristic.h>
#include <sbpl/discrete_space_information/environment_navxythetalat.h>

HomotopicBasedHeuristic::HomotopicBasedHeuristic(DiscreteSpaceInformation* environment) :
    Heuristic(environment)
{
}

int HomotopicBasedHeuristic::GetGoalHeuristic(int& hidx, int& state_id, std::vector<int>& sig)
{
  EnvironmentNAVXYTHETALAT* env_ptr = dynamic_cast<EnvironmentNAVXYTHETALAT*>(m_environment);
  if(env_ptr){
    std::pair<int, std::vector<int> > v = make_pair(state_id, sig);
    return env_ptr->GetHBSPCost(v);
  } else {
    SBPL_ERROR("Heuristic Function can only be used with EnvironmentNAVXYTHETALAT");
  }
}

// int HomotopicBasedHeuristic::GetStartHeuristic(int state_id)
// {
//     return m_environment->GetStartHeuristic(state_id);
// }

// int HomotopicBasedHeuristic::GetFromToHeuristic(int from_id, int to_id)
// {
//     return m_environment->GetFromToHeuristic(from_id, to_id);
// }
