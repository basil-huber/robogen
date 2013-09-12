/*
 * @(#) Mutator.cpp   1.0   Sep 2, 2013
 *
 * Titus Cieslewski (dev@titus-c.ch)
 *
 * The ROBOGEN Framework
 * Copyright © 2013-2014 Titus Cieslewski
 *
 * Laboratory of Intelligent Systems, EPFL
 *
 * This file is part of the ROBOGEN Framework.
 *
 * The ROBOGEN Framework is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License (GPL)
 * as published by the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * @(#) $Id$
 */

#include "evolution/engine/Mutator.h"
#include <boost/random/uniform_int_distribution.hpp>

namespace robogen {

Mutator::Mutator(double pBrainMutate, double brainMuteSigma,
		double pBrainCrossover, boost::random::mt19937 &rng) :
		type_(BRAIN_MUTATOR), weightMutate_(pBrainMutate),
		weightCrossover_(pBrainCrossover),
		weightDistribution_(0.,brainMuteSigma), rng_(rng){
}

Mutator::~Mutator() {
}

std::pair<Individual,Individual> Mutator::mutate(
		std::pair<Individual,Individual> parents){
	this->mutate(parents.first); this->mutate(parents.second);
	this->crossover(parents.first,parents.second);
	return parents;
}

bool Mutator::mutate(Individual &robot){
	bool mutated = false;
	// mutate brain TODO conf bits?
	if (type_ == BRAIN_MUTATOR || type_ == BRAIN_BODY_PARAM_MUTATOR ||
			type_ == FULL_MUTATOR){
		std::vector<double*> weights;
		std::vector<double*> biases;
		robot.robot->getBrainGenome(weights,biases);
		// mutate weights
		for (unsigned int i=0; i<weights.size(); ++i){
			if (weightMutate_(rng_)){
				mutated = true;
				*weights[i] += weightDistribution_(rng_);
			}
			// normalize
			if (*weights[i]>1.) *weights[i] = 1.;
			if (*weights[i]<-1.) *weights[i] = -1;
		}
		// mutate biases
		for (unsigned int i=0; i<biases.size(); ++i){
			if (weightMutate_(rng_)){
				mutated = true;
				*biases[i] += weightDistribution_(rng_);
			}
			// normalize
			if (*biases[i]>1.) *biases[i] = 1.;
			if (*biases[i]<-1.) *biases[i] = -1.;
		}
		if (mutated){
			robot.evaluated = false;
		}
	}
	return mutated;
}

bool Mutator::crossover(Individual &a, Individual &b){
	if (!weightCrossover_(rng_)) return false;
	// at first, only one-point TODO two-point

	// 1. get genomes
	std::vector<double*> weights[2];
	std::vector<double*> biases[2];
	a.robot->getBrainGenome(weights[0],biases[0]);
	b.robot->getBrainGenome(weights[1],biases[1]);

	// 2. select crossover point
	unsigned int maxpoint = weights[0].size() + biases[0].size() - 1;
	if (maxpoint != weights[1].size() + biases[1].size() - 1){
		//TODO exception, TODO what if sum same, but not parts?
		std::cout << "Genomes not of same size!" << std::endl;
	}
	boost::random::uniform_int_distribution<unsigned int> pointSel(1,maxpoint);
	int selectedPoint = pointSel(rng_);

	// 3. perform crossover
	for (unsigned int i=selectedPoint; i<=maxpoint; i++){
		if (i<weights[0].size()){
			std::swap(*weights[0][i],*weights[1][i]);
		}
		else{
			int j = i-weights[0].size();
			std::swap(*biases[0][j],*biases[1][j]);
		}
	}

	a.evaluated = false; b.evaluated = false;
	return true;
}

} /* namespace robogen */
