/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
	// Default random generator
	default_random_engine gen;

	// This line creates a normal (Gaussian) distribution for x.
	
	particles = vector<Particle>(N);

	normal_distribution<double> dist_x(x, std[0]);

	// TODO: Create normal distributions for y and theta.
	normal_distribution<double> dist_y(y, std[1]);
	normal_distribution<double> dist_theta(theta, std[2]);

	for (int i = 0; i < N; i++) {
		particles[i].id = i;
		particles[i].x = dist_x(gen);
		particles[i].y = dist_y(gen);
		particles[i].theta = dist_theta(gen);
		particles[i].weight = 1;

	}

}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

	default_random_engine gen;
	normal_distribution<double> dist_x(0, std_pos[0]);
	normal_distribution<double> dist_y(0, std_pos[1]);
	normal_distribution<double> dist_theta(0, std_pos[2]);

	for (int i = 0; i < N; i++) {
		double th = particles[i].theta;

		if (abs(yaw_rate) < 0.0001) {
			particles[i].x = particles[i].x + velocity * cos(th) + dist_x(gen);
			particles[i].y = particles[i].y + velocity * sin(th) + dist_y(gen);
			particles[i].theta = th + dist_theta(gen);
		}
		else {
			particles[i].x = particles[i].x + velocity / yaw_rate * (sin(th + yaw_rate) - sin(th)) + dist_x(gen);
			particles[i].y = particles[i].y + velocity / yaw_rate * (cos(th) - cos(th + yaw_rate)) + dist_y(gen);
			particles[i].theta = th + yaw_rate + dist_theta(gen);
		}
		
	}

}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		std::vector<LandmarkObs> &observations, Map &map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html

	double weight_sum = 0.0;

	for (int i = 0; i < N; i++) {
		double th = particles[i].theta;
		double xp = particles[i].x;
		double yp = particles[i].y;

		for (int j = 0; j < observations.size(); j++)
		{
			double xo = observations[j].x;
			double yo = observations[j].y;
			double xt = xo*cos(th) - yo*sin(th) + xp;
			double yt = xo*sin(th) + yo*cos(th) + yp;

			double min_distance = 99999;
			for (int k = 0; k < map_landmarks.landmark_list.size(); k++) {
				double distance = dist(xo, yo, map_landmarks.landmark_list[k].x, map_landmarks.landmark_list[k].y);

				if (distance < min_distance) {
					observations[j].id = map_landmarks.landmark_list[k].id;
					min_distance = distance;
				}
			}
			double x_ = observations[j].x;
			double y_ = observations[j].y;
			double mu_x = map_landmarks.landmark_list[k].x;
			double mu_y = map_landmarks.landmark_list[k].y;

			double prob = exp(-pow(x_ - mu_x, 2) / (2 * pow(std_landmark[0], 2)) - pow(y_ - mu_y, 2) / (2 * pow(std_landmark[1], 2))) / (2 * M_PI*std_landmark[0] * std_landmark[1]);
			particles[i].weight *= prob;
		}

		weight_sum += particles[i].weight;

	}

	for (int i = 0; i < N; i++) {
		particles[i].weight = particles[i].weight / weight_sum;
	}


}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

	default_random_engine gen;
	vector<double> new_weights(N);

	for (int i = 0; i < N; i++)
	{
		new_weights[i] = particles[i].weight;
	}

	std::discrete_distribution<> dist(new_weights.begin(), new_weights.end());

	for (int i = 0; i < N; i++) {
		particles[i].weight = new_weights[dist(gen)];
	}

}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, 
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

    particle.associations= associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
