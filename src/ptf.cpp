/*
 * particle_filter.cpp
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>

#include "ptf.h"

using namespace std;

void Particle_Filter::init(double x, double y, double theta, double std[]) {
	// Set how many particle you would use to describe the target function. Initialize all particles to first position being the ground truth data
	// (x, y, theta and their uncertainties coming from GPS) and set all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Refer ptf.h for more information about the methods assosiated with particle class (and others in this file).

    num_particles = 2444; //set to number of files in observation directory

    weights.resize(num_particles);
    particles.resize(num_particles);

    double std_x, std_y, std_theta; // Standard deviations for x, y, and theta
    std_x = std[0];
    std_y = std[1];
    std_theta = std[2];

    // Normal distribution for x, y and theta
    normal_distribution<double> dist_x(x, std_x); // mean is centered around the new measurement
    normal_distribution<double> dist_y(y, std_y);
    normal_distribution<double> dist_theta(theta, std_theta);

    default_random_engine gen; //http://www.cplusplus.com/reference/random/default_random_engine/

    // generate particles and set their values
    for(int i=0; i<num_particles; ++i){
        PARTICLE p;
        p.id = i;
        p.x = dist_x(gen); // take a random value from the Gaussian Normal distribution and update the attribute
        p.y = dist_y(gen);
        p.theta = dist_theta(gen);
        p.weight = 1;

        particles[i] = p;
        weights[i] = p.weight;
    }
    is_initialized = true;
}

void Particle_Filter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {

    double std_x, std_y, std_theta; // Standard deviations for x, y, and theta
    std_x = std_pos[0];
    std_y = std_pos[1];
    std_theta = std_pos[2];

    default_random_engine gen;

    for(int i=0; i<num_particles; ++i){
        PARTICLE *p = &particles[i]; // get address of particle to update

        // use the prediction equations from the Lesson 14
        double new_x = p->x + (velocity/yaw_rate) * (sin(p->theta + yaw_rate*delta_t) - sin(p->theta));
        double new_y = p->y + (velocity/yaw_rate) * (cos(p->theta) - cos(p->theta + yaw_rate*delta_t));
        double new_theta = p->theta + (yaw_rate*delta_t);

        // add Gaussian Noise to each measurement
        // Normal distribution for x, y and theta
        normal_distribution<double> dist_x(new_x, std_x);
        normal_distribution<double> dist_y(new_y, std_y);
        normal_distribution<double> dist_theta(new_theta, std_theta);

        // update the particle attributes
        p->x = dist_x(gen);
        p->y = dist_y(gen);
        p->theta = dist_theta(gen);
    }
}

void Particle_Filter::data_association(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	
    for(auto pred : predicted){
      double dist_min = std::numeric_limits<double>::max();
      for(auto observation : observations){
        double distance = dist(observation.x, observation.y, pred.x, pred.y); // distance b/w obs and landmark
        if(distance < dist_min){
          observation.id = pred.id;
        }
        dist_min = distance;
      }
    }
}

void Particle_Filter::update_weights(double sensor_range, double std_landmark[], 
		std::vector<LandmarkObs> observations, Map map_landmarks) {

    double std_x = std_landmark[0];
    double std_y = std_landmark[1];
    double weights_sum = 0;

    for(int i=0; i<num_particles; ++i){
        PARTICLE *p = &particles[i];
        double wt = 1.0;

        // Convert from robot's/vehicle's coordinate system to the map's coordinate system
        for(int j=0; j<observations.size(); ++j){
            LandmarkObs current_obs = observations[j];
            LandmarkObs transformed_obs;

            transformed_obs.x = (current_obs.x * cos(p->theta)) - (current_obs.y * sin(p->theta)) + p->x;
            transformed_obs.y = (current_obs.x * sin(p->theta)) + (current_obs.y * cos(p->theta)) + p->y;
            transformed_obs.id = current_obs.id;

            // choose the predicted measurement that is closest to the observed measurement and assign
            // the observed measurement to that particular landmark
            Map::single_landmark_s landmark;
            double distance_min = std::numeric_limits<double>::max();

            for(int k=0; k<map_landmarks.landmark_list.size(); ++k){
                Map::single_landmark_s cur_l = map_landmarks.landmark_list[k];
                double distance = dist(transformed_obs.x, transformed_obs.y, cur_l.x_f, cur_l.y_f);
                if(distance < distance_min){
                    distance_min = distance;
                    landmark = cur_l;
                }
            }

            // Multivariate Gaussian Distribution is used to update weights
            
            double num = exp(-0.5 * (pow((transformed_obs.x - landmark.x_f), 2) / pow(std_x, 2) + pow((transformed_obs.y - landmark.y_f), 2) / pow(std_y, 2)));
            double denom = 2 * M_PI * std_x * std_y;
            wt *= num/denom;
        }
        weights_sum += wt;
        p->weight = wt;
    }
    // weights have to be normalised and brought in (0, 1]
    for (int i = 0; i < num_particles; i++) {
        PARTICLE *p = &particles[i];
        p->weight /= weights_sum;
        weights[i] = p->weight;
    }
}

void Particle_Filter::resampling() {
	

    default_random_engine gen;

    // Random integers on the [0, n) range
    // the probability of each individual integer depends on its weight divided by the total summed weights.
    discrete_distribution<int> distribution(weights.begin(), weights.end());
    vector<PARTICLE> resampled_particles;

    for (int i = 0; i < num_particles; i++){
        resampled_particles.push_back(particles[distribution(gen)]);
    }

    particles = resampled_particles;

}

void Particle_Filter::write(std::string filename) {
	// You don't need to modify this file.
	std::ofstream dataFile;
	dataFile.open(filename, std::ios::app);
	for (int i = 0; i < num_particles; ++i) {
		dataFile << particles[i].x << " " << particles[i].y << " " << particles[i].theta << "\n";
	}
	dataFile.close();
}