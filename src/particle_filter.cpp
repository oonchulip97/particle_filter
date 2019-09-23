/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
 */

#include "particle_filter.h"

#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include "helper_functions.h"

using std::string;
using std::vector;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /**
   * TODO: Set the number of particles. Initialize all particles to 
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1. 
   * TODO: Add random Gaussian noise to each particle.
   * NOTE: Consult particle_filter.h for more information about this method 
   *   (and others in this file).
   */
  num_particles = 100;  // TODO: Set the number of particles

  std::default_random_engine gen;
  std::normal_distribution<double> dist_x(x, std[0]);
  std::normal_distribution<double> dist_y(y, std[1]);
  std::normal_distribution<double> dist_theta(theta, std[2]);

  for (size_t i = 0; i < num_particles; ++i) {

    Particle sample_particle;
    sample_particle.x = dist_x(gen);
    sample_particle.y = dist_y(gen);
    sample_particle.theta = dist_theta(gen);
    sample_particle.weight = 1.0;
    particles.push_back(sample_particle);
    weights.push_back(1.0);
  }

  is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], 
                                double velocity, double yaw_rate) {
  /**
   * TODO: Add measurements to each particle and add random Gaussian noise.
   * NOTE: When adding noise you may find std::normal_distribution 
   *   and std::default_random_engine useful.
   *  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
   *  http://www.cplusplus.com/reference/random/default_random_engine/
   */
  std::default_random_engine gen;
  std::normal_distribution<double> dist_x(0, std_pos[0]);
  std::normal_distribution<double> dist_y(0, std_pos[1]);
  std::normal_distribution<double> dist_theta(0, std_pos[2]);

  for (size_t i = 0; i < num_particles; ++i) {
    double theta = particles[i].theta;

    if (fabs(yaw_rate) < 0.001) { // yaw rate is small, yaw remains constant
      particles[i].x += velocity * delta_t * cos(theta);
      particles[i].y += velocity * delta_t * sin(theta);
    } else {
      particles[i].x += velocity / yaw_rate * (sin(theta + yaw_rate * delta_t) - sin(theta));
      particles[i].y += velocity / yaw_rate * (cos(theta) - cos(theta + yaw_rate * delta_t));
      particles[i].theta += yaw_rate * delta_t;
    }

    particles[i].x += dist_x(gen);
    particles[i].y += dist_y(gen);
    particles[i].theta += dist_theta(gen);
  }
}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, 
                                     vector<LandmarkObs>& observations) {
  /**
   * TODO: Find the predicted measurement that is closest to each 
   *   observed measurement and assign the observed measurement to this 
   *   particular landmark.
   * NOTE: this method will NOT be called by the grading code. But you will 
   *   probably find it useful to implement this method and use it as a helper 
   *   during the updateWeights phase.
   */
  for (size_t i = 0; i < observations.size(); ++i) {
    int closest_index = -1;
    double smallest_distance = std::numeric_limits<double>::infinity();

    double x_obs = observations[i].x;
    double y_obs = observations[i].y;
    for (size_t j = 0; j < predicted.size(); ++j){
      double x_pred = predicted[j].x;
      double y_pred = predicted[j].y;
      double distance = dist(x_obs, y_obs, x_pred, y_pred);

      if (distance <= smallest_distance) {
        smallest_distance = distance;
        closest_index = j;
      }
    }

    observations[i].id = predicted[closest_index].id;
  }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
                                   const vector<LandmarkObs> &observations, 
                                   const Map &map_landmarks) {
  /**
   * TODO: Update the weights of each particle using a mult-variate Gaussian 
   *   distribution. You can read more about this distribution here: 
   *   https://en.wikipedia.org/wiki/Multivariate_normal_distribution
   * NOTE: The observations are given in the VEHICLE'S coordinate system. 
   *   Your particles are located according to the MAP'S coordinate system. 
   *   You will need to transform between the two systems. Keep in mind that
   *   this transformation requires both rotation AND translation (but no scaling).
   *   The following is a good resource for the theory:
   *   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
   *   and the following is a good resource for the actual equation to implement
   *   (look at equation 3.33) http://planning.cs.uiuc.edu/node99.html
   */
  for (size_t i = 0; i < num_particles; ++i) {
    double x_part = particles[i].x;
    double y_part = particles[i].y;
    double theta = particles[i].theta;

    // Create predicted observation based on particle's position
    vector<LandmarkObs> predicted;
    for (size_t j = 0; j < map_landmarks.landmark_list.size(); ++j) {
      double x_map = map_landmarks.landmark_list[j].x_f;
      double y_map = map_landmarks.landmark_list[j].y_f;
      double distance = dist(x_part, y_part, x_map, y_map);
      if (distance <= sensor_range){
        LandmarkObs predicted_obs;
        predicted_obs.id = map_landmarks.landmark_list[j].id_i;
        predicted_obs.x = x_map;
        predicted_obs.y = y_map;
        predicted.push_back(predicted_obs);
      }
    }

    // Transform observation from particle's frame to map's frame
    vector<LandmarkObs> transformed;
    for (size_t j = 0; j < observations.size(); ++j) {
      LandmarkObs transformed_obs;
      double x_obs = observations[j].x;
      double y_obs = observations[j].y;
      transformed_obs.x = x_part + (cos(theta) * x_obs) - (sin(theta) * y_obs);
      transformed_obs.y = y_part + (sin(theta) * x_obs) + (cos(theta) * y_obs);
      transformed.push_back(transformed_obs);
    }

    // Find nearest landmark on the map with respect to each transformed measurement
    dataAssociation(predicted, transformed);

    // Update the weight of each particle based on all measurements
    double weight = 1.0;
    double sig_x = std_landmark[0];
    double sig_y = std_landmark[1];
    for (size_t j = 0; j < transformed.size(); ++j) {
      double x_obs = transformed[j].x;
      double y_obs = transformed[j].y;
      double mu_x, mu_y;
      for (size_t k = 0; k < map_landmarks.landmark_list.size(); k++) {
        if (transformed[j].id == map_landmarks.landmark_list[k].id_i) {
          mu_x = map_landmarks.landmark_list[k].x_f;
          mu_y = map_landmarks.landmark_list[k].y_f;
          break;
        }
      }
      weight *= multiv_prob(sig_x, sig_y, x_obs, y_obs, mu_x, mu_y);
    }
    particles[i].weight = weights[i] = weight;
  }
}

void ParticleFilter::resample() {
  /**
   * TODO: Resample particles with replacement with probability proportional 
   *   to their weight. 
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */
  std::default_random_engine gen;
  std::discrete_distribution<int> posterior_dist(weights.begin(), weights.end());

  std::vector<Particle> aux_particles;
  for (size_t i = 0; i < num_particles; ++i) {
    aux_particles.push_back(particles[posterior_dist(gen)]);
  }

  particles = aux_particles;
}

void ParticleFilter::SetAssociations(Particle& particle, 
                                     const vector<int>& associations, 
                                     const vector<double>& sense_x, 
                                     const vector<double>& sense_y) {
  // particle: the particle to which assign each listed association, 
  //   and association's (x,y) world coordinates mapping
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates
  particle.associations= associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) {
  vector<int> v = best.associations;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord) {
  vector<double> v;

  if (coord == "X") {
    v = best.sense_x;
  } else {
    v = best.sense_y;
  }

  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

double ParticleFilter::multiv_prob(double sig_x, double sig_y, double x_obs, double y_obs,
                                   double mu_x, double mu_y) {
  // calculate normalization term
  double gauss_norm;
  gauss_norm = 1 / (2 * M_PI * sig_x * sig_y);

  // calculate exponent
  double exponent;
  exponent = (pow(x_obs - mu_x, 2) / (2 * pow(sig_x, 2)))
              + (pow(y_obs - mu_y, 2) / (2 * pow(sig_y, 2)));

  // calculate weight using normalization terms and exponent
  double weight;
  weight = gauss_norm * exp(-exponent);

  return weight;
}
