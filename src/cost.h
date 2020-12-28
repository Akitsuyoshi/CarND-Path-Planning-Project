#ifndef COST_H
#define COST_H

// Weights for each cost functions
const float COLLISION = pow(10, 6);
const float BUFFER = pow(10, 5);

float collision_cost() {
  float cost = 0.;
  return cost;
}

float buffer_cost() {
  float cost = 0.;
  return cost;
}

float calculate_cost() {
  vector<std::function> cost_functions = {collision_cost, buffer_cost};
  vector<float> weights = {COLLISION, BUFFER};
  float cost = 0.;

  for (int i = 0; i < cost_functions.size(); i++) {
    cost += weights[i] * cost_functions[i]();
  }
  return cost;
}

#endif  // COST_H