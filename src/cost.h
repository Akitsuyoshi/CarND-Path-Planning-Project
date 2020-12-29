#ifndef COST_H
#define COST_H

using std::vector;

const int BUFFER_BETWEEN_CAR = 30;
// Weights for each cost functions
const float COLLISION = pow(10, 6);
const float BUFFER = pow(10, 5);

float logistic(double x) { return 2.0 / (1 + exp(-x)) - 1.0; }

float collision_cost(double car_s, double check_car_s) {
  if (abs(check_car_s - car_s) < BUFFER_BETWEEN_CAR) {
    return 1.;
  }
  return 0.;
}

float buffer_cost(double car_s, double check_car_s) {
  return logistic(BUFFER_BETWEEN_CAR / abs(check_car_s - car_s));
}

float calculate_cost(double car_s, double check_car_s) {
  vector<std::function<float(double car_s, double check_car_s)>>
      cost_functions = {collision_cost, buffer_cost};
  vector<float> weights = {COLLISION, BUFFER};
  float cost = 0.;

  for (int i = 0; i < cost_functions.size(); i++) {
    cost += weights[i] * cost_functions[i](car_s, check_car_s);
  }
  return cost;
}

#endif  // COST_H