#include "MultivariatePolynom.h"

MultivariatePolynom::MultivariatePolynom(std::vector<Particle> particles){
	for (auto it = particles.begin(); it != particles.end(); it++){
		(*this) += *it;
	}
}

MultivariatePolynom::MultivariatePolynom(Particle p){
	(*this) += p;
}

MultivariatePolynom::MultivariatePolynom(Particle p1, Particle p2, Particle p3, Particle p4){
	(*this) += p1;
	(*this) += p2;
	(*this) += p3;
	(*this) += p4;
}

MultivariatePolynom::MultivariatePolynom(){}

float MultivariatePolynom::Evaluate(std::vector<float> values) {

	float result = 0;

	for (auto it = particles_.begin(); it != particles_.end(); it++){
		result += it->Evaluate(values);
	}

	return result;
}

float MultivariatePolynom::Evaluate(float x, float y, float z){
	std::vector<float> args;
	args.push_back(x);
	args.push_back(y);
	args.push_back(z);
	return Evaluate(args);
}


MultivariatePolynom MultivariatePolynom::Derive(int position) {

	MultivariatePolynom derivative = MultivariatePolynom(*this);

	for (auto it = derivative.particles_.begin(); it != derivative.particles_.end(); it++)
	{
		it->Derive(position);
	}

	CleanUp();

	return derivative;
}

void MultivariatePolynom::CleanUp() {
	particles_.erase(std::remove_if(particles_.begin(), particles_.end(), [] (Particle &p) -> bool { 
		return p.Coefficient <= 0.001 && p.Coefficient >= -0.001; } ), particles_.end());
}

MultivariatePolynom MultivariatePolynom::operator+=(const Particle &other) {
	for (auto it = particles_.begin(); it != particles_.end(); it++){
		if(other.EqualVariablesAndExponents(*it)){
			*it += other;

			if(it->Coefficient == 0){
				particles_.erase(it);
			}

			return *this;
		}
	}

	// No matching particle found
	particles_.push_back(other);
	return *this;
}

MultivariatePolynom MultivariatePolynom::operator+(const Particle &otherParticle) {
	return MultivariatePolynom(*this) += otherParticle;
}

MultivariatePolynom MultivariatePolynom::operator+=(const MultivariatePolynom &otherPolynom) {
	for (auto it = otherPolynom.particles_.begin(); it != otherPolynom.particles_.end(); it++){
		*this += *it;
	}
	return *this;
}

MultivariatePolynom MultivariatePolynom::operator+(const MultivariatePolynom &otherPolynom) {
	return MultivariatePolynom(*this) += otherPolynom;
}

MultivariatePolynom MultivariatePolynom::operator*=(const Particle &otherParticle) {
	for (auto it = particles_.begin(); it != particles_.end(); it++){
		(*it) *= otherParticle;
	}
	// Remove particles with coefficients that are 0 
	CleanUp();

	return *this;
}

MultivariatePolynom MultivariatePolynom::operator*(const Particle &otherParticle) {
	return MultivariatePolynom(*this) *= otherParticle;
}

MultivariatePolynom MultivariatePolynom::operator*=(const float otherFloat) {

	if (otherFloat == 0){
		particles_.clear();
	} else {
		for (auto it = particles_.begin(); it != particles_.end(); it++){
			it->Coefficient *= otherFloat;
		}
	}

	return *this;
}

MultivariatePolynom MultivariatePolynom::operator*(const float otherFloat) {
	return MultivariatePolynom(*this) *= otherFloat;
}

MultivariatePolynom MultivariatePolynom::operator*=(const MultivariatePolynom &otherPolynom) {

	MultivariatePolynom result = MultivariatePolynom();

	for (auto selfIt = particles_.begin(); selfIt != particles_.end(); selfIt++){
		for (auto otherIt = otherPolynom.particles_.begin(); otherIt != otherPolynom.particles_.end(); otherIt++){
			result +=(*selfIt) * (*otherIt);
		}
	}

	*this = result;

	// Remove particles with coefficient == 0
	CleanUp();

	return *this;
}

MultivariatePolynom MultivariatePolynom::operator*(const MultivariatePolynom &otherPolynom) {
	return MultivariatePolynom(*this) *= otherPolynom;
}