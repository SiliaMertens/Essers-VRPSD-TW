#include <iostream>
#include <cstdlib>
#include <string>
#include <vector>
#include <numeric>
#include <time.h>
#include <ctime> 
#include <chrono>
#include <algorithm>

#include "ProbabilityEstimator.h"
#include "Heuristic.h"

using namespace std;

void show(vector<int> const& input) {
	cout << "shuffle ";
	for (auto const& i : input) {
		std::cout << i << " ";
	}
	cout << "\n";
}



string data_file = "General_Cargo_LTL_2018_v10072019_input_code 10 customers.txt";
string coordinates_file = "distance_matrix 10 klanten.txt";

const double time_window_violation_cost = 1;

/* Main function to run the algorithm */
int main(int argc, char* argv[]) {

	double perturbation_percentage = 0.2;
	int value_no_improvement = 2;

	srand(time(NULL));

	auto start = chrono::system_clock::now();

	struct problem p;

	p.collection_date = "2-Jan-2018";

	//if (argc > 1) {
	//	data_file = argv[1];
	//	p.collection_date = argv[2];
	//	coordinates_file = argv[3];
	//	time_window_violation_cost = atof(argv[4]);
	//}

	//if (argc > 1) {
	//	perturbation_percentage = atoi(argv[1]);
	//	value_no_improvement = atoi(argv[2]);

	//}

	read_data(p);
	read_distance_and_time_matrix(p);

	p.pe.readDistributions(p.collection_date); // read probabilities of 'p.collection_date'

	struct solution s_curr;
	struct solution s_prev;
	struct solution s_local_best;
	struct solution s_total_best;
	struct solution s_ILS_best;
	initialize_solution(p, s_curr);
	initialize_solution(p, s_prev);
	initialize_solution(p, s_local_best);
	initialize_solution(p, s_total_best);
	initialize_solution(p, s_ILS_best);

	//struct solution s_recourse;
	//initialize_solution(p, s_recourse);

	//s_curr.routes[0].route = { 0, 5, 7, 9, 0 };
	//s_curr.routes[1].route = { 0, 4, 8, 0 };
	//s_curr.routes[2].route = { 0, 10, 0 };
	//s_curr.routes[3].route = { 0, 1, 3, 2, 6, 0 };

	//update_solution(p, s_curr, s_recourse);

	//for (int vehicle_id = 0; vehicle_id < p.n_vehicles; vehicle_id++) {
	//	update_earliest_time(p, s_curr, vehicle_id);
	//	update_latest_time(p, s_curr, vehicle_id);
	//	update_schedule(p, s_curr, vehicle_id);
	//}

	//for (int vehicle_id = 0; vehicle_id < p.n_vehicles; vehicle_id++) {
	//	bereken_route_cost_zonder_recourse(p, s_curr, vehicle_id);
	//	bereken_route_cost(p, s_curr, vehicle_id);
	//	calculate_probabilities(p, s_curr, vehicle_id);
	//}



	//for (int vehicle_id = 0; vehicle_id < p.n_vehicles; vehicle_id++) {
	//	bereken_gewogen_route_cost(p, s_curr, s_recourse, vehicle_id);
	//}

	//calculate_total_cost(p, s_curr);

	//cout << "Initial solution with " << s_curr.number_of_vehicles_used << " vehicles and distance " << s_curr.total_distance_cost
	//	<< " route duration " << s_curr.total_route_duration << " time window violation " << s_curr.total_time_window_violation
	//	<< " overtime " << s_curr.total_overtime << " allowable operating time " << s_curr.total_driving_time << " total cost " << s_curr.total_cost << "\n";
	//for (int vehicle_id = 0; vehicle_id < p.n_vehicles; vehicle_id++) {
	//	for (size_t position = 0; position < s_curr.routes[vehicle_id].route.size(); position++) {
	//		cout << s_curr.routes[vehicle_id].route[position] << " ";
	//	}

	//	cout << "\n";
	//}

	//write_output_file(p, s_curr);


	vector<int> customers_to_be_inserted = {};

	for (int customer_id = 1; customer_id <= p.n_customers; customer_id++) {

		customers_to_be_inserted.push_back(customer_id);

	}

	//cout << "customers to be inserted " << customers_to_be_inserted.size() << "\n";

	for (int i = 0; i < customers_to_be_inserted.size() - 1; i++) {
		int j = i + rand() % (customers_to_be_inserted.size() - i);
		swap(customers_to_be_inserted[i], customers_to_be_inserted[j]);
	}

	show(customers_to_be_inserted);

	for (int customer_id = 0; customer_id <= customers_to_be_inserted.size() - 1; customer_id++) {

		perform_best_insertion(p, s_curr, customers_to_be_inserted[customer_id]);

	}

	//for (int customer_id = 1; customer_id <= p.n_customers; customer_id++) {
	//	perform_best_insertion(p, s_curr, customer_id);

	//	cout << "customer_id " << customer_id << "\n";
	//}
	
	

	update_solution(p, s_curr, s_local_best);

	cout << "Initial solution with " << s_local_best.number_of_vehicles_used << " vehicles and distance " << s_local_best.total_distance_cost
		<< " route duration " << s_local_best.total_route_duration << " time window violation " << s_local_best.total_time_window_violation <<
		" overtime " << s_local_best.total_overtime << " allowable operating time " << s_local_best.total_driving_time << " total cost " << s_local_best.total_cost << "\n";

	for (int vehicle_id = 0; vehicle_id < p.n_vehicles; vehicle_id++) {
		for (size_t position = 0; position < s_local_best.routes[vehicle_id].route.size(); position++) {
			cout << s_local_best.routes[vehicle_id].route[position] << " ";
		}

		cout << "\n";
	}

	update_solution(p, s_local_best, s_total_best);
	update_solution(p, s_local_best, s_ILS_best);

	cout << "total best " << s_total_best.total_cost << "\n";
	//cout << "ILS best " << s_ILS_best.total_cost << "\n";

	update_solution(p, s_local_best, s_prev);
	relocate(p, s_prev, s_curr, s_local_best);

	//cout << "\nNew best solution after first relocate with " << s_local_best.number_of_vehicles_used << " vehicles and distance " << s_local_best.total_distance_cost
	//	<< " route duration " << s_local_best.total_route_duration << " total cost " << s_local_best.total_cost << "\n";
	//for (int vehicle_id = 0; vehicle_id < p.n_vehicles; vehicle_id++) {
	//	for (size_t position = 0; position < s_local_best.routes[vehicle_id].route.size(); position++) {
	//		cout << s_local_best.routes[vehicle_id].route[position] << " ";
	//	}
	//	cout << "\n";
	//}

	//cout << "best cost na first relocate " << s_local_best.total_cost << "\n";

	//if (s_local_best.total_cost < s_total_best.total_cost) {
	//	update_solution(p, s_local_best, s_total_best);

	//	cout << "total best " << s_total_best.total_cost << "\n";
	//}


	while (s_local_best.total_cost < s_prev.total_cost) { // while loop uitvoeren op RELOCATE totdat er geen verbeteringen meer zijn 

		update_solution(p, s_local_best, s_prev);
		relocate(p, s_prev, s_curr, s_local_best);

		cout << "\nNew best solution after relocate with " << s_local_best.number_of_vehicles_used << " vehicles and distance " << s_local_best.total_distance_cost
			<< " route duration " << s_local_best.total_route_duration << " time window violation " << s_local_best.total_time_window_violation <<
			" overtime " << s_local_best.total_overtime << " allowable operating time " << s_local_best.total_driving_time << " total cost " << s_local_best.total_cost << "\n";
		for (int vehicle_id = 0; vehicle_id < p.n_vehicles; vehicle_id++) {
			for (size_t position = 0; position < s_local_best.routes[vehicle_id].route.size(); position++) {
				cout << s_local_best.routes[vehicle_id].route[position] << " ";
			}
			cout << "\n";
		}

		cout << "best cost na relocate " << s_local_best.total_cost << "\n";
	}

	if (s_local_best.total_cost < s_total_best.total_cost) {
		update_solution(p, s_local_best, s_total_best);
		update_solution(p, s_local_best, s_ILS_best);

		cout << "total best " << s_total_best.total_cost << "\n";
		//cout << "ILS best " << s_ILS_best.total_cost << "\n";
	}

	update_solution(p, s_local_best, s_prev);
	swap(p, s_prev, s_curr, s_local_best);

	while (s_local_best.total_cost < s_prev.total_cost) {// while loop uitvoeren op SWAP totdat er geen verbeteringen meer zijn 
		update_solution(p, s_local_best, s_prev);
		swap(p, s_prev, s_curr, s_local_best);

		cout << "\nNew best solution after swap with " << s_local_best.number_of_vehicles_used << " vehicles and distance " << s_local_best.total_distance_cost
			<< " route duration " << s_local_best.total_route_duration << " time window violation " << s_local_best.total_time_window_violation <<
			" overtime " << s_local_best.total_overtime << " allowable operating time " << s_local_best.total_driving_time << " total cost " << s_local_best.total_cost << "\n";
		for (int vehicle_id = 0; vehicle_id < p.n_vehicles; vehicle_id++) {
			for (size_t position = 0; position < s_local_best.routes[vehicle_id].route.size(); position++) {
				cout << s_local_best.routes[vehicle_id].route[position] << " ";
			}

			cout << "\n";
		}

		cout << "best cost na swap " << s_local_best.total_cost << "\n";
	}

	if (s_local_best.total_cost < s_total_best.total_cost) {
		update_solution(p, s_local_best, s_total_best);
		update_solution(p, s_local_best, s_ILS_best);

		cout << "total best " << s_total_best.total_cost << "\n";
		cout << "ILS best " << s_ILS_best.total_cost << "\n";
	}

	update_solution(p, s_local_best, s_prev);
	relocate(p, s_prev, s_curr, s_local_best);

	cout << "\nNew best solution after first relocate with " << s_local_best.number_of_vehicles_used << " vehicles and distance " << s_local_best.total_distance_cost
		<< " route duration " << s_local_best.total_route_duration << " time window violation " << s_local_best.total_time_window_violation <<
		" overtime " << s_local_best.total_overtime << " allowable operating time " << s_local_best.total_driving_time << " total cost " << s_local_best.total_cost << "\n";
	for (int vehicle_id = 0; vehicle_id < p.n_vehicles; vehicle_id++) {
		for (size_t position = 0; position < s_local_best.routes[vehicle_id].route.size(); position++) {
			cout << s_local_best.routes[vehicle_id].route[position] << " ";
		}
		cout << "\n";
	}

	cout << "best cost na first relocate " << s_local_best.total_cost << "\n";

	if (s_local_best.total_cost < s_total_best.total_cost) {
		update_solution(p, s_local_best, s_total_best);

		cout << "total best " << s_total_best.total_cost << "\n";
	}


	while (s_local_best.total_cost < s_prev.total_cost) { // while loop op RELOCATE uitvoeren totdat er geen verbeteringen meer zijn 

		update_solution(p, s_local_best, s_prev);
		relocate(p, s_prev, s_curr, s_local_best);

		cout << "\nNew best solution after relocate with " << s_local_best.number_of_vehicles_used << " vehicles and distance " << s_local_best.total_distance_cost
			<< " route duration " << s_local_best.total_route_duration << " time window violation " << s_local_best.total_time_window_violation <<
			" overtime " << s_local_best.total_overtime << " allowable operating time " << s_local_best.total_driving_time << " total cost " << s_local_best.total_cost << "\n";
		for (int vehicle_id = 0; vehicle_id < p.n_vehicles; vehicle_id++) {
			for (size_t position = 0; position < s_local_best.routes[vehicle_id].route.size(); position++) {
				cout << s_local_best.routes[vehicle_id].route[position] << " ";
			}
			cout << "\n";
		}

		cout << "best cost na relocate " << s_local_best.total_cost << "\n";
	}

	if (s_local_best.total_cost < s_total_best.total_cost) { // moet dit hier ook telkens, of pas na de laatste relocate? 
		update_solution(p, s_local_best, s_total_best);
		update_solution(p, s_local_best, s_ILS_best);

		cout << "total best " << s_total_best.total_cost << "\n";
		//cout << "ILS best " << s_ILS_best.total_cost << "\n";
	}


	int number_of_times_without_improvement = 0;

	cout << "value no improvement " << value_no_improvement << "\n";

	//for (int iteration = 0; iteration < iterations; iteration++) {
	while (number_of_times_without_improvement < value_no_improvement) {

		update_solution(p, s_total_best, s_local_best);

		vector<int> random_customers = {};

		while (random_customers.size() < perturbation_percentage * p.n_customers) {

			int customer = rand() % p.n_customers + 1;

			std::vector<int>::iterator it = find(random_customers.begin(), random_customers.end(), customer);
			if (it == random_customers.end()) {
				//cout << "curr " << j_curr << "\n";
				random_customers.push_back(customer);
			}

		}

		cout << "verwijderde klanten ";
		for (size_t i = 0; i < random_customers.size(); i++) {
			cout << random_customers[i] << " ";
		}

		cout << "\n";

		s_local_best.route_customer = {};
		s_local_best.position_customer = {};

		for (size_t i = 0; i < random_customers.size(); i++) {
			position_removed_customers(p, s_local_best, random_customers[i]);
			remove_customer(p, s_local_best, s_local_best.route_customer[i], s_local_best.position_customer[i]);
		}



		cout << "route na verwijdering " << "\n";
		for (int vehicle_id = 0; vehicle_id < p.n_vehicles; vehicle_id++) {
			for (size_t position = 0; position < s_local_best.routes[vehicle_id].route.size(); position++) {
				cout << s_local_best.routes[vehicle_id].route[position] << " ";
			}

			cout << "\n";

		}

		cout << "\n";



		for (size_t customer_id = 0; customer_id < random_customers.size(); customer_id++) { // verwijderde klanten terug invoegen 
			perform_best_insertion(p, s_local_best, random_customers[customer_id]);
		}

		cout << "New initial solution after perturbation with " << s_local_best.number_of_vehicles_used << " vehicles and distance " << s_local_best.total_distance_cost
			<< " route duration " << s_local_best.total_route_duration << " time window violation " << s_local_best.total_time_window_violation <<
			" overtime " << s_local_best.total_overtime << " allowable operating time " << s_local_best.total_driving_time << " total cost " << s_local_best.total_cost << "\n";
		for (int vehicle_id = 0; vehicle_id < p.n_vehicles; vehicle_id++) {
			for (size_t position = 0; position < s_local_best.routes[vehicle_id].route.size(); position++) {
				cout << s_local_best.routes[vehicle_id].route[position] << " ";
			}

			cout << "\n";

		}

		cout << "\n";

		write_output_file_perturbation(p, s_local_best);

		if (s_local_best.total_cost < s_total_best.total_cost) {
			update_solution(p, s_local_best, s_total_best);

			cout << "total best " << s_total_best.total_cost << "\n";
		}

		update_solution(p, s_local_best, s_prev);
		relocate(p, s_prev, s_curr, s_local_best);

		//cout << "\nNew best solution after first relocate with " << s_local_best.number_of_vehicles_used << " vehicles and distance " << s_local_best.total_distance_cost
		//	<< " route duration " << s_local_best.total_route_duration << " total cost " << s_local_best.total_cost << "\n";
		//for (int vehicle_id = 0; vehicle_id < p.n_vehicles; vehicle_id++) {
		//	for (size_t position = 0; position < s_local_best.routes[vehicle_id].route.size(); position++) {
		//		cout << s_local_best.routes[vehicle_id].route[position] << " ";
		//	}
		//	cout << "\n";
		//}

		//cout << "best cost na first relocate " << s_local_best.total_cost << "\n";

		//if (s_local_best.total_cost < s_total_best.total_cost) {
		//	update_solution(p, s_local_best, s_total_best);

		//	cout << "total best " << s_total_best.total_cost << "\n";
		//}

		while (s_local_best.total_cost < s_prev.total_cost) { // while loop uitvoeren totdat er geen verbeteringen meer zijn 

			update_solution(p, s_local_best, s_prev);
			relocate(p, s_prev, s_curr, s_local_best);

			cout << "\nNew best solution after relocate with " << s_local_best.number_of_vehicles_used << " vehicles and distance " << s_local_best.total_distance_cost
				<< " route duration " << s_local_best.total_route_duration << " time window violation " << s_local_best.total_time_window_violation <<
				" overtime " << s_local_best.total_overtime << " allowable operating time " << s_local_best.total_driving_time << " total cost " << s_local_best.total_cost << "\n";
			for (int vehicle_id = 0; vehicle_id < p.n_vehicles; vehicle_id++) {
				for (size_t position = 0; position < s_local_best.routes[vehicle_id].route.size(); position++) {
					cout << s_local_best.routes[vehicle_id].route[position] << " ";
				}

				cout << "\n";
			}

			cout << "best cost na relocate " << s_local_best.total_cost << "\n";

		}

		if (s_local_best.total_cost < s_total_best.total_cost) {
			update_solution(p, s_local_best, s_total_best);

			cout << "total best " << s_total_best.total_cost << "\n";
		}

		if (s_local_best.total_cost < s_ILS_best.total_cost) {
			update_solution(p, s_local_best, s_ILS_best);

			cout << "ILS best " << s_ILS_best.total_cost << "\n";
		}

		update_solution(p, s_local_best, s_prev);
		swap(p, s_prev, s_curr, s_local_best);

		while (s_local_best.total_cost < s_prev.total_cost) {
			update_solution(p, s_local_best, s_prev);
			swap(p, s_prev, s_curr, s_local_best);

			cout << "\nNew best solution after swap with " << s_local_best.number_of_vehicles_used << " vehicles and distance " << s_local_best.total_distance_cost
				<< " route duration " << s_local_best.total_route_duration << " time window violation " << s_local_best.total_time_window_violation <<
				" overtime " << s_local_best.total_overtime << " allowable operating time " << s_local_best.total_driving_time << " total cost " << s_local_best.total_cost << "\n";
			for (int vehicle_id = 0; vehicle_id < p.n_vehicles; vehicle_id++) {
				for (size_t position = 0; position < s_local_best.routes[vehicle_id].route.size(); position++) {
					cout << s_local_best.routes[vehicle_id].route[position] << " ";
				}

				cout << "\n";
			}

			cout << "best cost na swap " << s_local_best.total_cost << "\n";
		}

		if (s_local_best.total_cost < s_total_best.total_cost) {
			update_solution(p, s_local_best, s_total_best);

			cout << "total best " << s_total_best.total_cost << "\n";
		}

		if (s_local_best.total_cost < s_ILS_best.total_cost) {
			update_solution(p, s_local_best, s_ILS_best);

			cout << "ILS best " << s_ILS_best.total_cost << "\n";
		}

		update_solution(p, s_local_best, s_prev);
		relocate(p, s_prev, s_curr, s_local_best);

		//cout << "\nNew best solution after first relocate with " << s_local_best.number_of_vehicles_used << " vehicles and distance " << s_local_best.total_distance_cost
		//	<< " route duration " << s_local_best.total_route_duration << " total cost " << s_local_best.total_cost << "\n";
		//for (int vehicle_id = 0; vehicle_id < p.n_vehicles; vehicle_id++) {
		//	for (size_t position = 0; position < s_local_best.routes[vehicle_id].route.size(); position++) {
		//		cout << s_local_best.routes[vehicle_id].route[position] << " ";
		//	}
		//	cout << "\n";
		//}

		//cout << "best cost na first relocate " << s_local_best.total_cost << "\n";

		//if (s_local_best.total_cost < s_total_best.total_cost) {
		//	update_solution(p, s_local_best, s_total_best);

		//	cout << "total best " << s_total_best.total_cost << "\n";
		//}

		while (s_local_best.total_cost < s_prev.total_cost) { // while loop uitvoeren totdat er geen verbeteringen meer zijn 

			update_solution(p, s_local_best, s_prev);
			relocate(p, s_prev, s_curr, s_local_best);

			cout << "\nNew best solution after relocate with " << s_local_best.number_of_vehicles_used << " vehicles and distance " << s_local_best.total_distance_cost
				<< " route duration " << s_local_best.total_route_duration << " time window violation " << s_local_best.total_time_window_violation <<
				" overtime " << s_local_best.total_overtime << " allowable operating time " << s_local_best.total_driving_time << " total cost " << s_local_best.total_cost << "\n";
			for (int vehicle_id = 0; vehicle_id < p.n_vehicles; vehicle_id++) {
				for (size_t position = 0; position < s_local_best.routes[vehicle_id].route.size(); position++) {
					cout << s_local_best.routes[vehicle_id].route[position] << " ";
				}

				cout << "\n";
			}

			cout << "best cost na relocate " << s_local_best.total_cost << "\n";

		}

		if (s_local_best.total_cost < s_total_best.total_cost) {
			update_solution(p, s_local_best, s_total_best);

			cout << "total best " << s_total_best.total_cost << "\n";
		}

		if (s_local_best.total_cost < s_ILS_best.total_cost) {
			update_solution(p, s_local_best, s_ILS_best);

			cout << "ILS best " << s_ILS_best.total_cost << "\n";
		}

		else {
			number_of_times_without_improvement++;

			cout << "number of times without improvement " << number_of_times_without_improvement << "\n";
		}


		auto end = std::chrono::system_clock::now();

		std::chrono::duration<double> elapsed_seconds = end - start;
		std::time_t end_time = std::chrono::system_clock::to_time_t(end);

		//cout << "finished computation at " << ctime(&end_time) << "\n";
		//cout << "elapsed time: " << elapsed_seconds.count() << "s\n";

		write_output_file(p, s_ILS_best);
	}
	
}