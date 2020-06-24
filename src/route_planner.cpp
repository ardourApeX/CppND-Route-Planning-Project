#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;
    
    start_node = &m_Model.FindClosestNode(start_x, start_y);  //Closest Node to Start Node
    end_node = &m_Model.FindClosestNode(end_x, end_y);     //Closest Node to End Node

}



float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
     //Dereferencing end_node
	return(end_node -> distance(*node)); //Returning distance of node from end_node

}



void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
	current_node -> FindNeighbors(); //Calling FindNeighbors() using object current_node
	for(auto single_neighbor : current_node -> neighbors) //iterating over each node 
	{
		single_neighbor -> parent = current_node; //Setting up parent
        single_neighbor -> h_value = CalculateHValue(single_neighbor); //Setting up fvalue
		single_neighbor -> g_value = current_node -> g_value + current_node ->distance(*single_neighbor); //Setinng up gvalue
		open_list.push_back(single_neighbor); //Pushing single neighbor back to open_list
		single_neighbor -> visited = true; //Setting up visited attribute to ture.	

	}

}


//Created a new function to compare two nodes from open_list

/*bool Compare2Nodes(const RouteModel::Node *Node1, const RouteModel::Node *Node2)
{
	//Ternary Operator : It return true if Node1's Fvalue is greater than Node2's Fvalue otherwise false
	return (((Node1 -> g_value + Node1 -> h_value) > (Node2 -> g_value + Node2 -> h_value)) ? true : false);
}*/


RouteModel::Node *RoutePlanner::NextNode() {
//	sort(open_list.begin(), open_list.end(), Compare2Nodes); //Sorting open_list using Compare2Nodes Function
//Note : It is kinda impractical creating a whole function just for comparing two nodes
//Instead, use lambda function
    sort(open_list.begin(), open_list.end(), [](const RouteModel::Node *Node1, const RouteModel::Node *Node2) { return (Node1->h_value + Node1->g_value) > (Node2->h_value + Node2->g_value); });
	RouteModel::Node *Least = open_list.back(); //Just like the current node from the pervious exercise ie the one having least F_value
	open_list.pop_back();// Remove it from open_list as it is no longer for available for visit
	return Least;


}



std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;


    // TODO: Implement your solution here.
    while (current_node -> parent != nullptr)
    {
    	path_found.push_back(*current_node); // Adding current_node to path_found vector
    	distance += current_node -> distance(*(current_node -> parent)); // Adding up the distance btw current node and current node's parent
    	current_node = current_node -> parent;
    }

    path_found.push_back(*current_node); // Adding up last node
    std::reverse(path_found.begin(), path_found.end()); // Reversing Order
    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;


}



void RoutePlanner::AStarSearch() {

    RouteModel::Node *current_node = nullptr; 
    current_node = start_node; 
    open_list.push_back(start_node); // Added to open_list 
    start_node -> visited = true; 
    while(open_list.size() != 0) //Till open_list is empty
    {
    	AddNeighbors(current_node); // Finding out neighbors
    	current_node = NextNode(); // Sorted!
    	if(current_node == end_node) 
    	{
    		m_Model.path = ConstructFinalPath(current_node);
            return;
    	}

    }
}