/*

This file is a part of program liftsim for the course project of the course CS 293 - Data Structures and Algorithms Lab.

File Name : Customer.cpp

Copyright (C) 2011 Nivvedan, Ankush Das

    liftsim is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    liftsim is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with liftsim.  If not, see <http://www.gnu.org/licenses/>.
    
    
Authors:

Nivvedan
nivvedan@iitb.ac.in

Ankush Das
ankushdas@iitb.ac.in



File Created on : Wednesday, October 12, 2011.
Last modified on : Tuesday, 15 November, 2011.
Modified since 2026 by Bernardo Fioreze as part of OpenEGCS (see NOTICE).

*/

#include<iostream>
#include<string>
#include<cstdlib>
#include<ctime>

#include "Customer.h"

using namespace std;

namespace openEGCS {

Customer::Customer(unsigned long int cus_id, string cus_name, double t, int at_floor, int to_floor, int direction, double cus_weight) {
	id = cus_id;
	if (cus_name == "") cus_name = "Un-named Customer";
	name = cus_name;

	weight = cus_weight;
	timestamp = t;
	atFloor = at_floor;
	toFloor = to_floor;
	goes = direction;
	}

unsigned long int Customer::getId() const {
	return id;
	}


unsigned long int Customer::setId(unsigned long int newId) {
	id = newId;
	return id;
	}
			
string Customer::getName() const {
	return name;
	}

string Customer::setName(string newName) {
	if (newName == "") newName = "Un-named Customer";
	name = newName;
	return name;
	}

double Customer::getWeight() const {
	return weight;
}
	
double Customer::getTime() const {
	return timestamp;
	}
	
int Customer::getAt() const {
	return atFloor;
	}

int Customer::getTo() const {
	return toFloor;
	}

int Customer::getDir() const {
	return goes;
	}

bool Customer::operator ==(Customer* cust)
{
	return ((timestamp == cust->timestamp) && (atFloor == cust->atFloor) && (toFloor == cust->toFloor) && (goes == cust->goes));
}

bool Customer::operator ==(Customer cust)
{
	return ((timestamp == cust.timestamp) && (atFloor == cust.atFloor) && (toFloor == cust.toFloor) && (goes == cust.goes));
}

const Customer& Customer::operator =(const Customer& cust)
{	
	id = cust.id;
	name = cust.name;
	weight = cust.weight;
	timestamp = cust.timestamp;
	atFloor = cust.atFloor;
	toFloor = cust.toFloor;
	goes = cust.goes;
	return *this;
}

bool Customer::operator ==(vector<Customer>::iterator it) {
	// A value comparison; does not modify either side.
	return ((timestamp == it->timestamp) &&
	(atFloor == it->atFloor) &&
	(toFloor == it->toFloor) &&
	(goes == it->goes));
}

} // namespace openEGCS
