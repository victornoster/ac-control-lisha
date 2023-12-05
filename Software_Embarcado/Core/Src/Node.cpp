/*
   File Node.cpp

   Class Node

   Class Node stores integer values for a linked list.

   This file has the implementation for the Node's interface.

   Eduardo Augusto Bezerra <eduardo.bezerra@ufsc.br>
   Departamento de Engenharia Eletrica

   Data da criacao: Abril de 2006.
   Data da ultima alteracao: 8 de outubro de 2015.

*/

#include "Node.hpp"

// Constructor - initializes the node
//
Node::Node(int sensor_id, int sensor_value, int door_open, int ac_on, int hr, int min, int seg, int isPM, Node* nxt){
	id = sensor_id;
	sensorValue = sensor_value;
	doorOpen = door_open;
	acOn = ac_on;
	hora = hr;
	minuto = min;
	PM = isPM;
	segundo = seg;
	next = nxt;
}

// getVal returns the integer value stored in the node
//
int Node::getId(){
     return id;
}

int Node::getSensorValue() {
	return sensorValue;
}
int Node::getDoorOpen() {
	return doorOpen;
}
int Node::getAcOn() {
	return acOn;
}
int Node::getHora() {
	return hora;
}
int Node::getMinuto() {
	 return minuto;
}
int Node::getSegundo() {
	return segundo;
}
int Node::getPM() {
	return PM;
}
// getNext returns a pointer for the next node in the linked list
//
Node* Node::getNext(){
     return next;
}

// setVal stores the integer value in the node
//
void Node::setVal(int sensor_id, int sensor_value, int door_open, int ac_on, int hr, int min, int seg, int isPM){
     id = sensor_id;
     sensorValue = sensor_value;
     doorOpen = door_open;
     acOn = ac_on;
     hora = hr;
     minuto = min;
     segundo = seg;
     PM = isPM;
}

// setNext stores the pointer to the next node in the list in the "next" field
//
void Node::setNext(Node* nxt){
       next = nxt;
}


