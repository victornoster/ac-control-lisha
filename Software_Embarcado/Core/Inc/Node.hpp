/*
   File Node.hpp

   Class Node

   Class Node stores integer values for a linked list.

   This file has the Node's interface (header file).

   Eduardo Augusto Bezerra <eduardo.bezerra@ufsc.br>
   Departamento de Engenharia Eletrica

   Data da criacao: Abril de 2006.
   Data da ultima alteracao: 8 de outubro de 2015.
	Modified by Victor Noster
*/

#ifndef INC_NODE_HPP_
#define INC_NODE_HPP_


//#include <iostream>
//
//using namespace std;

class Node {

    int id;
    int sensorValue;
    int doorOpen;
    int acOn;
    int hora, minuto, segundo, PM;
    Node* next;

  public:

    Node(int sensor_id, int sensor_value, int door_open, int ac_on, int hr, int min, int seg, int isPM, Node* nxt);
    int getId();
    int getSensorValue();
    int getDoorOpen();
    int getAcOn();
    int getHora();
    int getMinuto();
    int getSegundo();
    int getPM();
    Node* getNext();
    void setVal(int sensor_id, int sensor_value, int door_open, int ac_on, int hr, int min, int seg, int isPM);
    void setNext(Node* nxt);
};


#endif /* INC_NODE_HPP_ */
