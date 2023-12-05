/*
 * Sensor.h
 */

#ifndef __SENSOR_H__
#define __SENSOR_H__

template<typename T>
class Sensor {
        int id;
        //float temp;
    protected:
        int getID();
        void setID(int newID);
        virtual T readSensor() = 0;      // funcao virtual pura
};

template<typename T>
int Sensor<T>::getID(){
    return id;
}

template<typename T>
void Sensor<T>::setID(int newID){
    id = newID;
}

#endif /* SENSOR_H_ */
