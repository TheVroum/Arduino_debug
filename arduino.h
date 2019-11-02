#ifndef ARDUINO_H
#define ARDUINO_H





#include <iostream>
#include <array>
#include <string>
#include <cstring>
#include <vector>
#include <tuple>
#include <chrono>
#include <thread>

#include <ctime>
#include <cassert>
#include <cstdlib>
#include <cstdio>
#include <cmath>




#define A0 14
#define A1 15
#define A2 16
#define A3 17
#define A4 18
#define A5 19
#define A6 20
#define A7 21




enum pinModes
{
    OUTPUT = 1,
    INPUT = 2,
    INPUT_PULLUP = 4
};


enum pinDigitalValue
{
    LOW = 0,
    HIGH
};







namespace myArd
{


extern const unsigned char digitalPinsRangeLow;
extern const unsigned char digitalPinsRangeHigh;
extern const unsigned char analogPinsRangeLow;
extern const unsigned char analogPinsRangeHigh;

#define controlablePinNumber 22//extern const size_t controlablePinNumber;

extern const float maximumPinTotalDrownCurrent;
extern const float maximumPinIndividualDrownCurrent;
extern const float internalPullupImpedence;
extern const float internalInputModeImpedence;
extern const float internalOutputModeImpedence;

#define eepromSize 512//extern const size_t eepromSize;

extern const float randomFactor;
extern const float INPUT_PULLUP_VOLTAGE;

extern const float stateImpedences[4];



///ORDER OF THE VECTOR IS IMPORTANT, AND CHANGES THE SECOND MEMBER OF THE RETURNED PAIR
inline std::pair <float/*voltage*/, float/*current through first pin, index 0 of the vector (order is important=*/>
    calculerResultat(std::vector <std::pair<float/*voltage*/, float/*impedance*/>>protagonistes)
{
    std::pair <float, float> ret(0, 0);
    std::vector <float> transmittances;
    for(auto a : protagonistes)
        transmittances.push_back(1.0/a.second);
    float totalTransmittance = 0;
    for(unsigned int i = 0; i < transmittances.size(); ++i)
        totalTransmittance += transmittances[i]
        , ret.first += transmittances[i]*protagonistes[i].first;
    ret.first /= totalTransmittance;
    ret.second = std::abs(protagonistes[0].first - ret.first) * transmittances[0];
    return ret;
}




enum arduinoState
{
    dead,
    working
};




class arduino
{
public:

    arduino();///

    //fonction pour récupérer (et non afficher) des infos sur l'arduino
    arduinoState getState();///
    std::array <char, 512> getWholeEeprom();///

    //fonction d'affichage
    arduino &operator<<(std::string s);///
    void printSentenceAndActualize(std::string s);///
    void clearAndActualize();///
    void actualize();///
    ///ICI penser à rajouter un effet pour que le courant maximal (et le voltage) soit toujours dans le pire cas.

    //fonction de "simulation" d'environnement et d'états interne de l'arduino
    void changePluggedOutput(unsigned char pin, float voltage, float impedance);///
    void setWholeEeprom(std::array <char, 512> input);///


    //fonctions de changement d'état interne
    void pinMode(unsigned char pin, pinModes m);///
    void digitalWrite(unsigned char pin, pinDigitalValue v);///
    pinDigitalValue digitalRead(unsigned char pin);///
    int analogRead(unsigned char pin);///
    void putEeprom(char *outputData, size_t index, size_t size);///
    void getEeprom(char *inputData, size_t index, size_t size);///



    static arduino defArd;


private:

    std::array <float, controlablePinNumber> pinWorstDrownCurrent;
    std::array<float, controlablePinNumber> pinVoltage;//if set as output, voltage doesnt changes as impedence is considered null
        // , except when digital write is called

    std::array <char, eepromSize> eeprom;///

    std::array <pinModes, controlablePinNumber> pinState;//1 = OUTPUT, 2 = INPUT, 4 = INPUT_PULLUP
    std::array<std::pair <float/*voltage*/, float/*impedance*/>, controlablePinNumber> pinExternalPlugged;
    std::array <float, controlablePinNumber> pinInternalPlugged;

    std::string currentlyPrinted;///
    arduinoState state;///

};




class eepromClass
{
public:
    static eepromClass eepromStaticMember;
    template <typename T>
        inline T& put(int adress, T &v)
        {
            arduino::defArd.getEeprom(reinterpret_cast<char*>(&v), adress, sizeof(v));
            return v;
        }
    template <typename T>
        inline T& get(int adress, T& v)
        {
            arduino::defArd.getEeprom(reinterpret_cast<char*>(&v), adress, sizeof(v));
            return v;
        }
    eepromClass() = default;
};





}



extern myArd::eepromClass &EEPROM;



inline void pinMode(unsigned char pin, pinModes m)
{
    myArd::arduino::defArd.pinMode(pin, m);
}

inline void digitalWrite(unsigned char pin, pinDigitalValue v)
{
    return myArd::arduino::defArd.digitalWrite(pin, v);
}

inline pinDigitalValue digitalRead(unsigned char pin)
{
    return myArd::arduino::defArd.digitalRead(pin);
}

inline int analogRead(unsigned char pin)
{
    return myArd::arduino::defArd.analogRead(pin);
}



void delay(unsigned long ms);

unsigned long millis();




#endif // ARDUINO_H
