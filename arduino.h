#ifndef ARDUINO_H
#define ARDUINO_H






#define TWO_INCLUDE_DIRECT_COMPILATION_MODE





#include <iostream>
#include <array>
#include <string>
#include <vector>
#include <tuple>
#include <chrono>
#include <thread>
#include <mutex>

#include <ctime>
#include <cassert>
#include <cstdlib>
#include <cstdio>
#include <cmath>
#include <cstring>




#include "settings.h"





#define A0 14
#define A1 15
#define A2 16
#define A3 17
#define A4 18
#define A5 19
#define A6 20
#define A7 21




enum
#ifndef TWO_INCLUDE_DIRECT_COMPILATION_MODE
        pinModes
#endif
{
    OUTPUT = 1,
    INPUT = 2,
    INPUT_PULLUP = 4
};


enum
#ifndef TWO_INCLUDE_DIRECT_COMPILATION_MODE
        pinDigitalValue
#endif
{
    LOW = 0,
    HIGH
};







namespace myArd
{


extern const std::array <std::string, 5> pinModesString;


extern const unsigned char digitalPinsRangeLow;
extern const unsigned char digitalPinsRangeHigh;
extern const unsigned char analogPinsRangeLow;
extern const unsigned char analogPinsRangeHigh;

#define controlablePinNumber 22//extern const size_t controlablePinNumber;

extern const float maximumPinTotalDrownCurrent;
extern const float maximumPinIndividualDrownCurrent;
extern const float internalPullupImpedence;
extern const float internalInputModeImpedence;
extern const float internalRailInputModeImpedence;
extern const float notRailMaximumVoltage;
extern const float notRailMinimumVoltage;
extern const float internalOutputModeImpedence;
extern const float upParasiteVoltage;
extern const float downParasiteVoltage;
extern const float maximumToleratedVoltage;
extern const float minimumToleratedVoltage;
extern const float veryHighResistance;
extern const float internalBaseLeakVoltage;

#define eepromSize 512//extern const size_t eepromSize;

extern const float randomFactor;
extern const float INPUT_PULLUP_VOLTAGE;
extern const float flatVoltageNoise;

extern const float stateImpedences[5];///problem here


enum alterationCallReason
{
    acrInit,
    acrTime,
    acrSetup,
    acrLoop,
    acrFAnalogRead,
    acrFDigitalRead,
    acrFMillis,
    acrFDelay,
    acrFDigitalWrite,
    acrFPinMode
};


bool breakAndAlter(alterationCallReason r);



extern std::array <bool, 10> breakingTicks;
extern unsigned long callbackInterval;



///ORDER OF THE VECTOR IS IMPORTANT, AND CHANGES THE SECOND MEMBER OF THE RETURNED PAIR
inline std::pair <float/*voltage*/, float/*current through first pin, index 0 of the vector (order is important=*/>
    calculerResultatClassic(std::vector <std::pair<float/*voltage*/, float/*impedance*/>>protagonistes)
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


inline std::pair <float/*voltage*/, float/*current through first pin, index 0 of the vector (order is important=*/>
    calculerResultat(std::vector <std::pair<float/*voltage*/, float/*impedance*/>>protagonistes)
{
    std::pair <float, float> ret(0, 0);
    std::vector <float> transmittances;
    std::vector <float> randomFactors;
    std::vector <std::pair<float/*voltage*/, float/*impedance*/>> futureProtagonist;
    float rfTot = 0;//which becomes ponderation of external random voltage
    for(auto &a : protagonistes)
    {
        if(std::isnan(a.first))
            randomFactors.push_back(1.0/a.second);
        else
            transmittances.push_back(1.0/a.second), futureProtagonist.push_back(a);
    }
    futureProtagonist.swap(protagonistes);
    assert(transmittances.size() == protagonistes.size());
    for(auto a : randomFactors)
        rfTot += a;
    float totalTransmittance = 0;
    for(unsigned int i = 0; i < transmittances.size(); ++i)
        totalTransmittance += transmittances[i]
        , ret.first += transmittances[i]*protagonistes[i].first;
    ret.first /= totalTransmittance;
    rfTot = rfTot/(rfTot + totalTransmittance);
    std::pair<float/*up*/, float/*down*/> worstVoltageCases
        (ret.first + (upParasiteVoltage*rfTot)
        , ret.first + (downParasiteVoltage*rfTot));
    ret.second = std::max(std::abs(protagonistes[0].first - worstVoltageCases.first)
        , std::abs(protagonistes[0].first - worstVoltageCases.second))* transmittances[0];
    ret.first = ((worstVoltageCases.first - maximumToleratedVoltage)
        > (minimumToleratedVoltage - worstVoltageCases.second)
        ? worstVoltageCases.first : worstVoltageCases.second);
    float noise = ((float)((rand() % 2001) - 1000)) / 1000.0;
    ret.first += (noise * flatVoltageNoise);
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
    std::array <float, controlablePinNumber> pinVoltage;/// //if set as output, voltage doesnt changes as impedence is considered null
        // , except when digital write is called

    std::array <char, eepromSize> eeprom;

    std::array <pinModes, controlablePinNumber> pinState;//1 = OUTPUT, 2 = INPUT, 4 = INPUT_PULLUP
    std::array <std::pair <float/*voltage*/, float/*impedance*/>, controlablePinNumber> pinExternalPlugged;
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
            arduino::defArd.putEeprom(reinterpret_cast<char*>(&v), adress, sizeof(v));
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


class millisClass
{
public:
    unsigned long operator ()();
    void startPoint();
    void pause();
    void unpause();
    bool paused();
    unsigned long long inClock();

    static millisClass millisStaticMember;

    void operator -=(unsigned long t);
    void operator +=(unsigned long t);

private:
    unsigned long long int substract;
    bool paused_m;

};




template <typename T, typename ...Args>
void caller(T f, unsigned long long period/*given directly to delay()*/
    , const bool &c, Args... a);




}




void altererDefArduino(myArd::arduino&, myArd::alterationCallReason);





extern myArd::eepromClass &EEPROM;
extern myArd::millisClass &millis;


inline void pinMode(unsigned char pin, pinModes m)
{
    if(myArd::breakAndAlter(myArd::acrFPinMode))
        __asm("int $3");
    myArd::arduino::defArd.pinMode(pin, m);
}

inline void digitalWrite(unsigned char pin, pinDigitalValue v)
{
    if(myArd::breakAndAlter(myArd::acrFDigitalWrite))
        __asm("int $3");
    return myArd::arduino::defArd.digitalWrite(pin, v);
}

inline pinDigitalValue digitalRead(unsigned char pin)
{
    pinDigitalValue dbtbm_0 = myArd::arduino::defArd.digitalRead(pin);
    if(myArd::breakAndAlter(myArd::acrFDigitalRead))
        __asm("int $3");
    return dbtbm_0;
}

inline int analogRead(unsigned char pin)
{
    int dbtbm_0 = myArd::arduino::defArd.analogRead(pin);
    if(myArd::breakAndAlter(myArd::acrFAnalogRead))
        __asm("int $3");
    return dbtbm_0;
}



void delay(unsigned long ms);




template <typename T, typename ...Args>
void myArd::caller(T f, unsigned long long period/*given directly to delay()*/
    , const bool &c, Args... a)
{
    for(; c; std::this_thread::sleep_for(
            std::chrono::milliseconds(period))
        , f(a...));
    return;
}



#endif // ARDUINO_H
