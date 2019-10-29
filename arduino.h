#ifndef ARDUINO_H
#define ARDUINO_H





#include <iostream>
#include <array>
#include <string>




#define A0 14
#define A1 15
#define A2 16
#define A3 17
#define A4 18
#define A5 19
#define A6 20
#define A7 21











namespace myArd
{


extern const unsigned char digitalPinsRangeLow;
extern const unsigned char digitalPinsRangeHigh;
extern const unsigned char analogPinsRangeLow;
extern const unsigned char analogPinsRangeHigh;

extern const size_t controlablePinNumber;

extern const float maximumPinTotalDrownCurrent;
extern const float maximumPinIndividualDrownCurrent;



enum arduinoState
{
    dead,
    working
};


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


class arduino
{
public:

    //fonction pour récupérer (et non afficher) des infos sur l'arduino
    arduinoState getState();
    std::array <char, 512> getEeprom();

    //fonction d'affichage
    arduino &operator<<(std::string s);
    void printSentenceAndActualize(std::string s);
    void clearAndActualize();
    void actualize();

    //fonction de "simulation" d'environnement et d'états interne de l'arduino
    void changePluggedOutput(float voltage, float impedance);
    void setEeprom(std::array <char, 512> input);


    //fonctiond de changement d'état interne
    void pinMode(unsigned char pin, pinModes m);
    void digitalWrite(unsigned char pin, pinDigitalValue v);
    void digitalRead(unsigned char pin);
    int analogRead(unsigned char pin);



    static arduino defArd;


private:

    std::array <char, controlablePinNumber> state;//1 = OUTPUT, 2 = INPUT, 4 = INPUT_PULLUP
    std::array <float, controlablePinNumber> worstDrownCurrent;
    std::array<float, controlablePinNumber> pinVoltage;//if set as output, voltage doesnt changes as impedence is considered null
        // , except when digital write is called
    std::string currentlyPrinted;




    bool checkedSumCurrent();//valid if = true, dead else
    bool checkVoltage();//valid if = true, dead else
};



}


void pinMode(unsigned char pin, pinModes m);
void digitalWrite(unsigned char pin, pinDigitalValue v);
void digitalRead(unsigned char pin);
int analogRead(unsigned char pin);


#endif // ARDUINO_H
