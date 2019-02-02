#include <iostream>
#include <wiringPi.h>
#include <time.h>
#include <unistd.h>

void initTime() {wiringPiSetup();}

double getTime() {
    pinMode(0, OUTPUT);
    clock_t start, end;
    digitalWrite(0, HIGH);
    usleep(50);
    digitalWrite(0, LOW);
    pinMode(0, INPUT);
    usleep(25);
    while (!digitalRead(0)) ;
    start = clock();
    while (digitalRead(0)) ;
    end = clock();
    unsigned long us = end - start;
    return (((double)us / 1000000.0) * 34300) / 2;
}
