#include <iostream>
#include <wiringPi.h>
#include <time.h>
#include <unistd.h>

void initTime() {wiringPiSetup();}

double getTime() {
    pinMode(1, OUTPUT);
    clock_t start, end;
    digitalWrite(1, HIGH);
    usleep(50);
    digitalWrite(1, LOW);
    pinMode(1, INPUT);
    usleep(25);
    while (!digitalRead(1)) ;
    start = clock();
    while (digitalRead(1)) ;
    end = clock();
    unsigned long us = end - start;
    return (((double)us / 1000000.0) * 34300) / 2;
}
