void setupLEDs()
{
    pinMode(Blue, OUTPUT);
    pinMode(Green, OUTPUT);
}

void onBlue()
{
    digitalWrite(Blue, HIGH);
}

void onGreen()
{
    digitalWrite(Green, HIGH);
}

void offBlue()
{
    digitalWrite(Blue, LOW);
}

void offGreen()
{
    digitalWrite(Green, LOW);
}