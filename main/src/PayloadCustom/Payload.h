#ifndef Payload_h
#define Payload_h

// Implement header code here
class Payload {
    public:
        Payload(); // Declare the constructor
        int deployPayload(void);
        void collectPayload(void);
        int getCycle();
    private:
        int readOneCycle(int mils);
};

#endif