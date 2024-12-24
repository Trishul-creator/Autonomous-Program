#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

class PIDController {
    public:
        PIDController(double kP, double kI, double kD);
        void setDesiredValue(int value);
        int calculate(int currentPosition);
        int signnum_c(int x);
        bool atTarget();

    private:
        double kP; // Proportional constant
        double kI; // Integral constant
        double kD; // Derivative constant
        int integralBound; // max integral value
        int maxIntegral; // max integral value

        int desiredValue; // how far it should go
        int error; // Error = desiredValue - currentPosition (Proportional Term of PID)
        int prevError; // Error from the last time calculate was called
        int totalError; // integral of error (Integral Term of PID)
        int derivative; // derivative of error (Derivative Term of PID) error - prevError

        
};

#endif //PIDCONTROLLER_H
