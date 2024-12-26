#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

class PIDController {
    public:
        PIDController(double kP, double kI, double kD, double turnkP, double turnkI, double turnkD);
        void setDesiredValue(int value);
        void setTurnDesiredValue(int value);
        int calculate(int currentPosition);
        int calculateTurn(int currentAngle);
        bool atTarget();
        bool atTurnTarget();

    private:
        double kP; // Proportional constant
        double kI; // Integral constant
        double kD; // Derivative constant

        double turnkP; // Turn Propotional constant
        double turnkI; // Turn Integral constant
        double turnkD; // Turn Derivative constant

        int integralBound; // max integral value
        int maxIntegral; // max integral value

        double desiredValue; // how far it should go
        double turnDesiredValue; // how far it should turn
        double error; // Error = desiredValue - currentPosition (Proportional Term of PID)
        double turnError; // Error = desiredValue - currentPosition (Proportional Term of PID)
        double prevError; // Error from the last time calculate was called
        double turnPrevError; // Error from the last time calculate was called
        double totalError; // integral of error (Integral Term of PID)
        double totalTurnError; // integral of error (Integral Term of PID)
        double derivative; // derivative of error (Derivative Term of PID) error - prevError
        double turnDerivative; // derivative of error (Derivative Term of PID) error - prevError

        
};

#endif //PIDCONTROLLER_H